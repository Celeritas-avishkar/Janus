"""Janus wire protocol: framed UDP datagrams with CRC32, fragmentation and
selective-repeat retransmission.

Datagram layout (network byte order):
    magic "JN" | ver u8 | type u8 | flags u8 | node u8 | seq u32 | msg_id u32 |
    frag_idx u8 | frag_cnt u8 | ts f64 | payload_len u16 | payload | crc32 u32
flags: 1 = reliable, 2 = ack, 4 = zlib-compressed payload

Reliable messages are acknowledged per fragment with a bitmap; the sender
retransmits only missing fragments. Messages with a `key` are latest-only: a
newer message with the same key replaces an older one still in flight (used
for hazard pictures and route updates).
"""
import json
import struct
import zlib
from collections import OrderedDict

from . import config as C

MAGIC = b"JN"
VERSION = 1
HDR = struct.Struct("!2sBBBBIIBBdH")
CRC = struct.Struct("!I")
F_REL, F_ACK, F_ZLIB = 1, 2, 4
MAX_FRAG = 1000
ZLIB_OVER = 400

TYPES = {"HB": 1, "TM": 2, "WX": 3, "MISSION": 4, "RESULT": 5, "CMD": 6, "ROUTE": 7,
         "HZR": 8, "BB": 9, "ACK": 10, "HELLO": 11}
NAMES = {v: k for k, v in TYPES.items()}


class Frame:
    __slots__ = ("mtype", "flags", "node", "seq", "msg_id", "fidx", "fcnt", "ts", "payload")

    def __init__(self, mtype, flags, node, seq, msg_id, fidx, fcnt, ts, payload):
        self.mtype, self.flags, self.node, self.seq = mtype, flags, node, seq
        self.msg_id, self.fidx, self.fcnt, self.ts, self.payload = msg_id, fidx, fcnt, ts, payload

    @property
    def name(self):
        return NAMES.get(self.mtype, "?")


def encode(mtype, flags, node, seq, msg_id, fidx, fcnt, ts, payload):
    body = HDR.pack(MAGIC, VERSION, mtype, flags, node, seq & 0xFFFFFFFF, msg_id & 0xFFFFFFFF,
                    fidx, fcnt, float(ts), len(payload)) + payload
    return body + CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


class BadFrame(Exception):
    pass


def decode(data):
    if len(data) < HDR.size + CRC.size:
        raise BadFrame("SHORT")
    body, crc = data[:-CRC.size], CRC.unpack(data[-CRC.size:])[0]
    magic, ver, mtype, flags, node, seq, msg_id, fidx, fcnt, ts, plen = HDR.unpack(body[:HDR.size])
    if magic != MAGIC or ver != VERSION:
        raise BadFrame("MAGIC")
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise BadFrame("CRC")
    payload = body[HDR.size:]
    if len(payload) != plen or fcnt == 0 or fidx >= fcnt:
        raise BadFrame("LENGTH")
    return Frame(mtype, flags, node, seq, msg_id, fidx, fcnt, ts, payload)


def peek_header(data):
    """Best-effort header read without CRC check (used by the relay for logging)."""
    try:
        magic, ver, mtype, flags, node, seq, msg_id, fidx, fcnt, ts, plen = HDR.unpack(data[:HDR.size])
        if magic != MAGIC:
            return None
        return {"type": NAMES.get(mtype, "?"), "flags": flags, "node": node, "seq": seq,
                "msg": msg_id, "fidx": fidx, "fcnt": fcnt, "len": len(data)}
    except struct.error:
        return None


def pack_payload(obj):
    raw = json.dumps(obj, separators=(",", ":")).encode()
    if len(raw) > ZLIB_OVER:
        comp = zlib.compress(raw, 6)
        if len(comp) < len(raw):
            return comp, F_ZLIB
    return raw, 0


def unpack_payload(data, flags):
    if flags & F_ZLIB:
        data = zlib.decompress(data)
    return json.loads(data.decode())


class _Out:
    __slots__ = ("msg_id", "mtype", "chunks", "flags", "acked", "attempts", "next_t", "key",
                 "max_attempts", "on_done", "on_fail", "first_t", "ts")


class Endpoint:
    """One side of the link. `sendfn(bytes)` puts a datagram on the wire.
    Time is always passed in explicitly so the same code runs in real time
    (separate processes) and in fast headless simulation."""

    def __init__(self, node, sendfn, peer=None):
        self.node = node
        self.peer = peer
        self.sendfn = sendfn
        self.seq = 0
        self.msg_id = 0
        self.out = OrderedDict()
        self.keyed = {}
        self.partial = {}
        self.done = OrderedDict()
        self.last_peer_seq = None
        self.gaps = []
        self.stats = {"tx": 0, "rx": 0, "crc_fail": 0, "bad": 0, "dup": 0, "retx": 0,
                      "timeouts": 0, "delivered": 0, "acked": 0, "gap_lost": 0, "stale": 0, "foreign": 0}

    # ---------------------------------------------------------- sending
    def _emit(self, mtype, flags, msg_id, fidx, fcnt, ts, payload):
        self.seq += 1
        self.stats["tx"] += 1
        self.sendfn(encode(mtype, flags, self.node, self.seq, msg_id, fidx, fcnt, ts, payload))

    def send(self, name, obj, now):
        data, zf = pack_payload(obj)
        if len(data) > MAX_FRAG:
            return self.send_reliable(name, obj, now, max_attempts=1)
        self._emit(TYPES[name], zf, 0, 0, 1, now, data)
        return 0

    def send_reliable(self, name, obj, now, key=None, max_attempts=C.MAX_ATTEMPTS, on_done=None, on_fail=None):
        data, zf = pack_payload(obj)
        chunks = [data[i:i + MAX_FRAG] for i in range(0, len(data), MAX_FRAG)] or [b""]
        if len(chunks) > 255:
            raise ValueError("message too large")
        if key is not None and key in self.keyed:
            old = self.out.pop(self.keyed[key], None)
            if old is not None:
                self.stats["superseded"] = self.stats.get("superseded", 0) + 1
        self.msg_id += 1
        o = _Out()
        o.msg_id, o.mtype, o.chunks, o.flags = self.msg_id, TYPES[name], chunks, F_REL | zf
        o.acked, o.attempts, o.key, o.max_attempts = set(), 0, key, max_attempts
        o.on_done, o.on_fail, o.first_t, o.ts = on_done, on_fail, now, now
        o.next_t = now
        self.out[o.msg_id] = o
        if key is not None:
            self.keyed[key] = o.msg_id
        self._transmit(o, now)
        return o.msg_id

    def _transmit(self, o, now):
        if o.attempts:
            self.stats["retx"] += 1
        o.attempts += 1
        for i, ch in enumerate(o.chunks):
            if i not in o.acked:
                self._emit(o.mtype, o.flags, o.msg_id, i, len(o.chunks), now, ch)
        back = min(1.0, C.RETRY_S * (1.25 ** min(o.attempts, 6)))
        o.next_t = now + back

    def tick(self, now):
        for mid in list(self.out.keys()):
            o = self.out.get(mid)
            if o is None or now < o.next_t:
                continue
            if o.max_attempts is not None and o.attempts >= o.max_attempts:
                self.out.pop(mid, None)
                if o.key is not None and self.keyed.get(o.key) == mid:
                    self.keyed.pop(o.key, None)
                self.stats["timeouts"] += 1
                if o.on_fail:
                    o.on_fail(mid)
                continue
            self._transmit(o, now)

    def pending(self):
        return [{"msg": o.msg_id, "type": NAMES[o.mtype], "attempts": o.attempts,
                 "frags": len(o.chunks), "acked": len(o.acked), "key": o.key} for o in self.out.values()]

    # ---------------------------------------------------------- receiving
    def on_datagram(self, data, now, max_age=None):
        """Returns a list of (name, payload, frame) for fully received messages."""
        try:
            f = decode(data)
        except BadFrame as e:
            self.stats["crc_fail" if str(e) == "CRC" else "bad"] += 1
            return []
        except Exception:
            self.stats["bad"] += 1
            return []
        if self.peer is not None and f.node != self.peer:
            self.stats["foreign"] += 1
            return []
        self.stats["rx"] += 1
        if self.last_peer_seq is not None and f.seq > self.last_peer_seq + 1:
            missing = f.seq - self.last_peer_seq - 1
            self.stats["gap_lost"] += missing
            if missing >= 5:
                self.gaps.append({"t": round(now, 2), "from": self.last_peer_seq, "to": f.seq, "missing": missing})
                self.gaps = self.gaps[-20:]
        if self.last_peer_seq is None or f.seq > self.last_peer_seq:
            self.last_peer_seq = f.seq

        if f.flags & F_ACK:
            self._on_ack(f, now)
            return []
        if not (f.flags & F_REL):
            if f.fcnt != 1:
                self.stats["bad"] += 1
                return []
            if max_age is not None and now - f.ts > max_age:
                self.stats["stale"] += 1
                return []
            try:
                obj = unpack_payload(f.payload, f.flags)
            except Exception:
                self.stats["bad"] += 1
                return []
            self.stats["delivered"] += 1
            return [(f.name, obj, f)]
        return self._on_reliable(f, now)

    def _on_reliable(self, f, now):
        if f.msg_id in self.done:
            self.stats["dup"] += 1
            self._ack(f.msg_id, (1 << f.fcnt) - 1, True, now)
            return []
        p = self.partial.get(f.msg_id)
        if p is None or p["cnt"] != f.fcnt:
            p = {"cnt": f.fcnt, "frags": {}, "t": now, "flags": f.flags, "mtype": f.mtype}
            self.partial[f.msg_id] = p
        p["frags"][f.fidx] = f.payload
        mask = 0
        for i in p["frags"]:
            mask |= 1 << i
        complete = len(p["frags"]) == p["cnt"]
        self._ack(f.msg_id, mask, complete, now)
        if not complete:
            return []
        del self.partial[f.msg_id]
        self.done[f.msg_id] = now
        while len(self.done) > 1024:
            self.done.popitem(last=False)
        data = b"".join(p["frags"][i] for i in range(p["cnt"]))
        try:
            obj = unpack_payload(data, p["flags"])
        except Exception:
            self.stats["bad"] += 1
            return []
        self.stats["delivered"] += 1
        f.payload = data
        return [(NAMES.get(p["mtype"], "?"), obj, f)]

    def _ack(self, msg_id, mask, complete, now):
        body = json.dumps({"m": msg_id, "b": mask, "d": 1 if complete else 0}, separators=(",", ":")).encode()
        self._emit(TYPES["ACK"], F_ACK, 0, 0, 1, now, body)

    def _on_ack(self, f, now):
        try:
            a = json.loads(f.payload.decode())
        except Exception:
            self.stats["bad"] += 1
            return
        o = self.out.get(a.get("m"))
        if o is None:
            return
        mask = int(a.get("b", 0))
        for i in range(len(o.chunks)):
            if mask & (1 << i):
                o.acked.add(i)
        if a.get("d") or len(o.acked) == len(o.chunks):
            self.out.pop(o.msg_id, None)
            if o.key is not None and self.keyed.get(o.key) == o.msg_id:
                self.keyed.pop(o.key, None)
            self.stats["acked"] += 1
            if o.on_done:
                o.on_done(o.msg_id, now - o.first_t, o.attempts)

    def purge_partial(self, now, older_than=10.0):
        for mid in [m for m, p in self.partial.items() if now - p["t"] > older_than]:
            del self.partial[mid]
