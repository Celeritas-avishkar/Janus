import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from janus import protocol as PR


def wire(loss=0.0, corrupt=0.0, seed=1):
    rng = random.Random(seed)
    a_out, b_out = [], []
    a = PR.Endpoint(0, a_out.append, peer=1)
    b = PR.Endpoint(1, b_out.append, peer=0)

    def flush(now):
        got = []
        for src, dst, out in ((a, b, a_out), (b, a, b_out)):
            msgs = list(out); out.clear()
            for d in msgs:
                if rng.random() < loss:
                    continue
                if corrupt and rng.random() < corrupt:
                    d = bytearray(d); d[rng.randrange(len(d))] ^= 0x10; d = bytes(d)
                for m in dst.on_datagram(d, now):
                    got.append((dst.node, m[0], m[1]))
        return got
    return a, b, flush


def test_roundtrip_and_crc():
    f = PR.encode(2, 0, 1, 7, 0, 0, 1, 1.5, b'{"x":1}')
    fr = PR.decode(f)
    assert fr.seq == 7 and fr.name == "TM"
    bad = bytearray(f); bad[20] ^= 1
    try:
        PR.decode(bytes(bad)); assert False
    except PR.BadFrame as e:
        assert str(e) == "CRC"


def test_reliable_fragmented_under_loss_and_corruption():
    a, b, flush = wire(loss=0.3, corrupt=0.2, seed=4)
    rnd = random.Random(99)
    big = {"route": [[i, i % 17, i % 9] for i in range(300)], "pad": "".join(rnd.choice("0123456789abcdef") for _ in range(6000))}
    done = []
    a.send_reliable("MISSION", big, 0.0, max_attempts=200, on_done=lambda m, rtt, att: done.append(att))
    got, t = [], 0.0
    for _ in range(600):
        t += 0.05
        got += flush(t)
        a.tick(t); b.tick(t)
        if done:
            break
    assert done, "message never acknowledged"
    assert [m for m in got if m[1] == "MISSION"] == [(1, "MISSION", big)], "delivered exactly once, intact"
    assert len(PR.pack_payload(big)[0]) > 2 * PR.MAX_FRAG, "test must exercise fragmentation"
    assert b.stats["crc_fail"] + a.stats["crc_fail"] > 0 and a.stats["retx"] > 0


def test_latest_only_key_supersedes():
    a, b, flush = wire(loss=1.0)
    a.send_reliable("WX", {"v": 1}, 0.0, key="WX", max_attempts=None)
    a.send_reliable("WX", {"v": 2}, 0.1, key="WX", max_attempts=None)
    assert [p["key"] for p in a.pending()] == ["WX"] and len(a.pending()) == 1


def test_duplicates_not_redelivered():
    a, b, flush = wire()
    a.send_reliable("CMD", {"c": "RTL"}, 0.0)
    out = []
    a.sendfn = out.append
    first = flush(0.05)
    assert [m[1] for m in first].count("CMD") == 1
    for o in list(a.out.values()):
        o.acked.clear()
    a.out.clear()
