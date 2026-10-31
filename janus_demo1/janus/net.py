"""Small UDP helpers shared by the processes."""
import json
import os
import socket
import sys

from . import config as C


def udp_socket(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if sys.platform.startswith("win") and hasattr(socket, "SIO_UDP_CONNRESET"):
        try:  # stop Windows reporting ICMP port-unreachable as recv errors
            s.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    try:
        s.bind((host, port))
    except OSError as e:
        raise SystemExit(f"Cannot bind UDP {host}:{port} ({e}). Another Janus may be running; "
                         f"use --port-base to pick other ports.")
    s.setblocking(False)
    return s


def recv_all(sock, limit=256):
    out = []
    for _ in range(limit):
        try:
            data, addr = sock.recvfrom(65535)
        except (BlockingIOError, InterruptedError):
            break
        except ConnectionResetError:
            continue
        except OSError:
            break
        out.append((data, addr))
    return out


def send_json(sock, obj, addr):
    try:
        sock.sendto(json.dumps(obj, separators=(",", ":")).encode(), addr)
    except OSError:
        pass


def send_bytes(sock, data, addr):
    try:
        sock.sendto(data, addr)
    except OSError:
        pass


def ports_from(base=None, override=None):
    p = dict(C.DEFAULT_PORTS)
    if base is not None:
        off = int(base) - C.DEFAULT_PORTS["gcs_radio"]
        p = {k: v + off for k, v in p.items()}
    if override:
        p.update(override)
    return p


def parse_ports(arg):
    return json.loads(arg) if arg else dict(C.DEFAULT_PORTS)


def pid():
    return os.getpid()
