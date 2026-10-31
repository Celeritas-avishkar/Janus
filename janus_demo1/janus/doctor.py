"""Health check for the Janus demonstrator: python -m janus.doctor [--quick]

Checks the interpreter, dependencies, ports, the team's main.py, a headless
simulation, and (unless --quick) a real 3-process UDP link on spare ports.
Exit code 0 = ready, 1 = something needs fixing.
"""
import argparse
import importlib
import os
import socket
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
OK, WARN, FAIL = "ok", "warn", "FAIL"
results = []


def report(status, name, detail=""):
    results.append(status)
    mark = {"ok": "[ ok ]", "warn": "[warn]", "FAIL": "[FAIL]"}[status]
    print(f"{mark} {name}" + (f": {detail}" if detail else ""), flush=True)


def port_free(port, kind=socket.SOCK_DGRAM, host="127.0.0.1"):
    s = socket.socket(socket.AF_INET, kind)
    try:
        s.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="skip the live 3-process link test")
    ap.add_argument("--port-base", type=int, default=None, help="UDP port base the UI will use")
    ap.add_argument("--port", type=int, default=7860, help="web UI port the UI will use")
    a = ap.parse_args()

    print(f"Janus doctor · {ROOT}\n")

    # 1. interpreter
    v = sys.version_info
    if v >= (3, 10):
        report(OK, "Python", f"{v.major}.{v.minor}.{v.micro} at {sys.executable}")
    else:
        report(FAIL, "Python", f"{v.major}.{v.minor} found; Janus needs 3.10+ (Gradio 6)")
        return finish()
    in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    report(OK if in_venv else WARN, "Virtual environment", sys.prefix if in_venv else "not active (run.sh uses .venv automatically)")

    # 2. dependencies
    want = None
    try:
        for line in open(os.path.join(ROOT, "requirements.txt"), encoding="utf-8"):
            if line.strip().startswith("gradio=="):
                want = line.strip().split("==")[1]
    except OSError:
        pass
    try:
        import gradio
        if want and gradio.__version__ != want:
            report(WARN, "Gradio", f"{gradio.__version__} installed, {want} pinned (tested version). Run: ./run.sh setup")
        else:
            report(OK, "Gradio", gradio.__version__)
    except Exception as e:
        report(FAIL, "Gradio", f"not importable ({e}). Run: ./run.sh setup")
    for mod in ("janus.planner", "janus.protocol", "janus.drone_brain", "janus.gcs_brain", "janus.runtime", "janus.ui.app"):
        try:
            importlib.import_module(mod)
        except Exception as e:
            report(FAIL, f"Import {mod}", str(e))
            return finish()
    report(OK, "Janus modules", "all import")
    for mod, why in (("pytest", "needed for ./run.sh test"), ("pygame", "lets the tests compare against main.py")):
        try:
            importlib.import_module(mod)
            report(OK, f"{mod} (optional)", "installed")
        except Exception:
            report(WARN, f"{mod} (optional)", f"missing; {why}. Run: ./run.sh setup --dev")

    # 3. ports
    from janus.net import ports_from
    ports = ports_from(a.port_base)
    busy = [f"{k}={p}" for k, p in ports.items() if not port_free(p)]
    if busy:
        report(FAIL, "UDP ports", "busy: " + ", ".join(busy) + ". Another Janus running? Or use --port-base 49800")
    else:
        report(OK, "UDP ports", f"{min(ports.values())}-{max(ports.values())} free")
    if port_free(a.port, socket.SOCK_STREAM):
        report(OK, "Web port", f"{a.port} free")
    else:
        report(FAIL, "Web port", f"{a.port} in use; run with --port 7861")

    # 4. team planner
    cands = [os.environ.get("JANUS_MAIN_PY", ""), os.path.join(ROOT, "..", "main.py"), os.path.join(ROOT, "..", "janus", "main.py")]
    found = next((c for c in cands if c and os.path.exists(c)), None)
    report(OK if found else WARN, "Team main.py", os.path.abspath(found) if found else
           "not found; parity test will be skipped (put janus_demo/ inside the Janus repo or set JANUS_MAIN_PY)")

    # 5. headless simulation
    try:
        from janus.config import make_profile
        from janus.scenarios import build, _sim
        t0 = time.time()
        prof = make_profile("small", "Scout-Q  light quadcopter")
        scn = build("nominal", 1, "small", prof)
        s = _sim(scn, prof)
        ok = s.drone.state == "COMPLETE" and s.drone.m["exposures"] == 0
        report(OK if ok else FAIL, "Headless simulation", f"nominal mission {s.drone.state} in {time.time() - t0:.1f} s")
    except Exception as e:
        report(FAIL, "Headless simulation", repr(e))

    # 6. live link over real UDP
    if a.quick:
        report(WARN, "Live UDP link", "skipped (--quick)")
    else:
        base = None
        for b in range(48900, 49600, 20):
            if all(port_free(p) for p in ports_from(b).values()):
                base = b
                break
        if base is None:
            report(FAIL, "Live UDP link", "no spare port block found for the test")
        else:
            rt = None
            try:
                from janus.runtime import Runtime
                rt = Runtime(port_base=base)
                res = rt.prepare({"script": "nominal", "seed": 1})
                t0, link = time.time(), None
                while time.time() - t0 < 15:
                    snap = rt.snapshot()
                    if all(p["alive"] for p in snap["procs"]) and rt.gcs and rt.gcs.link == "CONNECTED" \
                            and (rt.drone_obs or {}).get("link") == "CONNECTED":
                        link = time.time() - t0
                        break
                    time.sleep(0.2)
                pids = [p["pid"] for p in rt.snapshot()["procs"]]
                if res.get("ok") and link is not None:
                    report(OK, "Live UDP link", f"3 processes (pids {', '.join(map(str, pids))}) connected both ways in {link:.1f} s")
                else:
                    report(FAIL, "Live UDP link", f"no two-way link within 15 s; child logs in {rt.log_dir}. "
                           "Check firewall prompts for Python")
            except Exception as e:
                report(FAIL, "Live UDP link", repr(e))
            finally:
                if rt is not None:
                    rt.shutdown()
    return finish()


def finish():
    fails, warns = results.count(FAIL), results.count(WARN)
    print()
    if fails:
        print(f"Not ready: {fails} problem(s), {warns} warning(s).")
        return 1
    print(f"Ready to demo ({warns} warning(s)). Start with: ./run.sh ui   or   ./run.sh tour")
    return 0


if __name__ == "__main__":
    sys.exit(main())
