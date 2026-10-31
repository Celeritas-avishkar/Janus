"""Start the Janus demonstrator.

    python run_janus.py                 # local, opens on http://127.0.0.1:7860
    python run_janus.py --share         # also create a temporary public Gradio link
    python run_janus.py --port-base 49800   # if the default UDP ports are busy

Two-machine mode (experimental): run the ground station with --host 0.0.0.0
--no-spawn-drone --drone-host <drone-ip>, and on the drone machine:
    python -m janus.drone --host 0.0.0.0 --relay-host <gcs-ip> --gcs-host <gcs-ip>
"""
import argparse
import sys

if sys.version_info < (3, 10):
    sys.exit("Janus needs Python 3.10 or newer (Gradio 6). You have " + sys.version.split()[0])


def main():
    ap = argparse.ArgumentParser(description="Janus ground station / drone link demonstrator")
    ap.add_argument("--port", type=int, default=7860, help="web UI port")
    ap.add_argument("--share", action="store_true", help="create a temporary public Gradio link")
    ap.add_argument("--host", default="127.0.0.1", help="UDP bind address (0.0.0.0 for two-machine mode)")
    ap.add_argument("--ui-host", default="127.0.0.1", help="web UI bind address (0.0.0.0 to serve your LAN)")
    ap.add_argument("--port-base", type=int, default=None, help="first UDP port (default 47800)")
    ap.add_argument("--no-spawn-drone", action="store_true", help="do not start relay/drone processes here")
    ap.add_argument("--drone-host", default=None)
    ap.add_argument("--tour", action="store_true", help="start the hands-free tour on launch")
    a = ap.parse_args()

    from janus.runtime import Runtime
    from janus.ui.app import build_app, launch_kwargs

    rt = Runtime(host=a.host, port_base=a.port_base, spawn=not a.no_spawn_drone, drone_host=a.drone_host)
    if a.tour:
        rt.command("tour_start")
    demo = build_app(rt)
    print(f"[janus] UDP ports {rt.ports}\n[janus] logs in {rt.log_dir}", flush=True)
    try:
        demo.launch(server_name=a.ui_host, server_port=a.port, share=a.share, show_error=True, **launch_kwargs())
    finally:
        rt.shutdown()


if __name__ == "__main__":
    main()
