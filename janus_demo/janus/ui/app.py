"""Gradio front end for the Janus demonstrator."""
import json
import os
import tempfile
import time

import gradio as gr

from .. import config as C
from ..scenarios import SCRIPTS

HERE = os.path.dirname(os.path.abspath(__file__))
read = lambda n: open(os.path.join(HERE, n), encoding="utf-8").read()

MODE_LABEL = {"small": "Small UAS (0–400 ft)", "aircraft": "Aircraft (0–40,000 ft)"}
LABEL_MODE = {v: k for k, v in MODE_LABEL.items()}
SLIDERS = [  # key, label, min, max, step
    ("ceiling", "Ceiling (altitude layer)", 1, 8, 1),
    ("speed", "Cruise speed (cells / s)", 0.8, 6.5, 0.1),
    ("climb_mult", "Climb cost multiplier", 0.5, 4.0, 0.1),
    ("lookahead", "Forward sensor lookahead (cells along route)", 1, 12, 1),
    ("gust_limit_kt", "Gust limit (kt), small UAS", 5, 50, 1),
    ("turb_block", "Turbulence that blocks (1 LOW, 2 MOD, 3 HIGH), aircraft", 1, 3, 1),
    ("energy", "Energy capacity (units)", 15, 400, 1),
    ("reserve_pct", "Landing reserve (%)", 0, 50, 1),
    ("burn", "Energy burn factor", 0.3, 3.0, 0.05),
    ("tx_dbm", "Drone radio power (dBm)", 0, 47, 1),
    ("lost_link_s", "Lost-link timeout before failsafe (s)", 1, 30, 0.5),
    ("budget", "Replan compute budget (A* nodes)", 50, 200000, 50),
]

CSS = """
:root, .dark { --jn-bg:#0b121a; }
body, gradio-app, .gradio-container, .main, .wrap, .contain { background: #0b121a !important; }
.gradio-container { max-width: 1680px !important; padding-left: max(16px, env(safe-area-inset-left, 0px)) !important; padding-right: max(16px, env(safe-area-inset-right, 0px)) !important; }
gradio-app > div > footer, .gradio-container > footer, footer.svelte-1rjryqp { display: none !important; }
#jn-head { display:flex; justify-content:space-between; align-items:flex-end; gap:16px; flex-wrap:wrap; padding:6px 2px 12px; border-bottom:1px solid #223345; margin-bottom:6px; }
#jn-head h1 { font-family:"Barlow Semi Condensed","Arial Narrow",sans-serif; font-weight:700; font-size:34px; letter-spacing:.18em; margin:0; color:#dfe8f1; line-height:1; }
#jn-head h1 span { color:#f2a93b; }
#jn-head p { margin:4px 0 0; color:#8ba0b4; font-size:13.5px; }
#jn-head .meta { font-family:"IBM Plex Mono",monospace; font-size:11.5px; color:#5d7286; text-align:right; }
#jn-setup { background:#111b25; border:1px solid #223345; border-radius:8px; padding:12px !important; }
#jn-setup .gr-markdown h3, #jn-setup h3 { font-family:"Barlow Semi Condensed",sans-serif; letter-spacing:.1em; text-transform:uppercase; font-size:15px; color:#8ba0b4; margin:0 0 4px; }
#jn-status { font-size:12.5px; }
#jn-status p { margin: 3px 0; }
.jn-md table { font-size: 12.5px; border-collapse: collapse; }
.jn-md table, .jn-md th, .jn-md td { border-color: #223345 !important; }
.jn-md th { color: #8ba0b4; font-weight: 500; }
.jn-md code { font-size: 12px; }
"""

HEAD = """
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<meta name="color-scheme" content="dark">
"""

FORCE_DARK = """() => { document.body.classList.add('dark'); document.documentElement.style.colorScheme = 'dark'; }"""


def theme():
    t = gr.themes.Base(
        primary_hue=gr.themes.Color(c50="#fff6e8", c100="#fde9c8", c200="#fbd493", c300="#f7be5e", c400="#f4b04a",
                                    c500="#f2a93b", c600="#d48d22", c700="#a96e17", c800="#7d520f", c900="#523608", c950="#2e1e04"),
        neutral_hue=gr.themes.Color(c50="#dfe8f1", c100="#c4d2df", c200="#a6b7c7", c300="#8ba0b4", c400="#6f8599", c500="#5d7286",
                                    c600="#3b5168", c700="#2d4359", c800="#1b2a3a", c900="#111b25", c950="#0b121a"),
        font=[gr.themes.GoogleFont("IBM Plex Sans"), "Segoe UI", "system-ui", "sans-serif"],
        font_mono=[gr.themes.GoogleFont("IBM Plex Mono"), "ui-monospace", "Consolas", "monospace"],
        radius_size=gr.themes.sizes.radius_sm,
    )
    dark = dict(body_background_fill="#0b121a", body_text_color="#dfe8f1", block_background_fill="#111b25",
                block_border_color="#223345", block_label_text_color="#8ba0b4", block_title_text_color="#dfe8f1",
                input_background_fill="#16222f", input_border_color="#2d4359", border_color_primary="#2d4359",
                background_fill_primary="#111b25", background_fill_secondary="#0b121a", body_text_color_subdued="#8ba0b4",
                button_secondary_background_fill="#16222f", button_secondary_text_color="#dfe8f1",
                button_secondary_border_color="#2d4359", button_primary_text_color="#1a1205",
                slider_color="#f2a93b", checkbox_background_color="#16222f", panel_background_fill="#111b25",
                table_even_background_fill="#111b25", table_odd_background_fill="#16222f", table_border_color="#223345",
                color_accent_soft="#2d4359")
    kw = {}
    for k, v in dark.items():
        kw[k] = v
        kw[k + "_dark"] = v
    try:
        return t.set(**kw)
    except TypeError:
        return t.set(**{k: v for k, v in kw.items() if not k.endswith("_dark")})


def build_app(rt):
    def get_state(args=None):
        args = args or []
        scn_rev = args[0] if len(args) > 0 and args[0] is not None else -1
        last_evt = args[1] if len(args) > 1 and args[1] is not None else 0
        return rt.snapshot(scn_rev, last_evt)

    def command(args=None):
        args = args or []
        name = args[0] if args else None
        arg = args[1] if len(args) > 1 else None
        try:
            return rt.command(name, arg)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def preset_values(mode, preset):
        p = C.make_profile(mode, preset if preset in C.PRESETS[mode] else None)
        return p

    def on_mode(mode_label):
        mode = LABEL_MODE[mode_label]
        names = list(C.PRESETS[mode].keys())
        p = preset_values(mode, names[0])
        return [gr.update(choices=names, value=names[0])] + slider_updates(p, mode)

    def slider_updates(p, mode):
        ups = []
        for key, *_ in SLIDERS:
            vis = True
            if key == "gust_limit_kt":
                vis = mode == "small"
            if key == "turb_block":
                vis = mode == "aircraft"
            ups.append(gr.update(value=p[key], visible=vis))
        ups.append(gr.update(value=p["rain_rated"], visible=mode == "small"))
        ups.append(gr.update(value=p["failsafe"]))
        return ups

    def on_preset(mode_label, preset):
        mode = LABEL_MODE[mode_label]
        if preset not in C.PRESETS[mode]:
            return [gr.update() for _ in range(len(SLIDERS) + 2)]
        return slider_updates(preset_values(mode, preset), mode)

    def generate(mode_label, preset, script, weather, seed, uplink, *vals):
        mode = LABEL_MODE[mode_label]
        base = preset_values(mode, preset)
        keys = [k for k, *_ in SLIDERS] + ["rain_rated", "failsafe"]
        overrides = {}
        for k, v in zip(keys, vals):
            if v is None:
                continue
            bv = base[k]
            if isinstance(bv, bool):
                if bool(v) != bv:
                    overrides[k] = bool(v)
            elif isinstance(bv, str):
                if v != bv:
                    overrides[k] = v
            elif abs(float(v) - float(bv)) > 1e-9:
                overrides[k] = v
        res = rt.prepare({"mode": mode, "preset": preset, "script": script, "weather": weather,
                          "seed": int(seed or 1), "uplink": bool(uplink), "overrides": overrides})
        if not res.get("ok"):
            return (f"**Could not verify this story for this drone.** {res.get('error')}\n\n"
                    "Try another seed, another drone, or change the custom values.")
        st, prof = res["story"], res["profile"]
        lines = [f"**{SCRIPTS[script]}** · {prof['name']} · seed {st.get('seed_used')}"]
        lines += ["Checked in the fast simulator before you fly it:"] + [f"- {v}" for v in st.get("verified", [])]
        if overrides:
            lines.append("Custom values: " + ", ".join(f"`{k}={v}`" for k, v in overrides.items()))
        if not uplink:
            lines.append("Uplink **off**: the drone will only know what its own sensor sees.")
        lines.append("")
        lines.append("Press **Launch mission** in the console.")
        return "\n".join(lines)

    def evidence_md():
        ev = rt.evidence()
        if not ev:
            return "No run yet. Generate a scenario and launch it."
        prof = ev["profile"] or {}
        def n(v, d="–"):
            return d if v is None else v
        rows = [
            ("Drone / story", f"{prof.get('name')} · {SCRIPTS.get(ev['cfg']['script'], '')} · seed {(ev['story'] or {}).get('seed_used')}"),
            ("Final drone state", n(ev["drone_state"])),
            ("Uploads", "; ".join(f"{u['rid']} {u['status']}" + (f" ({u['reason']})" if u.get('reason') else "") +
                                   (f" in {u['rtt']} s, {u['attempts']} tx" if u.get('rtt') else "") for u in ev["uploads"]) or "–"),
            ("Replans", f"{n(ev['replans'], 0)} · median {n(ev['replan_ms_median'])} ms · max {n(ev['replan_ms_max'])} ms (this laptop)"),
            ("Hazard first detected (cells ahead)", ", ".join(f"{k}: {v}" for k, v in (ev["detect_cells"] or {}).items()) or "–"),
            ("Blocked-cell entries (must be 0)", n(ev["blocked_entries"], 0)),
            ("Unknown-hazard exposures", n(ev["exposures"], 0)),
            ("Hazards found by own sensor", n(ev["sensed"], 0)),
            ("Failsafes triggered", n(ev["failsafes"], 0)),
            ("Energy left", f"{n(ev['energy_left'])} of {prof.get('energy')} (reserve {prof.get('reserve_pct')}%)"),
            ("Outages", "; ".join(f"{(o['t1'] or 0) - (o['t0'] or 0):.1f} s, ghost error {o['err']} cells, black box {'recovered' if o['recovered'] else 'pending'} ({o.get('bb_records') or 0} records)" for o in ev["outages"]) or "none"),
        ]
        c = ev["air_counts"] or {"up": {}, "dn": {}}
        g, d = ev["gcs_ep"] or {}, ev["drone_ep"] or {}
        md = ["### This run, in numbers", "All figures come from this simulated run on this machine. They are not flight-test results.", "",
              "| Measure | Value |", "|---|---|"] + [f"| {a} | {b} |" for a, b in rows]
        md += ["", "### Packets", "| | Uplink | Downlink |", "|---|---|---|"]
        for k, lab in (("ok", "Delivered"), ("rf_lost", "Lost to link budget"), ("dropped", "Dropped by loss slider"),
                       ("corrupt", "Corrupted in flight"), ("blackout", "Cut link")):
            md.append(f"| {lab} | {c.get('up', {}).get(k, 0)} | {c.get('dn', {}).get(k, 0)} |")
        md += ["", "| Endpoint | Ground station | Drone |", "|---|---|---|"]
        for k, lab in (("crc_fail", "CRC rejects"), ("retx", "Retransmissions"), ("dup", "Duplicates ignored"),
                       ("gap_lost", "Sequence gap (packets)"), ("acked", "Reliable messages acknowledged")):
            md.append(f"| {lab} | {g.get(k, 0)} | {d.get(k, 0)} |")
        return "\n".join(md)

    def export():
        ev = rt.evidence() or {}
        path = os.path.join(tempfile.gettempdir(), f"janus_report_{int(time.time())}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"note": "Simulated run on the demo laptop. Illustrative values, not flight data.", **ev}, f, indent=2, default=str)
        return path

    with gr.Blocks(title="Janus · Team Celeritas", fill_width=True) as demo:
        gr.HTML("""<div id="jn-head"><div><h1>JAN<span>U</span>S</h1>
            <p>Ground station ⇄ drone link demonstrator · weather-aware replanning over a real UDP link · Team Celeritas, AVISHKAR</p></div>
            <div class="meta">Round 1 build · all flight, energy and radio figures are illustrative<br>Simulation, not flight test</div></div>""")
        with gr.Tabs():
            with gr.Tab("Live"):
                with gr.Row(equal_height=False):
                    with gr.Column(scale=1, min_width=290, elem_id="jn-setup"):
                        gr.Markdown("### Mission setup")
                        mode = gr.Radio(list(MODE_LABEL.values()), value=MODE_LABEL["small"], label="Platform mode")
                        names = list(C.PRESETS["small"].keys())
                        preset = gr.Dropdown(names, value=names[0], label="Drone profile")
                        script = gr.Dropdown([(v, k) for k, v in SCRIPTS.items()], value="nominal", label="Story")
                        weather = gr.Radio(["calm", "breezy", "stormy"], value="breezy", label="Weather profile")
                        seed = gr.Number(1, precision=0, label="Seed (same seed, same world)")
                        uplink = gr.Checkbox(True, label="Ground-station hazard uplink (off = sensor only)")
                        with gr.Accordion("Customize drone (beyond the presets)", open=False):
                            p0 = C.make_profile("small", names[0])
                            sl = []
                            for key, label, lo, hi, step in SLIDERS:
                                sl.append(gr.Slider(lo, hi, value=p0[key], step=step, label=label,
                                                    visible=key != "turb_block"))
                            rain = gr.Checkbox(p0["rain_rated"], label="Rain-rated airframe")
                            fs = gr.Dropdown([(C.FAILSAFE_LABEL[k], k) for k in C.FAILSAFES], value=p0["failsafe"], label="Lost-link failsafe")
                        go = gr.Button("Generate & pre-flight", variant="primary")
                        status = gr.Markdown("Pick a drone and a story, then press **Generate**. Or press **Play hands-free tour** in the console.", elem_id="jn-status")
                    with gr.Column(scale=4, min_width=360):
                        gr.HTML(html_template=read("live.html"), css_template=read("live.css"), js_on_load=read("live.js"),
                                server_functions=[get_state, command], apply_default_css=False, padding=False)
                mode.change(on_mode, [mode], [preset] + sl + [rain, fs])
                preset.change(on_preset, [mode, preset], sl + [rain, fs])
                go.click(generate, [mode, preset, script, weather, seed, uplink] + sl + [rain, fs], [status])
            with gr.Tab("Evidence"):
                with gr.Row():
                    refresh = gr.Button("Refresh numbers", size="sm")
                    dl = gr.DownloadButton("Download run report (JSON)", size="sm")
                ev_md = gr.Markdown(elem_classes=["jn-md"])
                refresh.click(evidence_md, None, ev_md)
                dl.click(export, None, dl)
                gr.Timer(2.0).tick(evidence_md, None, ev_md)
            with gr.Tab("How it works"):
                gr.Markdown(how_it_works(rt), elem_classes=["jn-md"])
    return demo


def launch_kwargs():
    return {"theme": theme(), "css": CSS, "head": HEAD, "js": FORCE_DARK, "footer_links": []}


def how_it_works(rt):
    p = rt.ports
    return f"""
### Three programs, one real radio path

| Program | What it is | UDP port(s) |
|---|---|---|
| Ground station | This window: weather picture, planner pre-check, uploads, link monitor, ghost estimate | `{p['gcs_radio']}` radio, `{p['gcs_obs']}` observer |
| Air relay | Separate process standing in for the radio: link budget, line-of-sight shadow, loss, latency, corruption, cut | `{p['air_gcs']}` / `{p['air_drone']}` |
| Drone agent | Separate process: onboard map, route validation, A* replanning, energy, failsafes, black box | `{p['drone_radio']}` radio |

Every byte between ground station and drone crosses the relay as a UDP datagram: header, sequence number, CRC32, fragmentation, and acknowledged retransmission for missions, hazard pictures and route updates.
You can watch it live in Wireshark on the loopback adapter with the filter `udp.port >= {p['gcs_radio']} && udp.port <= {p['drone_ctl']}`.

The **drone onboard** screen is fed by a separate simulation observer stream. It is not the radio link. It exists so you can compare what the drone really did with what the ground station believed.

### How a disconnection is proven
1. **Two truths side by side.** The left screen is drawn only from packets that arrived. The right screen is the drone's own computer.
2. **Ghost versus black box.** During an outage the ground station shows a dead-reckoned estimate with a growing uncertainty ring. When the link returns, the drone downlinks its black box and the real blind track is drawn over the estimate, with the error in cells.
3. **Packet seismograph.** Each packet's fate is shown as it happens, and sequence-number gaps are called out.
4. **Kill the relay.** "Pull antenna" terminates the relay process; its PID goes grey and nothing can cross.

### Planner
A* in a 25 × 17 × 9 grid, ported from the team's `main.py` and checked against it: equal optimal cost on 195 of 200 random scenarios, with the other 4 agreeing there is no route and 1 skipped. Ten moves per cell (8 horizontal without corner cutting, plus climb and descend), octile heuristic, climb and wind penalties. The drone plans only with what it knows: the map it carries, the uplinked hazard picture, and its own forward sensor. The safety invariant is that it never enters a cell its own map marks as blocked; every run counts this.

### What is real, and what is modelled
| Item | Status |
|---|---|
| UDP sockets, framing, CRC32, retransmission, three OS processes | Real |
| A* planner and replanning | Real (ported from main.py, parity-tested) |
| Weather feed (WeeWX field names, METAR format) | Simulated, seeded |
| Gust/turbulence severity | Our heuristic |
| Ash advisories, TFRs, geofences | Simulated |
| Radio link budget (free-space loss + obstruction) | Simplified model: no multipath, no Fresnel zones |
| Energy, speeds, limits | Illustrative values |
| Flight dynamics | Not modelled (grid cells) |
| Link security | Not implemented (packets are not authenticated) |

### Likely judge questions
- **Does it run on a real flight computer?** The planner is Python and is meant for a companion computer (Raspberry Pi or Jetson class) next to the autopilot. We have not benchmarked one yet; replan times shown are from this laptop.
- **Why UDP?** Telemetry links commonly use datagrams. With UDP we handle loss ourselves (sequence numbers, CRC, acknowledgements), so the failures can be seen instead of hidden by TCP.
- **How do you know the uplink helps?** The pop-up story is checked in simulation before it runs: the uplinked picture lets the drone replan 8–9 cells ahead, whereas its own sensor sees 4–6. It is a simulation result.
- **What if the link and the planner both fail?** The drone holds or returns on its last known map and never enters a blocked cell. If no safe route exists it holds and retries.
"""
