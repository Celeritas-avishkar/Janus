"""Gradio front end for the Janus demonstrator (Round 2)."""
import os

import gradio as gr

from .. import config as C
from ..scenarios import SCRIPTS
from ..holding import KINDS as HOLD_KINDS

HERE = os.path.dirname(os.path.abspath(__file__))
read = lambda n: open(os.path.join(HERE, n), encoding="utf-8").read()

MODE_LABEL = {"small": "Small UAS", "aircraft": "Aircraft"}
LABEL_MODE = {v: k for k, v in MODE_LABEL.items()}
WEATHER_LABEL = {"calm": "Calm", "breezy": "Breezy", "stormy": "Stormy"}
LABEL_WEATHER = {v: k for k, v in WEATHER_LABEL.items()}
ASH_CONC = {"Auto": "auto", "Weak": 1, "Moderate": 2, "Strong": 3}
MAP_CENTERS = [("Bengaluru city", "bengaluru"), ("Kempegowda Airport (VOBL)", "vobl")]

# key, label, min, max, step, basic?   Plain-language labels; the basic four are
# always visible, the rest sit under "More settings".
SLIDERS = [
    ("speed", "Cruise speed (cells per second)", 0.8, 6.5, 0.1, True),
    ("energy", "Battery size", 15, 400, 1, True),
    ("gust_limit_kt", "Strongest gust it can fly in (kt)", 5, 50, 1, True),
    ("turb_block", "Turbulence it must avoid (1 LOW, 2 MODERATE, 3 HIGH)", 1, 3, 1, True),
    ("ceiling", "Highest altitude layer", 1, 8, 1, False),
    ("climb_mult", "Cost of climbing (× level flight)", 0.5, 4.0, 0.1, False),
    ("reserve_pct", "Battery kept for landing (%)", 0, 50, 1, False),
    ("burn", "Energy use (× normal)", 0.3, 3.0, 0.05, False),
    ("tx_dbm", "Radio power (dBm)", 0, 47, 1, False),
    ("lost_link_s", "Seconds without link before failsafe", 1, 60, 0.5, False),
    ("br_ruler", "BendyRuler probe length (cells)", 2, 6, 1, False),
    ("opc_thr_mg", "OPC ash alarm (mg/m³)", 0.05, 2.0, 0.05, False),
    ("opc_period_s", "OPC sample period (s, OPC-N3: 1–30)", 1.0, 30.0, 1.0, False),
    ("so2_t90_s", "SO₂ sensor response t90 (s, SO2-B4: < 60)", 5.0, 120.0, 5.0, False),
    ("budget", "Planner search limit (nodes)", 50, 200000, 50, False),
]
KEYS = [s[0] for s in SLIDERS]

CSS = """
:root, .dark { --jn-bg:#0b121a; }
body, gradio-app { background:
    radial-gradient(1200px 700px at 12% -5%, rgba(242,169,59,0.13), transparent 60%),
    radial-gradient(900px 600px at 95% 10%, rgba(79,209,224,0.10), transparent 60%),
    radial-gradient(900px 700px at 55% 105%, rgba(155,123,242,0.09), transparent 60%), #0b121a !important;
  background-attachment: fixed !important; }
.gradio-container, .main, .wrap, .contain { background: transparent !important; }
.gradio-container { max-width: 1720px !important; padding-left: max(16px, env(safe-area-inset-left, 0px)) !important; padding-right: max(16px, env(safe-area-inset-right, 0px)) !important; }
gradio-app > div > footer, .gradio-container > footer { display: none !important; }
#jn-title { text-align: center; padding: 4px 0 10px; border-bottom: 1px solid #223345; margin-bottom: 4px; }
#jn-title h1 { font-family: "JetBrains Mono", "IBM Plex Mono", ui-monospace, Consolas, monospace; font-weight: 700; font-size: 38px;
  letter-spacing: .32em; margin: 0; padding-left: .32em; color: #f2a93b; line-height: 1.1; }
/* sidebar = mission setup */
.sidebar { background: rgba(14,22,32,0.72) !important; border-right: 1px solid rgba(255,255,255,0.08) !important;
  -webkit-backdrop-filter: blur(22px) saturate(160%); backdrop-filter: blur(22px) saturate(160%); }
.sidebar .block, .sidebar .form { background: rgba(255,255,255,0.03) !important; border-color: rgba(255,255,255,0.08) !important; border-radius: 14px !important; }
.sidebar .sidebar-content { padding: 14px 14px 24px !important; }
#jn-side-h { font-family: "JetBrains Mono", ui-monospace, monospace; font-size: 13px; letter-spacing: .16em; color: #8ba0b4; margin: 2px 0 6px; }
/* burger toggle */
/* clean burger: three rounded bars drawn as an SVG mask, crossfades to a close mark when open */
.sidebar .toggle-button { width: 38px !important; height: 38px !important; border-radius: 12px !important; display: grid !important; place-items: center !important;
  background: linear-gradient(180deg, rgba(255,255,255,0.12), rgba(255,255,255,0.03)), rgba(20,31,44,0.7) !important;
  border: 1px solid rgba(255,255,255,0.12) !important; box-shadow: inset 0 1px 0 rgba(255,255,255,0.15), 0 6px 16px rgba(0,0,0,0.35) !important;
  -webkit-backdrop-filter: blur(12px); backdrop-filter: blur(12px); }
.sidebar .toggle-button > * { display: none !important; }
.sidebar .toggle-button::before { content: ""; width: 18px; height: 18px; background: #dfe8f1;
  -webkit-mask: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'%3E%3Cpath d='M4 7h16M4 12h16M4 17h16' stroke='black' stroke-width='2.4' stroke-linecap='round'/%3E%3C/svg%3E") center / contain no-repeat;
  mask: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'%3E%3Cpath d='M4 7h16M4 12h16M4 17h16' stroke='black' stroke-width='2.4' stroke-linecap='round'/%3E%3C/svg%3E") center / contain no-repeat; }
.sidebar.open .toggle-button::before {
  -webkit-mask-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'%3E%3Cpath d='M6 6l12 12M18 6L6 18' stroke='black' stroke-width='2.4' stroke-linecap='round'/%3E%3C/svg%3E");
  mask-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24'%3E%3Cpath d='M6 6l12 12M18 6L6 18' stroke='black' stroke-width='2.4' stroke-linecap='round'/%3E%3C/svg%3E"); }
/* segmented radios that fill their row */
.jn-seg .wrap { display: grid !important; grid-template-columns: repeat(var(--n, 2), minmax(0, 1fr)) !important; gap: 6px !important; }
.jn-seg .wrap label { justify-content: center !important; margin: 0 !important; min-width: 0 !important; width: 100% !important; padding: 8px 6px !important; }
.jn-seg .wrap label input { display: none !important; }
.jn-seg .wrap label.selected, .jn-seg .wrap label:has(input:checked) { background: #f2a93b !important; color: #1a1205 !important; border-color: #f2a93b !important; font-weight: 600; }
.jn-seg.n3 { --n: 3; } .jn-seg.n4 { --n: 4; }
#jn-sar { border: 1px solid #3b2a2d; border-radius: 6px; padding: 6px 8px !important; background: #1a1418; }
#jn-status { font-size: 12.5px; } #jn-status p { margin: 3px 0; }
#jn-status ul { margin: 2px 0; padding-left: 18px; }
.jn-note { font-size: 11.5px; color: #8ba0b4; }
"""

HEAD = """
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
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
        font_mono=[gr.themes.GoogleFont("JetBrains Mono"), "ui-monospace", "Consolas", "monospace"],
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


def ash_top_choices(mode):
    ft = C.MODES[mode]["layer_ft"]
    out = [("Auto", "auto")]
    for z in range(2, C.LAYERS):
        lab = f"FL{z * ft // 100:03d}" if mode == "aircraft" else f"{z * ft} ft"
        out.append((lab, str(z)))
    return out


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

    def get_report(args=None):
        try:
            return rt.report()
        except Exception as e:
            return {"error": str(e)}

    def get_compare(args=None):
        try:
            return rt.compare()
        except Exception as e:
            return {"error": str(e), "rows": [], "sar": [], "sar_rules": []}

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

    def on_mode(mode_label):
        mode = LABEL_MODE[mode_label]
        names = list(C.PRESETS[mode].keys())
        p = C.make_profile(mode, names[0])
        return ([gr.update(choices=names, value=names[0]), gr.update(choices=ash_top_choices(mode), value="auto"),
                 gr.update(label="Gust cells" if mode == "small" else "Turbulence cells"),
                 gr.update(visible=mode == "small")] + slider_updates(p, mode))

    def on_preset(mode_label, preset):
        mode = LABEL_MODE[mode_label]
        if preset not in C.PRESETS[mode]:
            return [gr.update() for _ in range(len(SLIDERS) + 2)]
        return slider_updates(C.make_profile(mode, preset), mode)

    def generate(mode_label, preset, script, weather_label, seed, uplink, sar, planner, hold_pattern, map_center,
                 wx_custom, wind_dir, wind_kt, gust_kt, ash_plumes, ash_conc, ash_top, cells, rain, drift, temp_c, vis_km, *vals):
        mode = LABEL_MODE[mode_label]
        base = C.make_profile(mode, preset)
        overrides = {}
        for k, v in zip(KEYS + ["rain_rated", "failsafe"], vals):
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
        wx = {}
        if wx_custom:
            wx = {"wind_dir": wind_dir, "wind_kt": wind_kt, "gust_kt": gust_kt, "ash_plumes": int(ash_plumes),
                  "ash_conc": ASH_CONC.get(ash_conc, "auto"), "ash_top": ash_top, "cells": int(cells), "rain": int(rain),
                  "temp_c": temp_c, "vis_km": vis_km}
        wx["drift"] = {"Still": 0.0, "½ wind": 0.5, "Full wind": 1.0}.get(drift, 0.5)
        res = rt.prepare({"mode": mode, "preset": preset, "script": script, "weather": LABEL_WEATHER[weather_label],
                          "seed": int(seed or 1), "uplink": bool(uplink), "overrides": overrides, "sar": bool(sar),
                          "planner": planner, "wx_opts": wx, "map_center": map_center, "hold_pattern": hold_pattern})
        if not res.get("ok"):
            return (f"**Could not verify this story for this drone.** {res.get('error')}\n\n"
                    "Try another seed, another drone, or change the custom values.")
        st, prof = res["story"], res["profile"]
        lines = [f"**{SCRIPTS[script]}** · {prof['name']} · seed {st.get('seed_used')}",
                 "", "Checked in the fast simulator first:", ""] + [f"- {v}" for v in st.get("verified", [])] + [""]
        if overrides:
            lines.append("Your changes: " + ", ".join(f"`{k}={v}`" for k, v in overrides.items()))
        if wx_custom:
            lines.append("Weather: your settings.")
        if sar:
            lines.append("**SAR mode on:** wider weather limits; MEDIUM/HIGH ash still blocks.")
        if not uplink:
            lines.append("Uplink **off**: the drone only knows what its own sensor sees.")
        lines += ["", "Press **Launch mission** in the console."]
        return "\n".join(lines)

    with gr.Blocks(title="Janus", fill_width=True) as demo:
        with gr.Sidebar(label="Mission setup", open=True, width=360, position="left"):
            gr.HTML('<div id="jn-side-h">MISSION SETUP</div>')
            mode = gr.Radio(list(MODE_LABEL.values()), value=MODE_LABEL["small"], label="Platform",
                            elem_classes=["jn-seg"])
            names = list(C.PRESETS["small"].keys())
            preset = gr.Dropdown(names, value=names[0], label="Drone", filterable=False)
            script = gr.Dropdown([(v, k) for k, v in SCRIPTS.items()], value="escape", label="Story", filterable=False)
            weather = gr.Radio(list(WEATHER_LABEL.values()), value="Breezy", label="Weather",
                               elem_classes=["jn-seg", "n3"])
            with gr.Accordion("Weather and ash options", open=False):
                wx_custom = gr.Checkbox(False, label="Use these values (off: random from the seed)")
                with gr.Row():
                    wind_dir = gr.Slider(0, 350, value=270, step=10, label="Wind from (°)")
                    wind_kt = gr.Slider(0, 40, value=12, step=1, label="Wind (kt)")
                gust_kt = gr.Slider(0, 60, value=20, step=1, label="Gusts (kt)")
                ash_plumes = gr.Slider(0, 2, value=1, step=1, label="Active volcanic vents")
                ash_conc = gr.Radio(list(ASH_CONC), value="Auto", label="Eruption strength (ash at the vent)",
                                    elem_classes=["jn-seg", "n4"])
                ash_top = gr.Dropdown(ash_top_choices("small"), value="auto", label="Eruption column top", filterable=False)
                with gr.Row():
                    cells = gr.Slider(0, 5, value=2, step=1, label="Gust cells")
                    rain = gr.Slider(0, 4, value=1, step=1, label="Rain cells")
                drift = gr.Radio(["Still", "½ wind", "Full wind"], value="½ wind", label="Zones drift with the wind",
                                 elem_classes=["jn-seg", "n3"])
                with gr.Row():
                    temp_c = gr.Slider(-10, 45, value=27, step=1, label="Temperature (°C)")
                    vis_km = gr.Slider(1, 10, value=10, step=0.5, label="Visibility (km)")
            gr.HTML('<div class="jn-note">Onboard ash sensors: OPC particle counter + SO₂ cell. Both sample the air the drone is in, '
                    'so they cannot see ash ahead: forward warning comes from advisories over the uplink.</div>')
            sar = gr.Checkbox(False, label="Search & rescue (SAR) mode", elem_id="jn-sar",
                              info="Keeps flying in worse weather. MEDIUM/HIGH ash still blocks.")
            planner = gr.Dropdown([(v, k) for k, v in C.PLANNERS.items()], value="hybrid", label="Route planner", filterable=False)
            hold_pattern = gr.Dropdown([(v, k) for k, v in HOLD_KINDS.items()], value="racetrack", label="Holding pattern",
                                       filterable=False)
            seed = gr.Number(1, precision=0, label="Seed (same seed, same world)")
            uplink = gr.Checkbox(True, label="Ground-station advisory uplink (off: sensor only)")
            with gr.Accordion("Customize drone", open=False):
                p0 = C.make_profile("small", names[0])
                sl = {}
                for key, label, lo, hi, step, basic in SLIDERS:
                    if basic:
                        sl[key] = gr.Slider(lo, hi, value=p0[key], step=step, label=label, visible=key != "turb_block")
                fs = gr.Dropdown([(C.FAILSAFE_LABEL[k], k) for k in C.FAILSAFES], value=p0["failsafe"], label="If the link is lost", filterable=False)
                rain_rated = gr.Checkbox(p0["rain_rated"], label="Rain-rated airframe")
                with gr.Accordion("More settings", open=False):
                    for key, label, lo, hi, step, basic in SLIDERS:
                        if not basic:
                            sl[key] = gr.Slider(lo, hi, value=p0[key], step=step, label=label)
            map_center = gr.Dropdown(MAP_CENTERS, value="bengaluru", label="Map location (OpenStreetMap backdrop)", filterable=False)
            go = gr.Button("Generate & pre-flight", variant="primary")
            status = gr.Markdown("Pick a drone and a story, then press **Generate**.", elem_id="jn-status")
        gr.HTML('<div id="jn-title"><h1>JANUS</h1></div>')
        gr.HTML(html_template=read("live.html"), css_template=read("live.css"), js_on_load=read("live.js"),
                server_functions=[get_state, command, get_report, get_compare], apply_default_css=False, padding=False)
        sliders = [sl[k] for k in KEYS]
        mode.change(on_mode, [mode], [preset, ash_top, cells, rain] + sliders + [rain_rated, fs])
        preset.change(on_preset, [mode, preset], sliders + [rain_rated, fs])
        go.click(generate, [mode, preset, script, weather, seed, uplink, sar, planner, hold_pattern, map_center, wx_custom, wind_dir,
                            wind_kt, gust_kt, ash_plumes, ash_conc, ash_top, cells, rain, drift, temp_c, vis_km]
                 + sliders + [rain_rated, fs], [status])
    return demo


def launch_kwargs():
    return {"theme": theme(), "css": CSS, "head": HEAD, "js": FORCE_DARK, "footer_links": []}
