// Janus live console. Runs inside a Gradio gr.HTML component: `element` is the
// component root and `server` exposes the Python functions
// get_state / command / get_report / get_compare.
const root = element.querySelector('.jn');
const $ = (s) => root.querySelector(s);
const $$ = (s) => Array.from(root.querySelectorAll(s));
const C = {
  bg: '#0b121a', grid: 'rgba(120,150,180,0.07)', grid5: 'rgba(120,150,180,0.14)', ink: '#dfe8f1', muted: '#8ba0b4', dim: '#5d7286',
  amber: '#f2a93b', cyan: '#4fd1e0', ice: '#8ab8ff', ok: '#4cc98d', warn: '#f0b429', crit: '#f25560', mag: '#ff6fd4', drop: '#ff8a3d',
  building: '#2a3a4d', geofence: '#f25560', ridge: '#6d5a42', tfr: '#e06c9f',
  gust: '#2ec4b6', rain: '#4c8df6', turb: '#9b7bf2',
  ash1: '#3fd0e0', ash2: '#9aa6b2', ash3: '#f25560',
};
const FM = '"JetBrains Mono", "IBM Plex Mono", ui-monospace, Consolas, monospace';
const FB = '"IBM Plex Sans", "Segoe UI", system-ui, sans-serif';
const CENTERS = { bengaluru: [12.9716, 77.5946], vobl: [13.1986, 77.7066] };
const S = { scn: null, scnRev: -1, lastEvt: 0, st: null, recvAt: 0, ev: [], hist: [], disp: {}, oldRoute: null, oldRouteT: 0,
            droneRev: -1, gcsRev: -1, gcsOld: null, gcsOldT: 0, drag: {}, terrain: {}, lastText: 0, session: -1, fails: 0,
            tab: 'views', chips: { link: true, packets: true, updown: true, log: true }, bg: 'grid', tiles: {}, tileFail: 0,
            report: null, compare: null, lastReport: 0,
            altInt: 'auto', altSize: 'm', tool: 'none', zopt: { type: 'ash', sev: 3, shape: 'circle', size: 2, drift: false, hidden: false },
            hover: null };
const PROF_PX = { s: 52, m: 96, l: 156 };
const store = { get(k) { try { return window.localStorage.getItem('janus.' + k); } catch (e) { return null; } },
                set(k, v) { try { window.localStorage.setItem('janus.' + k, v); } catch (e) { /* storage unavailable */ } } };

// ------------------------------------------------------------------ polling
async function poll() {
  while (true) {
    const t0 = performance.now();
    try {
      const st = await server.get_state([S.scnRev, S.lastEvt, S.session]);
      if (st) onState(st);
      S.fails = 0;
    } catch (e) {
      S.fails++;
      if (S.fails === 3) toast('Lost contact with the Janus server. Is it still running?');
    }
    await new Promise((r) => setTimeout(r, Math.max(30, 110 - (performance.now() - t0))));
  }
}

function onState(st) {
  if (st.session !== S.session) {
    S.ev = []; S.hist = []; S.disp = {}; S.oldRoute = null; S.droneRev = -1; S.gcsRev = -1; S.gcsOld = null;
    S.session = st.session; S.compare = null;
  }
  if (st.scn) { S.scn = st.scn; S.scnRev = st.scn_rev; S.terrain = {}; }
  const evs = (st.air && st.air.events) || [];
  for (const e of evs) S.ev.push(e);
  if (st.air && st.air.evt_id) S.lastEvt = st.air.evt_id;
  const tcut = st.t - 45;
  while (S.ev.length && S.ev[0][1] < tcut) S.ev.shift();
  if (S.ev.length > 6000) S.ev.splice(0, S.ev.length - 6000);
  const g = st.gcs || {}, d = st.drone || {};
  S.hist.push([st.t, g.link || 'NO_LINK', d.link || 'NO_LINK', st.procs ? (st.procs[1].alive ? 1 : 0) : 0]);
  while (S.hist.length && S.hist[0][0] < tcut) S.hist.shift();
  if (d.rev !== undefined && d.rev !== S.droneRev) {
    if (S.droneRev >= 0 && S.st && S.st.drone && S.st.drone.route) { S.oldRoute = S.st.drone.route.slice(S.st.drone.seg || 0); S.oldRouteT = performance.now(); }
    S.droneRev = d.rev;
  }
  if (g.route_rev !== undefined && g.route_rev !== S.gcsRev) {
    if (S.gcsRev >= 0 && S.st && S.st.gcs && S.st.gcs.route) { S.gcsOld = S.st.gcs.route; S.gcsOldT = performance.now(); }
    S.gcsRev = g.route_rev;
  }
  S.st = st; S.recvAt = performance.now();
}

function tNow() { return S.st ? S.st.t + (S.st.paused ? 0 : Math.min(0.3, (performance.now() - S.recvAt) / 1000)) : 0; }

// ------------------------------------------------------------------ commands and navigation
async function cmd(name, arg) {
  try {
    const r = await server.command([name, arg === undefined ? null : arg]);
    if (r && r.ok === false && r.error) toast(r.error);
    return r;
  } catch (e) { toast('Command failed: ' + e); }
}
function toast(msg) {
  const el = $('.jn-toast'); el.textContent = msg; el.hidden = false;
  clearTimeout(S.toastT); S.toastT = setTimeout(() => { el.hidden = true; }, 4200);
}
root.addEventListener('click', (ev) => {
  const tab = ev.target.closest('[data-tab]');
  if (tab && tab.tagName === 'BUTTON') { setTab(tab.dataset.tab); return; }
  const chip = ev.target.closest('[data-chip]');
  if (chip) { S.chips[chip.dataset.chip] = !S.chips[chip.dataset.chip]; applyChips(); return; }
  const bg = ev.target.closest('[data-map-bg]');
  if (bg) { setBg(bg.dataset.mapBg); return; }
  const at = ev.target.closest('[data-alt-int]');
  if (at) { S.altInt = at.dataset.altInt; store.set('altInt', S.altInt); $$('[data-alt-int]').forEach((b) => b.classList.toggle('on', b === at)); return; }
  const as = ev.target.closest('[data-alt-size]');
  if (as) { S.altSize = as.dataset.altSize; store.set('altSize', S.altSize); $$('[data-alt-size]').forEach((b) => b.classList.toggle('on', b === as)); return; }
  const tl = ev.target.closest('[data-tool]');
  if (tl) { setTool(tl.dataset.tool); return; }
  const zs = ev.target.closest('[data-zsev]');
  if (zs) { S.zopt.sev = parseInt(zs.dataset.zsev, 10); $$('[data-zsev]').forEach((b) => b.classList.toggle('on', b === zs)); return; }
  const zh = ev.target.closest('[data-zshape]');
  if (zh) { S.zopt.shape = zh.dataset.zshape; $$('[data-zshape]').forEach((b) => b.classList.toggle('on', b === zh)); return; }
  const fo = ev.target.closest('[data-focus-open]');
  if (fo) { openFocus(fo.dataset.focusOpen); return; }
  const fl = ev.target.closest('[data-focus]');
  if (fl) { setFocusLayout(fl.dataset.focus); return; }
  if (ev.target.closest('[data-fs-exit]')) { if (document.fullscreenElement) document.exitFullscreen(); return; }
  const rep = ev.target.closest('[data-rep]');
  if (rep) { reportAction(rep.dataset.rep); return; }
  const b = ev.target.closest('[data-cmd]'); if (!b || b.disabled) return;
  const c = b.dataset.cmd, st = S.st || {};
  if (c === 'tour') cmd(st.tour && st.tour.active ? 'tour_stop' : 'tour_start');
  else if (c === 'cut') cmd('cut', !(st.impair && st.impair.blackout));
  else if (c === 'antenna') cmd(st.procs && st.procs[1].alive ? 'kill_relay' : 'restart_relay');
  else if (c === 'feed') cmd('feed', !(st.gcs && st.gcs.feed_paused));
  else if (c === 'reset') {
    b.classList.remove('flash'); void b.offsetWidth; b.classList.add('flash');
    S.ev = []; S.hist = []; S.oldRoute = null; S.gcsOld = null; S.report = null; S.compare = null;
    cmd('reset').then((r) => { if (r && r.ok) toast('Reset: drone on the pad, clock at zero, link healthy.'); });
  } else if (c === 'rtl') cmd('rtl').then((r) => { if (r && r.ok) toast('Return to launch sent. The drone confirms in the event log.'); });
  else cmd(c);
});
function setFocusLayout(v) {
  const f = $('.jn-focus');
  f.classList.toggle('solo', v !== 'split'); f.classList.toggle('solo-gcs', v === 'gcs'); f.classList.toggle('solo-drone', v === 'drone');
  $$('[data-focus]').forEach((b) => b.classList.toggle('on', b.dataset.focus === v));
}
function openFocus(v) {
  setFocusLayout(v);
  const f = $('.jn-focus');
  if (document.fullscreenElement) return;
  if (f.requestFullscreen) f.requestFullscreen().catch(() => toast('Full screen is not allowed here.'));
}
function setTool(t) {
  S.tool = t;
  $$('[data-tool]').forEach((b) => b.classList.toggle('on', b.dataset.tool === t));
  const bar = $('.jn-edit'); bar.classList.toggle('zone', t === 'zone'); bar.classList.toggle('active', t !== 'none');
  $$('canvas.map').forEach((c) => c.classList.toggle('editing', t !== 'none'));
  $('.edit-hint').textContent = t === 'none' ? 'Pick a tool, then click the ground-station or drone map.'
    : t === 'start' || t === 'goal' ? `Click to move the ${t} (before launch).` : t === 'zone' ? 'Click to place the zone. During flight it forms at once.' : 'Click a weather zone to remove it.';
}
$$('[data-zopt]').forEach((el) => el.addEventListener('input', () => {
  const k = el.dataset.zopt;
  S.zopt[k] = el.type === 'checkbox' ? el.checked : (k === 'size' ? parseInt(el.value, 10) : el.value);
  if (k === 'size') el.parentElement.querySelector('b').textContent = el.value;
}));
$$('canvas.map').forEach((cv) => {
  cv.addEventListener('mousemove', (ev) => { S.hover = S.tool === 'none' ? null : cellAt(cv, ev); });
  cv.addEventListener('mouseleave', () => { S.hover = null; });
  cv.addEventListener('click', (ev) => {
    if (S.tool === 'none') return;
    const c = cellAt(cv, ev); if (!c) return;
    const a = { tool: S.tool, x: c[0], y: c[1] };
    if (S.tool === 'zone') Object.assign(a, S.zopt);
    cmd('edit', a).then((r) => { if (r && r.ok && (S.tool === 'start' || S.tool === 'goal')) setTool('none'); });
  });
});
document.addEventListener('fullscreenchange', () => {
  $$('.fs-btn').forEach((b) => { b.title = document.fullscreenElement ? 'Leave full screen (Esc)' : 'Full screen (Esc to leave)'; });
});
function setTab(t) {
  S.tab = t; store.set('tab', t);
  $$('.jn-tabs button').forEach((b) => { b.classList.toggle('on', b.dataset.tab === t); b.setAttribute('aria-selected', b.dataset.tab === t); });
  $$('section.jn-tab').forEach((s) => { s.hidden = s.dataset.tab !== t; });
  if (t === 'reports') refreshReport();
}
function applyChips() {
  $$('[data-chip]').forEach((b) => { b.classList.toggle('on', !!S.chips[b.dataset.chip]); b.setAttribute('aria-pressed', !!S.chips[b.dataset.chip]); });
  $$('[data-panel]').forEach((p) => { p.hidden = !S.chips[p.dataset.panel]; });
  store.set('chips', JSON.stringify(S.chips));
}
function setBg(v) {
  S.bg = v; store.set('bg', v);
  $$('[data-map-bg]').forEach((b) => b.classList.toggle('on', b.dataset.mapBg === v));
  if (v === 'grid') $('.map-note').textContent = '';
}
$$('input[data-imp]').forEach((inp) => {
  const k = inp.dataset.imp;
  inp.addEventListener('input', () => { S.drag[k] = performance.now(); showImp(k, parseFloat(inp.value)); });
  inp.addEventListener('change', () => { S.drag[k] = performance.now(); const o = {}; o[k] = parseFloat(inp.value); cmd('impair', o); });
});
function showImp(k, v) {
  const el = root.querySelector('[data-v="' + k + '"]'); if (!el) return;
  el.textContent = k === 'latency_ms' ? Math.round(v) + ' ms' : Math.round(v * 100) + '%';
}

// ------------------------------------------------------------------ geometry
function layout(cv) {
  const scn = S.scn, w = cv.clientWidth, h = cv.clientHeight;
  const cols = scn ? scn.cols : 25, rows = scn ? scn.rows : 17;
  let prof = PROF_PX[S.altSize] || 96; const pad = 8;
  const cell = Math.min((w - pad * 2) / cols, (h - prof - pad * 2 - 6) / rows);
  // width-limited (e.g. full screen): give the spare height to the altitude profile
  const spare = h - pad * 2 - 6 - rows * cell - prof;
  if (spare > 20) prof = Math.min(prof + spare, 300);
  const ox = (w - cell * cols) / 2, oy = pad;
  return { w, h, cell, ox, oy, cols, rows, prof: { x: ox, y: oy + rows * cell + 8, w: cell * cols, h: prof - 10 } };
}
function fitCanvas(cv) {
  // keep map + altitude profile in proportion (except in full screen, where CSS sizes it)
  if (document.fullscreenElement && document.fullscreenElement.contains(cv)) { cv.style.height = ''; return; }
  const cols = S.scn ? S.scn.cols : 25, rows = S.scn ? S.scn.rows : 17;
  const want = Math.round((cv.clientWidth - 16) * rows / cols + (PROF_PX[S.altSize] || 96) + 22);
  cv.style.aspectRatio = 'auto';
  if (Math.abs(cv.clientHeight - want) > 2) cv.style.height = want + 'px';
}
function cellAt(cv, ev) {
  const L = layout(cv), r = cv.getBoundingClientRect();
  const x = (ev.clientX - r.left - L.ox) / L.cell - 0.5, y = (ev.clientY - r.top - L.oy) / L.cell - 0.5;
  if (x < -0.5 || y < -0.5 || x > L.cols - 0.5 || y > L.rows - 0.5) return null;
  return [x, y];
}
function px(L, x, y) { return [L.ox + (x + 0.5) * L.cell, L.oy + (y + 0.5) * L.cell]; }
function sizeCanvas(cv) {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = Math.max(10, Math.round(cv.clientWidth * dpr)), h = Math.max(10, Math.round(cv.clientHeight * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); return ctx;
}
function altFt(z) { const f = S.scn ? S.scn.layer_ft : 50; return Math.round(z * f).toLocaleString() + ' ft'; }
function rgba(hex, a) { const n = parseInt(hex.slice(1), 16); return `rgba(${n >> 16},${(n >> 8) & 255},${n & 255},${a})`; }
// zone geometry at time t: boxes and circles, drifting with vx, vy (cells/s) since t_ref
function zoneOff(h, t) {
  if (!h.vx && !h.vy) return [0, 0];
  const dt = Math.max(0, t - (h.t_ref !== undefined ? h.t_ref : (h.t_on || 0)));
  return [(h.vx || 0) * dt, (h.vy || 0) * dt];
}
function plumePuffs(h, t) {
  if (h.shape !== 'plume') return [h];
  if (h.freeze !== undefined && h.freeze !== null) t = h.freeze;
  const every = h.every, life = h.life, t0 = h.t_start || 0, frozen = h.freeze !== undefined && h.freeze !== null;
  const kLo = Math.max(0, Math.ceil((t - life - t0) / every)), kHi = Math.floor((Math.min(t, h.t_stop || 1e9) - t0) / every);
  const out = [];
  for (let k = kLo; k <= kHi; k++) {
    const tk = t0 + k * every, age = t - tk; if (age < 0 || age > life) continue;
    const wob = (h.meander || 0.3) * Math.sin(tk * 0.7 + (h.phase || 0));
    const r = h.r0 + h.dr * age, c = h.c0 * (h.r0 / r) ** 2 * Math.exp(-age / h.tau);
    if (c < 0.2) continue;
    out.push({ id: h.id, type: 'ash', shape: 'circle', cx: h.vent[0] + h.ux * h.speed * age - h.uy * wob, cy: h.vent[1] + h.uy * h.speed * age + h.ux * wob,
               r, conc_mg: c, sev: c >= 4 ? 3 : c >= 2 ? 2 : 1, vx: frozen ? 0 : h.vx, vy: frozen ? 0 : h.vy, t_ref: t, z0: h.z0, z1: h.z1, puff: true, age });
  }
  return out;
}
function zoneHas(h, x, y, t) {
  if (h.shape === 'plume') return plumePuffs(h, t).some((p) => zoneHas(p, x, y, t));
  const [dx, dy] = zoneOff(h, t);
  if (h.shape === 'circle') return (x - h.cx - dx) ** 2 + (y - h.cy - dy) ** 2 <= h.r * h.r;
  return x >= h.x0 + dx - 0.5 && x < h.x1 + dx + 0.5 && y >= h.y0 + dy - 0.5 && y < h.y1 + dy + 0.5;
}
function zoneR(h, t) { return h.r + (h.dr ? h.dr * Math.max(0, t - (h.t_ref !== undefined ? h.t_ref : (h.t_on || 0))) : 0); }
function zonePx(L, h, t) {
  const [dx, dy] = zoneOff(h, t);
  if (h.shape === 'circle') {
    const [cx, cy] = px(L, h.cx + dx, h.cy + dy), r = zoneR(h, t);
    return { circle: true, cx, cy, r: r * L.cell, x: cx - r * L.cell, y: cy - r * L.cell, w: 2 * r * L.cell, h: 2 * r * L.cell };
  }
  const x = L.ox + (h.x0 + dx) * L.cell, y = L.oy + (h.y0 + dy) * L.cell;
  const w = (h.x1 - h.x0 + 1) * L.cell, hh = (h.y1 - h.y0 + 1) * L.cell;
  return { circle: false, x, y, w, h: hh, cx: x + w / 2, cy: y + hh / 2 };
}
function zonePath(ctx, g, grow) {
  const e = grow || 0;
  ctx.beginPath();
  if (g.circle) ctx.arc(g.cx, g.cy, g.r + e, 0, Math.PI * 2);
  else if (ctx.roundRect) ctx.roundRect(g.x + 1 - e, g.y + 1 - e, g.w - 2 + 2 * e, g.h - 2 + 2 * e, Math.min(10, 4 + e));
  else ctx.rect(g.x + 1 - e, g.y + 1 - e, g.w - 2 + 2 * e, g.h - 2 + 2 * e);
}
function centerText(ctx, lines, cx, cy, maxW, colors, maxH) {
  const lh = 12;
  if (maxH) {                                                // keep the first line and any status line; drop detail that does not fit
    const fit = Math.max(1, Math.floor((maxH - 2) / lh));
    if (lines.length > fit) {
      const keep = colors && colors.length === lines.length && colors[lines.length - 1] !== colors[0] ? [lines.length - 1] : [];
      const idx = [...Array(lines.length).keys()].filter((i) => i < fit - keep.length || keep.includes(i));
      lines = idx.map((i) => lines[i]); if (colors) colors = idx.map((i) => colors[i]);
    }
  }
  let y = cy - ((lines.length - 1) * lh) / 2 + 3.5;
  ctx.textAlign = 'center';
  lines.forEach((t, i) => {
    let s = t; while (s.length > 3 && ctx.measureText(s).width > maxW) s = s.slice(0, -2);
    if (s !== t) s = s.slice(0, -1) + '…';
    const w = ctx.measureText(s).width + 6;
    ctx.fillStyle = 'rgba(8,12,18,0.55)'; ctx.fillRect(cx - w / 2, y - 9.5, w, 12.5);
    ctx.fillStyle = (colors && colors[i]) || 'rgba(255,255,255,0.95)'; ctx.fillText(s, cx, y); y += lh;
  });
  ctx.textAlign = 'left';
}
function hazColor(h) { return h.type === 'ash' ? C['ash' + (h.sev || 3)] : (C[h.type] || C.turb); }

// ------------------------------------------------------------------ backgrounds
function terrain(L, key) {
  if (L.w < 2 || L.h < 2) { const z = document.createElement('canvas'); z.width = z.height = 1; return z; }
  const k = key + ':' + Math.round(L.w) + 'x' + Math.round(L.h) + ':' + S.scnRev;
  if (S.terrain[k]) return S.terrain[k];
  const off = document.createElement('canvas'); const dpr = Math.min(2, window.devicePixelRatio || 1);
  off.width = Math.round(L.w * dpr); off.height = Math.round(L.h * dpr);
  const c = off.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0);
  const seed = (S.scn && S.scn.seed) || 1;
  const rnd = (i, j) => { const s = Math.sin(i * 127.1 + j * 311.7 + seed * 74.7) * 43758.5453; return s - Math.floor(s); };
  const noise = (x, y) => { const xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi;
    const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
    const a = rnd(xi, yi), b = rnd(xi + 1, yi), cc = rnd(xi, yi + 1), d = rnd(xi + 1, yi + 1);
    return a + (b - a) * u + (cc - a) * v + (a - b - cc + d) * u * v; };
  c.fillStyle = C.bg; c.fillRect(0, 0, L.w, L.h);
  for (let y = L.oy; y < L.oy + L.rows * L.cell; y += 4) {
    for (let x = L.ox; x < L.ox + L.cols * L.cell; x += 4) {
      const nx = (x - L.ox) / L.cell / 5, ny = (y - L.oy) / L.cell / 5;
      const v = noise(nx, ny) * 0.6 + noise(nx * 2.3, ny * 2.3) * 0.4;
      if (Math.floor(v * 9) % 2) { c.fillStyle = 'rgba(90,120,150,0.035)'; c.fillRect(x, y, 4, 4); }
    }
  }
  gridLines(c, L);
  S.terrain[k] = off; return off;
}
function gridLines(c, L, alpha) {
  const a = alpha === undefined ? 1 : alpha;
  for (let i = 0; i <= L.cols; i++) {
    c.strokeStyle = i % 5 === 0 ? `rgba(120,150,180,${0.14 * a})` : `rgba(120,150,180,${0.07 * a})`; c.lineWidth = 1; c.beginPath();
    c.moveTo(L.ox + i * L.cell + 0.5, L.oy); c.lineTo(L.ox + i * L.cell + 0.5, L.oy + L.rows * L.cell); c.stroke();
  }
  for (let j = 0; j <= L.rows; j++) {
    c.strokeStyle = j % 5 === 0 ? `rgba(120,150,180,${0.14 * a})` : `rgba(120,150,180,${0.07 * a})`; c.beginPath();
    c.moveTo(L.ox, L.oy + j * L.cell + 0.5); c.lineTo(L.ox + L.cols * L.cell, L.oy + j * L.cell + 0.5); c.stroke();
  }
}
// OpenStreetMap backdrop: standard slippy-map tiles, web-Mercator maths.
function mercX(lon, z) { return (lon + 180) / 360 * 256 * Math.pow(2, z); }
function mercY(lat, z) { const r = lat * Math.PI / 180; return (1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * 256 * Math.pow(2, z); }
function tile(z, x, y) {
  const k = `${z}/${x}/${y}`;
  let t = S.tiles[k];
  if (!t) {
    t = S.tiles[k] = { img: new Image(), ok: false, bad: false };
    t.img.onload = () => { t.ok = true; };
    t.img.onerror = () => { t.bad = true; S.tileFail++; };
    t.img.src = `https://tile.openstreetmap.org/${k}.png`;
  }
  return t;
}
function drawOsm(ctx, L) {
  const scn = S.scn, cfg = (S.st && S.st.cfg) || {};
  const [lat0, lon0] = CENTERS[cfg.map_center] || CENTERS.bengaluru;
  const cellM = scn.cell_m || 100, wM = L.cols * cellM, hM = L.rows * cellM;
  const dLat = hM / 111320, dLon = wM / (111320 * Math.cos(lat0 * Math.PI / 180));
  const north = lat0 + dLat / 2, south = lat0 - dLat / 2, west = lon0 - dLon / 2, east = lon0 + dLon / 2;
  const target = cellM / L.cell;                                   // metres per screen pixel
  const z = Math.max(2, Math.min(18, Math.ceil(Math.log2(156543.03 * Math.cos(lat0 * Math.PI / 180) / target))));
  const x0 = mercX(west, z), x1 = mercX(east, z), y0 = mercY(north, z), y1 = mercY(south, z);
  const sx = (L.cols * L.cell) / (x1 - x0), sy = (L.rows * L.cell) / (y1 - y0);
  ctx.save(); ctx.beginPath(); ctx.rect(L.ox, L.oy, L.cols * L.cell, L.rows * L.cell); ctx.clip();
  ctx.fillStyle = C.bg; ctx.fillRect(L.ox, L.oy, L.cols * L.cell, L.rows * L.cell);
  let loaded = 0, total = 0, bad = 0;
  for (let ty = Math.floor(y0 / 256); ty <= Math.floor(y1 / 256); ty++) {
    for (let tx = Math.floor(x0 / 256); tx <= Math.floor(x1 / 256); tx++) {
      total++;
      const t = tile(z, tx, ty);
      if (t.bad) { bad++; continue; }
      if (!t.ok) continue;
      loaded++;
      ctx.drawImage(t.img, L.ox + (tx * 256 - x0) * sx, L.oy + (ty * 256 - y0) * sy, 256 * sx + 0.5, 256 * sy + 0.5);
    }
  }
  ctx.fillStyle = 'rgba(11,18,26,0.52)'; ctx.fillRect(L.ox, L.oy, L.cols * L.cell, L.rows * L.cell);   // keep hazards readable
  ctx.restore();
  gridLines(ctx, L, 0.8);
  ctx.font = `10px ${FB}`; const att = '© OpenStreetMap contributors';
  const aw = ctx.measureText(att).width + 8, ax = L.ox + L.cols * L.cell - aw, ay = L.oy + L.rows * L.cell - 15;
  ctx.fillStyle = 'rgba(11,18,26,0.8)'; ctx.fillRect(ax, ay, aw, 15); ctx.fillStyle = C.ink; ctx.fillText(att, ax + 4, ay + 11);
  const note = bad === total && total ? 'Map tiles unreachable (offline?). Showing the grid only.'
    : loaded < total ? `Loading map tiles ${loaded}/${total}…` : 'Map backdrop only: obstacles and weather are simulated.';
  if (S.tab === 'views') { const n = $('.map-note'); if (n.textContent !== note) n.textContent = note; }
}

// ------------------------------------------------------------------ drawing helpers
function hazardBlocks(h, prof) {
  if (!prof) return true;
  if (h.type === 'tfr' && prof.sar) return false;
  if (['building', 'geofence', 'ridge', 'tfr'].includes(h.type)) return true;
  if (h.type === 'ash') return !(prof.sar && (h.sev || 3) <= 1);
  if (h.type === 'turb') return (h.sev || 2) >= prof.turb_block;
  if (h.type === 'gust') return (h.gust_kt || 0) > prof.gust_limit_kt;
  if (h.type === 'rain') return !prof.rain_rated;
  return false;
}
const BAND = ['', 'LOW', 'MED', 'HIGH'];
function altRange(z0, z1) {
  const f = S.scn ? S.scn.layer_ft : 50, aircraft = S.scn && S.scn.mode === 'aircraft';
  const lab = (z) => aircraft ? (z === 0 ? 'SFC' : 'FL' + String(Math.round(z * f / 100)).padStart(3, '0')) : String(Math.round(z * f));
  return aircraft ? `${lab(z0)}–${lab(z1)}` : `${lab(z0)}–${lab(z1)} ft`;
}
function hazLabel(h) {
  const full = S.scn && h.type === 'ash' && h.z1 >= S.scn.layers - 1 && S.scn.mode === 'aircraft';
  const z = full ? 'SFC–TOP' : altRange(h.z0, h.z1);
  if (h.type === 'ash') return `${h.id} ASH ${BAND[h.sev || 3]}${h.conc_mg ? ' · ' + (+h.conc_mg).toFixed(h.conc_mg >= 10 ? 0 : 1) + ' mg/m³' : ''} · ${z}`;
  if (h.type === 'gust') return `${h.id} GUST · ${h.gust_kt} kt · ${z}`;
  if (h.type === 'rain') return `${h.id} RAIN · ${h.rain_mmh || ''} mm/h`;
  if (h.type === 'turb') return `${h.id} TURB ${['', 'LOW', 'MOD', 'HIGH'][h.sev]} · ${z}`;
  if (h.type === 'building') return `${h.id} ${altFt(h.z1)}`;
  if (h.type === 'geofence') return 'NO-FLY';
  if (h.type === 'tfr') return `${h.id} TFR ${z}`;
  if (h.type === 'ridge') return `${h.id} ridge ${altFt(h.z1)}`;
  return h.id;
}
function fitText(ctx, text, x, y, maxW) {
  if (ctx.measureText(text).width <= maxW) { ctx.fillText(text, x, y); return; }
  const parts = text.split(' · '); let t = parts[0];
  if (ctx.measureText(t).width > maxW) { const words = t.split(' '); t = words.slice(0, 2).join(' '); }
  ctx.fillText(t, x, y);
  if (parts[1] && maxW > 40 && ctx.measureText(parts[1]).width <= maxW + 10) ctx.fillText(parts[1], x, y + 11);
}
function rectOf(L, h) { return [L.ox + h.x0 * L.cell, L.oy + h.y0 * L.cell, (h.x1 - h.x0 + 1) * L.cell, (h.y1 - h.y0 + 1) * L.cell]; }
function hatch(ctx, x, y, w, h, color, gap) {
  ctx.save(); ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip(); ctx.strokeStyle = color; ctx.lineWidth = 1;
  for (let i = -h; i < w; i += gap) { ctx.beginPath(); ctx.moveTo(x + i, y + h); ctx.lineTo(x + i + h, y); ctx.stroke(); }
  ctx.restore();
}
function speckle(ctx, x, y, w, h, color, seed) {
  ctx.save(); ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip(); ctx.fillStyle = color;
  const n = Math.round(w * h / 60);
  for (let i = 0; i < n; i++) { const a = Math.sin(i * 12.9898 + seed * 78.233) * 43758.5453, b = Math.sin(i * 39.346 + seed * 11.135) * 24634.6345;
    ctx.fillRect(x + (a - Math.floor(a)) * w, y + (b - Math.floor(b)) * h, 1.6, 1.6); }
  ctx.restore();
}
function drawStatic(ctx, L, list) {
  for (const o of list) {
    const [x, y, w, h] = rectOf(L, o);
    if (o.type === 'building' || o.type === 'ridge') {
      const top = (o.z1 + 1) / (S.scn ? S.scn.layers : 9);
      ctx.fillStyle = rgba(o.type === 'building' ? C.building : C.ridge, S.bg === 'osm' ? 0.85 : 1); ctx.fillRect(x + 1, y + 1, w - 2, h - 2);
      ctx.fillStyle = rgba(o.type === 'building' ? '#7fa3c8' : '#c4a27a', 0.10 + 0.32 * top); ctx.fillRect(x + 1, y + 1, w - 2, h - 2);
      if (o.rf_db) { ctx.strokeStyle = rgba('#7fa3c8', 0.9); ctx.lineWidth = 1.5; ctx.strokeRect(x + 1.5, y + 1.5, w - 3, h - 3); }
    } else {
      const col = o.type === 'geofence' ? C.geofence : C.tfr;
      ctx.fillStyle = rgba(col, 0.07); ctx.fillRect(x, y, w, h); hatch(ctx, x, y, w, h, rgba(col, 0.35), 7);
      ctx.strokeStyle = rgba(col, 0.8); ctx.setLineDash([5, 3]); ctx.lineWidth = 1.2; ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1); ctx.setLineDash([]);
    }
    if (L.cell > 15) { ctx.fillStyle = rgba('#dfe8f1', 0.75); ctx.font = `10px ${FM}`; fitText(ctx, hazLabel(o), x + 3, y + 11, w - 5); }
  }
}
function drawWeather(ctx, L, list, prof, style, t) {
  const sorted = list.slice().sort((a, b) => (a.type === 'ash') - (b.type === 'ash'));
  for (const h of sorted) {
    if (h.shape === 'plume') { drawPlume(ctx, L, h, prof, style, t); continue; }
    const g = zonePx(L, h, t);
    const col = hazColor(h);
    const blocks = hazardBlocks(h, prof);
    if (style === 'predicted') {
      ctx.save(); ctx.fillStyle = rgba(col, 0.10); zonePath(ctx, g); ctx.fill();
      ctx.strokeStyle = rgba(col, 0.75); ctx.setLineDash([6, 5]); ctx.lineWidth = 1.4; zonePath(ctx, g); ctx.stroke(); ctx.restore();
      if (h.type === 'ash' && (h.vx || h.vy)) drawTrajectory(ctx, L, h, t, 0);
      if (L.cell > 14) { ctx.font = `10px ${FM}`; centerText(ctx, [h.id + ' predicted'], g.cx, g.cy, g.w - 4, [rgba(col, 1)]); }
      continue;
    }
    if (style === 'unseen') {
      ctx.save(); ctx.strokeStyle = rgba(col, 0.6); ctx.setLineDash([2, 4]); ctx.lineWidth = 1.2; zonePath(ctx, g); ctx.stroke(); ctx.restore();
      if (L.cell > 14) { ctx.font = `10px ${FM}`; centerText(ctx, [h.unreported ? 'unreported ash' : 'not yet known'], g.cx, g.cy, g.w - 4, [rgba(col, 0.9)]); }
      continue;
    }
    ctx.save();
    if (h.type === 'ash') {
      // diffuse edge: the concentration the OPC sees falls off outside the core
      const halo = ctx.createRadialGradient(g.cx, g.cy, Math.max(1, Math.min(g.w, g.h) * 0.35), g.cx, g.cy, Math.max(g.w, g.h) * 0.5 + L.cell * 1.1);
      halo.addColorStop(0, rgba(col, 0.0)); halo.addColorStop(0.6, rgba(col, 0.10)); halo.addColorStop(1, rgba(col, 0));
      ctx.fillStyle = halo; zonePath(ctx, g, L.cell * 1.1); ctx.fill();
      const core = ctx.createRadialGradient(g.cx, g.cy, 1, g.cx, g.cy, Math.max(g.w, g.h) * 0.62);
      core.addColorStop(0, rgba(col, h.estimated ? 0.32 : 0.55)); core.addColorStop(1, rgba(col, h.estimated ? 0.1 : 0.2));
      ctx.fillStyle = core; zonePath(ctx, g); ctx.fill();
      ctx.save(); zonePath(ctx, g); ctx.clip(); speckle(ctx, g.x, g.y, g.w, g.h, rgba('#0b121a', 0.45), Math.round((h.x0 + 1) * (h.y0 + 3))); ctx.restore();
      ctx.lineWidth = blocks ? 2 : 1.2; ctx.strokeStyle = rgba(col, blocks ? 1 : 0.7);
      if (!blocks || h.estimated) ctx.setLineDash(h.estimated ? [6, 4] : [4, 3]);
      zonePath(ctx, g); ctx.stroke();
    } else {
      const sev = h.type === 'gust' ? Math.min(1, (h.gust_kt || 15) / 35) : (h.sev || 2) / 3;
      const gr = ctx.createRadialGradient(g.cx, g.cy, 1, g.cx, g.cy, Math.max(g.w, g.h) * 0.7);
      gr.addColorStop(0, rgba(col, 0.18 + 0.25 * sev)); gr.addColorStop(1, rgba(col, 0.05));
      ctx.fillStyle = gr; zonePath(ctx, g); ctx.fill();
      if (h.type === 'rain') { ctx.save(); zonePath(ctx, g); ctx.clip(); hatch(ctx, g.x, g.y, g.w, g.h, rgba(col, 0.25), 5); ctx.restore(); }
      if (h.type === 'turb') { ctx.save(); zonePath(ctx, g); ctx.clip(); ctx.strokeStyle = rgba(col, 0.35); ctx.lineWidth = 1;
        for (let yy = g.y + 6; yy < g.y + g.h; yy += 8) { ctx.beginPath(); for (let xx = g.x; xx <= g.x + g.w; xx += 4) ctx.lineTo(xx, yy + Math.sin(xx / 5) * 2.2); ctx.stroke(); } ctx.restore(); }
      ctx.lineWidth = blocks ? 1.6 : 1; ctx.strokeStyle = blocks ? rgba(col, 0.9) : rgba(col, 0.5);
      if (!blocks) ctx.setLineDash([4, 3]);
      zonePath(ctx, g); ctx.stroke();
    }
    ctx.restore();
    if (h.type === 'ash' && (h.vx || h.vy) && style !== 'unseen') drawTrajectory(ctx, L, h, t, style === 'predicted' ? 0 : 1);
    // drift arrow
    if ((h.vx || h.vy) && h.type !== 'ash') {
      const sp = Math.hypot(h.vx || 0, h.vy || 0), ux = (h.vx || 0) / sp, uy = (h.vy || 0) / sp, len = Math.min(g.w, g.h) * 0.5 + 10;
      const ax = g.cx + ux * (Math.min(g.w, g.h) * 0.5), ay = g.cy + uy * (Math.min(g.w, g.h) * 0.5);
      ctx.save(); ctx.strokeStyle = rgba(col, 0.9); ctx.fillStyle = rgba(col, 0.9); ctx.lineWidth = 1.6; ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax + ux * 12, ay + uy * 12); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(ax + ux * 16, ay + uy * 16); ctx.lineTo(ax + ux * 10 - uy * 4, ay + uy * 10 + ux * 4); ctx.lineTo(ax + ux * 10 + uy * 4, ay + uy * 10 - ux * 4); ctx.fill(); ctx.restore();
    }
    if (L.cell > 14) {
      ctx.font = `${h.type === 'ash' ? '600 ' : ''}10px ${FM}`;
      const lines = hazLabel(h).split(' · ');
      const cols = lines.map(() => 'rgba(255,255,255,0.95)');
      if (style === 'sensed') { lines.push(h.estimated ? 'measured on board' : 'found on board'); cols.push(C.amber); }
      else if (style === 'reported') { lines.push('reported by drone'); cols.push(C.amber); }
      else if (!blocks && g.h > 30) { lines.push(h.type === 'ash' ? 'SAR: passable' : 'passable'); cols.push(rgba(col, 1)); }
      centerText(ctx, lines, g.cx, g.cy, Math.max(30, (g.circle ? g.w * 0.86 : g.w) - 6), cols, g.circle ? g.h * 0.8 : g.h);
    }
  }
}
// Volcanic plume: puffs from a vent, dense and narrow near it, wide and faint downwind.
function drawPlume(ctx, L, h, prof, style, t) {
  const puffs = plumePuffs(h, t);
  const [vx0, vy0] = px(L, h.vent[0], h.vent[1]);
  if (style === 'predicted') {
    ctx.save(); ctx.fillStyle = rgba(C.ash2, 0.07);
    for (const p of puffs) { const g = zonePx(L, p, t); ctx.beginPath(); ctx.arc(g.cx, g.cy, g.r, 0, 7); ctx.fill(); }
    ctx.restore();
    ctx.font = `10px ${FM}`; centerText(ctx, [`${h.name || h.id} predicted`], vx0, vy0 + 18, 160, [C.warn]); return;
  }
  if (style === 'unseen') {
    ctx.save(); ctx.strokeStyle = rgba(C.ash3, 0.5); ctx.setLineDash([2, 4]);
    for (const p of puffs) { const g = zonePx(L, p, t); zonePath(ctx, g); ctx.stroke(); }
    ctx.restore(); return;
  }
  ctx.save();
  for (const p of puffs.slice().reverse()) {                    // oldest (faintest, widest) first
    const g = zonePx(L, p, t), col = C['ash' + p.sev], a = Math.min(0.55, 0.12 + 0.07 * p.conc_mg);
    const gr = ctx.createRadialGradient(g.cx, g.cy, 1, g.cx, g.cy, g.r * 1.25);
    gr.addColorStop(0, rgba(col, a)); gr.addColorStop(0.75, rgba(col, a * 0.6)); gr.addColorStop(1, rgba(col, 0));
    ctx.fillStyle = gr; ctx.beginPath(); ctx.arc(g.cx, g.cy, g.r * 1.25, 0, 7); ctx.fill();
    if (hazardBlocks(p, prof)) { ctx.strokeStyle = rgba(col, 0.35); ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(g.cx, g.cy, g.r, 0, 7); ctx.stroke(); }
  }
  ctx.restore();
  // vent
  ctx.save(); ctx.fillStyle = '#c0392b'; ctx.strokeStyle = '#ffd2c2'; ctx.lineWidth = 1.2;
  ctx.beginPath(); ctx.moveTo(vx0, vy0 - 9); ctx.lineTo(vx0 + 8, vy0 + 6); ctx.lineTo(vx0 - 8, vy0 + 6); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  if (h.speed && h.freeze == null) drawTrajectory(ctx, L, { cx: h.vent[0], cy: h.vent[1], r: h.r0, vx: h.ux * h.speed, vy: h.uy * h.speed, t_ref: t, dr: h.dr }, t, 1, h.life);
  ctx.font = `600 10px ${FM}`;
  const lines = [`${h.name || h.id} · ${(+h.c0).toFixed(1)} mg/m³ at vent`, altRange(h.z0, h.z1)];
  if (style === 'reported') lines.push('reported by drone');
  centerText(ctx, lines, vx0, vy0 + 22, 200, ['#fff', 'rgba(255,255,255,0.8)', C.amber]);
}
// Ash moves with the wind: project where it will be. Ticks every 5 s; the
// swath widens with spread (dr) and, for estimates, with wind uncertainty.
function drawTrajectory(ctx, L, h, t, solid, horizon) {
  const H = horizon ? Math.min(horizon, 25) : 15, steps = 5;
  const [dx, dy] = zoneOff(h, t);
  const cx0 = (h.cx !== undefined ? h.cx : (h.x0 + h.x1) / 2) + dx, cy0 = (h.cy !== undefined ? h.cy : (h.y0 + h.y1) / 2) + dy;
  const [ax, ay] = px(L, cx0, cy0), [bx, by] = px(L, cx0 + h.vx * H, cy0 + h.vy * H);
  const r0 = (h.r !== undefined ? zoneR(h, t) : Math.max(h.x1 - h.x0 + 1, h.y1 - h.y0 + 1) / 2);
  const grow = (h.dr || 0) * H + (h.estimated ? 0.06 * H : 0);
  const len = Math.hypot(bx - ax, by - ay); if (len < 6) return;
  const nx = -(by - ay) / len, ny = (bx - ax) / len, w0 = r0 * L.cell, w1 = (r0 + grow) * L.cell;
  ctx.save();
  ctx.fillStyle = rgba(h.estimated ? C.amber : C.ash2, 0.06);
  ctx.beginPath(); ctx.moveTo(ax + nx * w0, ay + ny * w0); ctx.lineTo(bx + nx * w1, by + ny * w1); ctx.lineTo(bx - nx * w1, by - ny * w1); ctx.lineTo(ax - nx * w0, ay - ny * w0); ctx.closePath(); ctx.fill();
  ctx.strokeStyle = rgba(h.estimated ? C.amber : '#cfd8e3', solid ? 0.75 : 0.4); ctx.setLineDash([5, 4]); ctx.lineWidth = 1.3;
  ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = rgba('#cfd8e3', 0.85); ctx.font = `9px ${FM}`;
  for (let k = 1; k <= steps; k++) {
    const f = k / steps, x = ax + (bx - ax) * f, y = ay + (by - ay) * f;
    ctx.beginPath(); ctx.arc(x, y, 2, 0, 7); ctx.fill();
    if (k === steps || k === Math.ceil(steps / 2)) ctx.fillText(`+${Math.round(H * f)}s`, x + 4, y - 3);
  }
  ctx.beginPath(); const ux = (bx - ax) / len, uy = (by - ay) / len;
  ctx.moveTo(bx + ux * 7, by + uy * 7); ctx.lineTo(bx - uy * 4, by + ux * 4); ctx.lineTo(bx + uy * 4, by - ux * 4); ctx.fill();
  ctx.restore();
}
function drawSamples(ctx, L, d, t) {
  const S_ = d.samples || [], w = d.wind_meas;
  if (!S_.length) return;
  const vx = w ? w[2] : 0, vy = w ? w[3] : 0;
  ctx.save();
  for (const s of S_) {
    const age = t - s[2]; if (age > 30) continue;
    const [x, y] = px(L, s[0] + vx * age, s[1] + vy * age);
    const col = s[3] >= 4 ? C.ash3 : s[3] >= 2 ? C.ash2 : C.ash1;
    ctx.fillStyle = rgba(col, Math.max(0.25, 0.9 - age / 35)); ctx.beginPath(); ctx.arc(x, y, 2.4, 0, 7); ctx.fill();
  }
  ctx.restore();
}
function drawPath(ctx, L, pts, color, width, dash, alpha, straight) {
  if (!pts || pts.length < 2) return;
  ctx.save(); ctx.globalAlpha = alpha === undefined ? 1 : alpha; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  if (dash) ctx.setLineDash(dash);
  const P = pts.map((c) => px(L, c[0], c[1]));
  ctx.beginPath(); ctx.moveTo(P[0][0], P[0][1]);
  if (straight || P.length < 3) { for (let i = 1; i < P.length; i++) ctx.lineTo(P[i][0], P[i][1]); }
  else {
    // rounded corners: quadratic Bézier through each cell centre, from mid-leg to mid-leg
    // (the drone flies exactly this curve: see DroneBrain._curve_pos)
    ctx.lineTo((P[0][0] + P[1][0]) / 2, (P[0][1] + P[1][1]) / 2);
    for (let i = 1; i < P.length - 1; i++) ctx.quadraticCurveTo(P[i][0], P[i][1], (P[i][0] + P[i + 1][0]) / 2, (P[i][1] + P[i + 1][1]) / 2);
    ctx.lineTo(P[P.length - 1][0], P[P.length - 1][1]);
  }
  ctx.stroke(); ctx.restore();
}
function drawTargets(ctx, L) {
  const scn = S.scn; if (!scn) return;
  const [hx, hy] = px(L, scn.home[0], scn.home[1]);
  ctx.strokeStyle = C.ice; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(hx, hy + 7); ctx.lineTo(hx, hy - 8); ctx.stroke();
  for (let r = 4; r <= 10; r += 3) { ctx.beginPath(); ctx.arc(hx, hy - 8, r, -2.4, -0.74); ctx.strokeStyle = rgba('#8ab8ff', 1 - r / 13); ctx.stroke(); }
  ctx.fillStyle = C.ice; ctx.font = `10px ${FM}`; ctx.fillText('GCS / HOME', hx + 8, hy + 12);
  scn.targets.forEach((t, i) => {
    const [x, y] = px(L, t[0], t[1]); const last = i === scn.targets.length - 1;
    ctx.strokeStyle = last ? C.ok : C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, Math.max(5, L.cell * 0.32), 0, 7); ctx.stroke();
    ctx.fillStyle = last ? C.ok : C.ink; ctx.font = `600 11px ${FB}`; ctx.textAlign = 'center';
    ctx.fillText(last ? 'GOAL' : 'WP' + (i + 1), x, y - Math.max(7, L.cell * 0.38)); ctx.textAlign = 'left';
  });
}
// edit-mode preview under the cursor
function drawHover(ctx, L, t) {
  if (S.tool === 'none' || !S.hover || !S.scn) return;
  const [hx, hy] = S.hover, x = Math.round(hx), y = Math.round(hy);
  ctx.save(); ctx.strokeStyle = rgba('#ffffff', 0.85); ctx.setLineDash([4, 3]); ctx.lineWidth = 1.4;
  if (S.tool === 'zone') {
    const o = S.zopt, col = o.type === 'ash' ? C['ash' + o.sev] : (C[o.type] || C.turb);
    const g = o.shape === 'circle' ? zonePx(L, { shape: 'circle', cx: x, cy: y, r: o.size + 0.2 }, 0)
      : zonePx(L, { x0: x - (o.size - 1), x1: x + (o.size - 1), y0: y - (o.size - 1), y1: y + (o.size - 1) }, 0);
    ctx.fillStyle = rgba(col, 0.25); zonePath(ctx, g); ctx.fill(); ctx.strokeStyle = rgba(col, 1); zonePath(ctx, g); ctx.stroke();
  } else {
    const [cx, cy] = px(L, x, y); ctx.beginPath(); ctx.arc(cx, cy, L.cell * 0.45, 0, 7); ctx.stroke();
    ctx.font = `600 10px ${FM}`; ctx.fillStyle = '#fff';
    ctx.fillText(S.tool === 'start' ? 'START' : S.tool === 'goal' ? 'GOAL' : 'ERASE', cx + L.cell * 0.5, cy - L.cell * 0.4);
  }
  ctx.restore();
}
function drawDrone(ctx, L, p, heading, color, hollow, label) {
  const [x, y] = px(L, p[0], p[1]); const r = Math.max(6, L.cell * 0.36);
  ctx.save(); ctx.translate(x, y); ctx.rotate(heading);
  ctx.beginPath(); ctx.moveTo(r, 0); ctx.lineTo(-r * 0.7, r * 0.62); ctx.lineTo(-r * 0.35, 0); ctx.lineTo(-r * 0.7, -r * 0.62); ctx.closePath();
  if (hollow) { ctx.setLineDash([3, 2]); ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.stroke(); }
  else { ctx.fillStyle = color; ctx.shadowColor = color; ctx.shadowBlur = 10; ctx.fill(); }
  ctx.restore();
  if (label) { ctx.fillStyle = color; ctx.font = `11px ${FM}`; ctx.fillText(label, x + r + 4, y - r); }
}
function heading(route, seg, fallback) {
  if (route && route.length > 1) {
    for (let i = Math.max(0, seg); i < route.length - 1; i++) {
      const a = route[i], b = route[i + 1];
      if (a[0] !== b[0] || a[1] !== b[1]) return Math.atan2(b[1] - a[1], b[0] - a[0]);
    }
  }
  return fallback || 0;
}
function smooth(key, target, k) {
  if (!target) return null;
  const cur = S.disp[key];
  if (!cur || Math.hypot(cur[0] - target[0], cur[1] - target[1]) > 3) { S.disp[key] = target.slice(); return S.disp[key]; }
  for (let i = 0; i < 3; i++) cur[i] += (target[i] - cur[i]) * k;
  return cur;
}
// In-situ sampling: OPC + SO2 measure the air at the drone. A ring around the
// drone shows the OPC reading; it pulses red inside ash.
function drawSampling(ctx, L, p, d) {
  if (!['ENROUTE', 'RTL', 'RTLINK', 'REPLAN', 'HOLD', 'HOLDING'].includes(d.state)) return;
  const s = d.sensors || {}, opc = s.opc || 0, thr = s.thr || 0.2;
  const [x, y] = px(L, p[0], p[1]);
  const lvl = opc >= 4 ? 3 : opc >= 2 ? 2 : opc >= thr ? 1 : 0;
  const col = lvl === 0 ? C.cyan : C['ash' + lvl];
  const pulse = lvl ? 0.5 + 0.5 * Math.sin(performance.now() / 110) : 0.5 + 0.5 * Math.sin(performance.now() / 600);
  const R = L.cell * (0.62 + 0.18 * pulse);
  ctx.save();
  const g = ctx.createRadialGradient(x, y, R * 0.3, x, y, R * (lvl ? 1.6 : 1.2));
  g.addColorStop(0, rgba(col, lvl ? 0.35 : 0.12)); g.addColorStop(1, rgba(col, 0));
  ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, R * (lvl ? 1.6 : 1.2), 0, 7); ctx.fill();
  ctx.strokeStyle = rgba(col, lvl ? 0.95 : 0.5); ctx.lineWidth = lvl ? 2 : 1; ctx.setLineDash(lvl ? [] : [3, 3]);
  ctx.beginPath(); ctx.arc(x, y, R, 0, 7); ctx.stroke(); ctx.restore();
  if (lvl) tag(ctx, x + R + 6, y + R + 10, `OPC ${opc.toFixed(opc >= 10 ? 0 : 1)} mg/m³${s.opc_sat ? ' SAT' : ''} · SO₂ ${(s.so2 || 0).toFixed(2)} ppm`, lvl === 3 ? C.crit : C.warn);
}
function drawHold(ctx, L, hold, t, color) {
  if (!hold) return;
  const col = color || C.warn;
  const [x, y] = px(L, hold.fix[0], hold.fix[1]);
  ctx.save();
  if (hold.pts && hold.pts.length > 2) {
    const P = hold.pts.map((q) => px(L, q[0], q[1]));
    ctx.strokeStyle = rgba(col, 0.85); ctx.lineWidth = 1.8; ctx.setLineDash([7, 4]); ctx.lineJoin = 'round';
    ctx.beginPath(); ctx.moveTo(P[0][0], P[0][1]); for (let i = 1; i < P.length; i++) ctx.lineTo(P[i][0], P[i][1]); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    // direction arrows along the pattern
    ctx.fillStyle = rgba(col, 0.95);
    for (let i = 3; i < P.length; i += Math.max(4, Math.floor(P.length / 5))) {
      const a2 = P[i - 1], b2 = P[i], ang = Math.atan2(b2[1] - a2[1], b2[0] - a2[0]);
      ctx.save(); ctx.translate(b2[0], b2[1]); ctx.rotate(ang); ctx.beginPath(); ctx.moveTo(5, 0); ctx.lineTo(-3, 3.5); ctx.lineTo(-3, -3.5); ctx.fill(); ctx.restore();
    }
  } else { ctx.strokeStyle = rgba(col, 0.85); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.arc(x, y, 0.42 * L.cell, 0, 7); ctx.stroke(); }
  // holding fix: a triangle, as on charts
  ctx.setLineDash([]); ctx.fillStyle = rgba(col, 0.25); ctx.strokeStyle = col; ctx.lineWidth = 1.4;
  ctx.beginPath(); ctx.moveTo(x, y - 7); ctx.lineTo(x + 6, y + 5); ctx.lineTo(x - 6, y + 5); ctx.closePath(); ctx.fill(); ctx.stroke();
  const [tx, ty] = px(L, hold.target[0], hold.target[1]);
  ctx.strokeStyle = rgba(col, 0.45); ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tx, ty); ctx.stroke();
  ctx.restore();
  const KN = { racetrack: 'RACETRACK (R)', racetrack_left: 'RACETRACK (L)', figure8: 'FIGURE-8', orbit: 'ORBIT' };
  tag(ctx, x + 10, y - 10, `HOLD ${KN[hold.kind] || ''}${hold.hdg !== undefined && hold.hdg !== null ? ' · ' + String(hold.hdg).padStart(3, '0') + '°' : ''} · ${hold.why}`, col);
}

// ------------------------------------------------------------------ altitude profile
function altInterval(route) {
  if (S.altInt !== 'auto') return parseInt(S.altInt, 10);
  if (!S.scn || S.scn.mode === 'small') return 100;
  const top = Math.max(0, ...((route || []).map((c) => c[2]))) * (S.scn.layer_ft || 5000);
  return top >= 10000 ? 10000 : 1000;
}
function drawProfile(ctx, L, route, seg, pos, hazards, prof, color, t) {
  const P0 = L.prof, layers = S.scn ? S.scn.layers : 9, lf = S.scn ? S.scn.layer_ft : 50;
  const gut = 46, P = { x: P0.x + gut, y: P0.y, w: P0.w - gut, h: P0.h };
  const maxFt = (layers - 1) * lf;
  ctx.fillStyle = 'rgba(12,19,27,0.85)'; ctx.fillRect(P0.x, P0.y, P0.w, P0.h);
  const Yft = (ft) => P.y + P.h - (ft / maxFt) * (P.h - 6) - 3;
  let iv = altInterval(route); const maxLines = Math.max(4, Math.floor(P.h / 9));
  while (maxFt / iv > maxLines) iv *= 10;                    // too dense for this height: step up a decade
  ctx.font = `9.5px ${FM}`; ctx.lineWidth = 1;
  for (let ft = 0; ft <= maxFt + 1e-6; ft += iv) {
    const y = Yft(ft); ctx.strokeStyle = ft % (iv * 5) === 0 ? 'rgba(120,150,180,0.22)' : 'rgba(120,150,180,0.10)';
    ctx.beginPath(); ctx.moveTo(P.x, y); ctx.lineTo(P.x + P.w, y); ctx.stroke();
    ctx.fillStyle = C.dim; const lab = ft >= 1000 && S.scn && S.scn.mode === 'aircraft' ? (ft >= 18000 ? 'FL' + String(Math.round(ft / 100)).padStart(3, '0') : (ft / 1000) + 'k ft') : ft + ' ft';
    ctx.fillText(lab, P0.x + 3, Math.min(P.y + P.h - 1, Math.max(P.y + 8, y + 3)));
  }
  ctx.fillStyle = C.dim; ctx.fillText(`ALTITUDE · grid ${iv.toLocaleString()} ft`, P.x + P.w - 150, P.y + 10);
  if (!route || route.length < 2) return;
  const n = route.length - 1, X = (i) => P.x + (i / n) * P.w, Yz = (z) => Yft(z * lf);
  for (let i = 0; i <= n; i++) {
    const c = route[i];
    for (const h of hazards || []) {
      const full = h.type === 'ash' && h.z1 >= layers - 1;
      const z1 = full ? layers - 1 : h.z1, z0 = full ? 0 : h.z0;
      if (zoneHas(h, c[0], c[1], t || 0)) {
        ctx.fillStyle = rgba(h.type === 'ash' ? hazColor(h) : (C[h.type] || C.building), hazardBlocks(h, prof) ? 0.4 : 0.15);
        ctx.fillRect(X(Math.max(0, i - 0.5)), Yz(z1 + 0.5), P.w / n, Yz(z0 - 0.5) - Yz(z1 + 0.5));
      }
    }
  }
  ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.lineJoin = 'round'; ctx.beginPath();
  route.forEach((c, i) => { if (i) ctx.lineTo(X(i), Yz(c[2])); else ctx.moveTo(X(i), Yz(c[2])); }); ctx.stroke();
  if (pos && seg !== undefined) { const x = X(Math.min(n, seg)), y = Yz(pos[2]); ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 7); ctx.fill();
    tag(ctx, Math.min(x + 6, P.x + P.w - 70), Math.max(P.y + 22, y - 6), altFt(pos[2]), color); }
}

// ------------------------------------------------------------------ panes
function clipMap(ctx, L) { ctx.save(); ctx.beginPath(); ctx.rect(L.ox, L.oy, L.cols * L.cell, L.rows * L.cell); ctx.clip(); }
function background(ctx, L, key) {
  if (S.bg === 'osm' && S.scn) { ctx.fillStyle = C.bg; ctx.fillRect(0, 0, L.w, L.h); drawOsm(ctx, L); }
  else ctx.drawImage(terrain(L, key), 0, 0, L.w, L.h);
}
function drawGcs(cv) {
  const ctx = sizeCanvas(cv), L = layout(cv), st = S.st, g = (st && st.gcs) || {};
  background(ctx, L, 'g');
  if (!S.scn) { emptyMsg(ctx, L, 'Generate a scenario to begin'); return; }
  clipMap(ctx, L);
  drawStatic(ctx, L, S.scn.static);
  const rep = new Set(g.reported || []);
  const tt = tNow();
  // Ground station views: LIVE while the link is up; with no link it switches
  // completely to a PREDICTED view: only what it can project from the last data.
  const predicted = !!g.tm && !(g.link === 'CONNECTED' || g.link === 'DEGRADED') && st.phase === 'MISSION';
  cv.closest('.jn-pane').classList.toggle('predicted', predicted);
  if (predicted) {
    drawWeather(ctx, L, (g.picture || []).filter((h) => !rep.has(h.id)), st.profile, 'predicted', tt);
    const age = g.age || 0;
    // drone-detected ash: projected from its last report along its measured drift, uncertainty growing with time
    for (const h of (g.picture || []).filter((h) => rep.has(h.id))) {
      const grown = Object.assign({}, h, { r: (h.r || 1.5) + 0.12 * age, estimated: true });
      drawWeather(ctx, L, [grown], st.profile, 'predicted', tt);
    }
    ctx.save(); ctx.fillStyle = 'rgba(8,12,18,0.28)'; ctx.fillRect(L.ox, L.oy, L.cols * L.cell, L.rows * L.cell); ctx.restore();
  } else {
    drawWeather(ctx, L, (g.picture || []).filter((h) => !rep.has(h.id)), st.profile, 'known', tt);
    drawWeather(ctx, L, (g.picture || []).filter((h) => rep.has(h.id)), st.profile, 'reported', tt);
  }
  drawTargets(ctx, L);
  const fade = Math.max(0, 1 - (performance.now() - S.gcsOldT) / 4000);
  if (S.gcsOld && fade > 0) drawPath(ctx, L, S.gcsOld, C.muted, 1.5, [5, 5], fade);
  if (!g.mission_live && g.plan) drawPath(ctx, L, g.plan, C.ice, 2, [6, 4], 0.9);
  if (g.route) drawPath(ctx, L, g.route, C.ice, 2, null, 0.6);
  const track = (g.track || []).map((r) => [r[1], r[2], r[3]]);
  if (track.length > 1) drawPath(ctx, L, track, C.ice, 1.2, [1, 4], 0.9, true);
  if (g.tm && g.tm.hold) drawHold(ctx, L, g.hold_path || { fix: g.tm.hold, target: S.scn.targets[S.scn.targets.length - 1], why: 'landing site to clear' }, tt, predicted ? C.dim : C.warn);
  else if (!g.mission_live && g.hold) drawHold(ctx, L, { fix: g.hold.fix, target: S.scn.targets[S.scn.targets.length - 1], why: g.hold.why }, tt);
  const outs = g.outages || [];
  const lastO = outs.length ? outs[outs.length - 1] : null;
  if (lastO && lastO.ghost && lastO.ghost.length > 1) drawPath(ctx, L, lastO.ghost.map((r) => [r[1], r[2], r[3]]), C.amber, 1.5, [4, 4], 0.7);
  if (lastO && lastO.recovered && lastO.blind && lastO.blind.length > 1) {
    drawPath(ctx, L, lastO.blind.map((r) => [r[1], r[2], r[3]]), C.mag, 2.6, null, 0.95);
    const e = lastO.blind[Math.floor(lastO.blind.length / 2)]; const [x, y] = px(L, e[1], e[2]);
    tag(ctx, x + 8, y - 10, 'BLACK BOX: TRUE BLIND TRACK', C.mag);
    if (lastO.est_end && lastO.true_end) {
      const [ax, ay] = px(L, lastO.est_end[0], lastO.est_end[1]), [bx, by] = px(L, lastO.true_end[0], lastO.true_end[1]);
      ctx.strokeStyle = C.warn; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
      tag(ctx, (ax + bx) / 2 + 6, (ay + by) / 2 + 4, `estimate off by ${lastO.err} cells`, C.warn);
    }
  }
  const tm = g.tm;
  if (tm) {
    const live = g.link === 'CONNECTED' || g.link === 'DEGRADED';
    const p = smooth('gcs', tm.p, live ? 0.25 : 1);
    const hd = heading(g.route, tm.seg, 0);
    if (g.ghost && g.link === 'LOST') {
      drawDrone(ctx, L, p, hd, C.dim, true, `last heard ${g.age ? g.age.toFixed(1) : '?'} s ago`);
      const gp = smooth('ghost', g.ghost.slice(0, 3), 0.3); const [gx, gy] = px(L, gp[0], gp[1]);
      const rr = g.ghost[3] * L.cell;
      ctx.fillStyle = rgba('#f2a93b', 0.08); ctx.beginPath(); ctx.arc(gx, gy, rr, 0, 7); ctx.fill();
      ctx.strokeStyle = rgba('#f2a93b', 0.7); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.arc(gx, gy, rr, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      if (g.outage && g.outage.ghost && g.outage.ghost.length > 1) drawPath(ctx, L, g.outage.ghost.map((r) => [r[1], r[2], r[3]]), C.amber, 1.5, [4, 4], 0.8);
      drawDrone(ctx, L, gp, hd, C.amber, true, 'ESTIMATE (dead reckoning)');
      tag(ctx, gx - rr * 0.7, gy + rr + 14, `± ${g.ghost[3].toFixed(1)} cells`, C.amber);
    } else {
      drawDrone(ctx, L, p, hd, C.ice, false, altFt(p[2]));
    }
  }
  ctx.restore();
  drawProfile(ctx, L, g.route || g.plan, tm ? tm.seg : 0, tm ? tm.p : null, (S.scn.static || []).concat(g.picture || []), st.profile, C.ice, tt);
  drawHover(ctx, L, tt);
  if (predicted) banner(ctx, L, `PREDICTED VIEW · no packets for ${(g.age || 0).toFixed(0)} s · everything shown is projected`, C.warn);
  else if (g.link === 'LOST') banner(ctx, L, 'NO PACKETS FROM DRONE', C.crit);
}
function drawDronePane(cv) {
  const ctx = sizeCanvas(cv), L = layout(cv), st = S.st, d = (st && st.drone) || null, air = (st && st.air) || {};
  background(ctx, L, 'd');
  if (!S.scn) { emptyMsg(ctx, L, 'Waiting for a scenario'); return; }
  clipMap(ctx, L);
  drawStatic(ctx, L, S.scn.static);
  const known = new Set((d && d.known) || []), sensed = new Set((d && d.sensed) || []);
  const t = d ? d.t + (st.paused ? 0 : Math.min(0.3, (performance.now() - S.recvAt) / 1000)) : tNow();
  const truth = (S.scn.hazards || []).filter((h) => (h.t_on || 0) <= t && t < (h.t_off || 1e9));
  const est = ((d && d.sensed_hz) || []).filter((h) => h.estimated);
  const estIds = new Set(est.map((h) => h.id));
  drawWeather(ctx, L, truth.filter((h) => known.has(h.id) && !sensed.has(h.id)), st.profile, 'known', t);
  drawWeather(ctx, L, truth.filter((h) => sensed.has(h.id) && !estIds.has(h.id)), st.profile, 'sensed', t);
  drawWeather(ctx, L, truth.filter((h) => !known.has(h.id)), st.profile, 'unseen', t);
  drawWeather(ctx, L, est, st.profile, 'sensed', t);
  drawTargets(ctx, L);
  if (!d) { ctx.restore(); emptyMsg(ctx, L, 'Drone process starting…'); return; }
  const fade = Math.max(0, 1 - (performance.now() - S.oldRouteT) / 3500);
  if (S.oldRoute && fade > 0) drawPath(ctx, L, S.oldRoute, C.crit, 2, [6, 5], fade);
  if (d.flown && d.flown.length > 1) drawPath(ctx, L, d.flown, C.amber, 1.5, null, 0.35);
  const rem = (d.route || []).slice(d.seg || 0);
  if (d.escaping && d.escape_end > (d.seg || 0)) {
    const escSeg = (d.route || []).slice(d.seg || 0, d.escape_end + 1);
    drawPath(ctx, L, (d.route || []).slice(d.escape_end), C.amber, 2.2, null, 0.95);
    drawPath(ctx, L, escSeg, C.crit, 3, [6, 4], 1);
    const e = escSeg[escSeg.length - 1]; const [ex, ey] = px(L, e[0], e[1]); tag(ctx, ex + 8, ey + 4, 'ESCAPE → CLEAN AIR', C.crit);
  } else drawPath(ctx, L, rem, C.amber, 2.2, null, 0.95);
  const p = smooth('drone', d.pos, d.state === 'HOLDING' ? 0.6 : 0.35);
  const hd = heading(d.route, d.seg, 0);
  drawHold(ctx, L, d.hold, t);
  drawSamples(ctx, L, d, t);
  drawSampling(ctx, L, p, d);
  const b = air.budget;
  if (b && S.scn.gcs && st.procs && st.procs[1].alive) {
    const [ax, ay] = px(L, S.scn.gcs[0], S.scn.gcs[1]), [bx, by] = px(L, p[0], p[1]);
    const blocked = b.blocked_by && b.blocked_by.length;
    const snr = Math.min(b.up_snr, b.dn_snr);
    ctx.strokeStyle = blocked ? C.crit : (snr < 10 ? C.warn : rgba('#8ab8ff', 0.45)); ctx.lineWidth = blocked ? 1.8 : 1;
    ctx.setLineDash(blocked ? [3, 4] : [8, 6]); ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
    if (blocked) tag(ctx, (ax + bx) / 2 + 6, (ay + by) / 2 - 6, `RADIO SHADOW · ${b.blocked_by.join(', ')} · -${Math.round(b.obstr_db)} dB`, C.crit);
  } else if (st.procs && !st.procs[1].alive) {
    const [bx, by] = px(L, p[0], p[1]); tag(ctx, bx + 10, by + 18, 'ANTENNA PULLED: NO RADIO PATH', C.crit);
  }
  drawDrone(ctx, L, p, hd, C.amber, false, altFt(p[2]));
  const hz = (S.scn.static || []).concat(truth.filter((h) => known.has(h.id)));
  ctx.restore();
  drawProfile(ctx, L, d.route, d.seg, d.pos, hz, st.profile, C.amber, t);
  drawHover(ctx, L, t);
  if (d.state === 'FORCED') banner(ctx, L, 'EMERGENCY LANDING', C.crit);
  else if (d.escaping) banner(ctx, L, `IN VOLCANIC ASH: ESCAPING · ${S.scn.mode === 'aircraft' ? 'ENGINE' : 'MOTORS'} ${Math.round(d.health)} %`, C.crit);
  else if (d.state === 'HOLDING') banner(ctx, L, 'HOLDING PATTERN: LANDING SITE BLOCKED', C.warn);
  else if (d.link === 'LOST') banner(ctx, L, d.failsafe ? `FLYING BLIND · FAILSAFE ${d.fs_reason || ''}` : 'FLYING BLIND: UPLINK SILENT', C.warn);
  else if (d.state === 'RTL' || (d.state === 'REPLAN' && d.resume_state === 'RTL')) banner(ctx, L, 'RETURNING TO LAUNCH', C.warn);
}
function tag(ctx, x, y, text, color) {
  ctx.font = `600 10.5px ${FM}`; const w = ctx.measureText(text).width + 8;
  const cw = ctx.canvas.clientWidth || 9999;
  if (x + w > cw - 4) x = Math.max(6, cw - 4 - w);
  ctx.fillStyle = 'rgba(11,18,26,0.88)'; ctx.fillRect(x - 4, y - 11, w, 15); ctx.fillStyle = color; ctx.fillText(text, x, y);
}
function banner(ctx, L, text, color) {
  ctx.font = `600 12.5px ${FM}`; const w = ctx.measureText(text).width + 22, x = L.ox + (L.cols * L.cell - w) / 2, y = L.oy + 8;
  ctx.fillStyle = rgba(color, 0.18); ctx.fillRect(x, y, w, 22); ctx.strokeStyle = color; ctx.strokeRect(x + 0.5, y + 0.5, w - 1, 21);
  ctx.fillStyle = color; ctx.fillText(text, x + 11, y + 15.5);
}
function emptyMsg(ctx, L, text) { ctx.fillStyle = C.muted; ctx.font = `15px ${FB}`; ctx.textAlign = 'center'; ctx.fillText(text, L.w / 2, L.h / 2 - 20); ctx.textAlign = 'left'; }

// ------------------------------------------------------------------ seismograph
function drawSeis(cv) {
  const ctx = sizeCanvas(cv), w = cv.clientWidth, h = cv.clientHeight, st = S.st;
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  const left = 78, right = w - 8, win = 30, now = tNow(), t0 = now - win;
  const X = (t) => left + ((t - t0) / win) * (right - left);
  const lanes = { up: { y: 38, label: 'UPLINK', sub: 'GCS → drone' }, dn: { y: 94, label: 'DOWNLINK', sub: 'drone → GCS' } };
  for (let s = Math.ceil(t0); s <= now; s++) {
    const x = X(s); ctx.strokeStyle = s % 5 === 0 ? 'rgba(120,150,180,0.16)' : 'rgba(120,150,180,0.06)'; ctx.beginPath(); ctx.moveTo(x, 8); ctx.lineTo(x, h - 18); ctx.stroke();
    if (s % 5 === 0 && s >= 0) { ctx.fillStyle = C.dim; ctx.font = `9.5px ${FM}`; ctx.fillText('T+' + s, x + 2, h - 6); }
  }
  const H = S.hist;
  for (let i = 1; i < H.length; i++) {
    const [ta, gl, dl, relay] = H[i - 1], tb = H[i][0]; if (tb < t0) continue;
    const xa = Math.max(left, X(ta)), xb = X(tb);
    const col = (s) => s === 'CONNECTED' ? rgba('#4cc98d', 0.55) : s === 'DEGRADED' ? rgba('#f0b429', 0.7) : s === 'LOST' ? rgba('#f25560', 0.8) : 'rgba(93,114,134,0.35)';
    ctx.fillStyle = col(dl); ctx.fillRect(xa, lanes.up.y + 20, xb - xa + 0.5, 3);
    ctx.fillStyle = col(gl); ctx.fillRect(xa, lanes.dn.y + 20, xb - xa + 0.5, 3);
    if (!relay) { ctx.fillStyle = 'rgba(122,42,51,0.18)'; ctx.fillRect(xa, 8, xb - xa + 0.5, h - 26); }
  }
  for (const k of ['up', 'dn']) {
    const ln = lanes[k];
    ctx.fillStyle = C.ink; ctx.font = `600 11px ${FM}`; ctx.fillText(ln.label, 6, ln.y + 2);
    ctx.fillStyle = C.dim; ctx.font = `9.5px ${FM}`; ctx.fillText(ln.sub, 6, ln.y + 14);
    ctx.strokeStyle = 'rgba(120,150,180,0.18)'; ctx.beginPath(); ctx.moveTo(left, ln.y); ctx.lineTo(right, ln.y); ctx.stroke();
  }
  for (const k of ['up', 'dn']) {
    const ln = lanes[k]; ctx.strokeStyle = rgba('#4fd1e0', 0.35); ctx.lineWidth = 1; ctx.beginPath(); let started = false;
    for (const e of S.ev) {
      if (e[2] !== k || e[1] < t0 || e[7] === null || e[7] === undefined) continue;
      const y = ln.y + 16 - Math.max(0, Math.min(32, e[7] * 0.8));
      if (started) ctx.lineTo(X(e[1]), y); else { ctx.moveTo(X(e[1]), y); started = true; }
    }
    ctx.stroke();
  }
  const REL = { MISSION: 1, WX: 1, ROUTE: 1, RESULT: 1, HZR: 1, CMD: 1 };
  for (const e of S.ev) {
    const t = e[1]; if (t < t0) continue;
    const ln = lanes[e[2]]; if (!ln) continue;
    const x = X(t), type = e[3], fate = e[4], big = REL[type] ? 1 : 0;
    let col = big ? C.ice : C.cyan; const up = big ? 17 : 10;
    if (fate === 'rf_lost') col = C.crit; else if (fate === 'dropped') col = C.drop; else if (fate === 'corrupt') col = C.warn; else if (fate === 'blackout') col = '#a33a46';
    ctx.strokeStyle = col; ctx.lineWidth = big ? 2.2 : 1.2; ctx.beginPath();
    if (fate === 'ok' || fate === 'dup') { ctx.moveTo(x, ln.y); ctx.lineTo(x, ln.y - up); }
    else if (fate === 'corrupt') { ctx.moveTo(x, ln.y); ctx.lineTo(x, ln.y - up); ctx.stroke(); ctx.fillStyle = col; ctx.fillRect(x - 2.5, ln.y - up - 3, 5, 5); continue; }
    else { ctx.moveTo(x - 2.5, ln.y + 3); ctx.lineTo(x + 2.5, ln.y + 9); ctx.moveTo(x + 2.5, ln.y + 3); ctx.lineTo(x - 2.5, ln.y + 9); }
    ctx.stroke();
    if ((type === 'MISSION' || type === 'CMD') && fate === 'ok' && x > left + 4) { ctx.fillStyle = C.ice; ctx.font = `9px ${FM}`; ctx.fillText(type, x + 2, ln.y - 19); }
  }
  const gaps = (st && st.gcs && st.gcs.gaps) || [];
  for (const gp of gaps) {
    if (gp.t < t0) continue; const x = X(gp.t);
    ctx.fillStyle = C.crit; ctx.font = `600 10px ${FM}`; ctx.fillText(`seq ${gp.from}→${gp.to}: ${gp.missing} missing`, Math.min(x + 3, right - 170), lanes.dn.y + 34);
  }
  if (st && st.procs && !st.procs[1].alive) { ctx.fillStyle = '#ff9aa1'; ctx.font = `600 11.5px ${FM}`; ctx.fillText('ANTENNA PULLED: AIR RELAY PROCESS STOPPED, NOTHING CAN CROSS', left + 10, h - 24); }
}

// ------------------------------------------------------------------ text panels (5 Hz)
function esc(s) { return String(s === undefined || s === null ? '' : s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }
function cell(v, cls) { return `<td class="${cls || ''}" title="${esc(String(v).replace(/<[^>]+>/g, ''))}">${v}</td>`; }
const NA = '<span class="na">—</span>';
function linkCls(l) { return l === 'CONNECTED' ? 'ok' : l === 'DEGRADED' ? 'warn' : l === 'LOST' ? 'crit' : ''; }
function fmtCell(c) { return c ? `(${c[0]}, ${c[1]}) · ${altFt(c[2])}` : NA; }
function updateViews(st) {
  const g = st.gcs || {}, d = st.drone || {}, tm = g.tm, m = d.metrics || {}, prof = st.profile || {};
  const ms = m.replan_ms || [], lastMs = ms.length ? ms[ms.length - 1] : null;
  const e0 = prof.energy || d.energy0 || 1, epct = d.energy !== undefined ? Math.max(0, d.energy / e0) : 0, rpct = d.reserve !== undefined ? d.reserve / e0 : 0;
  const up = (g.uploads || [])[(g.uploads || []).length - 1];
  const upTxt = up ? `${up.rid} ${up.status}${up.status === 'REJECTED' ? ': ' + esc(up.reason) : ''}${up.rtt ? ' · ' + up.rtt + ' s' : ''}` : NA;
  const upCls = !up ? '' : up.status === 'ACCEPTED' ? 'ok' : (up.status === 'REJECTED' || up.status === 'TIMEOUT') ? 'crit' : 'warn';
  const ashKnownG = (g.picture || []).filter((h) => h.type === 'ash').length;
  const ashKnownD = (S.scn ? S.scn.hazards : []).filter((h) => h.type === 'ash' && (d.known || []).includes(h.id)).length;
  const hit = (d.sensor_hits || []).slice(-1)[0];
  const sens = d.sensors || {};
  const algos = Object.entries(m.algo || {}).map(([k, v]) => `${k} ×${v}`).join(', ');
  const rows = [
    ['State', tm ? esc(tm.st) + (tm.fs ? ' · ' + esc(tm.fs) : '') : NA, d.state ? esc(d.state) + (d.failsafe ? ' · ' + esc(d.failsafe) : '') : NA, '', d.failsafe ? 'warn' : ''],
    ['Position', tm ? fmtCell(tm.c) : NA, d.cell ? fmtCell(d.cell) : NA],
    ['Link', g.link ? `${g.link.replace('_', ' ')}${g.age != null ? ' · heard ' + g.age.toFixed(1) + ' s ago' : ''}` : NA,
      d.link ? `${d.link.replace('_', ' ')}${d.link_age != null ? ' · heard ' + d.link_age.toFixed(1) + ' s ago' : ''}` : NA, linkCls(g.link), linkCls(d.link)],
    ['Energy', tm ? `${tm.e} / ${e0}` : NA,
      d.energy !== undefined ? `${d.energy.toFixed(1)} / ${e0} · reserve ${(d.reserve || 0).toFixed(1)}<div class="ebar"><i style="width:${(epct * 100).toFixed(1)}%"></i><u style="left:${(rpct * 100).toFixed(1)}%"></u></div>` : NA,
      '', epct < rpct * 1.3 && d.energy !== undefined ? 'crit' : ''],
    ['Route', g.route ? `rev ${g.route_rev} · ${g.route.length} cells` : (g.plan ? `plan ready · ${g.plan.length} cells` : NA),
      d.route ? `rev ${d.rev} · ${Math.max(0, d.route.length - 1 - (d.seg || 0))} cells to go` : NA],
    ['Planner', esc((st.cfg && PLANNER_NAMES[st.cfg.planner]) || '—'), d.algo ? `last: ${esc(d.algo)}` : 'no replan yet'],
    ['Replans', tm ? String(tm.rp) : NA, `${m.replans || 0}${m.bendy ? ' · ' + m.bendy + ' by BendyRuler' : ''}${lastMs !== null ? ' · last ' + lastMs.toFixed(1) + ' ms' : ''}`],
    ['OPC (particles)', tm && tm.opc !== undefined ? `${tm.opc} mg/m³` : NA, sens.opc !== undefined ? `${sens.opc.toFixed(2)} mg/m³${sens.opc_sat ? ' · SATURATED' : ''}${sens.in_ash ? ' · in ' + esc(sens.in_ash) : ''}` : NA,
      tm && tm.opc >= 0.2 ? 'warn' : '', sens.opc >= 2 ? 'crit' : sens.opc >= (sens.thr || 0.2) ? 'warn' : ''],
    ['SO₂ (gas)', tm && tm.so2 !== undefined ? `${tm.so2} ppm` : NA, sens.so2 !== undefined ? `${sens.so2.toFixed(3)} ppm · lags (t90 ${sens.so2_t90_s} s)` : NA],
    [S.scn && S.scn.mode === 'aircraft' ? 'Engine health' : 'Motor health', tm && tm.hp !== undefined ? `${tm.hp} %` : NA,
      d.health !== undefined ? `${d.health.toFixed(1)} %${(d.flags || []).length ? ' · ' + d.flags.join(', ').toLowerCase().replace('_', ' ') : ''}` : NA,
      tm && tm.hp < 60 ? 'crit' : '', d.health < 60 ? 'crit' : d.health < 90 ? 'warn' : 'ok'],
    ['Ash known', `${ashKnownG} plume${ashKnownG === 1 ? '' : 's'}${(g.reported || []).length ? ' · ' + g.reported.length + ' reported by drone' : ''}`,
      `${ashKnownD} plume${ashKnownD === 1 ? '' : 's'} · ${(d.sensed || []).length} found by own sensor`],
    ['Upload', upTxt, NA, upCls],
    ['Failsafe policy', NA, prof.failsafe ? esc(prof.failsafe.replace(/_/g, ' ').toLowerCase()) + (d.failsafe ? ' · ACTIVE' : '') + (prof.sar ? ' · SAR' : '') : NA, '', d.failsafe ? 'warn' : ''],
    ['Black box', `${(g.bb_have == null ? -1 : g.bb_have) + 1} records${(g.outages || []).length ? ' · ' + g.outages.length + ' outage(s)' : ''}`, d.bb_n !== undefined ? `${d.bb_n} records` : NA],
    ['Safety check', NA, `${m.blocked_entries || 0} blocked-cell entries · ${m.exposures || 0} entries into unknown hazards`, '', m.blocked_entries ? 'crit' : m.exposures ? 'warn' : 'ok'],
  ];
  if (d.hold || (tm && tm.hold)) rows.splice(1, 0, ['Holding', tm && tm.hold ? `at (${tm.hold[0]}, ${tm.hold[1]})` : NA, d.hold ? `at (${d.hold.fix[0]}, ${d.hold.fix[1]}) · waiting for ${esc(d.hold.why)}` : NA, 'warn', 'warn']);
  // sensor strip
  const opc = sens.opc || 0, thr = sens.thr || 0.2, hp = d.health === undefined ? 100 : d.health;
  const lvl = opc >= 4 ? 'crit' : opc >= 2 ? 'crit' : opc >= thr ? 'warn' : 'ok';
  const band = opc >= 4 ? 'HIGH' : opc >= 2 ? 'MEDIUM' : opc >= thr ? 'LOW' : 'clean';
  const bar = (v, col) => `<div class="bar"><i style="width:${Math.max(0, Math.min(100, v)).toFixed(0)}%;background:${col}"></i></div>`;
  $('.sens-strip').innerHTML =
    `<div><span class="sk">OPC particles · every ${sens.opc_period_s || 1} s</span><span class="sv ${lvl}">${opc.toFixed(opc >= 10 ? 0 : 2)}<small> mg/m³ ${sens.opc_sat ? 'SAT' : band}</small></span>${bar(opc / 8 * 100, opc >= 2 ? C.crit : opc >= thr ? C.warn : C.ok)}</div>` +
    `<div><span class="sk">SO₂ gas · t90 ≤ ${sens.so2_t90_s || 60} s</span><span class="sv ${(sens.so2 || 0) >= 0.1 ? 'warn' : 'ok'}">${(sens.so2 || 0).toFixed(2)}<small> ppm</small></span>${bar((sens.so2 || 0) / 3 * 100, C.warn)}</div>` +
    `<div><span class="sk">${S.scn && S.scn.mode === 'aircraft' ? 'Engine' : 'Motor'} health</span><span class="sv ${hp < 60 ? 'crit' : hp < 90 ? 'warn' : 'ok'}">${hp.toFixed(0)}<small> %</small></span>${bar(hp, hp < 60 ? C.crit : hp < 90 ? C.warn : C.ok)}</div>` +
    `<div><span class="sk">Ash dose · OPC fouling</span><span class="sv">${Math.round(d.dose || 0)}<small> mg·s/m³ · ${Math.round(d.fouling || 0)} %</small></span>${bar(d.fouling || 0, C.ash2)}</div>`;
  $('.tt-body').innerHTML = rows.map((r) => `<tr><th>${r[0]}</th>${cell(r[1], r[3])}${cell(r[2], r[4])}</tr>`).join('');
  // weather
  const wx = g.wx || {};
  $('.wx-src').textContent = wx.source ? wx.source + (wx.status === 'STALE' ? ` · STALE ${wx.age_s} s` : '') : '';
  const w = (k, v, cls) => `<div><span class="rk">${k}</span><span class="rv ${cls || ''}">${v}</span></div>`;
  $('.wx-grid').innerHTML = wx.windSpeed === undefined ? '<div><span class="rv">No weather yet</span></div>' :
    w('Wind', `${String(Math.round(wx.windDir)).padStart(3, '0')}° ${wx.windSpeed} kt`) + w('Gusts', `${wx.windGust} kt`, prof.gust_limit_kt && wx.windGust > prof.gust_limit_kt && S.scn && S.scn.mode === 'small' ? 'warn' : '') +
    w('Visibility', `${wx.visibility_km} km`, wx.visibility_km < 5 ? 'warn' : '') + w('Temp / dew point', `${wx.outTemp} / ${wx.dewpoint} °C`) +
    w('Humidity', `${wx.outHumidity} %`) + w('QNH', `${wx.barometer} hPa`) + w('Cloud base', `${(wx.cloud_ft || 0).toLocaleString()} ft`) +
    w('Rain', `${wx.rainRate} mm/h`) + w('Hazards in feed', `${(wx.counts || {}).ash || 0} ash · ${((wx.counts || {}).gust || 0) + ((wx.counts || {}).turb || 0)} ${S.scn && S.scn.mode === 'aircraft' ? 'turb' : 'gust'} · ${(wx.counts || {}).rain || 0} rain`);
  $('.wx-metar').innerHTML = wx.metar ? `<b>METAR (simulated)</b>${esc(wx.metar)}` : '';
  const ash = wx.ash || [];
  $('.wx-ash').innerHTML = ash.length ? ash.map((a) => `<li><span class="band ${a.band}">${a.band}</span><span class="vaa">${esc(a.id)} · ${esc(a.base)}/${esc(a.top)} · ${esc(a.mov)} · ${a.conc_mg} mg/m³ (${esc(a.range)})</span></li>`).join('')
    : '<li class="none">No ash advisories in the feed.</li>';
}
const PLANNER_NAMES = { hybrid: 'Hybrid: A* + BendyRuler + Dijkstra', astar: 'A* only', dijkstra: 'Dijkstra + BendyRuler' };
function updateTelemetry(st) {
  const g = st.gcs || {}, d = st.drone || {};
  (st.procs || []).forEach((p, i) => {
    const el = root.querySelector(`[data-proc="${i}"]`); if (!el) return;
    el.classList.toggle('alive', !!p.alive); el.classList.toggle('dead', !p.alive && i > 0);
    el.querySelector('.pinfo').textContent = `pid ${p.pid || '–'} · ${p.ports}${!p.alive && i > 0 ? ' · STOPPED' : ''}`;
  });
  const wu = $('[data-wire="up"]'), wd = $('[data-wire="dn"]');
  wu.className = 'jn-wire ' + (d.link || 'NO_LINK'); wd.className = 'jn-wire ' + (g.link || 'NO_LINK');
  wu.querySelector('.wstate').textContent = (d.link || '—').replace('_', ' ');
  wd.querySelector('.wstate').textContent = (g.link || '—').replace('_', ' ');
  const b = (st.air && st.air.budget) || {}, imp = st.impair || {};
  const kv = (k, v, cls) => `<div><span class="kk">${k}</span><span class="kv-v ${cls || ''}">${v}</span></div>`;
  $('.link-kv').innerHTML =
    kv('Drone hears ground station', d.link ? `${d.link.replace('_', ' ')}${d.link_age != null ? ' · ' + d.link_age.toFixed(1) + ' s' : ''}` : '—', linkCls(d.link)) +
    kv('Ground station hears drone', g.link ? `${g.link.replace('_', ' ')}${g.age != null ? ' · ' + g.age.toFixed(1) + ' s' : ''}` : '—', linkCls(g.link)) +
    kv('Uplink SNR', b.up_snr !== undefined ? `${b.up_snr > 40 ? '> 40' : Math.round(b.up_snr)} dB` : '—') +
    kv('Downlink SNR', b.dn_snr !== undefined ? `${b.dn_snr > 40 ? '> 40' : Math.round(b.dn_snr)} dB` : '—') +
    kv('Line of sight', b.blocked_by && b.blocked_by.length ? `blocked by ${b.blocked_by.join(', ')}` : (b.up_snr !== undefined ? 'clear' : '—'), b.blocked_by && b.blocked_by.length ? 'crit' : '') +
    kv('Antenna (relay process)', st.procs && st.procs[1].alive ? 'connected' : 'PULLED', st.procs && st.procs[1].alive ? 'ok' : 'crit') +
    kv('Link cut switch', imp.blackout ? 'CUT' : 'off', imp.blackout ? 'crit' : '') +
    kv('Latency setting', `${Math.round(imp.latency_ms || 0)} ms ± ${Math.round(imp.jitter_ms || 0)}`);
  const ge = g.ep || {}, de = d.ep || {};
  const row = (k, a, bb) => `<span>${k}</span><span class="r">${a === undefined ? 0 : a}</span><span class="r">${bb === undefined ? 0 : bb}</span>`;
  $('.pkt-stats').innerHTML = `<span class="h">Endpoint</span><span class="h r">GCS</span><span class="h r">Drone</span>` +
    row('sent', ge.tx, de.tx) + row('received', ge.rx, de.rx) + row('CRC rejects', ge.crc_fail, de.crc_fail) + row('retransmits', ge.retx, de.retx) +
    row('duplicates ignored', ge.dup, de.dup) + row('sequence gaps (pkts)', ge.gap_lost, de.gap_lost) + row('stale dropped', ge.stale, de.stale) +
    row('timeouts', ge.timeouts, de.timeouts);
  const cu = (st.air && st.air.counts) || { up: {}, dn: {} };
  const sum = (o) => Object.values(o || {}).reduce((a, v) => a + v, 0);
  const pct = (o) => { const n = sum(o); return n ? Math.round(100 * (o.ok || 0) / n) + ' %' : '—'; };
  $('.ud-stats').innerHTML = `<span class="h">Air relay</span><span class="h r">Uplink</span><span class="h r">Downlink</span>` +
    row('datagrams', sum(cu.up), sum(cu.dn)) + row('delivered', cu.up.ok, cu.dn.ok) + row('delivery rate', pct(cu.up), pct(cu.dn)) +
    row('lost (radio)', cu.up.rf_lost, cu.dn.rf_lost) + row('dropped (slider)', cu.up.dropped, cu.dn.dropped) +
    row('corrupted', cu.up.corrupt, cu.dn.corrupt) + row('link cut', cu.up.blackout, cu.dn.blackout);
  $('[data-cmd="feed"]').textContent = g.feed_paused ? 'Resume weather feed' : 'Pause weather feed';
  $$('input[data-imp]').forEach((inp) => {
    const k = inp.dataset.imp; if (S.drag[k] && performance.now() - S.drag[k] < 1500) return;
    const v = imp[k]; if (v !== undefined && parseFloat(inp.value) !== v) inp.value = v; showImp(k, v || 0);
  });
  const ev = (g.events || []).map((e) => Object.assign({ who: 'GCS' }, e)).concat((d.events || []).map((e) => Object.assign({ who: 'DRN' }, e)));
  ev.sort((a, c) => c.t - a.t);
  $('.log').innerHTML = ev.slice(0, 60).map((e) => `<li class="${e.lvl}"><span class="tt">T+${e.t.toFixed(1)}</span><span class="who ${e.who}">${e.who}</span><span class="tx">${esc(e.text)}</span></li>`).join('');
}
function updateText() {
  const st = S.st; if (!st) return;
  const g = st.gcs || {}, d = st.drone || {}, tour = st.tour || {};
  $$('.jn-clock').forEach((c) => { c.querySelector('.phase').textContent = st.paused ? st.phase + ' · STOPPED' : st.phase;
    c.querySelector('.t').textContent = 'T+' + tNow().toFixed(1); c.classList.toggle('paused', !!st.paused); });
  $$('[data-cmd="pause"]').forEach((b) => { b.classList.toggle('paused', !!st.paused); b.querySelector('.lbl').textContent = st.paused ? 'Start clock' : 'Stop clock';
    b.disabled = st.phase === 'SETUP'; });
  const cap = $('.jn-caption');
  cap.classList.toggle('err', !!st.err);
  if (tour.active) { $('.cap-main').textContent = tour.caption || tour.title || ''; $('.cap-sub').textContent = (tour.title ? tour.title + ' — ' : '') + (tour.sub || ''); }
  else { $('.cap-main').textContent = st.err ? st.err : (st.msg || ''); $('.cap-sub').textContent = st.scn_rev && S.scn ? storyLine() : ''; }
  $('[data-cmd="tour"]').textContent = tour.active ? '■ Stop tour' : '▶ Play hands-free tour';
  const lb = $('.link-gcs'); lb.className = 'badge link-gcs ' + (g.link || ''); lb.textContent = g.link ? g.link.replace('_', ' ') : 'NO LINK';
  const sb = $('.state-drone'); sb.className = 'badge state-drone ' + (d.state || ''); sb.textContent = d.state ? d.state + (d.failsafe ? ' · ' + d.failsafe : '') : 'IDLE';
  const relayAlive = st.procs && st.procs[1].alive, flying = ['ENROUTE', 'RTL', 'RTLINK', 'REPLAN', 'HOLD', 'HOLDING'].includes(d.state);
  $$('[data-cmd="launch"]').forEach((L) => { L.disabled = st.phase === 'SETUP'; L.textContent = st.phase === 'MISSION' ? 'Replay mission' : 'Launch mission'; });
  $$('[data-cmd="rtl"]').forEach((rtl) => { rtl.disabled = st.phase !== 'MISSION' || !(flying || d.state === 'COMPLETE');
    rtl.classList.toggle('active', d.state === 'RTL' || d.resume_state === 'RTL' && d.state === 'REPLAN'); });
  $('[data-cmd="resume"]').disabled = st.phase !== 'MISSION' || !['HOLD', 'HOLDING', 'RTL', 'RTLINK'].includes(d.state);
  ['popup', 'popup_hidden'].forEach((c) => { $(`[data-cmd="${c}"]`).disabled = st.phase !== 'MISSION' || !['ENROUTE', 'RTL', 'RTLINK'].includes(d.state); });
  $('[data-cmd="unsafe"]').disabled = st.phase === 'SETUP';
  const cut = $('[data-cmd="cut"]'); cut.classList.toggle('on', !!(st.impair && st.impair.blackout)); cut.textContent = st.impair && st.impair.blackout ? 'Restore link' : 'Cut link';
  $$('[data-cmd="antenna"]').forEach((ant) => { ant.classList.toggle('on', !relayAlive); ant.textContent = relayAlive ? 'Pull antenna' : 'Plug antenna back in'; });
  if (S.tab === 'views') updateViews(st);
  else if (S.tab === 'telemetry') updateTelemetry(st);
  else if (S.tab === 'reports' && performance.now() - S.lastReport > 2500) refreshReport();
}
function storyLine() {
  const s = S.scn && S.scn.story; if (!s) return '';
  return `${s.title || ''} · seed ${s.seed_used || ''} · checked in simulation: ${(s.verified || []).join(' · ')}`;
}

// ------------------------------------------------------------------ reports
async function refreshReport() {
  S.lastReport = performance.now();
  try { S.report = await server.get_report([]); } catch (e) { return; }
  renderReport();
}
async function reportAction(a) {
  if (a === 'refresh') { refreshReport(); return; }
  if (a === 'compare') {
    $('.rep-compare').innerHTML = '<p class="rep-empty">Running every planner on this map…</p>';
    try { S.compare = await server.get_compare([]); } catch (e) { toast('Comparison failed: ' + e); }
    renderCompare(); return;
  }
  if (a === 'download') {
    if (!S.report) await refreshReport();
    if (!S.report) { toast('No run yet.'); return; }
    const blob = new Blob([JSON.stringify({ report: S.report, compare: S.compare }, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = `janus_report_${Date.now()}.json`; document.body.appendChild(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
  }
}
function renderReport() {
  const r = S.report, body = $('.rep-body');
  if (!r) { body.innerHTML = '<p class="rep-empty">No run yet. Generate a scenario and launch it.</p>'; return; }
  const p = r.profile || {}, n = (v, dflt) => (v === null || v === undefined ? (dflt === undefined ? '—' : dflt) : v);
  const outcome = r.state || 'NOT FLOWN', ocls = ['COMPLETE', 'LANDED'].includes(outcome) ? 'ok' : outcome === 'FORCED' ? 'crit' : 'warn';
  const card = (k, v, cls) => `<div><span class="rk">${k}</span><span class="rv ${cls || ''}">${v}</span></div>`;
  const algos = Object.entries(r.algo || {}).map(([k, v]) => `${esc(k)} ×${v}`).join('<br>') || 'none';
  const hero = card('Outcome', esc(outcome), ocls) + card('Mission time', `${r.t}<small> s</small>`) + card('Distance flown', `${r.distance_km}<small> km</small>`) +
    card('Energy left', `${r.energy_left !== undefined && r.energy_left !== null ? r.energy_left.toFixed(1) : '—'}<small> / ${p.energy || '—'}</small>`) +
    card('Course changes', `${r.replans}<small>${r.bendy ? ' · ' + r.bendy + ' BendyRuler' : ''}</small>`) +
    card('Ash measured on board', String(r.sensed)) + card('Time in ash', `${r.ash_s}<small> s</small>`, r.ash_s > 1 ? 'warn' : 'ok') +
    card(p.mode === 'aircraft' ? 'Engine health' : 'Motor health', `${r.health === null || r.health === undefined ? '—' : Math.round(r.health)}<small> %</small>`, r.health < 60 ? 'crit' : r.health < 90 ? 'warn' : 'ok') +
    card('Escapes · holds', `${r.escapes} · ${r.holds}<small>${r.hold_s ? ' (' + r.hold_s + ' s)' : ''}</small>`, r.escapes ? 'warn' : '') +
    card('Unknown-hazard entries', String(r.exposures), r.exposures ? 'warn' : 'ok') +
    card('Failsafes', String(r.failsafes), r.failsafes ? 'warn' : '');
  const ashRows = (r.ash || []).map((a) => `<tr><td class="m">${esc(a.id)}</td><td><span class="band ${a.band}" style="font:600 10.5px var(--fm);padding:1px 5px;border-radius:3px;background:${a.band === 'LOW' ? 'var(--ash-lo)' : a.band === 'MEDIUM' ? 'var(--ash-md)' : 'var(--ash-hi)'};color:${a.band === 'HIGH' ? '#fff' : '#0b121a'}">${a.band}</span> ${a.conc_mg || ''} mg/m³</td><td>${a.hidden ? 'unreported' : 'advisory'}${a.t_on ? ' · formed T+' + a.t_on : ''}</td><td>${esc(a.known_by)}${a.detect_cells != null ? ' · ' + a.detect_cells + ' cells ahead' : ''}</td></tr>`).join('');
  const tl = (r.timeline || []).map((e) => `<li><span class="tt">T+${e.t.toFixed(1)}</span><span class="who ${e.who}">${e.who}</span><span>${esc(e.text)}</span></li>`).join('');
  const out = (r.outages || []).map((o) => `${((o.t1 || 0) - (o.t0 || 0)).toFixed(1)} s, estimate off by ${n(o.err)} cells, black box ${o.recovered ? 'recovered' : 'pending'}`).join('<br>') || 'none';
  const pk = r.packets || { up: {}, dn: {} }, sum = (o) => Object.values(o || {}).reduce((a, v) => a + v, 0);
  body.innerHTML = `<div class="rep-hero">${hero}</div>
    <div class="rep-cols">
      <div class="jn-card"><h5>Mission</h5><table class="rtable"><tbody>
        <tr><th>Drone</th><td>${esc(p.name)}</td></tr>
        <tr><th>Story</th><td>${esc((r.story || {}).title || '')} · seed ${n((r.story || {}).seed_used)}</td></tr>
        <tr><th>Planner</th><td>${esc(PLANNER_NAMES[(r.cfg || {}).planner] || '—')}</td></tr>
        <tr><th>Algorithms used</th><td class="m">${algos}</td></tr>
        <tr><th>Replan time</th><td class="m">median ${n(r.replan_ms_median)} ms · max ${n(r.replan_ms_max)} ms (this laptop)</td></tr>
        <tr><th>Cost field (Dijkstra)</th><td class="m">${r.field_ms != null ? r.field_ms + ' ms per map change' : 'not used'}</td></tr>
        <tr><th>SAR mode</th><td>${p.sar ? 'ON' : 'off'}</td></tr>
        <tr><th>Onboard sensors</th><td>OPC every ${p.opc_period_s || 1} s, alarm ${p.opc_thr_mg || 0.2} mg/m³ · SO₂ t90 ${p.so2_t90_s || 60} s · both in-situ (no look-ahead)</td></tr>
        <tr><th>Ash wear</th><td class="m">dose ${n(r.dose)} mg·s/m³ · OPC fouling ${n(r.fouling)} %${(r.flags || []).length ? ' · ' + r.flags.join(', ') : ''}</td></tr>
        <tr><th>Link outages</th><td>${out}</td></tr>
        <tr><th>Datagrams</th><td class="m">uplink ${sum(pk.up)} (${n(pk.up.ok, 0)} delivered) · downlink ${sum(pk.dn)} (${n(pk.dn.ok, 0)} delivered)</td></tr>
        <tr><th>Safety check</th><td>${r.blocked_entries} blocked-cell entries · ${r.exposures} exposures</td></tr>
      </tbody></table></div>
      <div class="jn-card"><h5>Volcanic ash on this map</h5><table class="rtable"><thead><tr><th>Plume</th><th>Concentration</th><th>Source</th><th>Drone learned it by</th></tr></thead><tbody>${ashRows || '<tr><td colspan="4" class="note">No ash on this map.</td></tr>'}</tbody></table>
        <h5 style="margin-top:12px">Timeline</h5><ol class="timeline">${tl || '<li class="note">Nothing yet.</li>'}</ol></div>
    </div>`;
}
function renderCompare() {
  const c = S.compare, el = $('.rep-compare');
  if (!c) { el.innerHTML = ''; return; }
  const ok = c.rows.filter((r) => r.ok && r.algo !== 'BendyRuler (one decision)' && !r.algo.includes('field'));
  const fastest = ok.length ? Math.min(...ok.map((r) => r.ms)) : null;
  const rows = c.rows.map((r) => `<tr><td>${esc(r.algo)}${r.note ? `<div class="note">${esc(r.note)}</div>` : ''}</td><td class="n ${r.ms === fastest ? 'best' : ''}">${r.ms}</td><td class="n">${r.nodes}</td><td class="n">${r.cost === null ? '—' : r.cost}</td><td class="n">${r.cells || '—'}</td></tr>`).join('');
  const sar = c.sar.map((s) => `<tr><td>${s.mode}</td><td class="n">${s.ok ? s.cost : 'no route'}</td><td class="n">${s.ok ? s.cells : '—'}</td><td class="n">${s.blocked_cells}</td></tr>`).join('');
  el.innerHTML = `<div class="rep-cols">
    <div class="jn-card"><h5>Planners on the same map <small>from (${c.from.join(', ')}) at T+${c.t}, this laptop</small></h5>
      <table class="rtable"><thead><tr><th>Algorithm</th><th>ms</th><th>nodes / probes</th><th>cost</th><th>cells</th></tr></thead><tbody>${rows}</tbody></table>
      <p class="note" style="color:var(--dim);font-size:11.5px;margin:6px 0 0">A* and Dijkstra find the same optimal cost; A* expands fewer nodes. BendyRuler decides in a fraction of a millisecond but only looks a few cells ahead.</p></div>
    <div class="jn-card"><h5>SAR off vs on <small>same map, A*</small></h5>
      <table class="rtable"><thead><tr><th>Mode</th><th>cost</th><th>cells</th><th>blocked cells</th></tr></thead><tbody>${sar}</tbody></table>
      <ul class="sar-rules">${c.sar_rules.map((x) => `<li>${esc(x)}</li>`).join('')}</ul></div>
  </div>`;
}

// ------------------------------------------------------------------ main loop
const cvG = root.querySelector('[data-map="gcs"]'), cvD = root.querySelector('[data-map="drone"]'), cvS = root.querySelector('canvas.seis');
function frame() {
  try {
    if (S.st) {
      if (S.tab === 'views' && cvG.clientWidth > 20) { fitCanvas(cvG); fitCanvas(cvD); drawGcs(cvG); drawDronePane(cvD); }
      if (S.tab === 'telemetry' && S.chips.updown && cvS.clientWidth > 20) drawSeis(cvS);
      if (performance.now() - S.lastText > 200) { updateText(); S.lastText = performance.now(); }
    }
  } catch (e) { console.error('janus render', e); }
  requestAnimationFrame(frame);
}
try { const c = JSON.parse(store.get('chips') || 'null'); if (c) Object.assign(S.chips, c); } catch (e) { /* ignore */ }
applyChips();
setBg(store.get('bg') === 'osm' ? 'osm' : 'grid');
S.altInt = ['auto', '100', '1000', '10000'].includes(store.get('altInt')) ? store.get('altInt') : 'auto';
S.altSize = ['s', 'm', 'l'].includes(store.get('altSize')) ? store.get('altSize') : 'm';
$$('[data-alt-int]').forEach((b) => b.classList.toggle('on', b.dataset.altInt === S.altInt));
$$('[data-alt-size]').forEach((b) => b.classList.toggle('on', b.dataset.altSize === S.altSize));
setTool('none');
setTab(['views', 'telemetry', 'reports'].includes(store.get('tab')) ? store.get('tab') : 'views');
poll();
requestAnimationFrame(frame);
