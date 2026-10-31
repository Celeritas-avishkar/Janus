// Janus live console. Runs inside a Gradio gr.HTML component: `element` is the
// component root and `server` exposes the Python functions get_state / command.
const root = element.querySelector('.jn');
const $ = (s) => root.querySelector(s);
const $$ = (s) => Array.from(root.querySelectorAll(s));
const C = {
  bg: '#0b121a', grid: 'rgba(120,150,180,0.07)', grid5: 'rgba(120,150,180,0.14)', ink: '#dfe8f1', muted: '#8ba0b4', dim: '#5d7286',
  amber: '#f2a93b', cyan: '#4fd1e0', ice: '#8ab8ff', ok: '#4cc98d', warn: '#f0b429', crit: '#f25560', mag: '#ff6fd4', drop: '#ff8a3d',
  building: '#2a3a4d', buildingTop: '#3b526b', geofence: '#f25560', ridge: '#6d5a42', tfr: '#e06c9f',
  gust: '#2ec4b6', rain: '#4c8df6', ash: '#c9853a', turb: '#9b7bf2',
};
const FM = '"IBM Plex Mono", ui-monospace, Consolas, monospace';
const FD = '"Barlow Semi Condensed", "Arial Narrow", sans-serif';
const S = { scn: null, scnRev: -1, lastEvt: 0, st: null, recvAt: 0, ev: [], hist: [], disp: {}, oldRoute: null, oldRouteT: 0,
            droneRev: -1, gcsRev: -1, gcsOld: null, gcsOldT: 0, busy: false, drag: {}, terrain: {}, lastText: 0, session: -1, fails: 0 };

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
    const wait = Math.max(30, 110 - (performance.now() - t0));
    await new Promise((r) => setTimeout(r, wait));
  }
}

function onState(st) {
  if (st.session !== S.session) {
    S.ev = []; S.hist = []; S.disp = {}; S.oldRoute = null; S.droneRev = -1; S.gcsRev = -1; S.gcsOld = null;
    S.session = st.session;
  }
  if (st.scn) { S.scn = st.scn; S.scnRev = st.scn_rev; S.terrain = {}; }
  const evs = (st.air && st.air.events) || [];
  for (const e of evs) S.ev.push(e);
  if (st.air && st.air.evt_id) S.lastEvt = st.air.evt_id;
  const tcut = st.t - 45;
  while (S.ev.length && S.ev[0][1] < tcut) S.ev.shift();
  if (S.ev.length > 6000) S.ev.splice(0, S.ev.length - 6000);
  const g = st.gcs || {}, d = st.drone || {};
  S.hist.push([st.t, g.link || 'NO_LINK', d.link || 'NO_LINK', st.air && st.procs ? (st.procs[1].alive ? 1 : 0) : 0]);
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

function tNow() { return S.st ? S.st.t + Math.min(0.3, (performance.now() - S.recvAt) / 1000) : 0; }

// ------------------------------------------------------------------ commands
async function cmd(name, arg) {
  try {
    const r = await server.command([name, arg === undefined ? null : arg]);
    if (r && r.ok === false && r.error) toast(r.error);
    return r;
  } catch (e) { toast('Command failed: ' + e); }
}
function toast(msg) {
  const el = $('.jn-toast'); el.textContent = msg; el.hidden = false;
  clearTimeout(S.toastT); S.toastT = setTimeout(() => { el.hidden = true; }, 3800);
}
root.addEventListener('click', (ev) => {
  const b = ev.target.closest('[data-cmd]'); if (!b || b.disabled) return;
  const c = b.dataset.cmd, st = S.st || {};
  if (c === 'tour') cmd(st.tour && st.tour.active ? 'tour_stop' : 'tour_start');
  else if (c === 'cut') cmd('cut', !(st.impair && st.impair.blackout));
  else if (c === 'antenna') cmd(st.procs && st.procs[1].alive ? 'kill_relay' : 'restart_relay');
  else if (c === 'feed') cmd('feed', !(st.gcs && st.gcs.feed_paused));
  else cmd(c);
});
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
  const prof = 52, pad = 8;
  const cell = Math.min((w - pad * 2) / cols, (h - prof - pad * 2 - 6) / rows);
  const ox = (w - cell * cols) / 2, oy = pad;
  return { w, h, cell, ox, oy, cols, rows, prof: { x: ox, y: oy + rows * cell + 8, w: cell * cols, h: prof - 10 } };
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

// ------------------------------------------------------------------ terrain backdrop (seeded, cached)
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
  const step = 4;
  for (let y = L.oy; y < L.oy + L.rows * L.cell; y += step) {
    for (let x = L.ox; x < L.ox + L.cols * L.cell; x += step) {
      const nx = (x - L.ox) / L.cell / 5, ny = (y - L.oy) / L.cell / 5;
      const v = noise(nx, ny) * 0.6 + noise(nx * 2.3, ny * 2.3) * 0.4;
      const band = Math.floor(v * 9) % 2;
      c.fillStyle = band ? 'rgba(90,120,150,0.035)' : 'rgba(0,0,0,0)';
      c.fillRect(x, y, step, step);
    }
  }
  for (let i = 0; i <= L.cols; i++) {
    c.strokeStyle = i % 5 === 0 ? C.grid5 : C.grid; c.lineWidth = 1; c.beginPath();
    c.moveTo(L.ox + i * L.cell + 0.5, L.oy); c.lineTo(L.ox + i * L.cell + 0.5, L.oy + L.rows * L.cell); c.stroke();
  }
  for (let j = 0; j <= L.rows; j++) {
    c.strokeStyle = j % 5 === 0 ? C.grid5 : C.grid; c.beginPath();
    c.moveTo(L.ox, L.oy + j * L.cell + 0.5); c.lineTo(L.ox + L.cols * L.cell, L.oy + j * L.cell + 0.5); c.stroke();
  }
  S.terrain[k] = off; return off;
}

// ------------------------------------------------------------------ drawing helpers
function hazardBlocks(h, prof) {
  if (!prof) return true;
  if (['building', 'geofence', 'ridge', 'tfr', 'ash'].includes(h.type)) return true;
  if (h.type === 'turb') return (h.sev || 2) >= prof.turb_block;
  if (h.type === 'gust') return (h.gust_kt || 0) > prof.gust_limit_kt;
  if (h.type === 'rain') return !prof.rain_rated;
  return false;
}
function hazLabel(h) {
  const z = (S.scn && h.type === 'ash' && h.z1 >= S.scn.layers - 1) ? 'SFC-TOP' : `${altFt(h.z0)}–${altFt(h.z1)}`;
  if (h.type === 'gust') return `${h.id} gust ${h.gust_kt} kt · ${z}`;
  if (h.type === 'rain') return `${h.id} rain ${h.rain_mmh || ''} mm/h`;
  if (h.type === 'ash') return `${h.id} ash D${h.sev} · ${z}`;
  if (h.type === 'turb') return `${h.id} turb ${['', 'LOW', 'MOD', 'HIGH'][h.sev]} · ${z}`;
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
function drawStatic(ctx, L, list) {
  for (const o of list) {
    const [x, y, w, h] = rectOf(L, o);
    if (o.type === 'building' || o.type === 'ridge') {
      const top = (o.z1 + 1) / (S.scn ? S.scn.layers : 9);
      const base = o.type === 'building' ? C.building : C.ridge;
      ctx.fillStyle = base; ctx.fillRect(x + 1, y + 1, w - 2, h - 2);
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
function drawWeather(ctx, L, list, prof, style) {
  for (const h of list) {
    const [x, y, w, hh] = rectOf(L, h);
    const col = C[h.type] || C.turb;
    const blocks = hazardBlocks(h, prof);
    if (style === 'unseen') {
      ctx.strokeStyle = rgba(col, 0.55); ctx.setLineDash([2, 4]); ctx.lineWidth = 1.2; ctx.strokeRect(x + 1.5, y + 1.5, w - 3, hh - 3); ctx.setLineDash([]);
      if (L.cell > 15) { ctx.fillStyle = rgba(col, 0.75); ctx.font = `10px ${FM}`; ctx.fillText('not yet known', x + 3, y + hh - 4); }
      continue;
    }
    const sev = h.type === 'gust' ? Math.min(1, (h.gust_kt || 15) / 35) : (h.sev || 2) / 3;
    const g = ctx.createRadialGradient(x + w / 2, y + hh / 2, 1, x + w / 2, y + hh / 2, Math.max(w, hh) * 0.75);
    g.addColorStop(0, rgba(col, 0.20 + 0.30 * sev)); g.addColorStop(1, rgba(col, 0.06));
    ctx.fillStyle = g; ctx.fillRect(x, y, w, hh);
    if (h.type === 'rain') hatch(ctx, x, y, w, hh, rgba(col, 0.25), 5);
    ctx.lineWidth = blocks ? 1.8 : 1; ctx.strokeStyle = blocks ? rgba(col, 0.95) : rgba(col, 0.5);
    if (!blocks) ctx.setLineDash([4, 3]);
    ctx.strokeRect(x + 1, y + 1, w - 2, hh - 2); ctx.setLineDash([]);
    if (L.cell > 15) {
      ctx.fillStyle = rgba('#ffffff', 0.85); ctx.font = `10px ${FM}`; fitText(ctx, hazLabel(h), x + 3, y + 11, w - 5);
      if (style === 'sensed') { ctx.fillStyle = C.amber; fitText(ctx, 'own sensor', x + 3, y + hh - 4, w - 5); }
      else if (!blocks && hh > 30) { ctx.fillStyle = rgba(col, 0.9); fitText(ctx, 'passable', x + 3, y + hh - 4, w - 5); }
    }
  }
}
function drawPath(ctx, L, pts, color, width, dash, alpha) {
  if (!pts || pts.length < 2) return;
  ctx.save(); ctx.globalAlpha = alpha === undefined ? 1 : alpha; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath(); pts.forEach((c, i) => { const [x, y] = px(L, c[0], c[1]); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); }); ctx.stroke(); ctx.restore();
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
    ctx.fillStyle = last ? C.ok : C.ink; ctx.font = `600 11px ${FD}`; ctx.textAlign = 'center';
    ctx.fillText(last ? 'GOAL' : 'WP' + (i + 1), x, y - Math.max(7, L.cell * 0.38)); ctx.textAlign = 'left';
  });
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

// ------------------------------------------------------------------ altitude profile
function drawProfile(ctx, L, route, seg, pos, hazards, prof, color) {
  const P = L.prof; const layers = S.scn ? S.scn.layers : 9;
  ctx.fillStyle = 'rgba(17,27,37,0.9)'; ctx.fillRect(P.x, P.y, P.w, P.h);
  ctx.strokeStyle = 'rgba(120,150,180,0.12)'; ctx.lineWidth = 1;
  for (let z = 0; z < layers; z += 2) { const y = P.y + P.h - (z / (layers - 1)) * (P.h - 4) - 2; ctx.beginPath(); ctx.moveTo(P.x, y); ctx.lineTo(P.x + P.w, y); ctx.stroke(); }
  ctx.fillStyle = C.dim; ctx.font = `9px ${FM}`; ctx.fillText(altFt(layers - 1), P.x + 2, P.y + 9); ctx.fillText('ALTITUDE PROFILE', P.x + P.w - 92, P.y + 9);
  if (!route || route.length < 2) return;
  const n = route.length - 1, X = (i) => P.x + (i / n) * P.w, Y = (z) => P.y + P.h - (z / (layers - 1)) * (P.h - 4) - 2;
  for (let i = 0; i <= n; i++) {
    const c = route[i];
    for (const h of hazards || []) {
      const z1 = (h.type === 'ash' && h.z1 >= layers - 1) ? layers - 1 : h.z1, z0 = (h.type === 'ash' && h.z1 >= layers - 1) ? 0 : h.z0;
      if (c[0] >= h.x0 && c[0] <= h.x1 && c[1] >= h.y0 && c[1] <= h.y1) {
        ctx.fillStyle = rgba(C[h.type] || C.building, hazardBlocks(h, prof) ? 0.35 : 0.15);
        ctx.fillRect(X(Math.max(0, i - 0.5)), Y(z1 + 0.5), P.w / n, Y(z0 - 0.5) - Y(z1 + 0.5));
      }
    }
  }
  ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.beginPath();
  route.forEach((c, i) => { if (i) ctx.lineTo(X(i), Y(c[2])); else ctx.moveTo(X(i), Y(c[2])); }); ctx.stroke();
  if (pos && seg !== undefined) { const x = X(Math.min(n, seg)), y = Y(pos[2]); ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 7); ctx.fill(); }
}

// ------------------------------------------------------------------ panes
function drawGcs(cv) {
  const ctx = sizeCanvas(cv), L = layout(cv), st = S.st, g = (st && st.gcs) || {};
  ctx.drawImage(terrain(L, 'g'), 0, 0, L.w, L.h);
  if (!S.scn) { emptyMsg(ctx, L, 'Generate a scenario to begin'); return; }
  drawStatic(ctx, L, S.scn.static);
  drawWeather(ctx, L, g.picture || [], st.profile, 'known');
  drawTargets(ctx, L);
  const fade = Math.max(0, 1 - (performance.now() - S.gcsOldT) / 4000);
  if (S.gcsOld && fade > 0) drawPath(ctx, L, S.gcsOld, C.muted, 1.5, [5, 5], fade);
  if (!g.mission_live && g.plan) drawPath(ctx, L, g.plan, C.ice, 2, [6, 4], 0.9);
  if (g.route) drawPath(ctx, L, g.route, C.ice, 2, null, 0.55);
  const track = (g.track || []).map((r) => [r[1], r[2], r[3]]);
  if (track.length > 1) drawPath(ctx, L, track, C.ice, 1.2, [1, 4], 0.9);
  // last finished outage: ghost guess vs black-box truth
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
  drawProfile(ctx, L, g.route || g.plan, tm ? tm.seg : 0, tm ? tm.p : null, (S.scn.static || []).concat(g.picture || []), st.profile, C.ice);
  if (g.link === 'LOST') banner(ctx, L, 'NO PACKETS FROM DRONE', C.crit);
}
function drawDrone_(cv) {
  const ctx = sizeCanvas(cv), L = layout(cv), st = S.st, d = (st && st.drone) || null, air = (st && st.air) || {};
  ctx.drawImage(terrain(L, 'd'), 0, 0, L.w, L.h);
  if (!S.scn) { emptyMsg(ctx, L, 'Waiting for a scenario'); return; }
  drawStatic(ctx, L, S.scn.static);
  const known = new Set((d && d.known) || []), sensed = new Set((d && d.sensed) || []);
  const t = d ? d.t : 0;
  const truth = (S.scn.hazards || []).filter((h) => (h.t_on || 0) <= t);
  drawWeather(ctx, L, truth.filter((h) => known.has(h.id) && !sensed.has(h.id)), st.profile, 'known');
  drawWeather(ctx, L, truth.filter((h) => sensed.has(h.id)), st.profile, 'sensed');
  drawWeather(ctx, L, truth.filter((h) => !known.has(h.id)), st.profile, 'unseen');
  drawTargets(ctx, L);
  if (!d) { emptyMsg(ctx, L, 'Drone process starting…'); return; }
  const fade = Math.max(0, 1 - (performance.now() - S.oldRouteT) / 3500);
  if (S.oldRoute && fade > 0) drawPath(ctx, L, S.oldRoute, C.crit, 2, [6, 5], fade);
  if (d.flown && d.flown.length > 1) drawPath(ctx, L, d.flown, C.amber, 1.5, null, 0.35);
  const rem = (d.route || []).slice(d.seg || 0);
  drawPath(ctx, L, rem, C.amber, 2.2, null, 0.95);
  const look = rem.slice(0, (d.lookahead || 0) + 1);
  if (look.length > 1 && ['ENROUTE', 'RTL', 'RTLINK'].includes(d.state)) drawPath(ctx, L, look, C.amber, 7, null, 0.18);
  const p = smooth('drone', d.pos, 0.35);
  // radio line of sight from the ground antenna (physics truth from the relay's link budget)
  const b = air.budget;
  if (b && S.scn.gcs && st.procs && st.procs[1].alive) {
    const [ax, ay] = px(L, S.scn.gcs[0], S.scn.gcs[1]), [bx, by] = px(L, p[0], p[1]);
    const blocked = b.blocked_by && b.blocked_by.length;
    const snr = Math.min(b.up_snr, b.dn_snr);
    ctx.strokeStyle = blocked ? C.crit : (snr < 10 ? C.warn : rgba('#4fd1e0', 0.55)); ctx.lineWidth = blocked ? 1.8 : 1.2;
    ctx.setLineDash(blocked ? [3, 4] : [8, 6]); ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
    const mx = (ax + bx) / 2, my = (ay + by) / 2;
    tag(ctx, mx + 6, my - 6, blocked ? `RADIO SHADOW · ${b.blocked_by.join(', ')} · -${Math.round(b.obstr_db)} dB` : `LOS · SNR ${snr > 40 ? '>40' : Math.round(snr)} dB`, blocked ? C.crit : C.cyan);
  } else if (st.procs && !st.procs[1].alive) {
    const [bx, by] = px(L, p[0], p[1]); tag(ctx, bx + 10, by + 18, 'NO RELAY PROCESS: NO RADIO PATH', C.crit);
  }
  drawDrone(ctx, L, p, heading(d.route, d.seg, 0), C.amber, false, altFt(p[2]));
  const hz = (S.scn.static || []).concat(truth.filter((h) => known.has(h.id)));
  drawProfile(ctx, L, d.route, d.seg, d.pos, hz, st.profile, C.amber);
  if (d.link === 'LOST') banner(ctx, L, d.failsafe ? `FLYING BLIND · FAILSAFE ${d.fs_reason || ''}` : 'FLYING BLIND: UPLINK SILENT', C.warn);
}
function tag(ctx, x, y, text, color) {
  ctx.font = `600 10.5px ${FM}`; const w = ctx.measureText(text).width + 8;
  ctx.fillStyle = 'rgba(11,18,26,0.85)'; ctx.fillRect(x - 4, y - 11, w, 15); ctx.fillStyle = color; ctx.fillText(text, x, y);
}
function banner(ctx, L, text, color) {
  ctx.font = `600 13px ${FD}`; const w = ctx.measureText(text).width + 22, x = L.ox + (L.cols * L.cell - w) / 2, y = L.oy + 8;
  ctx.fillStyle = rgba(color, 0.16); ctx.fillRect(x, y, w, 22); ctx.strokeStyle = color; ctx.strokeRect(x + 0.5, y + 0.5, w - 1, 21);
  ctx.fillStyle = color; ctx.fillText(text, x + 11, y + 15.5);
}
function emptyMsg(ctx, L, text) { ctx.fillStyle = C.muted; ctx.font = `15px ${FD}`; ctx.textAlign = 'center'; ctx.fillText(text, L.w / 2, L.h / 2 - 20); ctx.textAlign = 'left'; }

// ------------------------------------------------------------------ seismograph
function drawSeis(cv) {
  const ctx = sizeCanvas(cv), w = cv.clientWidth, h = cv.clientHeight, st = S.st;
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  const left = 78, right = w - 8, win = 30, now = tNow(), t0 = now - win;
  const X = (t) => left + ((t - t0) / win) * (right - left);
  const lanes = { up: { y: 34, label: 'UPLINK', sub: 'GCS → drone' }, dn: { y: 86, label: 'DOWNLINK', sub: 'drone → GCS' } };
  for (let s = Math.ceil(t0); s <= now; s++) {
    const x = X(s); ctx.strokeStyle = s % 5 === 0 ? 'rgba(120,150,180,0.16)' : 'rgba(120,150,180,0.06)'; ctx.beginPath(); ctx.moveTo(x, 8); ctx.lineTo(x, h - 18); ctx.stroke();
    if (s % 5 === 0 && s >= 0) { ctx.fillStyle = C.dim; ctx.font = `9.5px ${FM}`; ctx.fillText('T+' + s, x + 2, h - 6); }
  }
  // link-state band and relay-down shading
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
    ctx.fillStyle = C.ink; ctx.font = `600 11px ${FD}`; ctx.fillText(ln.label, 6, ln.y + 2);
    ctx.fillStyle = C.dim; ctx.font = `9.5px ${FM}`; ctx.fillText(ln.sub, 6, ln.y + 14);
    ctx.strokeStyle = 'rgba(120,150,180,0.18)'; ctx.beginPath(); ctx.moveTo(left, ln.y); ctx.lineTo(right, ln.y); ctx.stroke();
  }
  // SNR traces
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
    let col = big ? C.ice : C.cyan; let up = big ? 17 : 10;
    if (fate === 'rf_lost') { col = C.crit; } else if (fate === 'dropped') { col = C.drop; } else if (fate === 'corrupt') { col = C.warn; } else if (fate === 'blackout') { col = '#a33a46'; }
    ctx.strokeStyle = col; ctx.lineWidth = big ? 2.2 : 1.2; ctx.beginPath();
    if (fate === 'ok' || fate === 'dup') { ctx.moveTo(x, ln.y); ctx.lineTo(x, ln.y - up); }
    else if (fate === 'corrupt') { ctx.moveTo(x, ln.y); ctx.lineTo(x, ln.y - up); ctx.stroke(); ctx.fillStyle = col; ctx.fillRect(x - 2.5, ln.y - up - 3, 5, 5); continue; }
    else { ctx.moveTo(x - 2.5, ln.y + 3); ctx.lineTo(x + 2.5, ln.y + 9); ctx.moveTo(x + 2.5, ln.y + 3); ctx.lineTo(x - 2.5, ln.y + 9); }
    ctx.stroke();
    if (type === 'MISSION' && fate === 'ok' && x > left + 4) { ctx.fillStyle = C.ice; ctx.font = `9px ${FM}`; ctx.fillText('MISSION', x + 2, ln.y - 19); }
  }
  // sequence gaps as seen by the ground station
  const gaps = (st && st.gcs && st.gcs.gaps) || [];
  for (const gp of gaps) {
    if (gp.t < t0) continue; const x = X(gp.t);
    ctx.fillStyle = C.crit; ctx.font = `600 10px ${FM}`; ctx.fillText(`seq ${gp.from}→${gp.to}: ${gp.missing} missing`, Math.min(x + 3, right - 170), lanes.dn.y + 34);
  }
  if (st && st.procs && !st.procs[1].alive) { ctx.fillStyle = '#ff9aa1'; ctx.font = `600 12px ${FD}`; ctx.fillText('AIR RELAY PROCESS STOPPED: NOTHING CAN CROSS', left + 10, h - 24); }
}

// ------------------------------------------------------------------ text panels (5 Hz)
function rd(k, v, cls, wide) { return `<div class="${wide ? 'wide' : ''}" title="${esc(String(v).replace(/<[^>]+>/g, ''))}"><span class="rk">${k}</span><span class="rv ${cls || ''}">${v}</span></div>`; }
function esc(s) { return String(s === undefined || s === null ? '' : s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }
function updateText() {
  const st = S.st; if (!st) return;
  const g = st.gcs || {}, d = st.drone || {}, tour = st.tour || {};
  // process chain
  (st.procs || []).forEach((p, i) => {
    const el = root.querySelector(`[data-proc="${i}"]`); if (!el) return;
    el.classList.toggle('alive', !!p.alive); el.classList.toggle('dead', !p.alive && i > 0);
    el.querySelector('.pinfo').textContent = `pid ${p.pid || '–'} · ${p.ports}${!p.alive && i > 0 ? ' · STOPPED' : ''}`;
  });
  const wu = $('[data-wire="up"]'), wd = $('[data-wire="dn"]');
  wu.className = 'jn-wire ' + (d.link || 'NO_LINK'); wd.className = 'jn-wire ' + (g.link || 'NO_LINK');
  wu.querySelector('.wstate').textContent = 'drone hears GCS: ' + (d.link || '—').replace('_', ' ');
  wd.querySelector('.wstate').textContent = 'GCS hears drone: ' + (g.link || '—').replace('_', ' ');
  $('.jn-clock .phase').textContent = st.phase; $('.jn-clock .t').textContent = 'T+' + tNow().toFixed(1);
  // caption
  const cap = $('.jn-caption');
  cap.classList.toggle('err', !!st.err);
  if (tour.active) { $('.cap-main').textContent = tour.caption || tour.title || ''; $('.cap-sub').textContent = (tour.title ? tour.title + ' — ' : '') + (tour.sub || ''); }
  else { $('.cap-main').textContent = st.err ? st.err : (st.msg || ''); $('.cap-sub').textContent = st.scn_rev && S.scn ? storyLine() : ''; }
  $('[data-cmd="tour"]').textContent = tour.active ? '■ Stop tour' : '▶ Play hands-free tour';
  // badges
  const lb = $('.link-gcs'); lb.className = 'badge link-gcs ' + (g.link || ''); lb.textContent = g.link ? `${g.link.replace('_', ' ')}${g.age !== null && g.age !== undefined ? ' · ' + g.age.toFixed(1) + ' s' : ''}` : 'NO LINK';
  const sb = $('.state-drone'); sb.className = 'badge state-drone ' + (d.state || ''); sb.textContent = d.state ? d.state + (d.failsafe ? ' · ' + d.failsafe : '') : 'IDLE';
  // GCS readout
  const wx = g.wx || {}, up = (g.uploads || [])[ (g.uploads || []).length - 1 ];
  const upTxt = up ? `${up.rid} ${up.status}${up.status === 'REJECTED' ? ': ' + up.reason : ''}${up.rtt ? ' · ' + up.rtt + ' s' : ''}` : '—';
  const upCls = !up ? '' : up.status === 'ACCEPTED' ? 'ok' : up.status === 'REJECTED' || up.status === 'TIMEOUT' ? 'crit' : 'warn';
  const tm = g.tm;
  $('.gcs-read').innerHTML =
    rd('Weather · sim', wx.windSpeed !== undefined ? `${Math.round(wx.windDir)}° ${wx.windSpeed} kt G${wx.windGust} · ${wx.outTemp}°C · ${wx.barometer} hPa${wx.status === 'STALE' ? ' · STALE ' + wx.age_s + ' s' : ''}` : '—', wx.status === 'STALE' ? 'warn' : '') +
    rd('Hazard picture', `v${g.pic_v || 0} · ${(g.picture || []).length} volumes${(g.reported || []).length ? ' · ' + g.reported.length + ' from drone' : ''}`) +
    rd('Upload', esc(upTxt), upCls) +
    rd('Last telemetry', tm ? `#${tm.s} · ${tm.st} · ${g.age !== null ? g.age.toFixed(1) + ' s ago' : ''}` : '—', g.link === 'LOST' ? 'crit' : '') +
    rd('Black box', `${(g.bb_have || -1) + 1} records${(g.outages || []).length ? ' · ' + g.outages.length + ' outage(s)' : ''}`) +
    rd('Plan', g.plan_info ? (g.plan_info.ok ? `${g.plan_info.length} cells · cost ${g.plan_info.cost}` : 'no safe route: ' + g.plan_info.reason) : '—', g.plan_info && !g.plan_info.ok ? 'crit' : '') +
    (S.scn && S.scn.mode === 'aircraft' && wx.metar ? rd('Simulated METAR', esc(wx.metar), '', true) : '');
  // drone readout
  const m = d.metrics || {}, ms = (m.replan_ms || []), lastMs = ms.length ? ms[ms.length - 1] : null;
  const e0 = d.energy0 ? (st.profile ? st.profile.energy : d.energy0) : 1, epct = d.energy !== undefined ? Math.max(0, d.energy / e0) : 0;
  const rpct = d.reserve !== undefined ? d.reserve / e0 : 0;
  $('.drone-read').innerHTML =
    `<div><span class="rk">Energy</span><span class="rv ${epct < rpct * 1.3 ? 'crit' : ''}">${d.energy !== undefined ? d.energy.toFixed(1) + ' / ' + e0 : '—'}</span><div class="ebar"><i style="width:${(epct * 100).toFixed(1)}%"></i><u style="left:${(rpct * 100).toFixed(1)}%"></u></div></div>` +
    rd('Uplink heard', d.link ? `${d.link.replace('_', ' ')}${d.link_age !== null && d.link_age !== undefined ? ' · ' + d.link_age.toFixed(1) + ' s' : ''}` : '—', d.link === 'LOST' ? 'crit' : d.link === 'DEGRADED' ? 'warn' : 'ok') +
    rd('Replans', `${m.replans || 0}${lastMs !== null ? ' · last ' + lastMs.toFixed(1) + ' ms (laptop)' : ''}`) +
    rd('Knows', `${(d.known || []).length} hazards · ${(d.sensed || []).length} by own sensor`) +
    rd('Failsafe policy', st.profile ? st.profile.failsafe.replace(/_/g, ' ') + (d.failsafe ? ' · ACTIVE' : '') : '—', d.failsafe ? 'warn' : '') +
    rd('Safety invariant', `${m.blocked_entries || 0} blocked entries · ${m.exposures || 0} exposures`, (m.blocked_entries || m.exposures) ? 'crit' : 'ok');
  // controls
  const relayAlive = st.procs && st.procs[1].alive;
  const L = $('[data-cmd="launch"]'); L.disabled = st.phase === 'SETUP'; L.textContent = st.phase === 'MISSION' ? 'Replay mission' : 'Launch mission';
  const cut = $('[data-cmd="cut"]'); cut.classList.toggle('on', !!(st.impair && st.impair.blackout)); cut.textContent = st.impair && st.impair.blackout ? 'Restore link' : 'Cut link';
  const ant = $('[data-cmd="antenna"]'); ant.classList.toggle('on', !relayAlive); ant.textContent = relayAlive ? 'Pull antenna (kill relay)' : 'Restart relay process';
  $('[data-cmd="feed"]').textContent = g.feed_paused ? 'Resume weather feed' : 'Pause weather feed';
  ['rtl', 'resume', 'popup', 'unsafe'].forEach((c) => { $(`[data-cmd="${c}"]`).disabled = st.phase !== 'MISSION' && c !== 'unsafe'; });
  $(`[data-cmd="unsafe"]`).disabled = st.phase === 'SETUP';
  $$('input[data-imp]').forEach((inp) => {
    const k = inp.dataset.imp; if (S.drag[k] && performance.now() - S.drag[k] < 1500) return;
    const v = st.impair ? st.impair[k] : 0; if (v !== undefined && parseFloat(inp.value) !== v) inp.value = v; showImp(k, v || 0);
  });
  // log
  const ev = (g.events || []).map((e) => Object.assign({ who: 'GCS' }, e)).concat((d.events || []).map((e) => Object.assign({ who: 'DRN' }, e)));
  ev.sort((a, b) => b.t - a.t);
  $('.log').innerHTML = ev.slice(0, 40).map((e) => `<li class="${e.lvl}"><span class="tt">T+${e.t.toFixed(1)}</span><span class="who ${e.who}">${e.who}</span><span class="tx">${esc(e.text)}</span></li>`).join('');
  // stats
  const cu = (st.air && st.air.counts) || { up: {}, dn: {} }, ge = g.ep || {}, de = d.ep || {};
  const row = (k, a, b) => `<span>${k}</span><span class="r">${a === undefined ? 0 : a}</span><span class="r">${b === undefined ? 0 : b}</span>`;
  $('.stats').innerHTML = `<span class="h">Air relay</span><span class="h r">up</span><span class="h r">down</span>` +
    row('delivered', cu.up.ok, cu.dn.ok) + row('lost (radio)', cu.up.rf_lost, cu.dn.rf_lost) + row('dropped', cu.up.dropped, cu.dn.dropped) +
    row('corrupted', cu.up.corrupt, cu.dn.corrupt) + row('link cut', cu.up.blackout, cu.dn.blackout) + `<span class="sep"></span>` +
    `<span class="h">Endpoints</span><span class="h r">GCS</span><span class="h r">drone</span>` +
    row('CRC rejects', ge.crc_fail, de.crc_fail) + row('retransmits', ge.retx, de.retx) + row('duplicates', ge.dup, de.dup) +
    row('seq gaps (pkts)', ge.gap_lost, de.gap_lost) + row('stale dropped', ge.stale, de.stale);
}
function storyLine() {
  const s = S.scn && S.scn.story; if (!s) return '';
  const v = (s.verified || []).join(' · ');
  return `${s.title || ''} · seed ${s.seed_used || ''} · checked in simulation: ${v}`;
}

// ------------------------------------------------------------------ main loop
const cvG = root.querySelector('[data-map="gcs"]'), cvD = root.querySelector('[data-map="drone"]'), cvS = root.querySelector('canvas.seis');
function frame() {
  try {
    if (S.st && cvG.clientWidth > 20 && cvS.clientWidth > 20) {      // skip while the Live tab is hidden
      drawGcs(cvG); drawDrone_(cvD); drawSeis(cvS);
      if (performance.now() - S.lastText > 200) { updateText(); S.lastText = performance.now(); }
    }
  } catch (e) { console.error('janus render', e); }
  requestAnimationFrame(frame);
}
poll();
requestAnimationFrame(frame);
