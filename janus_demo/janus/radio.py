"""Simplified radio link budget used by the air relay.

received = tx + gains - free_space_path_loss - obstruction_loss + fading
SNR      = received - noise_floor
P(loss)  = logistic(SNR) around SNR50

Free-space path loss: 20 log10(d_km) + 20 log10(f_MHz) + 32.44 dB.
Obstruction: each physical obstacle (building or terrain ridge) crossed by the
straight line between antennas adds OBSTRUCTION_DB. This is a teaching model:
no multipath, no Fresnel zones, no earth curvature.
"""
import math

OBSTRUCTION_DB = 18.0          # default per obstacle; an obstacle may carry its own rf_db
MAX_OBSTRUCTION_DB = 45.0
SNR50_DB = 6.0
SNR_SLOPE_DB = 1.3
FADE_SIGMA_DB = 1.5
GCS_ANTENNA_LAYER = 0.35       # antenna mast height in layers


def fspl_db(d_m, f_mhz):
    d_km = max(d_m, 1.0) / 1000.0
    return 20.0 * math.log10(d_km) + 20.0 * math.log10(f_mhz) + 32.44


def loss_probability(snr_db):
    x = (snr_db - SNR50_DB) / SNR_SLOPE_DB
    if x > 40:
        return 0.0
    if x < -40:
        return 1.0
    return 1.0 / (1.0 + math.exp(x))


def obstructions(a, b, obstacles, physical_types):
    """Obstacles crossed by the segment a->b. a, b are (x, y, z) in cell units
    (cell centres at integer coordinates; z in layers). Returns list of ids."""
    ax, ay, az = a
    bx, by, bz = b
    dist = math.hypot(bx - ax, by - ay)
    n = max(2, int(dist * 4) + 2)
    hit = []
    phys = [o for o in obstacles if o["type"] in physical_types]
    if not phys:
        return hit
    for i in range(1, n):
        t = i / n
        x, y, z = ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t
        cx, cy = int(round(x)), int(round(y))
        for o in phys:
            if o["id"] in hit:
                continue
            if o["x0"] <= cx <= o["x1"] and o["y0"] <= cy <= o["y1"] and z <= o["z1"] + 0.5 and z >= o["z0"] - 0.5:
                hit.append(o["id"])
    return hit


def budget(mode_cfg, gcs_pos, drone_pos, obstacles, drone_tx_dbm, drone_gain_dbi, rng=None):
    """Return dict with uplink/downlink SNR and loss probability."""
    cell = mode_cfg["cell_m"]
    layer_m = mode_cfg["layer_ft"] * 0.3048
    dx = (drone_pos[0] - gcs_pos[0]) * cell
    dy = (drone_pos[1] - gcs_pos[1]) * cell
    dz = (drone_pos[2] - gcs_pos[2]) * layer_m
    d = math.sqrt(dx * dx + dy * dy + dz * dz)
    path = fspl_db(d, mode_cfg["freq_mhz"])
    hits = obstructions(gcs_pos, drone_pos, obstacles, mode_cfg["physical"])
    rf = {o["id"]: float(o.get("rf_db", OBSTRUCTION_DB)) for o in obstacles}
    obstr = min(MAX_OBSTRUCTION_DB, sum(rf.get(h, OBSTRUCTION_DB) for h in hits))
    fade_up = rng.gauss(0.0, FADE_SIGMA_DB) if rng else 0.0
    fade_dn = rng.gauss(0.0, FADE_SIGMA_DB) if rng else 0.0
    gains = mode_cfg["gcs_gain_dbi"] + drone_gain_dbi
    up_rx = mode_cfg["gcs_tx_dbm"] + gains - path - obstr + fade_up
    dn_rx = drone_tx_dbm + gains - path - obstr + fade_dn
    up_snr = up_rx - mode_cfg["noise_dbm"]
    dn_snr = dn_rx - mode_cfg["noise_dbm"]
    return {
        "dist_m": d, "fspl_db": path, "obstr_db": obstr, "blocked_by": hits,
        "up_snr": up_snr, "dn_snr": dn_snr,
        "up_loss": loss_probability(up_snr), "dn_loss": loss_probability(dn_snr),
    }
