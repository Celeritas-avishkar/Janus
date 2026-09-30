import pygame
import heapq
import math
import os
import random
import time

pygame.init()

# ============================================================
# WINDOW / SIMULATION CONSTANTS
# ============================================================

GRID_WIDTH, GRID_HEIGHT = 1000, 680
PANEL_WIDTH = 340
DASHBOARD_HEIGHT = 110
WIDTH = GRID_WIDTH + PANEL_WIDTH
HEIGHT = GRID_HEIGHT + DASHBOARD_HEIGHT

CELL_SIZE = 40
GRID_COLS = GRID_WIDTH // CELL_SIZE
GRID_ROWS = GRID_HEIGHT // CELL_SIZE

ALTITUDE_STEP_FT = 5000
GRID_LAYERS = 9
MAX_ALTITUDE_FT = (GRID_LAYERS - 1) * ALTITUDE_STEP_FT

# Real-time sensing / movement
SENSOR_RANGE_CELLS = 6
DRONE_SPEED_LABELS = ["SLOW", "CRUISE", "FAST", "RAPID", "MAX"]
DRONE_SPEED_VALUES = [1.2, 2.0, 3.2, 4.6, 6.5]  # cells / second
DRONE_SPEED_INDEX = 2

# Cost model for the demonstrator. These are illustrative, not flight-control limits.
VERTICAL_COST = 2
ENERGY_PER_COST_UNIT = 0.5
WIND_DIRECTION = (1, 0)
WIND_PENALTY = 3

TURBULENCE_LEVELS = ["LOW", "MODERATE", "HIGH"]
TURBULENCE_COST = {"LOW": 2, "MODERATE": 8, "HIGH": 28}
ASH_COST = {1: 8, 2: 16, 3: 26}

# ============================================================
# DISPLAY
# ============================================================

CANVAS_WIDTH, CANVAS_HEIGHT = WIDTH, HEIGHT
canvas = pygame.Surface((CANVAS_WIDTH, CANVAS_HEIGHT))
screen = canvas
display_surface = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Adaptive Drone Flight-Path Replanner")

is_fullscreen = False
_present_scale = 1.0
_present_offset = (0, 0)


def set_display_mode(fullscreen):
    global display_surface, is_fullscreen
    is_fullscreen = fullscreen
    if fullscreen:
        display_surface = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    else:
        display_surface = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)


def present():
    global _present_scale, _present_offset
    dw, dh = display_surface.get_size()
    scale = max(0.01, min(dw / CANVAS_WIDTH, dh / CANVAS_HEIGHT))
    sw, sh = max(1, int(CANVAS_WIDTH * scale)), max(1, int(CANVAS_HEIGHT * scale))
    ox, oy = (dw - sw) // 2, (dh - sh) // 2
    _present_scale, _present_offset = scale, (ox, oy)
    display_surface.fill((0, 0, 0))
    if (sw, sh) == (CANVAS_WIDTH, CANVAS_HEIGHT):
        display_surface.blit(canvas, (ox, oy))
    else:
        display_surface.blit(pygame.transform.smoothscale(canvas, (sw, sh)), (ox, oy))
    pygame.display.flip()


def to_canvas_pos(pos):
    x, y = pos
    scale = _present_scale if _present_scale else 1.0
    ox, oy = _present_offset
    return ((x - ox) / scale, (y - oy) / scale)


def mouse_canvas_pos():
    return to_canvas_pos(pygame.mouse.get_pos())


clock = pygame.time.Clock()

# UI-only typography. Logic/state/pathfinding is unchanged.
font_title = pygame.font.SysFont("Consolas", 22, bold=True)
font = pygame.font.SysFont("Consolas", 17, bold=True)
font_small = pygame.font.SysFont("Consolas", 14, bold=True)
font_tiny = pygame.font.SysFont("Consolas", 11, bold=True)
font_micro = pygame.font.SysFont("Consolas", 9, bold=True)

# ============================================================
# UNIFORM MILITARY / COMMAND-HUD PALETTE — UI ONLY
# ============================================================
BG_COLOR = (10, 15, 11)
GRID_COLOR = (29, 48, 31)
START_COLOR = (126, 184, 82)
GOAL_COLOR = (184, 70, 58)
DRONE_COLOR = (224, 176, 64)
DRONE_CLIMB_COLOR = (192, 210, 82)
PATH_COLOR = (76, 112, 73)
MAP_PATH_COLOR = (91, 154, 108)
HAZARD_PATH_COLOR = (184, 70, 58)
OLD_PATH_COLOR = (166, 132, 55)
OBSTACLE_FULL_COLOR = (69, 55, 38)
OBSTACLE_PARTIAL_COLOR = (101, 88, 47)

PANEL_BG_COLOR = (17, 25, 18)
PANEL_LINE_COLOR = (151, 169, 77)
PANEL_AXIS_COLOR = (67, 91, 62)
PANEL_MARKER_COLOR = (197, 215, 86)
WIND_ARROW_COLOR = (113, 157, 112)
WARNING_COLOR = (190, 73, 60)
SAFE_COLOR = (126, 184, 82)
REPLAN_FLASH_COLOR = (198, 184, 67)

DASHBOARD_BG_COLOR = (8, 13, 9)
DASHBOARD_LABEL_COLOR = (119, 143, 113)
DASHBOARD_VALUE_COLOR = (204, 218, 181)
TEXT_MAIN = (191, 207, 174)
LEGEND_BG_COLOR = (15, 23, 16)

BUTTON_COLOR = (30, 48, 32)
BUTTON_ACTIVE_COLOR = (73, 101, 47)
BUTTON_START_COLOR = (91, 137, 65)
BUTTON_OBSTACLE_COLOR = (83, 67, 42)
BUTTON_DANGER_COLOR = (84, 48, 43)

# Hazard overlays retain their meaning but fit the green command-HUD shell.
TURB_LOW = (80, 145, 105, 55)
TURB_MODERATE = (67, 132, 92, 90)
TURB_HIGH = (53, 111, 78, 125)
ASH_LOW = (150, 127, 58, 60)
ASH_MODERATE = (160, 112, 47, 95)
ASH_HIGH = (177, 91, 42, 125)

# Shared UI geometry.
UI_RADIUS = 4
UI_BORDER = (83, 108, 72)
UI_BORDER_SOFT = (47, 70, 50)
UI_PANEL_SHADOW = (5, 9, 6)
UI_SECTION = (119, 145, 73)
UI_MUTED = (111, 130, 104)
UI_WHITE = (220, 230, 205)

# ============================================================
# SHARED STATE / HELPERS
# ============================================================


def alt_ft(z_index):
    return int(z_index) * ALTITUDE_STEP_FT


def ease_in_out(t):
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(t * math.pi)


def lerp(a, b, t):
    return a + (b - a) * t


def draw_hud_corners(surface, rect, color=None, length=12, thickness=2):
    c = color if color else PANEL_AXIS_COLOR
    x, y, w, h = rect
    corners = [((x, y), (1, 1)), ((x + w, y), (-1, 1)),
               ((x, y + h), (1, -1)), ((x + w, y + h), (-1, -1))]
    for (cx, cy), (sx, sy) in corners:
        pygame.draw.line(surface, c, (cx, cy), (cx + sx * length, cy), thickness)
        pygame.draw.line(surface, c, (cx, cy), (cx, cy + sy * length), thickness)


def draw_dashed_line(surface, points, color, width=2, dash=10, gap=7):
    if len(points) < 2:
        return
    for a, b in zip(points[:-1], points[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        dist = math.hypot(dx, dy)
        if dist <= 0:
            continue
        ux, uy = dx / dist, dy / dist
        travelled = 0.0
        while travelled < dist:
            s = travelled
            e = min(travelled + dash, dist)
            if int(travelled / (dash + gap)) % 2 == 0:
                pygame.draw.line(surface, color,
                                 (int(ax + ux * s), int(ay + uy * s)),
                                 (int(ax + ux * e), int(ay + uy * e)), width)
            travelled += dash + gap


def announce(text, level="warning", duration=1.8):
    global event_banner_text, event_banner_until, event_banner_level
    event_banner_text = text
    event_banner_until = time.perf_counter() + duration
    event_banner_level = level


event_banner_text = ""
event_banner_until = 0.0
event_banner_level = "warning"


def banner_color(level):
    if level == "safe":
        return SAFE_COLOR
    if level == "info":
        return REPLAN_FLASH_COLOR
    return WARNING_COLOR


def draw_event_banner():
    if not event_banner_text:
        return
    now = time.perf_counter()
    if now > event_banner_until:
        return
    remain = max(0.0, event_banner_until - now)
    pulse = 0.6 + 0.4 * math.sin(now * 12.0)
    color = banner_color(event_banner_level)
    width = min(GRID_WIDTH - 60, max(430, font.render(event_banner_text, True, color).get_width() + 80))
    box = pygame.Rect((GRID_WIDTH - width) // 2, 48, width, 48)
    surf = pygame.Surface(box.size, pygame.SRCALPHA)
    alpha = int(200 * min(1.0, 0.75 + 0.25 * pulse))
    surf.fill((8, 10, 8, alpha))
    pygame.draw.rect(surf, (*color[:3], 235), surf.get_rect(), 2)
    draw_hud_corners(surf, surf.get_rect(), (*color[:3], 255), length=10, thickness=2)
    screen.blit(surf, box.topleft)
    text_surf = font.render(event_banner_text, True, color)
    screen.blit(text_surf, (box.centerx - text_surf.get_width() // 2,
                            box.centery - text_surf.get_height() // 2))


def turbulence_rgba(level):
    return {"LOW": TURB_LOW, "MODERATE": TURB_MODERATE, "HIGH": TURB_HIGH}.get(level, TURB_MODERATE)


def ash_rgba(severity):
    return {1: ASH_LOW, 2: ASH_MODERATE, 3: ASH_HIGH}.get(severity, ASH_MODERATE)


def hazard_cost(h):
    if h["type"] == "ash":
        return ASH_COST.get(h.get("severity", 2), 16)
    return TURBULENCE_COST.get(h.get("severity", "MODERATE"), 8)


def hazard_requires_replan(h):
    # Every turbulence zone and every ash zone is an avoidance volume.
    # This keeps the live route from ever intentionally crossing a
    # turbulence/ash cell while travelling to a waypoint.
    return h["type"] in ("ash", "turbulence")


def hazard_blocks_cell(cell, h):
    """Return True when a hazard occupies the exact 3-D cell.

    Ash is an exclusion volume, not merely a high-cost region. A plume whose
    ceiling reaches the drone ceiling is treated as occupying the complete
    vertical envelope from its configured floor through 40,000 ft. This prevents
    the planner from 'escaping' by climbing into the top boundary and then
    crossing the plume.
    """
    if not hazard_requires_replan(h):
        return False
    z_min = int(h.get("z_min", 0))
    z_max = int(h.get("z_max", GRID_LAYERS - 1))
    if h.get("type") == "ash" and z_max >= GRID_LAYERS - 1:
        z_min = 0
        z_max = GRID_LAYERS - 1
    bounds = hazard_bounds_grid(h) if "col_start" in h else map_hazard_bounds_grid(h)
    return cell_inside_bounds(cell, bounds, z_min, z_max)


def hazard_label(h):
    if h["type"] == "ash":
        return f"ASH D{h.get('severity', 2)}"
    return f"TURBULENCE {h.get('severity', 'MODERATE')}"


def make_obstacle(obstacle_id, col_start, width, row_start, height, z_min, z_max):
    return {"id": obstacle_id, "col_start": col_start, "width": width,
            "row_start": row_start, "height": height,
            "z_min": z_min, "z_max": z_max}


def make_hazard(hazard_id, htype, col_start, width, row_start, height,
                z_min=0, z_max=0, severity=None):
    # Hazards are static for deterministic replanning demonstrations.
    if severity is None:
        severity = random.choice([1, 2, 3]) if htype == "ash" else random.choice(TURBULENCE_LEVELS)
    # An ash ceiling at the drone's 40,000 ft ceiling represents a full plume
    # through the available flight envelope. Lock its floor to the ground so
    # the replanner cannot simply climb to the ceiling and cross it.
    if htype == "ash" and int(z_max) >= GRID_LAYERS - 1:
        z_min, z_max = 0, GRID_LAYERS - 1
    return {
        "id": hazard_id, "type": htype, "col_start": col_start, "width": width,
        "row_start": row_start, "height": height,
        "z_min": z_min, "z_max": z_max, "severity": severity,
    }


def obstacle_bounds_grid(o):
    return (o["col_start"], o["col_start"] + o["width"] - 1,
            o["row_start"], o["row_start"] + o["height"] - 1)


def hazard_bounds_grid(h):
    return (h["col_start"], h["col_start"] + h["width"] - 1,
            h["row_start"], h["row_start"] + h["height"] - 1)


def cell_inside_bounds(cell, bounds, z_min, z_max):
    x, y, z = cell
    xmin, xmax, ymin, ymax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax and z_min <= z <= z_max


# ============================================================
# GRID MODE
# ============================================================

start_cell = (1, 1, 0)
goal_cell = (18, 3, 0)
waypoints = []
waypoint_altitude = 2

obstacles = [
    make_obstacle(1, 7, 2, 1, 3, 0, GRID_LAYERS - 1),
    make_obstacle(2, 13, 2, 5, 4, 0, GRID_LAYERS - 1),
    make_obstacle(3, 11, 1, 0, GRID_ROWS, 0, 3),
]
hazards = [
    make_hazard(1, "ash", 5, 2, 1, 2, 0, 0, severity=2),
    make_hazard(2, "turbulence", 14, 4, 1, 3, 0, 0, severity="HIGH"),
    make_hazard(3, "turbulence", 16, 3, 2, 2, 1, 1, severity="LOW"),
]

next_obstacle_id = 4
next_hazard_id = 4
selected_obstacle_id = obstacles[0]["id"]
selected_hazard_id = hazards[0]["id"]


def obstacle_cols(o):
    return range(o["col_start"], o["col_start"] + o["width"])


def obstacle_rows(o):
    return range(o["row_start"], o["row_start"] + o["height"])


def in_obstacle_grid(cell, o):
    return cell_inside_bounds(cell, obstacle_bounds_grid(o), o["z_min"], o["z_max"])


def in_any_obstacle_grid(cell):
    return any(in_obstacle_grid(cell, o) for o in obstacles)


def hazard_cols(h):
    return range(h["col_start"], h["col_start"] + h["width"])


def hazard_rows(h):
    return range(h["row_start"], h["row_start"] + h["height"])


def in_hazard_grid(cell, h):
    return cell_inside_bounds(cell, hazard_bounds_grid(h), h["z_min"], h["z_max"])


def hazards_at_grid(cell):
    return [h for h in hazards if in_hazard_grid(cell, h)]


def get_selected_obstacle():
    return next((o for o in obstacles if o["id"] == selected_obstacle_id), None)


def get_selected_hazard():
    return next((h for h in hazards if h["id"] == selected_hazard_id), None)



def wind_cost_grid(a, b):
    if a[2] != b[2]:
        return 0
    dx, dy = b[0] - a[0], b[1] - a[1]
    return WIND_PENALTY if dx * WIND_DIRECTION[0] + dy * WIND_DIRECTION[1] < 0 else 0


def grid_neighbors(cell, goal=None):
    """Candidate moves for A*. Solid obstacles are always impassable. Hazards
    that require adaptive avoidance (ash, MODERATE/HIGH turbulence) are also
    treated as impassable here -- previously they only added cost, so A* would
    happily fly straight through an ash band or turbulence cell whenever that
    was cheaper than detouring around it. LOW turbulence remains a pass-through
    (soft-cost only) region. The goal cell itself is always allowed through so
    a goal placed inside a hazard zone stays reachable."""
    x, y, z = cell
    candidates = [(x + 1, y, z), (x - 1, y, z),
                  (x, y + 1, z), (x, y - 1, z),
                  (x, y, z + 1), (x, y, z - 1)]
    return [c for c in candidates
            if 0 <= c[0] < GRID_COLS and 0 <= c[1] < GRID_ROWS and 0 <= c[2] < GRID_LAYERS
            and not in_any_obstacle_grid(c)
            and not any(hazard_blocks_cell(c, h) for h in hazards)]


def grid_move_cost(a, b):
    base = VERTICAL_COST if a[2] != b[2] else 1
    total = base + wind_cost_grid(a, b)
    total += sum(hazard_cost(h) for h in hazards_at_grid(b))
    return total


def grid_heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + VERTICAL_COST * abs(a[2] - b[2])


def reconstruct_path(came_from, current):
    result = [current]
    while current in came_from:
        current = came_from[current]
        result.append(current)
    result.reverse()
    return result


def grid_astar(start, goal):
    if in_any_obstacle_grid(goal):
        return None
    open_set = [(grid_heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    counter = 1
    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in grid_neighbors(current, goal):
            tentative = g_score[current] + grid_move_cost(current, neighbor)
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                f = tentative + grid_heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, counter, neighbor))
                counter += 1
    return None


def direct_path(start, goal):
    """Nominal trajectory intentionally ignores hazards/obstacles.
    The sensor/replanner is what changes it during flight."""
    x, y, z = start
    gx, gy, gz = goal
    result = [(x, y, z)]
    # Move on the largest horizontal axis first for a readable nominal flight line.
    axes = []
    if abs(gx - x) >= abs(gy - y):
        axes = [("x", gx), ("y", gy), ("z", gz)]
    else:
        axes = [("y", gy), ("x", gx), ("z", gz)]
    for axis, target in axes:
        while True:
            current = {"x": x, "y": y, "z": z}[axis]
            if current == target:
                break
            step = 1 if target > current else -1
            if axis == "x":
                x += step
            elif axis == "y":
                y += step
            else:
                z += step
            result.append((x, y, z))
    return result


def grid_to_pixel(cell):
    return (cell[0] * CELL_SIZE + CELL_SIZE // 2,
            cell[1] * CELL_SIZE + CELL_SIZE // 2)


# ============================================================
# MAP MODE
# ============================================================

try:
    import requests
    from PIL import Image
except ImportError:
    requests = None
    Image = None

MAP_WIDTH, MAP_HEIGHT = GRID_WIDTH, GRID_HEIGHT
MAP_ZOOM_MIN, MAP_ZOOM_MAX = 3, 18
map_zoom_level = 13
map_zoom_actual = 13
MAP_CENTER_LAT = 12.9716
MAP_CENTER_LON = 77.5946
MAP_TILE_SIZE = 256
MAP_COLS = 80
MAP_ROWS = 60
MAP_CACHE_DIR = "osm_tile_cache"
MAP_USER_AGENT = "AdaptiveDroneFlightPathReplanner/4.0 (educational hackathon project)"

map_surface = None
map_origin_world = (0.0, 0.0)
map_is_panning = False
map_pan_last_pos = None
map_pan_live_offset = (0.0, 0.0)
MAP_PAN_CLICK_THRESHOLD = 4

map_start_latlon = (MAP_CENTER_LAT, MAP_CENTER_LON)
map_goal_latlon = (12.9352, 77.6245)
map_start_z = 0
map_goal_z = 0
map_waypoints = []
map_waypoint_altitude = 2

map_obstacles = []
map_hazards = []
next_map_obstacle_id = 1
next_map_hazard_id = 1
selected_map_obstacle_id = None
selected_map_hazard_id = None


def latlon_to_world(lat, lon, zoom):
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n * MAP_TILE_SIZE
    lat_rad = math.radians(max(-85.05112878, min(85.05112878, lat)))
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n * MAP_TILE_SIZE
    return x, y


def world_to_latlon(x, y, zoom):
    n = 2 ** zoom
    lon = x / (n * MAP_TILE_SIZE) * 360.0 - 180.0
    merc = math.pi * (1.0 - 2.0 * y / (n * MAP_TILE_SIZE))
    lat = math.degrees(math.atan(math.sinh(merc)))
    return lat, lon


def tile_xy_for_world(x, y):
    return int(math.floor(x / MAP_TILE_SIZE)), int(math.floor(y / MAP_TILE_SIZE))


def download_osm_tile(tx, ty, zoom):
    if requests is None or Image is None:
        raise RuntimeError("Map mode requires: pip install requests pillow")
    os.makedirs(MAP_CACHE_DIR, exist_ok=True)
    path = os.path.join(MAP_CACHE_DIR, f"{zoom}_{tx}_{ty}.png")
    if os.path.exists(path):
        return path
    max_tile = 2 ** zoom
    if not (0 <= tx < max_tile and 0 <= ty < max_tile):
        return None
    url = f"https://tile.openstreetmap.org/{zoom}/{tx}/{ty}.png"
    response = requests.get(url, headers={"User-Agent": MAP_USER_AGENT}, timeout=8)
    response.raise_for_status()
    with open(path, "wb") as f:
        f.write(response.content)
    return path


def load_real_map():
    global map_surface, map_origin_world, map_zoom_actual, map_status
    if requests is None or Image is None:
        map_status = "Install requests + Pillow for Real-World Map Mode."
        return False
    try:
        zoom = map_zoom_level
        cx, cy = latlon_to_world(MAP_CENTER_LAT, MAP_CENTER_LON, zoom)
        left_world = cx - MAP_WIDTH / 2
        top_world = cy - MAP_HEIGHT / 2
        tx0, ty0 = tile_xy_for_world(left_world, top_world)
        tx1, ty1 = tile_xy_for_world(left_world + MAP_WIDTH - 1, top_world + MAP_HEIGHT - 1)
        mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * MAP_TILE_SIZE,
                                    (ty1 - ty0 + 1) * MAP_TILE_SIZE))
        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                tile_path = download_osm_tile(tx, ty, zoom)
                if tile_path:
                    with Image.open(tile_path) as im:
                        mosaic.paste(im.convert("RGB"),
                                     ((tx - tx0) * MAP_TILE_SIZE,
                                      (ty - ty0) * MAP_TILE_SIZE))
        crop_x = int(round(left_world - tx0 * MAP_TILE_SIZE))
        crop_y = int(round(top_world - ty0 * MAP_TILE_SIZE))
        cropped = mosaic.crop((crop_x, crop_y, crop_x + MAP_WIDTH, crop_y + MAP_HEIGHT))
        map_surface = pygame.image.frombytes(cropped.tobytes(), cropped.size, "RGB")
        map_origin_world = (left_world, top_world)
        map_zoom_actual = zoom
        map_status = f"Map loaded — zoom {zoom}."
        return True
    except Exception as exc:
        map_status = f"Map load failed: {str(exc)[:64]}"
        return False


def screen_to_map_latlon(pos):
    if map_surface is None:
        return None
    ox, oy = map_pan_live_offset if map_is_panning else (0, 0)
    wx = map_origin_world[0] + pos[0] - ox
    wy = map_origin_world[1] + pos[1] - oy
    return world_to_latlon(wx, wy, map_zoom_actual)


def map_latlon_to_screen(lat, lon):
    wx, wy = latlon_to_world(lat, lon, map_zoom_actual)
    ox, oy = map_pan_live_offset if map_is_panning else (0, 0)
    return (int(round(wx - map_origin_world[0] + ox)),
            int(round(wy - map_origin_world[1] + oy)))


def map_coord_from_latlon(latlon, z):
    if map_surface is None:
        return 0, 0, z
    lat, lon = latlon
    wx, wy = latlon_to_world(lat, lon, map_zoom_actual)
    px = wx - map_origin_world[0]
    py = wy - map_origin_world[1]
    gx = max(0, min(MAP_COLS - 1, int(px / MAP_WIDTH * MAP_COLS)))
    gy = max(0, min(MAP_ROWS - 1, int(py / MAP_HEIGHT * MAP_ROWS)))
    return gx, gy, z


def map_cell_to_latlon(cell):
    gx, gy, _ = cell
    px = (gx + 0.5) / MAP_COLS * MAP_WIDTH
    py = (gy + 0.5) / MAP_ROWS * MAP_HEIGHT
    return screen_to_map_latlon((px, py))


def make_map_obstacle(obstacle_id, lat1, lon1, lat2, lon2, z_min, z_max):
    return {"id": obstacle_id, "lat1": lat1, "lon1": lon1,
            "lat2": lat2, "lon2": lon2, "z_min": z_min, "z_max": z_max}


def make_map_hazard(hazard_id, htype, lat1, lon1, lat2, lon2,
                    z_min=0, z_max=0, severity=None):
    if severity is None:
        severity = random.choice([1, 2, 3]) if htype == "ash" else random.choice(TURBULENCE_LEVELS)
    if htype == "ash" and int(z_max) >= GRID_LAYERS - 1:
        z_min, z_max = 0, GRID_LAYERS - 1
    return {"id": hazard_id, "type": htype, "lat1": lat1, "lon1": lon1,
            "lat2": lat2, "lon2": lon2, "z_min": z_min, "z_max": z_max,
            "severity": severity}


def map_zone_bounds(z):
    return (min(z["lat1"], z["lat2"]), max(z["lat1"], z["lat2"]),
            min(z["lon1"], z["lon2"]), max(z["lon1"], z["lon2"]))


def map_zone_grid_bounds(z):
    lat_min, lat_max, lon_min, lon_max = map_zone_bounds(z)
    a = map_coord_from_latlon((lat_min, lon_min), 0)
    b = map_coord_from_latlon((lat_max, lon_max), 0)
    return min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1])


def map_obstacle_bounds_grid(o):
    return map_zone_grid_bounds(o)


def map_hazard_bounds_grid(h):
    return map_zone_grid_bounds(h)


def map_waypoint_cell(wp):
    """Convert a stored map waypoint record or raw cell into a 3-D map grid cell."""
    if isinstance(wp, dict):
        latlon = wp.get("latlon")
        if latlon is None:
            raise ValueError("Map waypoint is missing latlon data.")
        return map_coord_from_latlon(latlon, int(wp.get("z", 0)))
    if isinstance(wp, (tuple, list)) and len(wp) == 2:
        return map_coord_from_latlon((float(wp[0]), float(wp[1])), 0)
    if isinstance(wp, (tuple, list)) and len(wp) == 3:
        return tuple(wp)
    raise ValueError("Unsupported map waypoint format.")


def in_map_obstacle(cell, o):
    """Collision check accepting either a 3-D grid cell or a map waypoint record."""
    cell = map_waypoint_cell(cell)
    return cell_inside_bounds(cell, map_obstacle_bounds_grid(o), o["z_min"], o["z_max"])


def in_any_map_obstacle(cell):
    return any(in_map_obstacle(cell, o) for o in map_obstacles)


def in_map_hazard(cell, h):
    return cell_inside_bounds(cell, map_hazard_bounds_grid(h), h["z_min"], h["z_max"])


def hazards_at_map(cell):
    return [h for h in map_hazards if in_map_hazard(cell, h)]


def get_selected_map_hazard():
    return next((h for h in map_hazards if h["id"] == selected_map_hazard_id), None)


def get_selected_map_obstacle():
    return next((o for o in map_obstacles if o["id"] == selected_map_obstacle_id), None)



def map_astar(start, goal):
    if in_any_map_obstacle(goal):
        return None
    obstacle_bounds = [(map_obstacle_bounds_grid(o), o["z_min"], o["z_max"]) for o in map_obstacles]
    hazard_bounds = [(map_hazard_bounds_grid(h), h) for h in map_hazards]

    def blocked(cell):
        return any(cell_inside_bounds(cell, b, z0, z1) for b, z0, z1 in obstacle_bounds)

    def hazard_blocked(cell):
        # Avoidance hazards are hard constraints. Never permit a goal-cell
        # exception here: a goal inside ash/high turbulence is not a valid safe
        # endpoint, and the drone must route around the exclusion volume.
        return any(hazard_blocks_cell(cell, h) for h in map_hazards)

    def neighbors(cell):
        x, y, z = cell
        candidates = [(x + 1, y, z), (x - 1, y, z),
                      (x, y + 1, z), (x, y - 1, z),
                      (x, y, z + 1), (x, y, z - 1)]
        return [c for c in candidates
                if 0 <= c[0] < MAP_COLS and 0 <= c[1] < MAP_ROWS and 0 <= c[2] < GRID_LAYERS
                and not blocked(c)
                and not hazard_blocked(c)]

    def cost(a, b):
        base = VERTICAL_COST if a[2] != b[2] else 1
        wind = 0
        if a[2] == b[2]:
            dx, dy = b[0] - a[0], b[1] - a[1]
            if dx * WIND_DIRECTION[0] + dy * WIND_DIRECTION[1] < 0:
                wind = WIND_PENALTY
        hazard_total = 0
        for bnd, h in hazard_bounds:
            if cell_inside_bounds(b, bnd, h["z_min"], h["z_max"]):
                hazard_total += hazard_cost(h)
        return base + wind + hazard_total

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + VERTICAL_COST * abs(a[2] - b[2])

    open_set = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    counter = 1
    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current):
            tentative = g_score[current] + cost(current, neighbor)
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(open_set, (tentative + heuristic(neighbor, goal), counter, neighbor))
                counter += 1
    return None


# ============================================================
# GENERIC REAL-TIME MISSION STATE
# ============================================================

sim_state = "HOME"
map_mode = False
home_screen_started = time.perf_counter()
home_screen_ready = False

# Grid route state
grid_path = []
grid_pixel_path = []
grid_current_segment = 0
grid_progress = 0.0
grid_target_index = 0

grid_hazard_ahead = False
grid_hazard_cells = []
grid_hazard_was_detected = False
grid_response_state = "IDLE"
grid_response_timer = 0.0
grid_pending_threat = None
grid_threat_lock = None
grid_replan_count = 0
grid_last_replan_ms = 0.0
grid_hazard_cells_crossed = 0
grid_last_counted_segment = -1
grid_mission_complete = False
grid_mission_start_ticks = 0
grid_mission_end_ticks = 0
grid_old_path = []
grid_old_path_timer = 0.0
grid_setup_mode = "idle"
grid_pending_placement = None
grid_pending_corner1 = None
grid_panel_scroll = 0
grid_panel_content_height = 0
grid_setup_message = ""
grid_show_report = False

# Map route state
map_path = []
map_pixel_path = []
map_current_segment = 0
map_progress = 0.0
map_target_index = 0
map_hazard_ahead = False
map_hazard_cells = []
map_hazard_was_detected = False
map_response_state = "IDLE"
map_response_timer = 0.0
map_pending_threat = None
map_threat_lock = None
map_replan_count = 0
map_last_replan_ms = 0.0
map_hazard_cells_crossed = 0
map_last_counted_segment = -1
map_mission_complete = False
map_mission_start_ticks = 0
map_mission_end_ticks = 0
map_old_path = []
map_old_path_timer = 0.0
map_setup_mode = "idle"
map_pending_placement = None
map_pending_corner1_latlon = None
map_panel_scroll = 0
map_panel_content_height = 0
map_status = "Map mode ready — press LOAD MAP / place start and goal."
map_show_report = False

show_report = False

# Altitude profile geometry
PANEL_MARGIN = 20
TICK_LABEL_GUTTER = 84
panel_rect = pygame.Rect(GRID_WIDTH + TICK_LABEL_GUTTER, 70,
                         PANEL_WIDTH - TICK_LABEL_GUTTER - PANEL_MARGIN,
                         GRID_HEIGHT - 150)


def build_targets_grid():
    return list(waypoints) + [goal_cell]


def build_targets_map():
    return [map_coord_from_latlon(wp["latlon"], wp["z"]) for wp in map_waypoints] + [
        map_coord_from_latlon(map_goal_latlon, map_goal_z)]


def grid_route_for_leg(start, target):
    # Build the actual leg with the same 3-D obstacle/hazard constraints used
    # by live replanning. The drone therefore starts each waypoint leg on a
    # safe trajectory instead of following a hazard-blind straight line.
    return grid_astar(start, target)


def map_route_for_leg(start, target):
    # Map mode uses the same 3-D constrained planner as the live replan path.
    return map_astar(start, target)


def refresh_grid_pixel_path():
    global grid_pixel_path
    grid_pixel_path = [grid_to_pixel(c) for c in grid_path]


def refresh_map_pixel_path():
    global map_pixel_path
    map_pixel_path = [map_latlon_to_screen(*map_cell_to_latlon(c)) for c in map_path]


def grid_target_advance():
    """Advance to the next waypoint/goal and immediately run the live sensor on the new leg.

    Every leg is initially a nominal, hazard-blind trajectory so the demonstration can
    visibly show adaptive replanning. However, the sensor must inspect the NEW leg before
    the drone is allowed to continue moving. This prevents a waypoint transition from
    giving the drone a frame in which it can enter a full/partial obstacle, ash plume,
    or avoidable turbulence region without triggering course correction.
    """
    global grid_path, grid_current_segment, grid_progress, grid_target_index
    global grid_mission_complete, grid_mission_end_ticks
    global grid_threat_lock, grid_hazard_ahead, grid_hazard_cells
    targets = build_targets_grid()
    if grid_target_index + 1 < len(targets):
        grid_target_index += 1
        new_path = grid_route_for_leg(grid_path[-1], targets[grid_target_index])
        if not new_path:
            grid_mission_complete = True
            grid_mission_end_ticks = pygame.time.get_ticks()
            announce("WAYPOINT ROUTE FAILED — MISSION HALTED", "warning", 2.0)
            return

        grid_path = new_path
        refresh_grid_pixel_path()
        grid_current_segment = 0
        grid_progress = 0.0
        # Every waypoint leg gets a fresh sensor lock and a fresh threat scan.
        grid_threat_lock = None
        grid_hazard_ahead = False
        grid_hazard_cells = []

        # Critical: inspect the new waypoint/goal leg immediately. This catches:
        # - full-height obstacles at any altitude,
        # - partial-altitude obstacles when the leg is at their altitude,
        # - all ash zones that intersect the leg,
        # - moderate/high turbulence that must be avoided.
        threats = grid_scan_threats()
        if threats:
            grid_start_replan_sequence(threats[0])
        else:
            lows = scan_noncritical_grid()
            if lows:
                announce("LOW TURBULENCE DETECTED — HOLDING COURSE", "safe", 1.0)
    else:
        grid_mission_complete = True
        grid_mission_end_ticks = pygame.time.get_ticks()
        announce("MISSION COMPLETE — GOAL REACHED", "safe", 2.5)


def map_target_advance():
    """Advance to the next geographic waypoint/goal and sensor-check that leg immediately."""
    global map_path, map_current_segment, map_progress, map_target_index
    global map_mission_complete, map_mission_end_ticks
    global map_threat_lock, map_hazard_ahead, map_hazard_cells
    targets = build_targets_map()
    if map_target_index + 1 < len(targets):
        map_target_index += 1
        new_path = map_route_for_leg(map_path[-1], targets[map_target_index])
        if not new_path:
            map_mission_complete = True
            map_mission_end_ticks = pygame.time.get_ticks()
            announce("WAYPOINT ROUTE FAILED — MISSION HALTED", "warning", 2.0)
            return

        map_path = new_path
        refresh_map_pixel_path()
        map_current_segment = 0
        map_progress = 0.0
        # Every geographic waypoint leg gets a fresh sensor lock and scan.
        map_threat_lock = None
        map_hazard_ahead = False
        map_hazard_cells = []

        # Immediately inspect the new nominal leg before movement resumes. This
        # applies the same 3-D obstacle/ash/turbulence rules used during replanning.
        threats = map_scan_threats()
        if threats:
            map_start_replan_sequence(threats[0])
        else:
            lows = scan_noncritical_map()
            if lows:
                announce("LOW TURBULENCE DETECTED — HOLDING COURSE", "safe", 1.0)
    else:
        map_mission_complete = True
        map_mission_end_ticks = pygame.time.get_ticks()
        announce("MISSION COMPLETE — GOAL REACHED", "safe", 2.5)


def grid_scan_threats():
    if not grid_path:
        return []
    end = min(len(grid_path), grid_current_segment + 1 + SENSOR_RANGE_CELLS)
    threats = []
    seen = set()
    for idx in range(grid_current_segment, end):
        cell = grid_path[idx]
        for o in obstacles:
            if in_obstacle_grid(cell, o):
                key = ("obstacle", o["id"], idx)
                if key not in seen:
                    seen.add(key)
                    threats.append({"kind": "obstacle", "obj": o, "index": idx})
        for h in hazards:
            if hazard_blocks_cell(cell, h):
                key = ("hazard", h["id"], idx)
                if key not in seen:
                    seen.add(key)
                    threats.append({"kind": "hazard", "obj": h, "index": idx})
    return threats


def map_scan_threats():
    if not map_path:
        return []
    end = min(len(map_path), map_current_segment + 1 + SENSOR_RANGE_CELLS)
    threats = []
    seen = set()
    obs_bounds = [(map_obstacle_bounds_grid(o), o) for o in map_obstacles]
    haz_bounds = [(map_hazard_bounds_grid(h), h) for h in map_hazards]
    for idx in range(map_current_segment, end):
        cell = map_path[idx]
        for bnd, o in obs_bounds:
            if cell_inside_bounds(cell, bnd, o["z_min"], o["z_max"]):
                key = ("obstacle", o["id"], idx)
                if key not in seen:
                    seen.add(key)
                    threats.append({"kind": "obstacle", "obj": o, "index": idx})
        for bnd, h in haz_bounds:
            if hazard_blocks_cell(cell, h):
                key = ("hazard", h["id"], idx)
                if key not in seen:
                    seen.add(key)
                    threats.append({"kind": "hazard", "obj": h, "index": idx})
    return threats


def scan_noncritical_grid():
    if not grid_path:
        return []
    end = min(len(grid_path), grid_current_segment + 1 + SENSOR_RANGE_CELLS)
    found = []
    for idx in range(grid_current_segment, end):
        for h in hazards_at_grid(grid_path[idx]):
            if not hazard_requires_replan(h):
                found.append(h)
    return found


def scan_noncritical_map():
    if not map_path:
        return []
    end = min(len(map_path), map_current_segment + 1 + SENSOR_RANGE_CELLS)
    found = []
    bnds = [(map_hazard_bounds_grid(h), h) for h in map_hazards]
    for idx in range(map_current_segment, end):
        cell = map_path[idx]
        for bnd, h in bnds:
            if cell_inside_bounds(cell, bnd, h["z_min"], h["z_max"]) and not hazard_requires_replan(h):
                found.append(h)
    return found


def grid_current_cell():
    return grid_path[grid_current_segment] if grid_path else start_cell


def map_current_cell():
    return map_path[map_current_segment] if map_path else map_coord_from_latlon(map_start_latlon, map_start_z)


def grid_threat_signature(threat):
    o = threat["obj"]
    if threat["kind"] == "obstacle":
        return ("obstacle", o["id"])
    return ("hazard", o["id"], o.get("severity"))


def map_threat_signature(threat):
    o = threat["obj"]
    if threat["kind"] == "obstacle":
        return ("obstacle", o["id"])
    return ("hazard", o["id"], o.get("severity"))


def grid_start_replan_sequence(threat):
    global grid_response_state, grid_response_timer, grid_pending_threat, grid_threat_lock
    global grid_hazard_ahead, grid_hazard_cells
    grid_pending_threat = threat
    grid_threat_lock = grid_threat_signature(threat)
    grid_response_state = "DETECTED"
    grid_response_timer = 0.55
    grid_hazard_ahead = True
    label = "OBSTACLE" if threat["kind"] == "obstacle" else hazard_label(threat["obj"])
    announce(f"HAZARD DETECTED — {label} — CHANGING COURSE", "warning", 1.7)


def map_start_replan_sequence(threat):
    global map_response_state, map_response_timer, map_pending_threat, map_threat_lock
    global map_hazard_ahead, map_hazard_cells
    map_pending_threat = threat
    map_threat_lock = map_threat_signature(threat)
    map_response_state = "DETECTED"
    map_response_timer = 0.55
    map_hazard_ahead = True
    label = "OBSTACLE" if threat["kind"] == "obstacle" else hazard_label(threat["obj"])
    announce(f"HAZARD DETECTED — {label} — CHANGING COURSE", "warning", 1.7)


def grid_execute_replan():
    global grid_path, grid_pixel_path, grid_replan_count, grid_last_replan_ms
    global grid_response_state, grid_response_timer, grid_pending_threat, grid_old_path, grid_old_path_timer
    global grid_hazard_ahead, grid_progress
    if not grid_pending_threat or not grid_path:
        grid_response_state = "IDLE"
        return
    targets = build_targets_grid()
    if grid_target_index >= len(targets):
        grid_response_state = "IDLE"
        return
    start = grid_current_cell()
    goal = targets[grid_target_index]
    old_path = grid_path[grid_current_segment:]
    t0 = time.perf_counter()
    new_remaining = grid_astar(start, goal)
    elapsed = (time.perf_counter() - t0) * 1000.0
    grid_last_replan_ms = elapsed
    if new_remaining is not None:
        grid_old_path = old_path
        grid_old_path_timer = 1.25
        grid_path = grid_path[:grid_current_segment] + new_remaining
        refresh_grid_pixel_path()
        grid_replan_count += 1
        grid_progress = 0.0
        grid_response_state = "CORRECTED"
        grid_response_timer = 1.0
        announce("COURSE CORRECTION — NEW TRAJECTORY LOCKED", "info", 1.5)
    else:
        grid_response_state = "IDLE"
        grid_response_timer = 0.0
        announce("REPLAN FAILED — HOLDING CURRENT TRAJECTORY", "warning", 2.0)
    grid_pending_threat = None
    grid_hazard_ahead = True


def map_execute_replan():
    global map_path, map_pixel_path, map_replan_count, map_last_replan_ms
    global map_response_state, map_response_timer, map_pending_threat, map_old_path, map_old_path_timer
    global map_hazard_ahead, map_progress
    if not map_pending_threat or not map_path:
        map_response_state = "IDLE"
        return
    targets = build_targets_map()
    if map_target_index >= len(targets):
        map_response_state = "IDLE"
        return
    start = map_current_cell()
    goal = targets[map_target_index]
    old_path = map_path[map_current_segment:]
    t0 = time.perf_counter()
    new_remaining = map_astar(start, goal)
    elapsed = (time.perf_counter() - t0) * 1000.0
    map_last_replan_ms = elapsed
    if new_remaining is not None:
        map_old_path = old_path
        map_old_path_timer = 1.25
        map_path = map_path[:map_current_segment] + new_remaining
        refresh_map_pixel_path()
        map_replan_count += 1
        map_progress = 0.0
        map_response_state = "CORRECTED"
        map_response_timer = 1.0
        announce("COURSE CORRECTION — NEW TRAJECTORY LOCKED", "info", 1.5)
    else:
        map_response_state = "IDLE"
        map_response_timer = 0.0
        announce("REPLAN FAILED — HOLDING CURRENT TRAJECTORY", "warning", 2.0)
    map_pending_threat = None
    map_hazard_ahead = True


def grid_update_mission(dt):
    global grid_progress, grid_current_segment, grid_hazard_ahead, grid_hazard_cells, grid_hazard_cells_crossed
    global grid_hazard_was_detected, grid_response_state, grid_response_timer
    global grid_old_path_timer, grid_last_counted_segment, grid_threat_lock
    if grid_mission_complete or not grid_path:
        return

    if grid_old_path_timer > 0:
        grid_old_path_timer = max(0.0, grid_old_path_timer - dt)

    if grid_response_state == "DETECTED":
        grid_response_timer -= dt
        if grid_response_timer <= 0:
            grid_execute_replan()
        return
    if grid_response_state == "CORRECTED":
        grid_response_timer -= dt
        if grid_response_timer <= 0:
            grid_response_state = "IDLE"

    # Detect on cell boundaries so a newly detected threat never causes a visible back-jump.
    at_boundary = grid_progress <= 0.001
    if at_boundary:
        threats = grid_scan_threats()
        grid_hazard_cells = [grid_path[i] for i in range(grid_current_segment,
                                                           min(len(grid_path), grid_current_segment + 1 + SENSOR_RANGE_CELLS))
                             if hazards_at_grid(grid_path[i])]
        grid_hazard_ahead = bool(grid_hazard_cells or threats)
        if threats:
            threat = threats[0]
            signature = grid_threat_signature(threat)
            if signature != grid_threat_lock:
                grid_start_replan_sequence(threat)
                return
        else:
            lows = scan_noncritical_grid()
            if lows:
                announce("LOW TURBULENCE DETECTED — HOLDING COURSE", "safe", 1.0)

    grid_progress += DRONE_SPEED_VALUES[DRONE_SPEED_INDEX] * dt
    while grid_progress >= 1.0 and not grid_mission_complete:
        grid_progress -= 1.0
        if grid_current_segment < len(grid_path) - 1:
            grid_current_segment += 1
            if grid_current_segment != grid_last_counted_segment:
                if hazards_at_grid(grid_path[grid_current_segment]):
                    grid_hazard_cells_crossed += 1
                grid_last_counted_segment = grid_current_segment
        else:
            grid_target_advance()
            # Never consume leftover frame time on a newly entered waypoint leg.
            # The new-leg sensor decision must be processed before movement resumes.
            break


def map_update_mission(dt):
    global map_progress, map_current_segment, map_hazard_ahead, map_hazard_cells, map_hazard_cells_crossed
    global map_hazard_was_detected, map_response_state, map_response_timer
    global map_old_path_timer, map_last_counted_segment, map_threat_lock
    if map_mission_complete or not map_path:
        return

    if map_old_path_timer > 0:
        map_old_path_timer = max(0.0, map_old_path_timer - dt)

    if map_response_state == "DETECTED":
        map_response_timer -= dt
        if map_response_timer <= 0:
            map_execute_replan()
        return
    if map_response_state == "CORRECTED":
        map_response_timer -= dt
        if map_response_timer <= 0:
            map_response_state = "IDLE"

    if map_progress <= 0.001:
        threats = map_scan_threats()
        end = min(len(map_path), map_current_segment + 1 + SENSOR_RANGE_CELLS)
        map_hazard_cells = [map_path[i] for i in range(map_current_segment, end) if hazards_at_map(map_path[i])]
        map_hazard_ahead = bool(map_hazard_cells or threats)
        if threats:
            threat = threats[0]
            signature = map_threat_signature(threat)
            if signature != map_threat_lock:
                map_start_replan_sequence(threat)
                return
        else:
            lows = scan_noncritical_map()
            if lows:
                announce("LOW TURBULENCE DETECTED — HOLDING COURSE", "safe", 1.0)

    map_progress += DRONE_SPEED_VALUES[DRONE_SPEED_INDEX] * dt
    while map_progress >= 1.0 and not map_mission_complete:
        map_progress -= 1.0
        if map_current_segment < len(map_path) - 1:
            map_current_segment += 1
            if map_current_segment != map_last_counted_segment:
                if hazards_at_map(map_path[map_current_segment]):
                    map_hazard_cells_crossed += 1
                map_last_counted_segment = map_current_segment
        else:
            map_target_advance()
            # Never consume leftover frame time on a newly entered waypoint leg.
            # The new-leg sensor decision must be processed before movement resumes.
            break


# ============================================================
# PATH / METRICS
# ============================================================


def path_cost_grid(p):
    return sum(grid_move_cost(p[i - 1], p[i]) for i in range(1, len(p)))


def path_altitude_changes(p):
    return sum(p[i][2] != p[i - 1][2] for i in range(1, len(p)))


def map_path_cost(p):
    if not p:
        return 0
    # Cache map geometry bounds for metric calculation.
    bnds = [(map_hazard_bounds_grid(h), h) for h in map_hazards]
    total = 0
    for i in range(1, len(p)):
        a, b = p[i - 1], p[i]
        total += VERTICAL_COST if a[2] != b[2] else 1
        if a[2] == b[2]:
            dx, dy = b[0] - a[0], b[1] - a[1]
            if dx * WIND_DIRECTION[0] + dy * WIND_DIRECTION[1] < 0:
                total += WIND_PENALTY
        for bound, h in bnds:
            if cell_inside_bounds(b, bound, h["z_min"], h["z_max"]):
                total += hazard_cost(h)
    return total


def mission_elapsed(start_ticks, end_ticks, complete):
    end = end_ticks if complete else pygame.time.get_ticks()
    return max(0.0, (end - start_ticks) / 1000.0)


# ============================================================
# RESET / RANDOMIZATION
# ============================================================


def clear_grid_scenario(clear_waypoints=True):
    global obstacles, hazards, next_obstacle_id, next_hazard_id
    global selected_obstacle_id, selected_hazard_id, grid_path, grid_pixel_path
    global grid_current_segment, grid_progress, grid_target_index, grid_mission_complete
    global grid_setup_mode, grid_pending_placement, grid_pending_corner1, grid_setup_message
    global grid_replan_count, grid_last_replan_ms, grid_hazard_cells_crossed, grid_hazard_ahead
    global grid_response_state, grid_response_timer, grid_pending_threat, grid_threat_lock
    global grid_old_path, grid_old_path_timer, grid_show_report
    obstacles = []
    hazards = []
    next_obstacle_id = 1
    next_hazard_id = 1
    selected_obstacle_id = None
    selected_hazard_id = None
    if clear_waypoints:
        waypoints.clear()
    grid_path = []
    grid_pixel_path = []
    grid_current_segment = 0
    grid_progress = 0.0
    grid_target_index = 0
    grid_mission_complete = False
    grid_setup_mode = "idle"
    grid_pending_placement = None
    grid_pending_corner1 = None
    grid_setup_message = "Scenario cleared — add hazards or randomize."
    grid_replan_count = 0
    grid_last_replan_ms = 0.0
    grid_hazard_cells_crossed = 0
    grid_hazard_ahead = False
    grid_response_state = "IDLE"
    grid_response_timer = 0.0
    grid_pending_threat = None
    grid_threat_lock = None
    grid_old_path = []
    grid_old_path_timer = 0.0
    grid_show_report = False
    globals()["sim_state"] = "SETUP"


def reset_grid_flight():
    global grid_path, grid_pixel_path, grid_current_segment, grid_progress, grid_target_index
    global grid_mission_complete, grid_replan_count, grid_last_replan_ms, grid_hazard_cells_crossed
    global grid_hazard_ahead, grid_hazard_cells, grid_hazard_was_detected, grid_response_state
    global grid_response_timer, grid_pending_threat, grid_threat_lock, grid_old_path, grid_old_path_timer
    global grid_show_report, grid_setup_message
    grid_path = []
    grid_pixel_path = []
    grid_current_segment = 0
    grid_progress = 0.0
    grid_target_index = 0
    grid_mission_complete = False
    grid_replan_count = 0
    grid_last_replan_ms = 0.0
    grid_hazard_cells_crossed = 0
    grid_hazard_ahead = False
    grid_hazard_cells = []
    grid_hazard_was_detected = False
    grid_response_state = "IDLE"
    grid_response_timer = 0.0
    grid_pending_threat = None
    grid_threat_lock = None
    grid_old_path = []
    grid_old_path_timer = 0.0
    grid_show_report = False
    grid_setup_message = "Flight reset — scenario retained."
    globals()["sim_state"] = "SETUP"


def clear_map_scenario(clear_waypoints=True):
    global map_obstacles, map_hazards, next_map_obstacle_id, next_map_hazard_id
    global selected_map_obstacle_id, selected_map_hazard_id
    global map_path, map_pixel_path, map_current_segment, map_progress, map_target_index
    global map_mission_complete, map_status, map_setup_mode, map_pending_placement, map_pending_corner1_latlon
    global map_replan_count, map_last_replan_ms, map_hazard_cells_crossed, map_hazard_ahead
    global map_hazard_cells, map_response_state, map_response_timer, map_pending_threat, map_threat_lock
    global map_old_path, map_old_path_timer, map_show_report
    map_obstacles = []
    map_hazards = []
    next_map_obstacle_id = 1
    next_map_hazard_id = 1
    selected_map_obstacle_id = None
    selected_map_hazard_id = None
    if clear_waypoints:
        map_waypoints.clear()
    map_path = []
    map_pixel_path = []
    map_current_segment = 0
    map_progress = 0.0
    map_target_index = 0
    map_mission_complete = False
    map_status = "Scenario cleared — add zones or randomize."
    map_setup_mode = "idle"
    map_pending_placement = None
    map_pending_corner1_latlon = None
    map_replan_count = 0
    map_last_replan_ms = 0.0
    map_hazard_cells_crossed = 0
    map_hazard_ahead = False
    map_hazard_cells = []
    map_response_state = "IDLE"
    map_response_timer = 0.0
    map_pending_threat = None
    map_threat_lock = None
    map_old_path = []
    map_old_path_timer = 0.0
    map_show_report = False
    globals()["sim_state"] = "SETUP"


def reset_map_flight():
    global map_path, map_pixel_path, map_current_segment, map_progress, map_target_index
    global map_mission_complete, map_replan_count, map_last_replan_ms, map_hazard_cells_crossed
    global map_hazard_ahead, map_hazard_cells, map_response_state, map_response_timer
    global map_pending_threat, map_threat_lock, map_old_path, map_old_path_timer, map_show_report, map_status
    map_path = []
    map_pixel_path = []
    map_current_segment = 0
    map_progress = 0.0
    map_target_index = 0
    map_mission_complete = False
    map_replan_count = 0
    map_last_replan_ms = 0.0
    map_hazard_cells_crossed = 0
    map_hazard_ahead = False
    map_hazard_cells = []
    map_response_state = "IDLE"
    map_response_timer = 0.0
    map_pending_threat = None
    map_threat_lock = None
    map_old_path = []
    map_old_path_timer = 0.0
    map_show_report = False
    map_status = "Flight reset — scenario retained."
    globals()["sim_state"] = "SETUP"


def add_random_grid_scenario():
    global next_obstacle_id, next_hazard_id, selected_obstacle_id, selected_hazard_id
    clear_grid_scenario(clear_waypoints=True)
    random.seed()
    reserved = {start_cell[:2], goal_cell[:2]}

    def safe_zone(cs, w, rs, h):
        occupied = {(x, y) for x in range(cs, cs + w) for y in range(rs, rs + h)}
        return not occupied.intersection(reserved)

    # Keep the random scenario challengeable rather than building an unintentional wall.
    for _ in range(4):
        for _attempt in range(50):
            w, h = random.choice([(1, 2), (2, 2), (2, 3), (3, 2)])
            cs = random.randint(3, GRID_COLS - w - 3)
            rs = random.randint(1, GRID_ROWS - h - 2)
            if safe_zone(cs, w, rs, h):
                z_min, z_max = (0, GRID_LAYERS - 1) if random.random() < 0.65 else sorted(random.sample(range(GRID_LAYERS), 2))
                o = make_obstacle(next_obstacle_id, cs, w, rs, h, z_min, z_max)
                obstacles.append(o)
                next_obstacle_id += 1
                break

    for htype in ["ash", "ash", "turbulence", "turbulence", "turbulence"]:
        for _attempt in range(50):
            w, h = random.choice([(2, 2), (3, 2), (3, 3), (4, 2)])
            cs = random.randint(2, GRID_COLS - w - 2)
            rs = random.randint(1, GRID_ROWS - h - 2)
            if safe_zone(cs, w, rs, h):
                z = random.randint(0, GRID_LAYERS - 1)
                severity = random.randint(1, 3) if htype == "ash" else random.choice(TURBULENCE_LEVELS)
                hz = make_hazard(next_hazard_id, htype, cs, w, rs, h, z, z, severity)
                hazards.append(hz)
                next_hazard_id += 1
                break

    # Guarantee one visible, route-intersecting high-turbulence trigger so the demo
    # reliably shows live detection/replanning on every randomize press.
    nominal = direct_path(start_cell, goal_cell)
    if len(nominal) >= 8:
        trigger = nominal[min(SENSOR_RANGE_CELLS + 1, len(nominal) - 2)]
        cs = max(1, min(GRID_COLS - 3, trigger[0] - 1))
        rs = max(0, min(GRID_ROWS - 2, trigger[1] - 1))
        hz = make_hazard(next_hazard_id, "turbulence", cs, 3, rs, 2, trigger[2], trigger[2], "HIGH")
        hazards.append(hz)
        next_hazard_id += 1

    selected_obstacle_id = obstacles[0]["id"] if obstacles else None
    selected_hazard_id = hazards[0]["id"] if hazards else None
    globals()["grid_setup_message"] = "RANDOM SCENARIO GENERATED — sensor will detect hazards in flight."
    announce("RANDOMIZER — SCENARIO ARMED", "info", 1.5)


def random_map_rect(latlon_center=None):
    # Choose a rectangle in current screen coordinates, then convert corners back to lat/lon.
    mx = random.randint(120, MAP_WIDTH - 220)
    my = random.randint(90, MAP_HEIGHT - 180)
    mw = random.randint(50, 170)
    mh = random.randint(45, 130)
    p1 = (mx, my)
    p2 = (min(MAP_WIDTH - 8, mx + mw), min(MAP_HEIGHT - 8, my + mh))
    ll1 = screen_to_map_latlon(p1)
    ll2 = screen_to_map_latlon(p2)
    return ll1, ll2


def add_random_map_scenario():
    global next_map_obstacle_id, next_map_hazard_id, selected_map_obstacle_id, selected_map_hazard_id
    clear_map_scenario(clear_waypoints=True)
    if map_surface is None:
        load_real_map()
    if map_surface is None:
        map_status = "Map unavailable — cannot randomize geographic zones."
        globals()["map_status"] = map_status
        return
    random.seed()
    start_px = map_latlon_to_screen(*map_start_latlon)
    goal_px = map_latlon_to_screen(*map_goal_latlon)

    def away_from_points(ll1, ll2):
        p1 = map_latlon_to_screen(*ll1)
        p2 = map_latlon_to_screen(*ll2)
        cx = (p1[0] + p2[0]) / 2
        cy = (p1[1] + p2[1]) / 2
        return (math.hypot(cx - start_px[0], cy - start_px[1]) > 80 and
                math.hypot(cx - goal_px[0], cy - goal_px[1]) > 80)

    for _ in range(4):
        for _attempt in range(50):
            ll1, ll2 = random_map_rect()
            if away_from_points(ll1, ll2):
                zmin, zmax = (0, GRID_LAYERS - 1) if random.random() < 0.65 else sorted(random.sample(range(GRID_LAYERS), 2))
                map_obstacles.append(make_map_obstacle(next_map_obstacle_id, ll1[0], ll1[1], ll2[0], ll2[1], zmin, zmax))
                next_map_obstacle_id += 1
                break

    for htype in ["ash", "turbulence", "turbulence", "ash", "turbulence"]:
        for _attempt in range(50):
            ll1, ll2 = random_map_rect()
            if away_from_points(ll1, ll2):
                z = random.randint(0, GRID_LAYERS - 1)
                severity = random.randint(1, 3) if htype == "ash" else random.choice(TURBULENCE_LEVELS)
                hz = make_map_hazard(next_map_hazard_id, htype, ll1[0], ll1[1], ll2[0], ll2[1], z, z, severity)
                map_hazards.append(hz)
                next_map_hazard_id += 1
                break

    # Guarantee one visible, route-intersecting high-turbulence trigger for the map demo.
    start = map_coord_from_latlon(map_start_latlon, map_start_z)
    goal = map_coord_from_latlon(map_goal_latlon, map_goal_z)
    nominal = direct_path(start, goal)
    if len(nominal) >= 8:
        trigger = nominal[min(SENSOR_RANGE_CELLS + 1, len(nominal) - 2)]
        px = ((trigger[0] + 0.5) / MAP_COLS * MAP_WIDTH,
              (trigger[1] + 0.5) / MAP_ROWS * MAP_HEIGHT)
        p1 = (max(20, px[0] - 28), max(20, px[1] - 22))
        p2 = (min(MAP_WIDTH - 20, px[0] + 28), min(MAP_HEIGHT - 20, px[1] + 22))
        ll1, ll2 = screen_to_map_latlon(p1), screen_to_map_latlon(p2)
        hz = make_map_hazard(next_map_hazard_id, "turbulence", ll1[0], ll1[1], ll2[0], ll2[1],
                             trigger[2], trigger[2], "HIGH")
        map_hazards.append(hz)
        next_map_hazard_id += 1

    selected_map_obstacle_id = map_obstacles[0]["id"] if map_obstacles else None
    selected_map_hazard_id = map_hazards[0]["id"] if map_hazards else None
    globals()["map_status"] = "RANDOM GEOGRAPHIC SCENARIO GENERATED — sensor live."
    announce("RANDOMIZER — GEO SCENARIO ARMED", "info", 1.5)


# ============================================================
# SETUP / LAUNCH ACTIONS
# ============================================================


def start_grid_simulation():
    global sim_state, grid_path, grid_pixel_path, grid_current_segment, grid_progress
    global grid_target_index, grid_mission_complete, grid_replan_count, grid_last_replan_ms
    global grid_hazard_cells_crossed, grid_hazard_ahead, grid_hazard_cells, grid_hazard_was_detected
    global grid_response_state, grid_response_timer, grid_pending_threat, grid_threat_lock
    global grid_old_path, grid_old_path_timer, grid_show_report, grid_mission_start_ticks, grid_mission_end_ticks
    global grid_setup_message
    if start_cell == goal_cell and not waypoints:
        grid_setup_message = "Start and goal cannot be the same cell."
        return
    if in_any_obstacle_grid(start_cell) or in_any_obstacle_grid(goal_cell):
        grid_setup_message = "START or GOAL is inside a solid obstacle."
        return
    if any(hazard_blocks_cell(start_cell, h) for h in hazards):
        grid_setup_message = "START is inside an avoidance hazard."
        return
    if any(hazard_blocks_cell(goal_cell, h) for h in hazards):
        grid_setup_message = "GOAL is inside an avoidance hazard — move it outside the plume."
        return
    for idx, wp in enumerate(waypoints, start=1):
        # Waypoints are validated as full 3-D cells at their selected altitude.
        # The actual leg is still initially nominal; the live sensor then catches
        # threats along the way and replans around them.
        if in_any_obstacle_grid(wp):
            grid_setup_message = f"Waypoint #{idx} is inside an obstacle at {alt_ft(wp[2]):,} ft."
            return
        blocking = [h for h in hazards if hazard_blocks_cell(wp, h)]
        if blocking:
            grid_setup_message = (f"Waypoint #{idx} is inside {hazard_label(blocking[0])} "
                                  f"at {alt_ft(wp[2]):,} ft.")
            return
    targets = build_targets_grid()
    first_path = grid_route_for_leg(start_cell, targets[0])
    if not first_path:
        grid_setup_message = "Could not build the nominal first leg."
        return
    grid_path = first_path
    refresh_grid_pixel_path()
    grid_current_segment = 0
    grid_progress = 0.0
    grid_target_index = 0
    grid_mission_complete = False
    grid_replan_count = 0
    grid_last_replan_ms = 0.0
    grid_hazard_cells_crossed = 0
    grid_hazard_ahead = False
    grid_hazard_cells = []
    grid_hazard_was_detected = False
    grid_response_state = "IDLE"
    grid_response_timer = 0.0
    grid_pending_threat = None
    grid_threat_lock = None
    grid_old_path = []
    grid_old_path_timer = 0.0
    grid_show_report = False
    grid_mission_start_ticks = pygame.time.get_ticks()
    grid_mission_end_ticks = grid_mission_start_ticks
    grid_setup_message = "LIVE SENSOR ENABLED — obstacle, turbulence and ash avoidance active."
    sim_state = "RUNNING"
    announce("MISSION STARTED — LIVE HAZARD DETECTION ACTIVE", "safe", 1.8)


def start_map_simulation():
    global sim_state, map_path, map_pixel_path, map_current_segment, map_progress
    global map_target_index, map_mission_complete, map_replan_count, map_last_replan_ms
    global map_hazard_cells_crossed, map_hazard_ahead, map_hazard_cells, map_response_state
    global map_response_timer, map_pending_threat, map_threat_lock, map_old_path, map_old_path_timer
    global map_show_report, map_mission_start_ticks, map_mission_end_ticks, map_status
    if map_surface is None and not load_real_map():
        return
    start = map_coord_from_latlon(map_start_latlon, map_start_z)
    goal = map_coord_from_latlon(map_goal_latlon, map_goal_z)
    if start == goal and not map_waypoints:
        map_status = "START and GOAL cannot be the same geographic cell."
        return
    if in_any_map_obstacle(start) or in_any_map_obstacle(goal):
        map_status = "START or GOAL is inside a solid obstacle."
        return
    if any(hazard_blocks_cell(start, h) for h in map_hazards):
        map_status = "START is inside an avoidance hazard."
        return
    if any(hazard_blocks_cell(goal, h) for h in map_hazards):
        map_status = "GOAL is inside an avoidance hazard — move it outside the plume."
        return
    targets = build_targets_map()
    for idx, wp in enumerate(map_waypoints, start=1):
        wp_cell = map_waypoint_cell(wp)
        # Validate the exact waypoint altitude against solid obstacles and
        # avoidance hazards before launching the mission.
        if in_any_map_obstacle(wp_cell):
            map_status = f"Waypoint #{idx} is inside an obstacle at {alt_ft(wp_cell[2]):,} ft."
            return
        blocking = [h for h in map_hazards if hazard_blocks_cell(wp_cell, h)]
        if blocking:
            map_status = (f"Waypoint #{idx} is inside {hazard_label(blocking[0])} "
                          f"at {alt_ft(wp_cell[2]):,} ft.")
            return
    map_path = map_route_for_leg(start, targets[0])
    if not map_path:
        map_status = "Could not build the nominal first geographic leg."
        return
    refresh_map_pixel_path()
    map_current_segment = 0
    map_progress = 0.0
    map_target_index = 0
    map_mission_complete = False
    map_replan_count = 0
    map_last_replan_ms = 0.0
    map_hazard_cells_crossed = 0
    map_hazard_ahead = False
    map_hazard_cells = []
    map_response_state = "IDLE"
    map_response_timer = 0.0
    map_pending_threat = None
    map_threat_lock = None
    map_old_path = []
    map_old_path_timer = 0.0
    map_show_report = False
    map_mission_start_ticks = pygame.time.get_ticks()
    map_mission_end_ticks = map_mission_start_ticks
    map_status = "LIVE GEO SENSOR ENABLED — adaptive route replanning active."
    sim_state = "RUNNING"
    announce("MAP MISSION STARTED — LIVE GEO HAZARD SCAN ACTIVE", "safe", 1.8)


# ============================================================
# PROFILE PANEL
# ============================================================


def profile_to_pixel(distance, altitude_ft, total_dist=1, rect=None):
    if rect is None:
        rect = panel_rect
    tx = 0.0 if total_dist <= 0 else distance / total_dist
    ty = 0.0 if MAX_ALTITUDE_FT <= 0 else altitude_ft / MAX_ALTITUDE_FT
    return (rect.left + tx * rect.width,
            rect.bottom - ty * rect.height)


def compute_profile(path, is_map=False, rect=None):
    """Build monotonic route-distance samples for the altitude profile.

    The x-axis is cumulative route cost/distance units and the y-axis is altitude.
    Map mode intentionally uses the same normalized grid-distance convention so
    Grid and Map profiles remain visually comparable.

    `rect` must be the exact same plot rectangle used to draw the axis ticks /
    threat bands (the `graph` rect in draw_altitude_panel), otherwise the
    plotted route line and the current-position marker will not line up with
    the gridlines and hazard bands drawn on top of them.
    """
    if rect is None:
        rect = panel_rect
    if not path:
        return [], [], 1.0

    distances = [0.0]
    for i in range(1, len(path)):
        if is_map:
            step = 1.0 if path[i][2] == path[i - 1][2] else float(VERTICAL_COST)
        else:
            step = float(grid_move_cost(path[i - 1], path[i]))
        distances.append(distances[-1] + max(0.01, step))

    total = max(distances[-1], 1.0)
    points = [profile_to_pixel(distances[i], alt_ft(path[i][2]), total, rect) for i in range(len(path))]
    return points, distances, total


def profile_hazard_intervals(path, distances, hazards_to_draw, obstacles_to_draw,
                              is_map=False):
    """Return threat intervals only where a threat actually intersects the active route.

    This fixes the old graph behavior where every hazard was rendered as a full-width
    horizontal band, even when it was nowhere near the drone's trajectory.
    """
    if not path or not distances:
        return []

    intervals = []
    route_len = len(path)

    def inside(cell, obj, obstacle=False):
        if obstacle:
            if is_map:
                return cell_inside_bounds(cell, map_obstacle_bounds_grid(obj),
                                          obj["z_min"], obj["z_max"])
            return cell_inside_bounds(cell, obstacle_bounds_grid(obj),
                                      obj["z_min"], obj["z_max"])
        if is_map:
            return cell_inside_bounds(cell, map_hazard_bounds_grid(obj),
                                      obj["z_min"], obj["z_max"])
        return cell_inside_bounds(cell, hazard_bounds_grid(obj),
                                  obj["z_min"], obj["z_max"])

    def add_intervals(objects, obstacle=False):
        for obj in objects:
            hit_indices = [i for i, cell in enumerate(path) if inside(cell, obj, obstacle)]
            if not hit_indices:
                continue

            # Split non-contiguous encounters so each threat segment occupies its true
            # x-position on the graph.
            runs = []
            run_start = prev = hit_indices[0]
            for idx in hit_indices[1:]:
                if idx == prev + 1:
                    prev = idx
                    continue
                runs.append((run_start, prev))
                run_start = prev = idx
            runs.append((run_start, prev))

            for i0, i1 in runs:
                d0 = distances[max(0, i0)]
                d1 = distances[min(route_len - 1, i1)]
                # Give a one-step threat a visible width without changing its position.
                if d1 <= d0:
                    if i1 + 1 < route_len:
                        d1 = distances[i1 + 1]
                    else:
                        d1 = min(total_profile_distance(distances), d0 + 0.8)
                intervals.append({
                    "d0": d0,
                    "d1": d1,
                    "z_min": obj["z_min"],
                    "z_max": obj["z_max"],
                    "label": (f"OBSTACLE #{obj['id']}" if obstacle else hazard_label(obj)),
                    "color": (OBSTACLE_FULL_COLOR if obstacle and
                              obj["z_min"] == 0 and obj["z_max"] == GRID_LAYERS - 1
                              else OBSTACLE_PARTIAL_COLOR if obstacle
                              else ash_rgba(obj.get("severity", 2)) if obj["type"] == "ash"
                              else turbulence_rgba(obj.get("severity", "MODERATE"))),
                    "critical": True if obstacle else hazard_requires_replan(obj),
                    "kind": "obstacle" if obstacle else obj["type"],
                })

    add_intervals(hazards_to_draw, obstacle=False)
    add_intervals(obstacles_to_draw, obstacle=True)
    return intervals


def total_profile_distance(distances):
    return distances[-1] if distances else 1.0


def draw_altitude_panel(path, hazards_to_draw, obstacles_to_draw=None, is_map=False,
                        current_segment=0, progress=0.0):
    """Render an altitude-vs-distance profile with route-localized threat regions."""
    if obstacles_to_draw is None:
        obstacles_to_draw = map_obstacles if is_map else obstacles

    panel_box = pygame.Rect(GRID_WIDTH, 0, PANEL_WIDTH, GRID_HEIGHT)
    pygame.draw.rect(screen, UI_PANEL_SHADOW, panel_box.move(-3, 0))
    pygame.draw.rect(screen, PANEL_BG_COLOR, panel_box)
    pygame.draw.line(screen, PANEL_LINE_COLOR, (GRID_WIDTH, 0), (GRID_WIDTH, GRID_HEIGHT), 2)
    draw_hud_corners(screen, pygame.Rect(GRID_WIDTH + 5, 5, PANEL_WIDTH - 10, GRID_HEIGHT - 10),
                     UI_BORDER, length=12, thickness=1)

    title = font_title.render("ALTITUDE / THREAT PROFILE", True, TEXT_MAIN)
    screen.blit(title, (GRID_WIDTH + 14, 20))
    subtitle = font_micro.render("ACTIVE ROUTE  •  THREAT EXTENT / ALTITUDE", True, UI_MUTED)
    screen.blit(subtitle, (GRID_WIDTH + 14, 47))

    # Plot geometry. The existing panel_rect is deliberately kept inset from the
    # altitude labels so the graph remains readable at the current 340 px sidebar.
    # NOTE: `graph` must be computed BEFORE compute_profile() and passed into it,
    # otherwise the plotted route/marker (which used to default to `panel_rect`)
    # remains aligned with the axis ticks / hazard bands below, which
    # were always drawn against `graph`. That mismatch was the root cause of the
    # altitude/threat profile looking broken (route line and current-position
    # marker not lining up with the grid, waypoints or hazard bands).
    graph = pygame.Rect(panel_rect.left, panel_rect.top + 8,
                        panel_rect.width, panel_rect.height - 8)

    points, distances, total = compute_profile(path, is_map=is_map, rect=graph)

    # Grid + altitude ticks.
    pygame.draw.line(screen, PANEL_AXIS_COLOR, graph.bottomleft, graph.topleft, 2)
    pygame.draw.line(screen, PANEL_AXIS_COLOR, graph.bottomleft, graph.bottomright, 2)
    for z in range(GRID_LAYERS):
        ft = alt_ft(z)
        y = graph.bottom - (ft / max(1, MAX_ALTITUDE_FT)) * graph.height
        y = int(y)
        pygame.draw.line(screen, (38, 39, 50), (graph.left, y), (graph.right, y), 1)
        label = font_micro.render(f"{ft:,} ft", True, (150, 150, 165))
        screen.blit(label, (graph.left - label.get_width() - 6, y - label.get_height() // 2))

    # Localize threat bands to the portion of the active route that intersects the threat.
    intervals = profile_hazard_intervals(path, distances, hazards_to_draw,
                                          obstacles_to_draw, is_map=is_map)
    for item in intervals:
        x0 = graph.left + (item["d0"] / total) * graph.width
        x1 = graph.left + (item["d1"] / total) * graph.width
        y_top = graph.bottom - (item["z_max"] / max(1, GRID_LAYERS - 1)) * graph.height
        y_bottom = graph.bottom - (item["z_min"] / max(1, GRID_LAYERS - 1)) * graph.height
        rect = pygame.Rect(int(min(x0, x1)), int(min(y_top, y_bottom)),
                           max(3, int(abs(x1 - x0))), max(4, int(abs(y_bottom - y_top))))
        band = pygame.Surface(rect.size, pygame.SRCALPHA)
        color = item["color"]
        band.fill(color if len(color) == 4 else (*color, 100))
        screen.blit(band, rect.topleft)
        border = WARNING_COLOR if item["critical"] else SAFE_COLOR
        pygame.draw.rect(screen, border, rect, 1)

        # Only label sizeable threat regions to prevent the profile becoming unreadable.
        if rect.width >= 32:
            label = font_micro.render(item["label"], True, TEXT_MAIN)
            screen.blit(label, (rect.x + 3, min(rect.bottom - label.get_height(), rect.y + 2)))

    # Active altitude trajectory.
    if len(points) > 1:
        pygame.draw.lines(screen, PANEL_LINE_COLOR, False,
                          [(int(x), int(y)) for x, y in points], 3)
    elif points:
        p = points[0]
        pygame.draw.circle(screen, PANEL_LINE_COLOR, (int(p[0]), int(p[1])), 4)

    # Mark route waypoints at their actual distance/altitude positions.
    if path:
        waypoint_cells = build_targets_map() if is_map else build_targets_grid()
        waypoint_lookup = {cell: i + 1 for i, cell in enumerate(waypoint_cells[:-1])}
        for idx, cell in enumerate(path):
            if cell in waypoint_lookup:
                d = distances[idx]
                x = graph.left + (d / total) * graph.width
                y = graph.bottom - (cell[2] / max(1, GRID_LAYERS - 1)) * graph.height
                pygame.draw.line(screen, (130, 190, 255), (int(x), graph.top), (int(x), graph.bottom), 1)
                tag = font_micro.render(f"WP{waypoint_lookup[cell]}", True, (175, 210, 255))
                screen.blit(tag, (int(x) + 2, graph.top + 2))

    # Current drone position.
    if points:
        seg = min(max(0, current_segment), max(0, len(points) - 2))
        if len(points) == 1 or seg >= len(points) - 1:
            marker = points[-1]
        else:
            t = ease_in_out(progress)
            marker = (lerp(points[seg][0], points[seg + 1][0], t),
                      lerp(points[seg][1], points[seg + 1][1], t))
        pygame.draw.circle(screen, PANEL_MARKER_COLOR,
                           (int(marker[0]), int(marker[1])), 8)
        pygame.draw.circle(screen, (255, 255, 255),
                           (int(marker[0]), int(marker[1])), 8, 1)
        current_alt = path[min(current_segment, len(path) - 1)][2]
        current_label = font_micro.render(f"CURRENT {alt_ft(current_alt):,} FT", True, PANEL_MARKER_COLOR)
        screen.blit(current_label,
                    (graph.right - current_label.get_width(), max(graph.top, int(marker[1]) - 18)))

    # X-axis ticks / distance scale.
    tick_count = 4
    for k in range(tick_count + 1):
        frac = k / tick_count
        x = int(graph.left + frac * graph.width)
        pygame.draw.line(screen, PANEL_AXIS_COLOR, (x, graph.bottom), (x, graph.bottom + 5), 1)
        val = total * frac
        label = font_micro.render(f"{val:.0f}", True, (140, 145, 130))
        screen.blit(label, (x - label.get_width() // 2, graph.bottom + 7))
    x_label = font_micro.render("ROUTE DISTANCE / COST  →", True, (150, 150, 165))
    screen.blit(x_label, (graph.left, graph.bottom + 23))

    # Compact status key.
    key_y = graph.bottom + 42
    screen.blit(font_micro.render("● ROUTE", True, PANEL_LINE_COLOR), (graph.left, key_y))
    screen.blit(font_micro.render("■ THREAT", True, WARNING_COLOR), (graph.left + 70, key_y))
    screen.blit(font_micro.render("■ LOW / PASS", True, SAFE_COLOR), (graph.left + 142, key_y))


# ============================================================
# DRAWING — GRID
# ============================================================


def draw_grid():
    for x in range(0, GRID_WIDTH + 1, CELL_SIZE):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, GRID_HEIGHT))
    for y in range(0, GRID_HEIGHT + 1, CELL_SIZE):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (GRID_WIDTH, y))


def draw_grid_hazards():
    for h in hazards:
        color = ash_rgba(h["severity"]) if h["type"] == "ash" else turbulence_rgba(h["severity"])
        surf = pygame.Surface((GRID_WIDTH, GRID_HEIGHT), pygame.SRCALPHA)
        rect = pygame.Rect(h["col_start"] * CELL_SIZE, h["row_start"] * CELL_SIZE,
                           h["width"] * CELL_SIZE, h["height"] * CELL_SIZE)
        pygame.draw.rect(surf, color, rect)
        screen.blit(surf, (0, 0))
        border = WARNING_COLOR if hazard_requires_replan(h) else SAFE_COLOR
        pygame.draw.rect(screen, border, rect, 1)
        label = font_micro.render(hazard_label(h), True, TEXT_MAIN)
        if rect.width > 45:
            screen.blit(label, (rect.x + 3, rect.y + 3))


def draw_grid_obstacles():
    for o in obstacles:
        full = o["z_min"] == 0 and o["z_max"] == GRID_LAYERS - 1
        c = OBSTACLE_FULL_COLOR if full else OBSTACLE_PARTIAL_COLOR
        rect = pygame.Rect(o["col_start"] * CELL_SIZE, o["row_start"] * CELL_SIZE,
                           o["width"] * CELL_SIZE, o["height"] * CELL_SIZE)
        pygame.draw.rect(screen, c, rect)
        pygame.draw.rect(screen, (235, 235, 220), rect, 1)
        if rect.width > 45:
            screen.blit(font_micro.render(f"OBSTACLE #{o['id']}", True, TEXT_MAIN),
                        (rect.x + 3, rect.y + 3))


def draw_grid_path():
    if len(grid_pixel_path) > 1:
        if grid_old_path_timer > 0 and grid_old_path:
            old_px = [grid_to_pixel(c) for c in grid_old_path]
            draw_dashed_line(screen, old_px, OLD_PATH_COLOR, 2, dash=8, gap=8)
        color = HAZARD_PATH_COLOR if grid_hazard_ahead else PATH_COLOR
        pygame.draw.lines(screen, color, False, grid_pixel_path, 4)

    # Waypoint markers
    for i, wp in enumerate(waypoints, start=1):
        p = grid_to_pixel(wp)
        pygame.draw.circle(screen, (130, 190, 255), p, 7)
        screen.blit(font_micro.render(f"WP{i}", True, (220, 230, 255)), (p[0] + 8, p[1] - 6))


def draw_grid_drone():
    if not grid_path:
        return
    if grid_current_segment >= len(grid_path) - 1:
        pos = grid_to_pixel(grid_path[-1])
        current_z = grid_path[-1][2]
        climbing = False
    else:
        a = grid_to_pixel(grid_path[grid_current_segment])
        b = grid_to_pixel(grid_path[grid_current_segment + 1])
        t = ease_in_out(grid_progress)
        pos = (int(lerp(a[0], b[0], t)), int(lerp(a[1], b[1], t)))
        current_z = grid_path[grid_current_segment][2]
        climbing = grid_path[grid_current_segment][2] != grid_path[grid_current_segment + 1][2]
    color = DRONE_CLIMB_COLOR if climbing else DRONE_COLOR
    radius = SENSOR_RANGE_CELLS * CELL_SIZE
    sensor = pygame.Surface((int(radius * 2 + 4), int(radius * 2 + 4)), pygame.SRCALPHA)
    pygame.draw.circle(sensor, (*color, 16), (sensor.get_width() // 2, sensor.get_height() // 2), int(radius), 1)
    screen.blit(sensor, (pos[0] - sensor.get_width() // 2, pos[1] - sensor.get_height() // 2))
    glow = pygame.Surface((34, 34), pygame.SRCALPHA)
    pygame.draw.circle(glow, (*color, 55), (17, 17), 17)
    screen.blit(glow, (pos[0] - 17, pos[1] - 17))
    pygame.draw.circle(screen, color, pos, 8)
    pygame.draw.circle(screen, (255, 255, 255), pos, 9, 1)
    screen.blit(font_small.render(f"{alt_ft(current_z):,} ft", True, color),
                (pos[0] + 12, pos[1] - 20))


def draw_grid_setup_markers():
    pygame.draw.circle(screen, START_COLOR, grid_to_pixel(start_cell), 10)
    pygame.draw.circle(screen, (255, 255, 255), grid_to_pixel(start_cell), 11, 1)
    pygame.draw.circle(screen, GOAL_COLOR, grid_to_pixel(goal_cell), 10)
    pygame.draw.circle(screen, (255, 255, 255), grid_to_pixel(goal_cell), 11, 1)
    for i, wp in enumerate(waypoints, start=1):
        p = grid_to_pixel(wp)
        pygame.draw.circle(screen, (130, 190, 255), p, 8)
        screen.blit(font_micro.render(f"WP{i}", True, (230, 235, 255)), (p[0] + 10, p[1] - 6))


def draw_wind_indicator():
    ax, ay = 90, GRID_HEIGHT - 42
    tip = (ax + WIND_DIRECTION[0] * 42, ay + WIND_DIRECTION[1] * 42)
    pygame.draw.line(screen, WIND_ARROW_COLOR, (ax, ay), tip, 3)
    pygame.draw.circle(screen, WIND_ARROW_COLOR, tip, 5)
    screen.blit(font_micro.render("WIND", True, WIND_ARROW_COLOR), (ax - 14, ay + 10))


# ============================================================
# DRAWING — MAP
# ============================================================


def draw_real_map():
    if map_surface is not None:
        screen.fill((20, 24, 30), pygame.Rect(0, 0, MAP_WIDTH, MAP_HEIGHT))
        ox, oy = map_pan_live_offset if map_is_panning else (0, 0)
        screen.blit(map_surface, (int(ox), int(oy)))
    else:
        screen.fill((35, 45, 55), pygame.Rect(0, 0, MAP_WIDTH, MAP_HEIGHT))
        screen.blit(font.render("REAL-WORLD MAP UNAVAILABLE", True, TEXT_MAIN), (24, 24))
        screen.blit(font_small.render("Check internet access and install requests + Pillow.", True, (180, 185, 200)), (24, 56))

    for h in map_hazards:
        p1 = map_latlon_to_screen(h["lat1"], h["lon1"])
        p2 = map_latlon_to_screen(h["lat2"], h["lon2"])
        x1, x2 = sorted((p1[0], p2[0])); y1, y2 = sorted((p1[1], p2[1]))
        rect = pygame.Rect(x1, y1, max(3, x2 - x1), max(3, y2 - y1))
        surf = pygame.Surface(rect.size, pygame.SRCALPHA)
        surf.fill(ash_rgba(h["severity"]) if h["type"] == "ash" else turbulence_rgba(h["severity"]))
        screen.blit(surf, rect.topleft)
        border = WARNING_COLOR if hazard_requires_replan(h) else SAFE_COLOR
        pygame.draw.rect(screen, border, rect, 1)
        if rect.width > 55:
            screen.blit(font_micro.render(hazard_label(h), True, (235, 240, 235)), (rect.x + 3, rect.y + 3))

    for o in map_obstacles:
        p1 = map_latlon_to_screen(o["lat1"], o["lon1"])
        p2 = map_latlon_to_screen(o["lat2"], o["lon2"])
        x1, x2 = sorted((p1[0], p2[0])); y1, y2 = sorted((p1[1], p2[1]))
        rect = pygame.Rect(x1, y1, max(3, x2 - x1), max(3, y2 - y1))
        color = OBSTACLE_FULL_COLOR if o["z_min"] == 0 and o["z_max"] == GRID_LAYERS - 1 else OBSTACLE_PARTIAL_COLOR
        surf = pygame.Surface(rect.size, pygame.SRCALPHA)
        surf.fill((*color, 205))
        screen.blit(surf, rect.topleft)
        pygame.draw.rect(screen, (255, 255, 255), rect, 1)
        if rect.width > 55:
            screen.blit(font_micro.render(f"OBSTACLE #{o['id']}", True, (245, 245, 235)), (rect.x + 3, rect.y + 3))

    if map_old_path_timer > 0 and map_old_path:
        old = [map_latlon_to_screen(*map_cell_to_latlon(c)) for c in map_old_path]
        draw_dashed_line(screen, old, OLD_PATH_COLOR, 2, dash=8, gap=8)
    if len(map_pixel_path) > 1:
        pygame.draw.lines(screen, HAZARD_PATH_COLOR if map_hazard_ahead else MAP_PATH_COLOR,
                          False, map_pixel_path, 4)

    start_screen = map_latlon_to_screen(*map_start_latlon)
    goal_screen = map_latlon_to_screen(*map_goal_latlon)
    pygame.draw.circle(screen, START_COLOR, start_screen, 10)
    pygame.draw.circle(screen, (255, 255, 255), start_screen, 11, 1)
    pygame.draw.circle(screen, GOAL_COLOR, goal_screen, 10)
    pygame.draw.circle(screen, (255, 255, 255), goal_screen, 11, 1)
    for i, wp in enumerate(map_waypoints, start=1):
        p = map_latlon_to_screen(*wp["latlon"])
        pygame.draw.circle(screen, (130, 190, 255), p, 8)
        pygame.draw.circle(screen, (255, 255, 255), p, 9, 1)
        screen.blit(font_micro.render(f"WP{i}", True, (230, 235, 255)), (p[0] + 10, p[1] - 6))

    if sim_state == "RUNNING" and map_path:
        if map_current_segment >= len(map_path) - 1:
            pos = map_pixel_path[-1]
            current_z = map_path[-1][2]
            climbing = False
        else:
            a = map_pixel_path[map_current_segment]
            b = map_pixel_path[map_current_segment + 1]
            t = ease_in_out(map_progress)
            pos = (int(lerp(a[0], b[0], t)), int(lerp(a[1], b[1], t)))
            current_z = map_path[map_current_segment][2]
            climbing = map_path[map_current_segment][2] != map_path[map_current_segment + 1][2]
        color = DRONE_CLIMB_COLOR if climbing else DRONE_COLOR
        radius = SENSOR_RANGE_CELLS * (MAP_WIDTH / MAP_COLS)
        sensor = pygame.Surface((int(radius * 2 + 4), int(radius * 2 + 4)), pygame.SRCALPHA)
        pygame.draw.circle(sensor, (*color, 20), (sensor.get_width() // 2, sensor.get_height() // 2), int(radius), 1)
        screen.blit(sensor, (pos[0] - sensor.get_width() // 2, pos[1] - sensor.get_height() // 2))
        glow = pygame.Surface((34, 34), pygame.SRCALPHA)
        pygame.draw.circle(glow, (*color, 55), (17, 17), 17)
        screen.blit(glow, (pos[0] - 17, pos[1] - 17))
        pygame.draw.circle(screen, color, pos, 8)
        pygame.draw.circle(screen, (255, 255, 255), pos, 9, 1)
        screen.blit(font_small.render(f"{alt_ft(current_z):,} ft", True, color), (pos[0] + 12, pos[1] - 20))

    # Keep only the required OSM attribution on the imagery. The map title is
    # rendered in the right-hand panel instead of over the geographic map.
    screen.blit(font_micro.render("© OpenStreetMap contributors", True, (20, 25, 35)), (12, MAP_HEIGHT - 18))


# ============================================================
# LEGENDS / DASHBOARDS / REPORTS
# ============================================================


def draw_legend(is_map=False):
    items = [(START_COLOR, "Start"), (GOAL_COLOR, "Goal"),
             (DRONE_COLOR, "Drone"), ((130, 190, 255), "Waypoint"),
             (OBSTACLE_FULL_COLOR, "Solid obstacle"),
             (ASH_MODERATE[:3], "Volcanic ash"),
             (TURB_LOW[:3], "Low turbulence (pass-through)"),
             (TURB_HIGH[:3], "High turbulence (avoid)")]
    row_h = 18
    w = 250
    h = 10 + row_h * len(items)
    box = pygame.Rect(GRID_WIDTH - w - 12, 12, w, h)
    layer = pygame.Surface(box.size, pygame.SRCALPHA)
    layer.fill((*LEGEND_BG_COLOR, 205))
    screen.blit(layer, box.topleft)
    pygame.draw.rect(screen, UI_BORDER, box, 1, border_radius=UI_RADIUS)
    draw_hud_corners(screen, box, UI_BORDER, length=7, thickness=1)
    y = 8
    for color, label in items:
        pygame.draw.rect(screen, color, (box.left + 8, box.top + y + 3, 11, 11))
        screen.blit(font_micro.render(label, True, TEXT_MAIN), (box.left + 26, box.top + y))
        y += row_h


def draw_dashboard(is_map=False):
    pygame.draw.rect(screen, DASHBOARD_BG_COLOR, dashboard_rect)
    pygame.draw.line(screen, PANEL_LINE_COLOR, (0, GRID_HEIGHT), (WIDTH, GRID_HEIGHT), 2)
    pygame.draw.line(screen, UI_BORDER_SOFT, (0, GRID_HEIGHT + 3), (WIDTH, GRID_HEIGHT + 3), 1)
    if is_map:
        elapsed = mission_elapsed(map_mission_start_ticks, map_mission_end_ticks, map_mission_complete)
        cost = map_path_cost(map_path)
        alt_changes = path_altitude_changes(map_path)
        crossed = map_hazard_cells_crossed
        replans = map_replan_count
        replan_ms = map_last_replan_ms
        complete = map_mission_complete
        status = "MISSION COMPLETE" if complete else ("HAZARD DETECTED" if map_hazard_ahead else "CLEAR")
    else:
        elapsed = mission_elapsed(grid_mission_start_ticks, grid_mission_end_ticks, grid_mission_complete)
        cost = path_cost_grid(grid_path)
        alt_changes = path_altitude_changes(grid_path)
        crossed = grid_hazard_cells_crossed
        replans = grid_replan_count
        replan_ms = grid_last_replan_ms
        complete = grid_mission_complete
        status = "MISSION COMPLETE" if complete else ("HAZARD DETECTED" if grid_hazard_ahead else "CLEAR")

    cols = [
        [("MISSION TIME", f"{elapsed:.1f} s"), ("PATH COST", str(cost)), ("ALTITUDE CHANGES", str(alt_changes))],
        [("HAZARD CELLS CROSSED", str(crossed)), ("REPLANS", str(replans)), ("LAST REPLAN", f"{replan_ms:.2f} ms")],
        [("EST. ENERGY", f"{cost * ENERGY_PER_COST_UNIT:.1f} units"), ("ROUTE STATUS", status), ("SPEED", DRONE_SPEED_LABELS[DRONE_SPEED_INDEX])]
    ]
    for x, entries in zip((16, 294, 590), cols):
        y = GRID_HEIGHT + 9
        for label, value in entries:
            screen.blit(font_micro.render(label, True, DASHBOARD_LABEL_COLOR), (x, y))
            value_color = DASHBOARD_VALUE_COLOR
            if label == "ROUTE STATUS":
                value_color = SAFE_COLOR if complete else (WARNING_COLOR if "HAZARD" in value else DASHBOARD_VALUE_COLOR)
            screen.blit(font_small.render(value, True, value_color), (x, y + 14))
            y += 31
    screen.blit(font_micro.render("A* is invoked only after live hazard detection. Energy/cost are illustrative.", True, UI_MUTED), (16, HEIGHT - 17))
    screen.blit(font_micro.render("F11 fullscreen  |  R reset flight  |  P report  |  M map/grid", True, UI_MUTED), (WIDTH - 330, HEIGHT - 17))


def draw_report(is_map=False):
    overlay = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 190))
    screen.blit(overlay, (0, 0))
    if is_map:
        elapsed = mission_elapsed(map_mission_start_ticks, map_mission_end_ticks, map_mission_complete)
        rows = [
            ("Mode", "REAL-WORLD MAP"),
            ("Mission Time", f"{elapsed:.1f} s"),
            ("Replans", str(map_replan_count)),
            ("Last Replan", f"{map_last_replan_ms:.2f} ms"),
            ("Hazard Cells Crossed", str(map_hazard_cells_crossed)),
            ("Altitude Changes", str(path_altitude_changes(map_path))),
            ("Path Cost", str(map_path_cost(map_path))),
            ("Waypoints", str(len(map_waypoints))),
        ]
    else:
        elapsed = mission_elapsed(grid_mission_start_ticks, grid_mission_end_ticks, grid_mission_complete)
        rows = [
            ("Mode", "GRID"),
            ("Mission Time", f"{elapsed:.1f} s"),
            ("Replans", str(grid_replan_count)),
            ("Last Replan", f"{grid_last_replan_ms:.2f} ms"),
            ("Hazard Cells Crossed", str(grid_hazard_cells_crossed)),
            ("Altitude Changes", str(path_altitude_changes(grid_path))),
            ("Path Cost", str(path_cost_grid(grid_path))),
            ("Waypoints", str(len(waypoints))),
        ]
    card_w = 480
    card_h = 64 + 30 * len(rows)
    card = pygame.Rect((WIDTH - card_w) // 2, (HEIGHT - card_h) // 2, card_w, card_h)
    pygame.draw.rect(screen, UI_PANEL_SHADOW, card.move(5, 5), border_radius=UI_RADIUS)
    pygame.draw.rect(screen, PANEL_BG_COLOR, card, border_radius=UI_RADIUS)
    pygame.draw.rect(screen, PANEL_LINE_COLOR, card, 2, border_radius=UI_RADIUS)
    draw_hud_corners(screen, card, BUTTON_ACTIVE_COLOR, length=18)
    title = font_title.render("MISSION PERFORMANCE REPORT", True, TEXT_MAIN)
    screen.blit(title, (card.centerx - title.get_width() // 2, card.y + 14))
    y = card.y + 55
    for label, value in rows:
        screen.blit(font_small.render(label, True, DASHBOARD_LABEL_COLOR), (card.x + 22, y))
        v = font_small.render(value, True, DASHBOARD_VALUE_COLOR)
        screen.blit(v, (card.right - 22 - v.get_width(), y))
        y += 30
    screen.blit(font_micro.render("P to close", True, (165, 165, 180)), (card.centerx - 24, card.bottom - 20))


def draw_banner_and_common_text(is_map=False):
    # Map Mode owns the full left canvas for geographic imagery. Do not paint
    # the global application heading/status over the map. Map setup/running
    # panels already provide their own headings, while the event banner is
    # intentionally retained for dramatic real-time hazard notifications.
    if is_map:
        draw_event_banner()
        return

    title = "Adaptive Flight-Path Replanner"
    subtitle = "REAL-TIME COURSE CORRECTION • SENSOR RANGE 6 CELLS"
    screen.blit(font_title.render(title, True, TEXT_MAIN), (12, 9))
    screen.blit(font_micro.render(subtitle, True, UI_MUTED), (12, 33))
    status = grid_setup_message if sim_state == "SETUP" else "LIVE SENSOR: scanning ahead"
    screen.blit(font_tiny.render(status[:100], True, UI_MUTED), (12, 56))
    draw_event_banner()


dashboard_rect = pygame.Rect(0, GRID_HEIGHT, WIDTH, DASHBOARD_HEIGHT)


# ============================================================
# SCROLLABLE SETUP PANELS
# ============================================================


def button_on_surface(panel, buttons, x, y, label, action, w=306, h=28,
                      target=None, active=False, color=None, text_color=TEXT_MAIN):
    # Presentation-only helper: same rect/action bookkeeping, upgraded command-console styling.
    rect = pygame.Rect(x, y, w, h)
    bg = color if color is not None else (BUTTON_ACTIVE_COLOR if active else BUTTON_COLOR)
    border = PANEL_LINE_COLOR if active else UI_BORDER
    pygame.draw.rect(panel, UI_PANEL_SHADOW, rect.move(2, 2), border_radius=UI_RADIUS)
    pygame.draw.rect(panel, bg, rect, border_radius=UI_RADIUS)
    pygame.draw.rect(panel, border, rect, 1, border_radius=UI_RADIUS)
    # Small left status marker makes active controls immediately readable.
    if active:
        pygame.draw.rect(panel, PANEL_MARKER_COLOR, (rect.x + 5, rect.y + 7, 3, rect.height - 14), border_radius=1)
        tx = rect.x + 14
    else:
        tx = rect.x + 9
    txt = font_small.render(label, True, text_color)
    panel.blit(txt, (tx, rect.y + (rect.height - txt.get_height()) // 2))
    buttons.append((rect, action, target))
    return y + h + 6


def label_on_surface(panel, x, y, text, color=(150, 150, 165), dy=16):
    panel.blit(font_tiny.render(text, True, color), (x, y))
    return y + dy


def draw_scrollbar(scroll, content_height, x):
    visible_h = GRID_HEIGHT
    if content_height <= visible_h:
        return
    track = pygame.Rect(x, 6, 6, visible_h - 12)
    pygame.draw.rect(screen, UI_BORDER_SOFT, track, border_radius=3)
    thumb_h = max(44, int(track.height * visible_h / content_height))
    max_scroll = content_height - visible_h
    thumb_y = track.y + int((track.height - thumb_h) * (scroll / max_scroll))
    pygame.draw.rect(screen, BUTTON_ACTIVE_COLOR,
                     (track.x, thumb_y, track.width, thumb_h), border_radius=3)


def draw_grid_setup_panel():
    global grid_panel_scroll, grid_panel_content_height
    panel = pygame.Surface((PANEL_WIDTH, 3600), pygame.SRCALPHA)
    panel.fill(PANEL_BG_COLOR)
    pygame.draw.rect(panel, UI_BORDER_SOFT, (5, 5, PANEL_WIDTH - 10, 3590), 1, border_radius=UI_RADIUS)
    buttons = []
    x, y = 12, 12

    panel.blit(font_title.render("GRID MISSION PLANNING", True, TEXT_MAIN), (x, y)); y += 30
    y = label_on_surface(panel, x, y, "Launch a nominal path. A* waits until the live sensor sees a threat.")
    y = label_on_surface(panel, x, y, f"Altitude levels: 0–{MAX_ALTITUDE_FT:,} ft / {GRID_LAYERS} levels")
    y += 6

    y = button_on_surface(panel, buttons, x, y, "PLACE START", "grid_place_start", active=grid_setup_mode == "placing_start")
    y = label_on_surface(panel, x + 4, y - 22, f"START ({start_cell[0]}, {start_cell[1]}, {alt_ft(start_cell[2]):,} ft)", dy=16)
    y = button_on_surface(panel, buttons, x, y, "PLACE GOAL", "grid_place_goal", active=grid_setup_mode == "placing_goal")
    y = label_on_surface(panel, x + 4, y - 22, f"GOAL ({goal_cell[0]}, {goal_cell[1]}, {alt_ft(goal_cell[2]):,} ft)", dy=16)
    y = button_on_surface(panel, buttons, x, y, f"START ALTITUDE: {alt_ft(start_cell[2]):,} FT", "grid_cycle_start_alt")
    y = button_on_surface(panel, buttons, x, y, f"GOAL ALTITUDE: {alt_ft(goal_cell[2]):,} FT", "grid_cycle_goal_alt")
    y += 2

    panel.blit(font_small.render("WAYPOINT SYSTEM", True, TEXT_MAIN), (x, y)); y += 21
    y = button_on_surface(panel, buttons, x, y, f"WAYPOINT ALTITUDE: {alt_ft(waypoint_altitude):,} FT", "grid_cycle_waypoint_alt")
    y = button_on_surface(panel, buttons, x, y, "+ ADD WAYPOINT — click grid", "grid_add_waypoint", active=grid_setup_mode == "placing_waypoint")
    for i, wp in enumerate(waypoints, start=1):
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_COLOR, row); pygame.draw.rect(panel, PANEL_AXIS_COLOR, row, 1)
        panel.blit(font_micro.render(f"WP{i}: ({wp[0]}, {wp[1]}) @ {alt_ft(wp[2]):,} ft", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "noop", None))
        rm = pygame.Rect(x + 280, y, 26, 24)
        pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "grid_remove_waypoint", i - 1))
        y += 29
    if not waypoints:
        y = label_on_surface(panel, x + 4, y, "No waypoints — mission goes directly to GOAL.", dy=16)
    y += 7

    panel.blit(font_small.render("ADD HAZARDS / OBSTACLES", True, TEXT_MAIN), (x, y)); y += 22
    y = button_on_surface(panel, buttons, x, y, "+ ADD VOLCANIC ASH ZONE", "grid_add_ash", active=grid_pending_placement == "ash")
    y = button_on_surface(panel, buttons, x, y, "+ ADD TURBULENCE ZONE", "grid_add_turbulence", active=grid_pending_placement == "turbulence")
    y = button_on_surface(panel, buttons, x, y, "+ ADD FULL-HEIGHT OBSTACLE", "grid_add_obstacle_full", active=grid_pending_placement == "obstacle_full", color=BUTTON_OBSTACLE_COLOR if grid_pending_placement != "obstacle_full" else None)
    y = button_on_surface(panel, buttons, x, y, "+ ADD PARTIAL-ALT OBSTACLE", "grid_add_obstacle_partial", active=grid_pending_placement == "obstacle_partial", color=BUTTON_OBSTACLE_COLOR if grid_pending_placement != "obstacle_partial" else None)
    y += 2

    if grid_setup_mode != "idle":
        text = {
            "placing_start": "CLICK A CELL TO PLACE START",
            "placing_goal": "CLICK A CELL TO PLACE GOAL",
            "placing_waypoint": f"CLICK A CELL FOR WP @ {alt_ft(waypoint_altitude):,} FT",
            "corner1": "CLICK FIRST CORNER OF ZONE",
            "corner2": "CLICK SECOND CORNER OF ZONE",
        }.get(grid_setup_mode, "")
        y = label_on_surface(panel, x, y, text, PANEL_MARKER_COLOR, dy=18)

    panel.blit(font_small.render(f"HAZARDS ({len(hazards)})", True, TEXT_MAIN), (x, y)); y += 21
    for h in hazards:
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_ACTIVE_COLOR if h["id"] == selected_hazard_id else BUTTON_COLOR, row)
        panel.blit(font_micro.render(f"#{h['id']} {hazard_label(h)}", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "grid_select_hazard", h["id"]))
        rm = pygame.Rect(x + 280, y, 26, 24); pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "grid_remove_hazard", h["id"]))
        y += 29
    if not hazards:
        y = label_on_surface(panel, x + 4, y, "None", dy=16)

    hsel = get_selected_hazard()
    if hsel:
        y = label_on_surface(panel, x, y + 2, f"EDITING HAZARD #{hsel['id']}", UI_SECTION, 16)
        y = button_on_surface(panel, buttons, x, y, f"SEVERITY / EXTENT: {hazard_label(hsel).replace('ASH D', 'ASH D')}", "grid_cycle_hazard_severity")
        y = button_on_surface(panel, buttons, x, y, f"MIN ALTITUDE: {alt_ft(hsel['z_min']):,} FT", "grid_cycle_hazard_minz")
        y = button_on_surface(panel, buttons, x, y, f"MAX ALTITUDE: {alt_ft(hsel['z_max']):,} FT", "grid_cycle_hazard_maxz")
    y += 4
    panel.blit(font_small.render(f"OBSTACLES ({len(obstacles)})", True, TEXT_MAIN), (x, y)); y += 21
    for o in obstacles:
        kind = "FULL" if o["z_min"] == 0 and o["z_max"] == GRID_LAYERS - 1 else f"{alt_ft(o['z_min']):,}-{alt_ft(o['z_max']):,}ft"
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_ACTIVE_COLOR if o["id"] == selected_obstacle_id else BUTTON_COLOR, row)
        panel.blit(font_micro.render(f"#{o['id']} OBSTACLE [{kind}]", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "grid_select_obstacle", o["id"]))
        rm = pygame.Rect(x + 280, y, 26, 24); pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "grid_remove_obstacle", o["id"]))
        y += 29
    if not obstacles:
        y = label_on_surface(panel, x + 4, y, "None", dy=16)

    osel = get_selected_obstacle()
    if osel:
        y = label_on_surface(panel, x, y + 2, f"EDITING OBSTACLE #{osel['id']}", UI_SECTION, 16)
        y = button_on_surface(panel, buttons, x, y, f"MIN ALTITUDE: {alt_ft(osel['z_min']):,} FT", "grid_cycle_obstacle_minz")
        y = button_on_surface(panel, buttons, x, y, f"MAX ALTITUDE: {alt_ft(osel['z_max']):,} FT", "grid_cycle_obstacle_maxz")

    y += 7
    panel.blit(font_small.render("DEMO CONTROLS", True, TEXT_MAIN), (x, y)); y += 22
    y = button_on_surface(panel, buttons, x, y, "RANDOMIZE SCENARIO", "grid_randomize", color=(65, 82, 45))
    y = button_on_surface(panel, buttons, x, y, "RESET FLIGHT (KEEP SCENARIO)", "grid_reset")
    y = button_on_surface(panel, buttons, x, y, "NEW SCENARIO / CLEAR ALL", "grid_clear", color=BUTTON_DANGER_COLOR)
    y = button_on_surface(panel, buttons, x, y, f"DRONE SPEED: {DRONE_SPEED_LABELS[DRONE_SPEED_INDEX]}", "cycle_drone_speed")

    y += 6
    panel.blit(font_small.render("MISSION", True, TEXT_MAIN), (x, y)); y += 22
    y = button_on_surface(panel, buttons, x, y, "LAUNCH LIVE-REPLAN MISSION", "grid_start_sim", color=BUTTON_START_COLOR, text_color=(255, 255, 255), h=36)
    y = button_on_surface(panel, buttons, x, y, "SWITCH TO REAL-WORLD MAP", "switch_map")
    y = label_on_surface(panel, x, y, "M: toggle mode • F11 fullscreen • wheel: panel scroll", UI_MUTED, dy=16)

    grid_panel_content_height = y + 12
    max_scroll = max(0, grid_panel_content_height - GRID_HEIGHT)
    grid_panel_scroll = max(0, min(grid_panel_scroll, max_scroll))
    screen.blit(panel, (GRID_WIDTH, 0), area=pygame.Rect(0, grid_panel_scroll, PANEL_WIDTH, GRID_HEIGHT))
    pygame.draw.line(screen, PANEL_LINE_COLOR, (GRID_WIDTH, 0), (GRID_WIDTH, GRID_HEIGHT), 2)
    draw_scrollbar(grid_panel_scroll, grid_panel_content_height, GRID_WIDTH + PANEL_WIDTH - 7)

    visible = pygame.Rect(GRID_WIDTH, 0, PANEL_WIDTH, GRID_HEIGHT)
    return [{"rect": pygame.Rect(GRID_WIDTH + r.x, r.y - grid_panel_scroll, r.w, r.h),
             "action": a, "target": t}
            for r, a, t in buttons
            if pygame.Rect(GRID_WIDTH + r.x, r.y - grid_panel_scroll, r.w, r.h).colliderect(visible)]


def draw_map_setup_panel():
    global map_panel_scroll, map_panel_content_height
    panel = pygame.Surface((PANEL_WIDTH, 4200), pygame.SRCALPHA)
    panel.fill(PANEL_BG_COLOR)
    pygame.draw.rect(panel, UI_BORDER_SOFT, (5, 5, PANEL_WIDTH - 10, 4190), 1, border_radius=UI_RADIUS)
    buttons = []
    x, y = 12, 12
    panel.blit(font_title.render("REAL-WORLD MAP PLANNING", True, TEXT_MAIN), (x, y)); y += 30
    y = label_on_surface(panel, x, y, "OSM map + synthetic sensor zones. No full route is precomputed.")
    y = label_on_surface(panel, x, y, "Wheel over MAP = zoom • drag MAP = pan • wheel over PANEL = scroll")
    y += 5

    y = button_on_surface(panel, buttons, x, y, "REFRESH MAP TILES", "map_refresh")
    y = button_on_surface(panel, buttons, x, y, "ZOOM IN", "map_zoom_in")
    y = button_on_surface(panel, buttons, x, y, "ZOOM OUT", "map_zoom_out")

    y += 4
    panel.blit(font_small.render("START / GOAL", True, TEXT_MAIN), (x, y)); y += 21
    y = button_on_surface(panel, buttons, x, y, "PLACE START ON MAP", "map_place_start", active=map_setup_mode == "placing_start")
    y = label_on_surface(panel, x + 4, y - 22, f"START {map_start_latlon[0]:.4f}, {map_start_latlon[1]:.4f} @ {alt_ft(map_start_z):,} ft", dy=16)
    y = button_on_surface(panel, buttons, x, y, "PLACE GOAL ON MAP", "map_place_goal", active=map_setup_mode == "placing_goal")
    y = label_on_surface(panel, x + 4, y - 22, f"GOAL {map_goal_latlon[0]:.4f}, {map_goal_latlon[1]:.4f} @ {alt_ft(map_goal_z):,} ft", dy=16)
    y = button_on_surface(panel, buttons, x, y, f"START ALTITUDE: {alt_ft(map_start_z):,} FT", "map_cycle_start_alt")
    y = button_on_surface(panel, buttons, x, y, f"GOAL ALTITUDE: {alt_ft(map_goal_z):,} FT", "map_cycle_goal_alt")

    y += 5
    panel.blit(font_small.render("WAYPOINT SYSTEM", True, TEXT_MAIN), (x, y)); y += 21
    y = button_on_surface(panel, buttons, x, y, f"WAYPOINT ALTITUDE: {alt_ft(map_waypoint_altitude):,} FT", "map_cycle_waypoint_alt")
    y = button_on_surface(panel, buttons, x, y, "+ ADD WAYPOINT — click map", "map_add_waypoint", active=map_setup_mode == "placing_waypoint")
    for i, wp in enumerate(map_waypoints, start=1):
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_COLOR, row); pygame.draw.rect(panel, PANEL_AXIS_COLOR, row, 1)
        panel.blit(font_micro.render(f"WP{i}: {wp['latlon'][0]:.4f},{wp['latlon'][1]:.4f} @ {alt_ft(wp['z']):,}ft", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "noop", None))
        rm = pygame.Rect(x + 280, y, 26, 24); pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "map_remove_waypoint", i - 1))
        y += 29
    if not map_waypoints:
        y = label_on_surface(panel, x + 4, y, "No waypoints — direct to GOAL.", dy=16)

    y += 7
    panel.blit(font_small.render("ADD HAZARDS / OBSTACLES", True, TEXT_MAIN), (x, y)); y += 22
    y = button_on_surface(panel, buttons, x, y, "+ ADD VOLCANIC ASH ZONE", "map_add_ash", active=map_pending_placement == "ash")
    y = button_on_surface(panel, buttons, x, y, "+ ADD TURBULENCE ZONE", "map_add_turbulence", active=map_pending_placement == "turbulence")
    y = button_on_surface(panel, buttons, x, y, "+ ADD FULL-HEIGHT OBSTACLE", "map_add_obstacle_full", active=map_pending_placement == "obstacle_full", color=BUTTON_OBSTACLE_COLOR if map_pending_placement != "obstacle_full" else None)
    y = button_on_surface(panel, buttons, x, y, "+ ADD PARTIAL-ALT OBSTACLE", "map_add_obstacle_partial", active=map_pending_placement == "obstacle_partial", color=BUTTON_OBSTACLE_COLOR if map_pending_placement != "obstacle_partial" else None)

    if map_setup_mode != "idle":
        text = {"placing_start": "CLICK MAP TO PLACE START",
                "placing_goal": "CLICK MAP TO PLACE GOAL",
                "placing_waypoint": f"CLICK MAP FOR WP @ {alt_ft(map_waypoint_altitude):,} FT",
                "corner1": "CLICK FIRST CORNER OF ZONE",
                "corner2": "CLICK SECOND CORNER OF ZONE"}.get(map_setup_mode, "")
        y = label_on_surface(panel, x, y, text, PANEL_MARKER_COLOR, 18)

    panel.blit(font_small.render(f"HAZARDS ({len(map_hazards)})", True, TEXT_MAIN), (x, y)); y += 21
    for h in map_hazards:
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_ACTIVE_COLOR if h["id"] == selected_map_hazard_id else BUTTON_COLOR, row)
        panel.blit(font_micro.render(f"#{h['id']} {hazard_label(h)}", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "map_select_hazard", h["id"]))
        rm = pygame.Rect(x + 280, y, 26, 24); pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "map_remove_hazard", h["id"]))
        y += 29
    if not map_hazards:
        y = label_on_surface(panel, x + 4, y, "None", dy=16)

    hsel = get_selected_map_hazard()
    if hsel:
        y = label_on_surface(panel, x, y + 2, f"EDITING HAZARD #{hsel['id']}", UI_SECTION, 16)
        y = button_on_surface(panel, buttons, x, y, f"SEVERITY / EXTENT: {hazard_label(hsel)}", "map_cycle_hazard_severity")
        y = button_on_surface(panel, buttons, x, y, f"MIN ALTITUDE: {alt_ft(hsel['z_min']):,} FT", "map_cycle_hazard_minz")
        y = button_on_surface(panel, buttons, x, y, f"MAX ALTITUDE: {alt_ft(hsel['z_max']):,} FT", "map_cycle_hazard_maxz")
    y += 4
    panel.blit(font_small.render(f"OBSTACLES ({len(map_obstacles)})", True, TEXT_MAIN), (x, y)); y += 21
    for o in map_obstacles:
        kind = "FULL" if o["z_min"] == 0 and o["z_max"] == GRID_LAYERS - 1 else f"{alt_ft(o['z_min']):,}-{alt_ft(o['z_max']):,}ft"
        row = pygame.Rect(x, y, 276, 24)
        pygame.draw.rect(panel, BUTTON_ACTIVE_COLOR if o["id"] == selected_map_obstacle_id else BUTTON_COLOR, row)
        panel.blit(font_micro.render(f"#{o['id']} OBSTACLE [{kind}]", True, TEXT_MAIN), (x + 5, y + 6))
        buttons.append((row, "map_select_obstacle", o["id"]))
        rm = pygame.Rect(x + 280, y, 26, 24); pygame.draw.rect(panel, BUTTON_DANGER_COLOR, rm)
        panel.blit(font_micro.render("X", True, TEXT_MAIN), (x + 289, y + 6))
        buttons.append((rm, "map_remove_obstacle", o["id"]))
        y += 29
    if not map_obstacles:
        y = label_on_surface(panel, x + 4, y, "None", dy=16)

    osel = get_selected_map_obstacle()
    if osel:
        y = label_on_surface(panel, x, y + 2, f"EDITING OBSTACLE #{osel['id']}", UI_SECTION, 16)
        y = button_on_surface(panel, buttons, x, y, f"MIN ALTITUDE: {alt_ft(osel['z_min']):,} FT", "map_cycle_obstacle_minz")
        y = button_on_surface(panel, buttons, x, y, f"MAX ALTITUDE: {alt_ft(osel['z_max']):,} FT", "map_cycle_obstacle_maxz")

    y += 8
    panel.blit(font_small.render("DEMO CONTROLS", True, TEXT_MAIN), (x, y)); y += 22
    y = button_on_surface(panel, buttons, x, y, "RANDOMIZE GEO SCENARIO", "map_randomize", color=(65, 82, 45))
    y = button_on_surface(panel, buttons, x, y, "RESET FLIGHT (KEEP SCENARIO)", "map_reset")
    y = button_on_surface(panel, buttons, x, y, "CLEAR & REFRESH SCENARIO", "map_clear", color=BUTTON_DANGER_COLOR)
    y = button_on_surface(panel, buttons, x, y, f"DRONE SPEED: {DRONE_SPEED_LABELS[DRONE_SPEED_INDEX]}", "cycle_drone_speed")

    y += 6
    y = button_on_surface(panel, buttons, x, y, "LAUNCH LIVE-REPLAN MAP MISSION", "map_start_sim", color=BUTTON_START_COLOR, text_color=(255, 255, 255), h=36)
    y = button_on_surface(panel, buttons, x, y, "SWITCH TO GRID MODE", "switch_grid")
    y = label_on_surface(panel, x, y, map_status[:90], WARNING_COLOR if "fail" in map_status.lower() else UI_MUTED, 16)

    map_panel_content_height = y + 12
    max_scroll = max(0, map_panel_content_height - GRID_HEIGHT)
    map_panel_scroll = max(0, min(map_panel_scroll, max_scroll))
    screen.blit(panel, (GRID_WIDTH, 0), area=pygame.Rect(0, map_panel_scroll, PANEL_WIDTH, GRID_HEIGHT))
    pygame.draw.line(screen, PANEL_LINE_COLOR, (GRID_WIDTH, 0), (GRID_WIDTH, GRID_HEIGHT), 2)
    draw_scrollbar(map_panel_scroll, map_panel_content_height, GRID_WIDTH + PANEL_WIDTH - 7)
    visible = pygame.Rect(GRID_WIDTH, 0, PANEL_WIDTH, GRID_HEIGHT)
    return [{"rect": pygame.Rect(GRID_WIDTH + r.x, r.y - map_panel_scroll, r.w, r.h),
             "action": a, "target": t}
            for r, a, t in buttons
            if pygame.Rect(GRID_WIDTH + r.x, r.y - map_panel_scroll, r.w, r.h).colliderect(visible)]


# ============================================================
# START / MODE SELECTION SCREEN
# ============================================================


def draw_start_screen():
    """Boot/loading screen shown before either simulation mode."""
    global home_screen_ready
    elapsed = time.perf_counter() - home_screen_started
    home_screen_ready = elapsed >= 1.15

    screen.fill((7, 12, 8))

    # Background grid / HUD texture
    for x in range(0, WIDTH, 40):
        pygame.draw.line(screen, (18, 35, 21), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 40):
        pygame.draw.line(screen, (18, 35, 21), (0, y), (WIDTH, y), 1)

    # Outer command-console frame.
    frame = pygame.Rect(28, 24, WIDTH - 56, HEIGHT - 48)
    pygame.draw.rect(screen, UI_BORDER_SOFT, frame, 1, border_radius=UI_RADIUS)
    draw_hud_corners(screen, frame, PANEL_LINE_COLOR, length=18, thickness=2)

    cx, cy = WIDTH // 2, 220
    pulse = 0.5 + 0.5 * math.sin(time.perf_counter() * 3.0)
    ring_r = int(74 + pulse * 7)
    pygame.draw.circle(screen, (45, 80, 48), (cx, cy), ring_r, 2)
    pygame.draw.circle(screen, BUTTON_ACTIVE_COLOR, (cx, cy), 52, 2)
    pygame.draw.circle(screen, (108, 176, 84), (cx, cy), 8)
    pygame.draw.line(screen, (108, 176, 84), (cx - 34, cy), (cx + 34, cy), 1)
    pygame.draw.line(screen, (108, 176, 84), (cx, cy - 34), (cx, cy + 34), 1)

    title = font_title.render("ADAPTIVE DRONE FLIGHT-REPLANNER", True, TEXT_MAIN)
    subtitle = font_small.render("VOLCANIC ASH / TURBULENCE / OBSTACLE AVOIDANCE", True, UI_MUTED)
    screen.blit(title, (cx - title.get_width() // 2, 86))
    screen.blit(subtitle, (cx - subtitle.get_width() // 2, 118))

    status_messages = [
        "INITIALIZING FLIGHT-PLANNING CORE",
        "LOADING SENSOR / THREAT MODEL",
        "READY FOR MISSION MODE SELECTION",
    ]
    idx = min(2, int(elapsed / 0.38))
    status = status_messages[idx]
    status_s = font_small.render(status, True, BUTTON_ACTIVE_COLOR if home_screen_ready else REPLAN_FLASH_COLOR)
    screen.blit(status_s, (cx - status_s.get_width() // 2, 326))

    bar = pygame.Rect(cx - 240, 360, 480, 12)
    pygame.draw.rect(screen, (25, 35, 27), bar)
    fill_ratio = min(1.0, elapsed / 1.15)
    pygame.draw.rect(screen, BUTTON_ACTIVE_COLOR, (bar.x, bar.y, int(bar.width * fill_ratio), bar.height))
    pygame.draw.rect(screen, PANEL_AXIS_COLOR, bar, 1)

    buttons = []
    if home_screen_ready:
        button_w, button_h, gap = 300, 62, 28
        y = 440
        left = pygame.Rect(cx - button_w - gap // 2, y, button_w, button_h)
        right = pygame.Rect(cx + gap // 2, y, button_w, button_h)
        pygame.draw.rect(screen, BUTTON_ACTIVE_COLOR, left)
        pygame.draw.rect(screen, BUTTON_START_COLOR, right)
        pygame.draw.rect(screen, (215, 230, 210), left, 1)
        pygame.draw.rect(screen, (255, 255, 255), right, 1)
        lt = font.render("GRID MODE - PRESS G", True, (235, 240, 225))
        rt = font.render("REAL-WORLD MAP MODE - PRESS M", True, (255, 255, 255))
        screen.blit(lt, (left.centerx - lt.get_width() // 2, left.centery - lt.get_height() // 2))
        screen.blit(rt, (right.centerx - rt.get_width() // 2, right.centery - rt.get_height() // 2))
        buttons = [
            {"rect": left, "action": "home_grid", "target": None},
            {"rect": right, "action": "home_map", "target": None},
        ]

        hint = font_tiny.render("Choose a simulation environment to begin", True, UI_MUTED)
        screen.blit(hint, (cx - hint.get_width() // 2, 525))
    else:
        loading = font_tiny.render("SYSTEM CHECK IN PROGRESS...", True, (120, 140, 120))
        screen.blit(loading, (cx - loading.get_width() // 2, 412))

    footer = font_micro.render("ADAPTIVE REPLANNING DEMONSTRATOR  •  HACKATHON BUILD", True, UI_MUTED)
    screen.blit(footer, (cx - footer.get_width() // 2, HEIGHT - 36))
    return buttons


# ============================================================
# SETUP CLICKS / ACTION HANDLERS
# ============================================================


def handle_grid_map_click(pos):
    global grid_setup_mode, grid_pending_corner1, grid_pending_placement
    global start_cell, goal_cell, selected_hazard_id, selected_obstacle_id, next_hazard_id, next_obstacle_id
    if not (0 <= pos[0] < GRID_WIDTH and 0 <= pos[1] < GRID_HEIGHT):
        return
    col = max(0, min(GRID_COLS - 1, int(pos[0] // CELL_SIZE)))
    row = max(0, min(GRID_ROWS - 1, int(pos[1] // CELL_SIZE)))
    if grid_setup_mode == "placing_start":
        start_cell = (col, row, start_cell[2])
        grid_setup_mode = "idle"
        return
    if grid_setup_mode == "placing_goal":
        goal_cell = (col, row, goal_cell[2])
        grid_setup_mode = "idle"
        return
    if grid_setup_mode == "placing_waypoint":
        waypoints.append((col, row, waypoint_altitude))
        grid_setup_mode = "idle"
        announce(f"WAYPOINT {len(waypoints)} ADDED — {alt_ft(waypoint_altitude):,} FT", "info", 1.0)
        return
    if grid_setup_mode == "corner1":
        grid_pending_corner1 = (col, row)
        grid_setup_mode = "corner2"
        return
    if grid_setup_mode == "corner2" and grid_pending_corner1 is not None:
        c1 = grid_pending_corner1
        c2 = (col, row)
        cs, ce = sorted((c1[0], c2[0])); rs, re = sorted((c1[1], c2[1]))
        width, height = ce - cs + 1, re - rs + 1
        if grid_pending_placement in ("ash", "turbulence"):
            htype = grid_pending_placement
            h = make_hazard(next_hazard_id, htype, cs, width, rs, height,
                            0, 0, random.randint(1, 3) if htype == "ash" else random.choice(TURBULENCE_LEVELS))
            hazards.append(h)
            selected_hazard_id = h["id"]
            next_hazard_id += 1
        else:
            full = grid_pending_placement == "obstacle_full"
            o = make_obstacle(next_obstacle_id, cs, width, rs, height,
                              0, GRID_LAYERS - 1 if full else 0)
            obstacles.append(o)
            selected_obstacle_id = o["id"]
            next_obstacle_id += 1
        grid_pending_corner1 = None
        grid_pending_placement = None
        grid_setup_mode = "idle"


def handle_map_click(pos):
    global map_setup_mode, map_pending_placement, map_pending_corner1_latlon
    global map_start_latlon, map_goal_latlon, selected_map_hazard_id, selected_map_obstacle_id
    global next_map_hazard_id, next_map_obstacle_id
    ll = screen_to_map_latlon(pos)
    if ll is None:
        return
    if map_setup_mode == "placing_start":
        map_start_latlon = ll; map_setup_mode = "idle"; return
    if map_setup_mode == "placing_goal":
        map_goal_latlon = ll; map_setup_mode = "idle"; return
    if map_setup_mode == "placing_waypoint":
        map_waypoints.append({"latlon": ll, "z": map_waypoint_altitude})
        map_setup_mode = "idle"
        announce(f"WAYPOINT {len(map_waypoints)} ADDED — {alt_ft(map_waypoint_altitude):,} FT", "info", 1.0)
        return
    if map_setup_mode == "corner1":
        map_pending_corner1_latlon = ll
        map_setup_mode = "corner2"
        return
    if map_setup_mode == "corner2" and map_pending_corner1_latlon is not None:
        lat1, lon1 = map_pending_corner1_latlon
        lat2, lon2 = ll
        if map_pending_placement in ("ash", "turbulence"):
            htype = map_pending_placement
            h = make_map_hazard(next_map_hazard_id, htype, lat1, lon1, lat2, lon2,
                                0, 0,
                                random.randint(1, 3) if htype == "ash" else random.choice(TURBULENCE_LEVELS))
            map_hazards.append(h); selected_map_hazard_id = h["id"]; next_map_hazard_id += 1
        else:
            full = map_pending_placement == "obstacle_full"
            o = make_map_obstacle(next_map_obstacle_id, lat1, lon1, lat2, lon2,
                                  0, GRID_LAYERS - 1 if full else 0)
            map_obstacles.append(o); selected_map_obstacle_id = o["id"]; next_map_obstacle_id += 1
        map_setup_mode = "idle"
        map_pending_placement = None
        map_pending_corner1_latlon = None


def cycle_hazard_severity(h):
    if not h:
        return
    if h["type"] == "ash":
        h["severity"] = 1 + (int(h.get("severity", 1)) % 3)
    else:
        idx = TURBULENCE_LEVELS.index(h.get("severity", "MODERATE")) if h.get("severity") in TURBULENCE_LEVELS else 1
        h["severity"] = TURBULENCE_LEVELS[(idx + 1) % len(TURBULENCE_LEVELS)]


def handle_grid_action(action, target=None):
    global grid_setup_mode, grid_pending_placement, selected_hazard_id, selected_obstacle_id
    global map_mode, sim_state
    global grid_panel_scroll, waypoint_altitude, DRONE_SPEED_INDEX
    global start_cell, goal_cell, hazards, obstacles, grid_show_report
    global home_screen_started, home_screen_ready
    global grid_setup_message
    if action == "noop": return
    if action == "home_grid":
        map_mode = False; sim_state = "SETUP"; grid_panel_scroll = 0
        return
    if action == "home_map":
        map_mode = True; sim_state = "SETUP"; map_panel_scroll = 0; load_real_map()
        return
    if action == "grid_place_start": grid_setup_mode = "placing_start"
    elif action == "grid_place_goal": grid_setup_mode = "placing_goal"
    elif action == "grid_cycle_start_alt":
        start_cell = (start_cell[0], start_cell[1], (start_cell[2] + 1) % GRID_LAYERS)
    elif action == "grid_cycle_goal_alt":
        goal_cell = (goal_cell[0], goal_cell[1], (goal_cell[2] + 1) % GRID_LAYERS)
    elif action == "grid_cycle_waypoint_alt": waypoint_altitude = (waypoint_altitude + 1) % GRID_LAYERS
    elif action == "grid_add_waypoint": grid_setup_mode = "placing_waypoint"
    elif action == "grid_remove_waypoint":
        if target is not None and 0 <= target < len(waypoints): waypoints.pop(target)
    elif action == "grid_add_ash": grid_pending_placement = "ash"; grid_setup_mode = "corner1"
    elif action == "grid_add_turbulence": grid_pending_placement = "turbulence"; grid_setup_mode = "corner1"
    elif action == "grid_add_obstacle_full": grid_pending_placement = "obstacle_full"; grid_setup_mode = "corner1"
    elif action == "grid_add_obstacle_partial": grid_pending_placement = "obstacle_partial"; grid_setup_mode = "corner1"
    elif action == "grid_select_hazard": selected_hazard_id = target
    elif action == "grid_remove_hazard":
        hazards[:] = [h for h in hazards if h["id"] != target]
        selected_hazard_id = hazards[0]["id"] if hazards else None
    elif action == "grid_cycle_hazard_severity": cycle_hazard_severity(get_selected_hazard())
    elif action == "grid_cycle_hazard_minz":
        h = get_selected_hazard()
        if h:
            h["z_min"] = (h["z_min"] + 1) % GRID_LAYERS
            h["z_max"] = max(h["z_max"], h["z_min"])
    elif action == "grid_cycle_hazard_maxz":
        h = get_selected_hazard()
        if h:
            h["z_max"] = (h["z_max"] + 1) % GRID_LAYERS
            h["z_min"] = min(h["z_min"], h["z_max"])
    elif action == "grid_select_obstacle": selected_obstacle_id = target
    elif action == "grid_remove_obstacle":
        obstacles[:] = [o for o in obstacles if o["id"] != target]
        selected_obstacle_id = obstacles[0]["id"] if obstacles else None
    elif action == "grid_cycle_obstacle_minz":
        o = get_selected_obstacle()
        if o:
            o["z_min"] = (o["z_min"] + 1) % GRID_LAYERS
            o["z_max"] = max(o["z_max"], o["z_min"])
    elif action == "grid_cycle_obstacle_maxz":
        o = get_selected_obstacle()
        if o:
            o["z_max"] = (o["z_max"] + 1) % GRID_LAYERS
            o["z_min"] = min(o["z_min"], o["z_max"])
    elif action == "grid_randomize": add_random_grid_scenario()
    elif action == "grid_reset": reset_grid_flight()
    elif action == "grid_clear": clear_grid_scenario(True)
    elif action == "grid_start_sim": start_grid_simulation()
    elif action == "cycle_drone_speed": DRONE_SPEED_INDEX = (DRONE_SPEED_INDEX + 1) % len(DRONE_SPEED_VALUES)
    elif action == "switch_map":
        map_mode = True
        globals()["map_panel_scroll"] = 0
        globals()["sim_state"] = "SETUP"
        load_real_map()
    elif action == "grid_refresh":
        reset_grid_flight()
    elif action == "grid_report": grid_show_report = not grid_show_report


def handle_map_action(action, target=None):
    global map_setup_mode, map_pending_placement, selected_map_hazard_id, selected_map_obstacle_id
    global map_panel_scroll, map_waypoint_altitude, map_start_z, map_goal_z, map_zoom_level
    global DRONE_SPEED_INDEX, map_show_report, map_mode, sim_state, map_status
    if action == "noop": return
    if action == "home_grid":
        map_mode = False; sim_state = "SETUP"; grid_panel_scroll = 0
        return
    if action == "home_map":
        map_mode = True; sim_state = "SETUP"; map_panel_scroll = 0; load_real_map()
        return
    if action == "map_refresh": load_real_map()
    elif action == "map_zoom_in":
        map_zoom_level = min(MAP_ZOOM_MAX, map_zoom_level + 1); load_real_map()
    elif action == "map_zoom_out":
        map_zoom_level = max(MAP_ZOOM_MIN, map_zoom_level - 1); load_real_map()
    elif action == "map_place_start": map_setup_mode = "placing_start"
    elif action == "map_place_goal": map_setup_mode = "placing_goal"
    elif action == "map_cycle_start_alt": map_start_z = (map_start_z + 1) % GRID_LAYERS
    elif action == "map_cycle_goal_alt": map_goal_z = (map_goal_z + 1) % GRID_LAYERS
    elif action == "map_cycle_waypoint_alt": map_waypoint_altitude = (map_waypoint_altitude + 1) % GRID_LAYERS
    elif action == "map_add_waypoint": map_setup_mode = "placing_waypoint"
    elif action == "map_remove_waypoint":
        if target is not None and 0 <= target < len(map_waypoints): map_waypoints.pop(target)
    elif action == "map_add_ash": map_pending_placement = "ash"; map_setup_mode = "corner1"
    elif action == "map_add_turbulence": map_pending_placement = "turbulence"; map_setup_mode = "corner1"
    elif action == "map_add_obstacle_full": map_pending_placement = "obstacle_full"; map_setup_mode = "corner1"
    elif action == "map_add_obstacle_partial": map_pending_placement = "obstacle_partial"; map_setup_mode = "corner1"
    elif action == "map_select_hazard": selected_map_hazard_id = target
    elif action == "map_remove_hazard":
        map_hazards[:] = [h for h in map_hazards if h["id"] != target]
        selected_map_hazard_id = map_hazards[0]["id"] if map_hazards else None
    elif action == "map_cycle_hazard_severity": cycle_hazard_severity(get_selected_map_hazard())
    elif action == "map_cycle_hazard_minz":
        h = get_selected_map_hazard()
        if h:
            h["z_min"] = (h["z_min"] + 1) % GRID_LAYERS; h["z_max"] = max(h["z_max"], h["z_min"])
    elif action == "map_cycle_hazard_maxz":
        h = get_selected_map_hazard()
        if h:
            h["z_max"] = (h["z_max"] + 1) % GRID_LAYERS; h["z_min"] = min(h["z_min"], h["z_max"])
    elif action == "map_select_obstacle": selected_map_obstacle_id = target
    elif action == "map_remove_obstacle":
        map_obstacles[:] = [o for o in map_obstacles if o["id"] != target]
        selected_map_obstacle_id = map_obstacles[0]["id"] if map_obstacles else None
    elif action == "map_cycle_obstacle_minz":
        o = get_selected_map_obstacle()
        if o:
            o["z_min"] = (o["z_min"] + 1) % GRID_LAYERS; o["z_max"] = max(o["z_max"], o["z_min"])
    elif action == "map_cycle_obstacle_maxz":
        o = get_selected_map_obstacle()
        if o:
            o["z_max"] = (o["z_max"] + 1) % GRID_LAYERS; o["z_min"] = min(o["z_min"], o["z_max"])
    elif action == "map_randomize": add_random_map_scenario()
    elif action == "map_reset": reset_map_flight()
    elif action == "map_clear": clear_map_scenario(True); load_real_map()
    elif action == "map_start_sim": start_map_simulation()
    elif action == "cycle_drone_speed": DRONE_SPEED_INDEX = (DRONE_SPEED_INDEX + 1) % len(DRONE_SPEED_VALUES)
    elif action == "switch_grid":
        map_mode = False; sim_state = "SETUP"; map_setup_mode = "idle"; grid_panel_scroll = 0
    elif action == "map_report": map_show_report = not map_show_report


# ============================================================
# MAP CAMERA INPUT
# ============================================================


def zoom_map_at_cursor(direction, cursor):
    global map_zoom_level, MAP_CENTER_LAT, MAP_CENTER_LON
    if map_surface is None:
        return
    anchor = screen_to_map_latlon(cursor)
    if anchor is None:
        return
    old_zoom = map_zoom_level
    new_zoom = max(MAP_ZOOM_MIN, min(MAP_ZOOM_MAX, old_zoom + direction))
    if new_zoom == old_zoom:
        return
    # Keep the point under the cursor fixed during the zoom.
    anchor_world_new = latlon_to_world(anchor[0], anchor[1], new_zoom)
    desired_center_world = (anchor_world_new[0] - cursor[0] + MAP_WIDTH / 2,
                            anchor_world_new[1] - cursor[1] + MAP_HEIGHT / 2)
    MAP_CENTER_LAT, MAP_CENTER_LON = world_to_latlon(desired_center_world[0], desired_center_world[1], new_zoom)
    map_zoom_level = new_zoom
    load_real_map()


def finish_map_pan():
    global map_is_panning, map_pan_live_offset, MAP_CENTER_LAT, MAP_CENTER_LON
    if not map_is_panning:
        return
    off_x, off_y = map_pan_live_offset
    map_is_panning = False
    if abs(off_x) > MAP_PAN_CLICK_THRESHOLD or abs(off_y) > MAP_PAN_CLICK_THRESHOLD:
        cx, cy = latlon_to_world(MAP_CENTER_LAT, MAP_CENTER_LON, map_zoom_actual)
        MAP_CENTER_LAT, MAP_CENTER_LON = world_to_latlon(cx - off_x, cy - off_y, map_zoom_actual)
        load_real_map()
    map_pan_live_offset = (0, 0)


# ============================================================
# CORNER PREVIEWS
# ============================================================


def draw_grid_preview():
    if grid_setup_mode == "corner2" and grid_pending_corner1 is not None:
        mx, my = mouse_canvas_pos()
        if 0 <= mx < GRID_WIDTH and 0 <= my < GRID_HEIGHT:
            c2 = (int(mx // CELL_SIZE), int(my // CELL_SIZE))
            c1 = grid_pending_corner1
            x = min(c1[0], c2[0]) * CELL_SIZE
            y = min(c1[1], c2[1]) * CELL_SIZE
            w = (abs(c1[0] - c2[0]) + 1) * CELL_SIZE
            h = (abs(c1[1] - c2[1]) + 1) * CELL_SIZE
            pygame.draw.rect(screen, (255, 255, 255), (x, y, w, h), 2)


def draw_map_preview():
    if map_setup_mode == "corner2" and map_pending_corner1_latlon is not None:
        mx, my = mouse_canvas_pos()
        if 0 <= mx < MAP_WIDTH and 0 <= my < MAP_HEIGHT:
            ll = screen_to_map_latlon((mx, my))
            if ll:
                p1 = map_latlon_to_screen(*map_pending_corner1_latlon)
                p2 = map_latlon_to_screen(*ll)
                x1, x2 = sorted((p1[0], p2[0])); y1, y2 = sorted((p1[1], p2[1]))
                pygame.draw.rect(screen, (255, 255, 255), pygame.Rect(x1, y1, max(2, x2 - x1), max(2, y2 - y1)), 2)


# ============================================================
# MAIN LOOP
# ============================================================

running = True
active_buttons = []
last_time = time.perf_counter()

while running:
    now = time.perf_counter()
    dt = min(0.05, max(0.001, now - last_time))
    last_time = now

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.VIDEORESIZE and not is_fullscreen:
            display_surface = pygame.display.set_mode(event.size, pygame.RESIZABLE)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_F11:
                set_display_mode(not is_fullscreen)
            elif event.key == pygame.K_m and sim_state == "HOME":
                map_mode = True; sim_state = "SETUP"; map_panel_scroll = 0; load_real_map()
            elif event.key == pygame.K_g and sim_state == "HOME":
                map_mode = False; sim_state = "SETUP"; grid_panel_scroll = 0
            elif event.key == pygame.K_m and sim_state == "SETUP":
                map_mode = not map_mode
                if map_mode:
                    map_panel_scroll = 0
                    load_real_map()
                else:
                    grid_panel_scroll = 0
            elif event.key == pygame.K_r:
                if map_mode:
                    reset_map_flight()
                else:
                    reset_grid_flight()
            elif event.key == pygame.K_p and sim_state == "RUNNING":
                if map_mode:
                    map_show_report = not map_show_report
                else:
                    grid_show_report = not grid_show_report
            elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS) and map_mode and sim_state == "SETUP":
                zoom_map_at_cursor(1, mouse_canvas_pos())
            elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS) and map_mode and sim_state == "SETUP":
                zoom_map_at_cursor(-1, mouse_canvas_pos())
            elif event.key == pygame.K_ESCAPE:
                if map_mode:
                    map_setup_mode = "idle"; map_pending_placement = None; map_pending_corner1_latlon = None
                else:
                    grid_setup_mode = "idle"; grid_pending_placement = None; grid_pending_corner1 = None

        elif event.type == pygame.MOUSEWHEEL and sim_state == "SETUP":
            mx, my = mouse_canvas_pos()
            if mx >= GRID_WIDTH:
                if map_mode:
                    map_panel_scroll -= event.y * 48
                else:
                    grid_panel_scroll -= event.y * 48
            elif map_mode and 0 <= mx < MAP_WIDTH and 0 <= my < MAP_HEIGHT:
                zoom_map_at_cursor(1 if event.y > 0 else -1, (mx, my))

        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and sim_state == "SETUP":
            pos = to_canvas_pos(event.pos)
            clicked = next((b for b in active_buttons if b["rect"].collidepoint(pos)), None)
            if sim_state == "HOME":
                if clicked:
                    if clicked["action"] == "home_grid":
                        map_mode = False; sim_state = "SETUP"; grid_panel_scroll = 0
                    elif clicked["action"] == "home_map":
                        map_mode = True; sim_state = "SETUP"; map_panel_scroll = 0; load_real_map()
            elif clicked:
                if map_mode:
                    handle_map_action(clicked["action"], clicked.get("target"))
                else:
                    handle_grid_action(clicked["action"], clicked.get("target"))
            elif map_mode and 0 <= pos[0] < MAP_WIDTH and 0 <= pos[1] < MAP_HEIGHT:
                if map_setup_mode != "idle":
                    handle_map_click(pos)
                else:
                    map_is_panning = True
                    map_pan_last_pos = pos
                    map_pan_live_offset = (0.0, 0.0)
            elif (not map_mode) and 0 <= pos[0] < GRID_WIDTH and 0 <= pos[1] < GRID_HEIGHT:
                handle_grid_map_click(pos)

        elif event.type == pygame.MOUSEMOTION and map_is_panning:
            pos = to_canvas_pos(event.pos)
            if map_pan_last_pos is not None:
                dx = pos[0] - map_pan_last_pos[0]
                dy = pos[1] - map_pan_last_pos[1]
                map_pan_live_offset = (map_pan_live_offset[0] + dx, map_pan_live_offset[1] + dy)
                map_pan_last_pos = pos

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1 and map_is_panning:
            finish_map_pan()

    # Clamp panel scroll every frame so wheel input can never run it off-screen.
    if map_mode:
        map_panel_scroll = max(0, min(map_panel_scroll, max(0, map_panel_content_height - GRID_HEIGHT)))
    else:
        grid_panel_scroll = max(0, min(grid_panel_scroll, max(0, grid_panel_content_height - GRID_HEIGHT)))

    # ---------------- START SCREEN ----------------
    if sim_state == "HOME":
        active_buttons = draw_start_screen()
        present()
        clock.tick(60)
        continue

    # ---------------- MAP ----------------
    if map_mode:
        if sim_state == "RUNNING":
            map_update_mission(dt)

        screen.fill(BG_COLOR)
        if sim_state == "SETUP":
            draw_real_map()
            draw_map_preview()
            active_buttons = draw_map_setup_panel()
            draw_banner_and_common_text(is_map=True)
            hint = font_micro.render("MAP INPUT: drag to pan • wheel on map to zoom • wheel on right panel to scroll", True, TEXT_MAIN)
            screen.blit(hint, (MAP_WIDTH - hint.get_width() - 12, MAP_HEIGHT - 24))
        else:
            draw_real_map()
            draw_altitude_panel(map_path, map_hazards, is_map=True,
                                current_segment=map_current_segment, progress=map_progress)
            draw_dashboard(is_map=True)
            draw_banner_and_common_text(is_map=True)
            draw_legend(is_map=True)
            if map_show_report:
                draw_report(is_map=True)
        present()
        clock.tick(60)
        continue

    # ---------------- GRID ----------------
    if sim_state == "RUNNING":
        grid_update_mission(dt)

    screen.fill(BG_COLOR)
    draw_grid()
    draw_grid_hazards()
    draw_grid_obstacles()
    if sim_state == "SETUP":
        draw_grid_setup_markers()
        draw_grid_preview()
        active_buttons = draw_grid_setup_panel()
    else:
        draw_grid_path()
        draw_grid_drone()
        draw_wind_indicator()
        draw_altitude_panel(grid_path, hazards, is_map=False,
                            current_segment=grid_current_segment, progress=grid_progress)
        draw_dashboard(is_map=False)
        draw_banner_and_common_text(is_map=False)
        draw_legend(is_map=False)
        if grid_show_report:
            draw_report(is_map=False)

    if sim_state == "SETUP":
        pygame.draw.line(screen, PANEL_LINE_COLOR, (GRID_WIDTH, 0), (GRID_WIDTH, GRID_HEIGHT), 2)
    present()
    clock.tick(60)

pygame.quit()