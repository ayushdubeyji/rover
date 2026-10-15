import math
from PIL import Image, ImageDraw, ImageFont

# 240x240 screen simulation
W, H = 240, 240
CH = 156
STRIP_Y = 156

# Read car_model.h directly
with open('s3_remote/car_model.h', 'r') as f:
    code = f.read()

import re
verts_raw = code.split('CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {')[1].split('};')[0]
vert_strs = re.findall(r'\{([-\d\s,]+)\}', verts_raw)
verts = [list(map(int, v.split(','))) for v in vert_strs]

tris_raw = code.split('CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {')[1].split('};')[0]
tri_strs = re.findall(r'\{([-\d\s,]+)\}', tris_raw)
tris = [list(map(int, t.split(','))) for t in tri_strs]

cols_raw = code.split('CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {')[1].split('};')[0]
col_vals = [int(x.strip(), 16) for x in cols_raw.replace('\n', '').split(',') if x.strip()]

def rgb565_to_rgb888(c):
    r = ((c >> 11) & 31) * 255 // 31
    g = ((c >> 5) & 63) * 255 // 63
    b = (c & 31) * 255 // 31
    return (r, g, b)

cT = 0.8387
sT = 0.5446

def to_view(wx, wy, wz):
    vx = -wy
    vy = wz * cT + wx * sT
    vd = wx * cT - wz * sT
    return vx, vy, vd

def rot_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    m = [
        1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
        2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
        2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)
    ]
    return m

def render_frame(roll_deg, pitch_deg, yaw_deg):
    im = Image.new('RGB', (W, H), (0, 0, 0)) # Pure black background
    draw = ImageDraw.Draw(im)

    # 1. Header (y=0..14)
    draw.line([(0, 14), (W, 14)], fill=(49, 134, 134))
    draw.text((4, 2), "ATTITUDE", fill=(0, 255, 255))
    draw.ellipse((147, 5, 153, 11), fill=(0, 255, 0))
    draw.text((157, 2), "LINK", fill=(0, 255, 0))
    draw.text((194, 2), "4.1V", fill=(255, 255, 255))

    # 2. Car 3D projection
    R = rot_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    V = []
    for bx, by, bz in verts:
        fx, fy, fz = bx/127.0, by/127.0, bz/127.0
        wx = R[0]*fx + R[1]*fy + R[2]*fz
        wy = R[3]*fx + R[4]*fy + R[5]*fz
        wz = R[6]*fx + R[7]*fy + R[8]*fz
        V.append(to_view(wx, wy, wz))

    cx, cy, scale = 100, 85, 68
    # Ground shadow
    draw.ellipse((cx - int(scale*0.62), cy + int(scale*0.42) - int(scale*0.16),
                  cx + int(scale*0.62), cy + int(scale*0.42) + int(scale*0.16)), fill=(33, 16, 4))

    tri_data = []
    for t_idx, (a, b, c) in enumerate(tris):
        vxa, vya, vda = V[a]
        vxb, vyb, vdb = V[b]
        vxc, vyc, vdc = V[c]
        key = (vda + vdb + vdc) / 3.0
        ux, uy, ud = vxb - vxa, vyb - vya, vdb - vda
        wx, wy, wd = vxc - vxa, vyc - vya, vdc - vda
        nx = uy*wd - ud*wy
        ny = ud*wx - ux*wd
        nd = ux*wy - uy*wx
        nl = math.sqrt(nx*nx + ny*ny + nd*nd) + 1e-6
        dot = abs((-0.4*nx + 0.6*ny - 0.7*nd) / nl)
        s = int(170 + 85 * dot)
        col = col_vals[t_idx]
        r = (((col >> 11) & 31) * s) >> 8
        g = (((col >> 5) & 63) * s) >> 8
        bl = ((col & 31) * s) >> 8
        c_rgb = (r * 255 // 31, g * 255 // 63, bl * 255 // 31)
        pts = [
            (cx + int(vxa * scale), cy - int(vya * scale)),
            (cx + int(vxb * scale), cy - int(vyb * scale)),
            (cx + int(vxc * scale), cy - int(vyc * scale))
        ]
        tri_data.append((key, pts, c_rgb))

    # Painter's sort far to near
    tri_data.sort(key=lambda x: x[0], reverse=True)
    for _, pts, col in tri_data:
        draw.polygon(pts, fill=col, outline=col)

    # 3. Gizmo at (196, 48, 20)
    gx, gy, glen = 196, 48, 20
    gcols = [(255, 0, 0), (0, 255, 0), (0, 100, 255)]
    glabels = ['X', 'Y', 'Z']
    for a in range(3):
        vx_, vy_, _ = to_view(R[a], R[3+a], R[6+a])
        ex = gx + int(vx_ * glen)
        ey = gy - int(vy_ * glen)
        draw.line([(gx, gy), (ex, ey)], fill=gcols[a], width=2)
        draw.ellipse((ex-2, ey-2, ex+2, ey+2), fill=gcols[a])
        lx = ex + (4 if ex >= gx else -10)
        ly = ey - 4
        draw.rectangle((lx-1, ly-1, lx+8, ly+9), fill=(0,0,0))
        draw.text((lx, ly), glabels[a], fill=gcols[a])
    draw.ellipse((gx-2, gy-2, gx+2, gy+2), fill=(255, 255, 255))

    # Telemetry text on upper canvas
    draw.text((4, 22), f"R {roll_deg:+5.1f}", fill=(123, 239, 239))
    draw.text((4, 32), f"P {pitch_deg:+5.1f}", fill=(123, 239, 239))
    draw.text((4, 42), f"Y {yaw_deg:+5.1f}", fill=(123, 239, 239))

    # 4. Flawed bottom strip (y=156..240) in Pure Solid Black
    draw.rectangle((0, STRIP_Y, W, H), fill=(0, 0, 0))
    draw.line([(0, STRIP_Y - 1), (W, STRIP_Y - 1)], fill=(0, 255, 0)) # Green divider

    # Row 0: Speed & Battery in bold green
    # Row 1: Heading & Mode in bold yellow
    # Row 2: Sensors in bold cyan
    r0 = "SPD 0.00  BAT 4.1V"
    r1 = "HDG   0  NORMAL  "
    r2 = "TOF  45cm US  60cm"

    # Simulated bold 2-pass (size 2 is 16px high)
    try:
        font = ImageFont.truetype("courbd.ttf", 16)
    except:
        font = ImageFont.load_default()

    y0 = STRIP_Y + 4
    y1 = y0 + 26
    y2 = y1 + 26

    draw.text((12, y0), r0, fill=(255, 255, 255), font=font)
    draw.text((13, y0), r0, fill=(255, 255, 255), font=font)

    draw.text((12, y1), r1, fill=(255, 255, 255), font=font)
    draw.text((13, y1), r1, fill=(255, 255, 255), font=font)

    draw.text((12, y2), r2, fill=(255, 255, 255), font=font)
    draw.text((13, y2), r2, fill=(255, 255, 255), font=font)

    return im

# Render 4 test frames: Flat, Tilted Forward 15, Rolled Right 20, Yaw 45
f1 = render_frame(0, 0, 0)
f2 = render_frame(0, 15, 0)
f3 = render_frame(20, 0, 0)
f4 = render_frame(0, 0, 45)

grid = Image.new('RGB', (W*4, H), (0,0,0))
grid.paste(f1, (0, 0))
grid.paste(f2, (W, 0))
grid.paste(f3, (W*2, 0))
grid.paste(f4, (W*3, 0))
grid.save('tools/screen_preview_all_modes.png')
print("Rendered and saved tools/screen_preview_all_modes.png")
