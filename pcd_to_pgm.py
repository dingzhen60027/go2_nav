#!/usr/bin/env python3
"""PCD → PGM: 读二进制 PCD (PointXYZINormal), 转 2D occupancy grid."""

import struct, sys, math, os

# ---- 配置 ----
PCD_FILE = "/home/wjg/go2_nav/src/faster-lio/PCD/scans.pcd"
OUT_DIR = "/home/wjg/go2_nav/maps"
RESOLUTION = 0.05          # 每像素 5cm
Z_MIN = 0.3                # 地面以上（排除地面点）
Z_MAX = 3.0                # 最高到 3m（足够覆盖墙壁/障碍物）
OCC_THRESH = 2              # 一个格子至少 2 个点才算 occupied
# -------------

os.makedirs(OUT_DIR, exist_ok=True)
out_pgm = os.path.join(OUT_DIR, "map.pgm")
out_yaml = os.path.join(OUT_DIR, "map.yaml")

print(f"读取 PCD: {PCD_FILE}")

# 读二进制 PCD
with open(PCD_FILE, "rb") as f:
    header_bytes = b""
    while True:
        line = f.readline()
        header_bytes += line
        if line.startswith(b"DATA"):
            break
    data = f.read()

step = 32  # PointXYZINormal = 8 floats * 4 bytes
n = len(data) // step

# 提取地面附近点
points = []
for i in range(n):
    off = i * step
    x, y, z = struct.unpack("fff", data[off : off + 12])
    if Z_MIN <= z <= Z_MAX and abs(x) < 200 and abs(y) < 200:
        points.append((x, y))

print(f"地面附近点: {len(points)}")

if not points:
    print("ERROR: 没有符合条件的点")
    sys.exit(1)

# 计算地图范围
xs = [p[0] for p in points]
ys = [p[1] for p in points]
x_min, x_max = min(xs), max(xs)
y_min, y_max = min(ys), max(ys)

width = math.ceil((x_max - x_min) / RESOLUTION)
height = math.ceil((y_max - y_min) / RESOLUTION)

print(f"地图: {width}x{height} @ {RESOLUTION}m/pix")
print(f"范围: x[{x_min:.2f}, {x_max:.2f}] y[{y_min:.2f}, {y_max:.2f}]")

# 构建 occupancy grid (0=free, 255=occupied for PGM)
grid = [0] * (width * height)

for px, py in points:
    i = int((px - x_min) / RESOLUTION)
    j = int((py - y_min) / RESOLUTION)
    idx = i + j * width
    if 0 <= idx < len(grid):
        grid[idx] += 1

# 阈值: 有 OCC_THRESH 以上点标记为 occupied
occ_count = 0
for idx in range(len(grid)):
    if grid[idx] >= OCC_THRESH:
        grid[idx] = 0    # PGM: 0 = black = occupied
        occ_count += 1
    else:
        grid[idx] = 255  # PGM: 255 = white = free

print(f"occupied: {occ_count}, free: {len(grid) - occ_count}")

# 写 PGM (binary P5)
with open(out_pgm, "wb") as f:
    f.write(f"P5\n{width} {height}\n255\n".encode())
    f.write(bytes(grid))

# 写 YAML
with open(out_yaml, "w") as f:
    f.write(f"""image: map.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{x_min:.3f}, {y_min:.3f}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")

print(f"\n完成!")
print(f"  {out_pgm}")
print(f"  {out_yaml}")
