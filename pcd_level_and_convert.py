#!/usr/bin/env python3
"""PCD 校平 + 转 PGM: 先绕 X/Y 轴旋转使地面水平，再输出 2D 地图。"""

import struct, math, os, sys

PCD_FILE = "/home/wjg/go2_nav/src/faster-lio/PCD/scans.pcd"
OUT_PCD = "/home/wjg/go2_nav/src/faster-lio/PCD/scans_leveled.pcd"
OUT_DIR = "/home/wjg/go2_nav/maps"
RES = 0.05

# 检测到的倾斜角（度）
ROT_X_DEG = 3.6   # 绕 X 轴修正（正方向）
ROT_Y_DEG = 7.8   # 绕 Y 轴修正（正方向）

os.makedirs(OUT_DIR, exist_ok=True)

# ---- 读 PCD ----
with open(PCD_FILE, "rb") as f:
    hdr = b""
    while True:
        l = f.readline()
        hdr += l
        if l.startswith(b"DATA"):
            break
    data = f.read()

step = 32; n = len(data) // step
print(f"读取 {n} 个点")

# ---- 旋转变换 ----
rx = ROT_X_DEG * math.pi / 180
ry = ROT_Y_DEG * math.pi / 180
cx, sx = math.cos(rx), math.sin(rx)
cy, sy = math.cos(ry), math.sin(ry)

out = bytearray()
pts = 0
for i in range(n):
    off = i * step
    x, y, z, intensity = struct.unpack("ffff", data[off:off+16])
    # 绕 X 轴转
    y1 = y * cx - z * sx
    z1 = y * sx + z * cx
    # 绕 Y 轴转
    x2 = x * cy + z1 * sy
    z2 = -x * sy + z1 * cy
    if abs(x2) < 500 and abs(y1) < 500:
        out.extend(struct.pack("ffff", x2, y1, z2, intensity))
        pts += 1

print(f"校平后: {pts} 个点")

# ---- 写校平后的 PCD (binary, PointXYZI) ----
with open(OUT_PCD, "wb") as f:
    f.write(f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {pts}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {pts}\nDATA binary\n".encode())
    for i in range(pts):
        f.write(out[i*16:i*16+12])
print(f"校平 PCD: {OUT_PCD}")

# ---- 转 2D PGM ----
# 用全部点投影，但只保留地面以上的
points_xy = []
for i in range(pts):
    off = i * 16
    x2 = struct.unpack("f", out[off:off+4])[0]
    y1 = struct.unpack("f", out[off+4:off+8])[0]
    z2 = struct.unpack("f", out[off+8:off+12])[0]
    if -5.0 <= z2 <= 20.0:  # 全范围
        points_xy.append((x2, y1))

if not points_xy:
    print("错误: 没有有效点"); sys.exit(1)

xs = [p[0] for p in points_xy]
ys = [p[1] for p in points_xy]
x_min, x_max = min(xs), max(xs)
y_min, y_max = min(ys), max(ys)
w = math.ceil((x_max - x_min) / RES)
h = math.ceil((y_max - y_min) / RES)

print(f"地图: {w}x{h} @ {RES}m/pix, 范围: x[{x_min:.1f},{x_max:.1f}] y[{y_min:.1f},{y_max:.1f}]")

grid = [0] * (w * h)
for px, py in points_xy:
    i = int((px - x_min) / RES)
    j = int((py - y_min) / RES)
    idx = i + j * w
    if 0 <= idx < len(grid):
        grid[idx] += 1

occ = 0
for idx in range(len(grid)):
    if grid[idx] >= 2:
        grid[idx] = 0; occ += 1
    else:
        grid[idx] = 255

with open(os.path.join(OUT_DIR, "map.pgm"), "wb") as f:
    f.write(f"P5\n{w} {h}\n255\n".encode())
    f.write(bytes(grid))

with open(os.path.join(OUT_DIR, "map.yaml"), "w") as f:
    f.write(f"image: map.pgm\nmode: trinary\nresolution: {RES}\norigin: [{x_min:.3f}, {y_min:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

print(f"完成! 地图: {os.path.join(OUT_DIR, 'map.pgm')} ({occ} 障碍格)")