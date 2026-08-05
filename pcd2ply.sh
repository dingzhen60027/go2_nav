#!/bin/bash
# pcd2ply.sh — PCD 转 PLY，自动命名不覆盖

set -e

export GO2_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCD_DIR="${GO2_NAV_ROOT}/src/faster-lio/PCD"
OUT_DIR="${GO2_NAV_ROOT}/PCD"
PCD_FILE="${PCD_DIR}/scans.pcd"

if [ ! -f "$PCD_FILE" ]; then
    echo "错误: 未找到 $PCD_FILE"
    echo "请先建图"
    exit 1
fi

mkdir -p "$OUT_DIR"

# 自动生成带时间戳的文件名
TIMESTAMP=$(date +%m%d_%H%M)
OUT_FILE="${OUT_DIR}/scans_${TIMESTAMP}.ply"

echo "===== PCD → PLY ====="
echo "输入: $PCD_FILE ($(du -h "$PCD_FILE" | cut -f1))"
echo "输出: $OUT_FILE"

# 读文件头获取点数
HEADER=$(python3 -c "
with open('$PCD_FILE', 'rb') as f:
    while True:
        l = f.readline()
        if l.startswith(b'DATA'): break
        if l.startswith(b'POINTS'): print(int(l.split()[1]))
" 2>/dev/null)

echo "PCD 点数: $HEADER"

# 转 PLY（PointXYZI，采样最多 80 万点）
python3 << PYEOF
import struct, random

with open('$PCD_FILE', 'rb') as f:
    while True:
        l = f.readline()
        if l.startswith(b'DATA'): break
    data = f.read()

step = 32; n = len(data) // step
sample_n = min(n, 800000)
random.seed(42)
indices = sorted(random.sample(range(n), sample_n))

out = bytearray()
for i in indices:
    off = i * step
    x, y, z, intensity = struct.unpack('ffff', data[off:off+16])
    out.extend(struct.pack('ffff', x, y, z, intensity))

header = f'''ply
format binary_little_endian 1.0
element vertex {sample_n}
property float x
property float y
property float z
property float intensity
end_header
'''.encode()

with open('$OUT_FILE', 'wb') as f:
    f.write(header)
    f.write(bytes(out))

print(f"写入 {sample_n} 个点")
PYEOF

echo "完成: $OUT_FILE"
