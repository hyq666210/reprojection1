#!/usr/bin/env bash
#
# 一键构建并运行全部检查。
# 用法：在项目根目录执行  bash build.sh
#
# 哪一步报错就停在哪一步，往上翻看那一步的提示即可。

set -euo pipefail
cd "$(dirname "$0")"

echo "== [1/4] 检查工具链 =="
if ! command -v cmake >/dev/null 2>&1; then
    echo "找不到 cmake。请先执行：sudo apt install -y cmake"
    exit 1
fi
if ! command -v g++ >/dev/null 2>&1; then
    echo "找不到 g++。请先执行：sudo apt install -y build-essential"
    exit 1
fi
cmake --version | head -1
g++ --version | head -1
echo "检查通过：cmake 与 g++ 都在 PATH 里。"

echo
echo "== [2/4] 配置 =="
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

echo
echo "== [3/4] 编译 =="
JOBS="$(nproc 2>/dev/null || echo 2)"
cmake --build build -j"$JOBS"

echo
echo "== [4/4] 运行 =="
echo "--- 单元测试 ---"
./build/bin/reprojection_tests
echo
echo "--- 数据文件重投影 ---"
./build/bin/reprojection_app data/points.txt

echo
echo "全部完成。"
