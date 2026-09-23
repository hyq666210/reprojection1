```markdown
# reprojection

27 赛季算法组视觉侧第一次培训作业：针孔相机模型下的重投影计算。

## 这是什么

输入一个三维点、相机内参和外参，输出它应该落在图像的哪个像素上，并计算与观测点之间的像素欧氏距离。理想针孔模型，忽略畸变。

## 约定

- 外参：世界 -> 相机，`Pc = R * Pw + t`（`R` 为 3x3 旋转矩阵，行主序；`t` 为 3x1 平移向量）
- 三维坐标与平移量**统一用米**
- 像素坐标系：原点在图像左上角，`u` 向右、`v` 向下
- 深度 `Zc` 必须为正，否则该点不可投影（返回 `std::nullopt`）

## 目录结构

```
reprojection/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/
│   ├── vec_mat.hpp             # 最小向量/矩阵工具
│   └── camera.hpp              # 相机模型与重投影接口（只放声明）
├── src/
│   ├── camera.cpp              # 重投影实现
│   └── main.cpp                # 命令行程序
├── tests/
│   └── test_reprojection.cpp   # 单元测试（自带极简断言框架）
└── data/
    └── points.txt              # 自造测试数据
```

`include/` 与 `src/` 分离是作业要求：头文件是「说明书」，实现文件是「怎么做」。

## 依赖

零第三方依赖。只需要 C++17 编译器和 CMake >= 3.16。

## 构建与运行

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/bin/reprojection_app data/points.txt    # 投影整份数据，打印表格与统计
./build/bin/reprojection_tests                  # 单元测试
ctest --test-dir build --output-on-failure      # 一次跑完两项
```

不想用 CMake 时，也可以直接调编译器：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/camera.cpp src/main.cpp -o app
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/camera.cpp tests/test_reprojection.cpp -o tests
```

## 数据文件格式

```
intrinsics fx fy cx cy
extrinsics R00 R01 R02 R10 R11 R12 R20 R21 R22 tx ty tz
point <id> Xw Yw Zw [u_obs v_obs]
```

- `#` 开头到行尾是注释
- `extrinsics` 的 9 个旋转矩阵元素按**行主序**排列
- `point` 行末尾的观测值可省略；省略时只投影、不计算误差
- 改数据不需要重新编译，直接重跑程序即可

## 测试结果

单元测试 60 个断言全部通过；主程序对数据文件的运行结果见作业报告。
```

这个文件不是作业硬性要求的，但它有两个用处：一是 clone 你仓库的人（也就是助教）第一眼看到的就是它；二是它逼你把数据格式写清楚——而数据格式本来就要在报告里说明。

---

## 十三、Step 10：构建、运行、核对结果

进到项目目录，标准三步：

```bash
cd ~/reprojection
rm -rf build                                        # 干净起步
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release      # 1. 配置
cmake --build build -j                              # 2. 编译（-j 用满 CPU 核心）
./build/bin/reprojection_app data/points.txt        # 3. 跑数据
./build/bin/reprojection_tests                      # 4. 跑单测
```

### 主程序的实测输出（本机已跑过，你的应当逐行一致）

```text
数据文件: data/points.txt
内参: fx=900 fy=900 cx=640 cy=360
外参约定: Pc = R * Pw + t  (世界 -> 相机)

id              world (Xw, Yw, Zw)              projected (u, v)              observed (u, v)           error(px)    status
----------------------------------------------------------------------------------------------------------------------
P1              (0, 0, 5)                       (648.8702, 308.2546)          (648.8702, 308.2546)      5.95182e-05  OK
P2              (0.5, 0.3, 6)                   (719.7097, 355.9817)          (718.9097, 356.7817)      1.13139e+00  OK
P3              (-0.4, 0.2, 4.5)                (569.9535, 344.3215)          (569.9535, 344.3215)      1.19154e-05  OK
P4              (0.1, -0.6, 7)                  (661.8478, 233.2085)          (661.8478, 233.2085)      5.72937e-05  OK
P5              (-0.25, -0.15, 3.2)             (586.3928, 261.9677)          (586.3928, 261.9677)      3.20884e-05  OK
P6              (1, 1, 8)                       (752.3998, 424.7243)          -                         -            OK
P7              (-0.6, 1, 5)                    (537.3372, 480.3834)          (537.3372, 480.3834)      2.46362e-05  OK
BAD_ZC_ZERO     (0, 0, -0.08010978768)          -                             -                         -            SKIPPED: Zc=-3.33067e-16 <= 0 (点在相机背后或光心平面)
EDGE_TINY_ZC    (0, 0, -0.08010978668)          (u=4.5000e+10, v=-2.3227e+10) -                         -            OK [数值爆炸: Zc=1.000e-09]
BAD_BEHIND_1    (0, 0, -1)                      -                             -                         -            SKIPPED: Zc=-0.91863 <= 0 (点在相机背后或光心平面)
BAD_BEHIND_2    (0.5, 0.5, -3)                  -                             -                         -            SKIPPED: Zc=-2.88882 <= 0 (点在相机背后或光心平面)
----------------------------------------------------------------------------------------------------------------------
统计: 成功重投影 8 个，因非正深度跳过 3 个
平均像素误差: 0.188596 px（基于 6 个带观测值的点）
最大像素误差: 1.13139 px (点 P2)
```