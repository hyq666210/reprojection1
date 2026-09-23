// 单元测试：不依赖 gtest，自带极简断言框架，零依赖，clone 下来就能跑。
//
// 六个测试组，每一组对应报告里的一个论证点：
//   组 1  光轴上的点必须落在主点      —— 证明公式没写错
//   组 2  旋转与平移的方向             —— 符号写反了这里会挂
//   组 3  像素欧氏距离                 —— 基本的几何性质
//   组 4  异常输入                     —— 非正深度/NaN/inf 必须被拦住，但 1e-9 不能误杀
//   组 5  已知偏移量的误差回归          —— 不是「差不多」，是构造已知真值
//   组 6  旋转矩阵自检                 —— 确认 R 是合法的旋转矩阵

#include "camera.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_passed = 0;
int g_failed = 0;

void expectNear(const std::string& name, double actual, double expected, double tol) {
    const double diff = std::fabs(actual - expected);
    if (diff <= tol) {
        ++g_passed;
        std::cout << "[PASS] " << name << "\n";
    } else {
        ++g_failed;
        std::cout << "[FAIL] " << name << "\n"
                  << "         actual   = " << actual << "\n"
                  << "         expected = " << expected << "\n"
                  << "         diff     = " << diff << "  (tol = " << tol << ")\n";
    }
}

void expectTrue(const std::string& name, bool cond) {
    if (cond) {
        ++g_passed;
        std::cout << "[PASS] " << name << "\n";
    } else {
        ++g_failed;
        std::cout << "[FAIL] " << name << "\n";
    }
}

using namespace reproj;

const double kPi  = 3.14159265358979323846;
const double kDeg = kPi / 180.0;

const CameraIntrinsics kK{900.0, 900.0, 640.0, 360.0};

CameraExtrinsics identityExt() {
    return CameraExtrinsics{Mat3::identity(), Vec3{0.0, 0.0, 0.0}};
}

// 与 data/points.txt 里的外参一致：R = Rx(3deg) * Rz(2deg), t = (0.05, -0.03, 0.08)
CameraExtrinsics datasetExt() {
    return CameraExtrinsics{rotationX(3.0 * kDeg) * rotationZ(2.0 * kDeg), Vec3{0.05, -0.03, 0.08}};
}

double round4(double v) { return std::round(v * 1.0e4) / 1.0e4; }

void expectInvalid(const std::string& name, const Vec3& Pw, const CameraIntrinsics& K,
                   const CameraExtrinsics& ext) {
    expectTrue(name + "  ->  应判定为不可投影", !reproject(Pw, K, ext).has_value());
}

void section(const std::string& title) {
    std::cout << "\n--- " << title << " ---\n";
}

// ---------------------------------------------------------------- 组 1
void group1_optical_axis() {
    section("组 1：光轴上的点必须精确落在主点");
    const CameraExtrinsics ext = identityExt();

    for (double depth : {1.0, 5.0, 100.0}) {
        const auto r = reproject(Vec3{0.0, 0.0, depth}, kK, ext);
        const std::string tag = "1.x 光轴点 z=" + std::to_string(static_cast<int>(depth));
        expectTrue(tag + " 有效", r.has_value());
        if (r) {
            expectNear(tag + " u == cx", r->u, 640.0, 1e-12);
            expectNear(tag + " v == cy", r->v, 360.0, 1e-12);
        }
    }

    // fx 翻倍，像素偏移量应翻倍（线性关系）
    CameraIntrinsics K2 = kK;
    K2.fx = 1800.0;
    K2.fy = 1800.0;
    const auto a = reproject(Vec3{1.0, 0.0, 2.0}, kK, ext);
    const auto b = reproject(Vec3{1.0, 0.0, 2.0}, K2, ext);
    expectTrue("1.4 fx 翻倍后两个结果都有效", a.has_value() && b.has_value());
    if (a && b) expectNear("1.5 fx 翻倍 -> (u-cx) 翻倍", b->u - 640.0, 2.0 * (a->u - 640.0), 1e-9);

    // 深度翻倍，像素偏移量应减半
    const auto c = reproject(Vec3{1.0, 0.0, 4.0}, kK, ext);
    expectNear("1.6 深度翻倍 -> 像素偏移减半", 2.0 * (c->u - 640.0), a->u - 640.0, 1e-9);
}

// ---------------------------------------------------------------- 组 2
void group2_rotation_translation() {
    section("组 2：旋转与平移的方向");

    const CameraExtrinsics ext = identityExt();

    // 绕 Z 轴转 90 度：+X 应该被转到 +Y。Pw=(2,0,3) -> Pc=(0,2,3)
    CameraExtrinsics rz90 = ext;
    rz90.R               = rotationZ(90.0 * kDeg);
    const auto p1        = reproject(Vec3{2.0, 0.0, 3.0}, kK, rz90);
    expectTrue("2.1 Rz(90) * (2,0,3) 有效", p1.has_value());
    if (p1) {
        expectNear("2.2 Rz(90) 后 u 回到主点", p1->u, 640.0, 1e-9);
        expectNear("2.3 Rz(90) 后 v = 900*(2/3)+360", p1->v, 960.0, 1e-9);
    }

    // 反向：+Y 应该被转到 -X。Pw=(0,2,3) -> Pc=(-2,0,3)
    const auto p2 = reproject(Vec3{0.0, 2.0, 3.0}, kK, rz90);
    expectTrue("2.4 Rz(90) * (0,2,3) 有效", p2.has_value());
    if (p2) {
        expectNear("2.5 Rz(90) 后 u = 640-600", p2->u, 40.0, 1e-9);
        expectNear("2.6 Rz(90) 后 v 回到主点", p2->v, 360.0, 1e-9);
    }

    // 纯平移在 z 方向：等于把点推远，但光轴上的点结果不变
    CameraExtrinsics tz = ext;
    tz.t                = Vec3{0.0, 0.0, 1.0};
    const auto p3       = reproject(Vec3{0.0, 0.0, 4.0}, kK, tz);
    expectTrue("2.7 t=(0,0,1) 把 z=4 变成 z=5 仍有效", p3.has_value());
    if (p3) {
        expectNear("2.8 光轴点平移后仍在主点 (u)", p3->u, 640.0, 1e-12);
        expectNear("2.9 光轴点平移后仍在主点 (v)", p3->v, 360.0, 1e-12);
        expectNear("2.10 深度确实是 5", p3->Zc, 5.0, 1e-12);
    }

    // 纯平移在 x 方向：横向移动等价于横向平移像素
    CameraExtrinsics tx = ext;
    tx.t                = Vec3{1.0, 0.0, 0.0};
    const auto p4       = reproject(Vec3{0.0, 0.0, 1.0}, kK, tx);
    expectTrue("2.11 t=(1,0,0) 有效", p4.has_value());
    if (p4) expectNear("2.12 平移 1 m / 深 1 m -> 偏移 900 px", p4->u, 1540.0, 1e-9);

    // 与数据文件里同一组外参做交叉验证（数值来自独立计算）
    const auto p5 = reproject(Vec3{0.0, 0.0, 5.0}, kK, datasetExt());
    expectTrue("2.13 数据集外参下 P1 有效", p5.has_value());
    if (p5) {
        expectNear("2.14 数据集外参 P1 的 u", p5->u, 648.8702326236, 1e-6);
        expectNear("2.15 数据集外参 P1 的 v", p5->v, 308.2546497808, 1e-6);
    }
}

// ---------------------------------------------------------------- 组 3
void group3_pixel_distance() {
    section("组 3：像素欧氏距离");

    expectNear("3.1 3-4-5 直角三角形", pixelDistance(0.0, 0.0, 3.0, 4.0), 5.0, 1e-12);
    expectNear("3.2 同一个点距离为 0", pixelDistance(1.0, 1.0, 1.0, 1.0), 0.0, 1e-12);
    expectNear("3.3 交换顺序结果不变",
               pixelDistance(10.0, 20.0, 13.0, 24.0) - pixelDistance(13.0, 24.0, 10.0, 20.0), 0.0,
               1e-12);
    expectNear("3.4 对角线 sqrt(2)", pixelDistance(10.0, 10.0, 9.0, 11.0), std::sqrt(2.0), 1e-12);
    expectNear("3.5 纯水平距离", pixelDistance(100.0, 100.0, 103.0, 100.0), 3.0, 1e-12);
    expectNear("3.6 纯垂直距离", pixelDistance(100.0, 100.0, 100.0, 103.0), 3.0, 1e-12);
}

// ---------------------------------------------------------------- 组 4
void group4_invalid_inputs() {
    section("组 4：异常输入");
    const CameraExtrinsics ext = identityExt();

    expectInvalid("4.1 点在相机背后 Zc=-2", Vec3{0.0, 0.0, -2.0}, kK, ext);
    expectInvalid("4.2 点落在光心平面上 Zc=0", Vec3{0.0, 0.0, 0.0}, kK, ext);
    expectInvalid("4.3 极小的负深度 Zc=-1e-12", Vec3{0.0, 0.0, -1.0e-12}, kK, ext);
    expectInvalid("4.4 坐标含 NaN", Vec3{NAN, 0.0, 1.0}, kK, ext);
    expectInvalid("4.5 坐标含 +inf", Vec3{INFINITY, 0.0, 1.0}, kK, ext);
    expectInvalid("4.6 坐标含 -inf", Vec3{0.0, -INFINITY, 1.0}, kK, ext);
    expectTrue("4.7 projectPoint 直接吃 NaN 深度也应被拦住",
               !projectPoint(Vec3{0.0, 0.0, NAN}, kK).has_value());

    // 反向确认：正的极小深度不能被误杀，否则就是「过度防御」
    const auto tiny = reproject(Vec3{0.0, 0.0, 1.0e-9}, kK, ext);
    expectTrue("4.8 Zc=1e-9 且 Xc=Yc=0 必须仍然有效（不能误杀）", tiny.has_value());
    if (tiny) {
        expectNear("4.9 此时仍精确落在主点 (u)", tiny->u, 640.0, 1e-12);
        expectNear("4.10 此时仍精确落在主点 (v)", tiny->v, 360.0, 1e-12);
    }

    const auto blowup = reproject(Vec3{1.0, 0.0, 1.0e-9}, kK, ext);
    expectTrue("4.11 Zc=1e-9 且有横向偏移时仍然有效（判据只看正负）", blowup.has_value());
    if (blowup) expectTrue("4.12 但结果大到天文数字，需要上层做可读化处理",
                           std::fabs(blowup->u) > 1.0e6);
}

// ---------------------------------------------------------------- 组 5
void group5_error_formula() {
    section("组 5：已知偏移量的误差回归");
    const CameraExtrinsics ext = identityExt();

    // 手算：Pc = (0.5, 0.3, 6) -> u = 900*0.5/6+640 = 715, v = 900*0.3/6+360 = 405
    const auto r = reproject(Vec3{0.5, 0.3, 6.0}, kK, ext);
    expectTrue("5.1 基准点有效", r.has_value());
    if (!r) return;

    expectNear("5.2 手算 u", r->u, 715.0, 1e-9);
    expectNear("5.3 手算 v", r->v, 405.0, 1e-9);

    // 注入已知偏移 (-0.8, +0.8)，理论误差 = sqrt(0.8^2 + 0.8^2)
    const double kTheoretical = std::sqrt(0.8 * 0.8 + 0.8 * 0.8);  // 1.13137085...
    expectNear("5.4 已知偏移的理论值 = sqrt(1.28)",
               pixelDistance(715.0, 405.0, 714.2, 405.8), kTheoretical, 1e-12);

    // 观测值等于投影值 -> 误差必须精确为 0
    expectNear("5.5 观测值等于投影值时误差为 0", pixelDistance(r->u, r->v, r->u, r->v), 0.0, 1e-15);

    // 模拟数据文件的做法：观测值保留 4 位小数，误差应落在 sqrt(1.28) 附近（1e-5 量级的舍入）
    const auto rd = reproject(Vec3{0.5, 0.3, 6.0}, kK, datasetExt());
    expectTrue("5.6 数据集外参下的 P2 有效", rd.has_value());
    if (rd) {
        const double err = pixelDistance(rd->u, rd->v, round4(rd->u - 0.8), round4(rd->v + 0.8));
        expectNear("5.7 加入观测值舍入后仍贴合 sqrt(1.28)", err, kTheoretical, 1e-4);
    }
}

// ---------------------------------------------------------------- 组 6
void group6_rotation_sanity() {
    section("组 6：旋转矩阵自检（R^T * R = I, det = 1）");

    const Mat3 R = datasetExt().R;
    const Mat3 M = transpose(R) * R;

    bool orthogonal = true;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const double expected = (i == j) ? 1.0 : 0.0;
            if (std::fabs(M(i, j) - expected) > 1e-12) orthogonal = false;
        }
    expectTrue("6.1 数据集外参的 R 满足 R^T * R = I", orthogonal);
    expectNear("6.2 数据集外参的 det(R) = 1", determinant(R), 1.0, 1e-12);

    expectNear("6.3 rotationX(0.7) 的 det = 1", determinant(rotationX(0.7)), 1.0, 1e-12);
    expectNear("6.4 rotationY(-1.2) 的 det = 1", determinant(rotationY(-1.2)), 1.0, 1e-12);
    expectNear("6.5 rotationZ(2.5) 的 det = 1", determinant(rotationZ(2.5)), 1.0, 1e-12);

    // R * (R^T * v) 必须还原 v
    const Vec3 v{0.3, -0.7, 1.1};
    const Vec3 back = R * (transpose(R) * v);
    expectNear("6.6 R * (R^T * v) 还原 v.x", back.x, v.x, 1e-12);
    expectNear("6.7 R * (R^T * v) 还原 v.y", back.y, v.y, 1e-12);
    expectNear("6.8 R * (R^T * v) 还原 v.z", back.z, v.z, 1e-12);
}

}  // namespace

int main() {
    group1_optical_axis();
    group2_rotation_translation();
    group3_pixel_distance();
    group4_invalid_inputs();
    group5_error_formula();
    group6_rotation_sanity();

    std::cout << "\n================ 汇总 ================\n";
    std::cout << "通过: " << g_passed << "  失败: " << g_failed << "\n";
    if (g_failed == 0) {
        std::cout << "全部用例通过。\n";
        return 0;
    }
    std::cout << "存在失败用例，请检查上面的 [FAIL] 输出。\n";
    return 1;
}
