#pragma once

// 最小向量/矩阵工具。本作业刻意不依赖 Eigen，零第三方依赖，
// clone 下来就能编 —— 这样才能亲手写出 Pc = R * Pw + t。

#include <cmath>

namespace reproj {

// ---------------------------------------------------------------- Vec3
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

// ---------------------------------------------------------------- Mat3
// 行主序：m[行][列]。也就是按数学书上那样写，第一行就是 m[0][0..2]。
// 数据文件里的 9 个数也按这个顺序排列。
struct Mat3 {
    double m[3][3] = {{1.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0},
                      {0.0, 0.0, 1.0}};

    double&       operator()(int r, int c)       { return m[r][c]; }
    const double& operator()(int r, int c) const { return m[r][c]; }

    static Mat3 identity() { return Mat3{}; }
};

// 矩阵乘向量：R * Pw，也就是 Pc 的旋转部分
inline Vec3 operator*(const Mat3& R, const Vec3& v) {
    return {R(0, 0) * v.x + R(0, 1) * v.y + R(0, 2) * v.z,
            R(1, 0) * v.x + R(1, 1) * v.y + R(1, 2) * v.z,
            R(2, 0) * v.x + R(2, 1) * v.y + R(2, 2) * v.z};
}

// 矩阵乘矩阵（测试里验证 R^T * R = I 要用）
inline Mat3 operator*(const Mat3& A, const Mat3& B) {
    Mat3 C;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A(i, k) * B(k, j);
            C(i, j) = s;
        }
    }
    return C;
}

inline Mat3 transpose(const Mat3& A) {
    Mat3 T;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) T(i, j) = A(j, i);
    return T;
}

inline double determinant(const Mat3& A) {
    return A(0, 0) * (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1))
         - A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0))
         + A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));
}

// ------------------------------------------------- 绕各轴旋转（角度单位：弧度）
// 这三个函数不是重投影本身需要的，而是用来「构造一个已知姿态的相机」，
// 这样自造测试数据时才能一句话讲清相机站在哪、朝哪看。
inline Mat3 rotationX(double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    Mat3 R;
    R(0, 0) = 1; R(0, 1) = 0; R(0, 2) = 0;
    R(1, 0) = 0; R(1, 1) = c; R(1, 2) = -s;
    R(2, 0) = 0; R(2, 1) = s; R(2, 2) = c;
    return R;
}

inline Mat3 rotationY(double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    Mat3 R;
    R(0, 0) = c; R(0, 1) = 0; R(0, 2) = s;
    R(1, 0) = 0; R(1, 1) = 1; R(1, 2) = 0;
    R(2, 0) = -s; R(2, 1) = 0; R(2, 2) = c;
    return R;
}

inline Mat3 rotationZ(double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    Mat3 R;
    R(0, 0) = c; R(0, 1) = -s; R(0, 2) = 0;
    R(1, 0) = s; R(1, 1) = c; R(1, 2) = 0;
    R(2, 0) = 0; R(2, 1) = 0; R(2, 2) = 1;
    return R;
}

}  // namespace reproj
