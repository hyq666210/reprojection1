#include "camera.hpp"

#include <cmath>

namespace reproj {

Vec3 worldToCamera(const Vec3& Pw, const CameraExtrinsics& ext) {
    // Pc = R * Pw + t
    return ext.R * Pw + ext.t;
}

std::optional<ReprojectionResult> projectPoint(const Vec3& Pc, const CameraIntrinsics& K) {
    // 异常处理 1：深度必须为正。
    // 写成 !(z > 0) 而不是 (z <= 0)：如果 Pc.z 是 NaN，
    // NaN <= 0 是 false（漏掉），而 !(NaN > 0) 是 true（拦住）。
    if (!(Pc.z > 0.0)) {
        return std::nullopt;
    }

    // 异常处理 2：坐标必须是有限值，防止 inf 一路传播到像素坐标。
    if (!std::isfinite(Pc.x) || !std::isfinite(Pc.y) || !std::isfinite(Pc.z)) {
        return std::nullopt;
    }

    const double x = Pc.x / Pc.z;  // 透视除法
    const double y = Pc.y / Pc.z;

    ReprojectionResult out;
    out.u  = K.fx * x + K.cx;
    out.v  = K.fy * y + K.cy;
    out.Zc = Pc.z;
    return out;
}

std::optional<ReprojectionResult> reproject(const Vec3& Pw, const CameraIntrinsics& K,
                                            const CameraExtrinsics& ext) {
    return projectPoint(worldToCamera(Pw, ext), K);
}

double pixelDistance(double u1, double v1, double u2, double v2) {
    const double du = u1 - u2;
    const double dv = v1 - v2;
    return std::sqrt(du * du + dv * dv);
}

}  // namespace reproj
