#pragma once

// 相机模型与重投影接口。这个文件是「说明书」：只放声明，不放实现。
// 别人想看这个模块能干什么，读这一个文件就够了。

#include <optional>

#include "vec_mat.hpp"

namespace reproj {

// 相机内参（理想针孔模型，无畸变）
struct CameraIntrinsics {
    double fx = 0.0;  // 焦距（像素）
    double fy = 0.0;  // 焦距（像素）
    double cx = 0.0;  // 主点 x（像素）
    double cy = 0.0;  // 主点 y（像素）
};

// 相机外参：世界坐标系 -> 相机坐标系，Pc = R * Pw + t
struct CameraExtrinsics {
    Mat3 R;  // 3x3 旋转矩阵（行主序）
    Vec3 t;  // 3x1 平移向量，单位与 Pw 一致
};

// 一次重投影的结果
struct ReprojectionResult {
    double u  = 0.0;  // 像素横坐标
    double v  = 0.0;  // 像素纵坐标
    double Zc = 0.0;  // 相机坐标系下的深度，方便调试和打印
};

// 第一步：世界坐标 -> 相机坐标
Vec3 worldToCamera(const Vec3& Pw, const CameraExtrinsics& ext);

// 第二步 + 第三步：相机坐标 -> 像素坐标。
// 深度非正或坐标非有限时返回 std::nullopt，表示「这个点不可投影」。
std::optional<ReprojectionResult> projectPoint(const Vec3& Pc, const CameraIntrinsics& K);

// 上面两步的组合：一个三维点直接得到像素坐标
std::optional<ReprojectionResult> reproject(const Vec3& Pw, const CameraIntrinsics& K,
                                            const CameraExtrinsics& ext);

// 两个像素点之间的欧氏距离，单位：像素
double pixelDistance(double u1, double v1, double u2, double v2);

}  // namespace reproj
