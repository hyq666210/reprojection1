// 命令行程序：读数据 -> 求重投影 -> 打印表格与统计
//
// 为什么不把测试数据写死在代码里？
// 因为那样每改一个数据点都要重新编译。数据放在 data/points.txt 里，
// 改完直接重跑就行，报告里也能直接贴「输入 -> 输出」对照表。

#include "camera.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using reproj::CameraExtrinsics;
using reproj::CameraIntrinsics;
using reproj::ReprojectionResult;
using reproj::Vec3;

struct SamplePoint {
    std::string id;
    Vec3        world;
    bool        hasObs = false;
    double      uObs   = 0.0;
    double      vObs   = 0.0;
};

struct Dataset {
    CameraIntrinsics         K;
    CameraExtrinsics         ext;
    std::vector<SamplePoint> points;
};

// ------------------------------------------------------------ 格式化助手
std::string general(double v, int precision) {
    std::ostringstream os;
    os << std::setprecision(precision) << std::defaultfloat << v;
    return os.str();
}

std::string fixed(double v, int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

std::string scientific(double v, int precision) {
    std::ostringstream os;
    os << std::scientific << std::setprecision(precision) << v;
    return os.str();
}

// ------------------------------------------------------------ 解析
bool parseExtrinsics(std::istringstream& ss, CameraExtrinsics& ext) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            if (!(ss >> ext.R(r, c))) return false;
    return static_cast<bool>(ss >> ext.t.x >> ext.t.y >> ext.t.z);
}

bool readDataset(const std::string& path, Dataset& ds, std::string& err) {
    std::ifstream in(path);
    if (!in) {
        err = "打不开数据文件: " + path;
        return false;
    }

    bool        gotK   = false;
    bool        gotExt = false;
    std::string line;
    int         lineNo = 0;

    while (std::getline(in, line)) {
        ++lineNo;
        const std::string::size_type hash = line.find('#');  // 砍掉注释
        if (hash != std::string::npos) line.erase(hash);

        std::istringstream ss(line);
        std::string        tag;
        if (!(ss >> tag)) continue;  // 空行

        if (tag == "intrinsics") {
            if (!(ss >> ds.K.fx >> ds.K.fy >> ds.K.cx >> ds.K.cy)) {
                err = "第 " + std::to_string(lineNo) + " 行: intrinsics 需要 4 个数 (fx fy cx cy)";
                return false;
            }
            gotK = true;
        } else if (tag == "extrinsics") {
            if (!parseExtrinsics(ss, ds.ext)) {
                err = "第 " + std::to_string(lineNo) + " 行: extrinsics 需要 12 个数 (R00..R22 tx ty tz)";
                return false;
            }
            gotExt = true;
        } else if (tag == "point") {
            SamplePoint p;
            if (!(ss >> p.id >> p.world.x >> p.world.y >> p.world.z)) {
                err = "第 " + std::to_string(lineNo) + " 行: point 需要 id 和 3 个坐标";
                return false;
            }
            if (ss >> p.uObs >> p.vObs) p.hasObs = true;  // 观测值是可选的
            ds.points.push_back(p);
        } else {
            err = "第 " + std::to_string(lineNo) + " 行: 不认识的关键词 '" + tag + "'";
            return false;
        }
    }

    if (!gotK)             { err = "数据文件缺少 intrinsics 行";   return false; }
    if (!gotExt)           { err = "数据文件缺少 extrinsics 行";   return false; }
    if (ds.points.empty()) { err = "数据文件里没有任何 point 行";  return false; }
    return true;
}

// 没给命令行参数时，自动找一份能打开的数据文件。
// 为什么需要这个：相对路径是相对「你敲命令时所在的目录」，
// 而不是可执行文件所在的目录。从项目根跑、从 build/ 里跑、
// 用 VSCode 的「运行」按钮跑，工作目录都不一样。
std::string defaultDataPath() {
    const char* candidates[] = {"data/points.txt", "../data/points.txt",
                                "../../data/points.txt", "points.txt"};
    for (const char* c : candidates) {
        std::ifstream probe(c);
        if (probe) return c;
    }
    return "data/points.txt";  // 一个都找不到，就报这个路径，让报错信息有意义
}

}  // namespace

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : defaultDataPath();

    Dataset     ds;
    std::string err;
    if (!readDataset(path, ds, err)) {
        std::cerr << "读数据失败: " << err << "\n";
        if (argc == 1) {
            std::cerr << "提示: 没找到默认数据文件。请显式指定路径，例如在项目根目录执行:\n"
                      << "      ./build/bin/reprojection_app data/points.txt\n";
        }
        return 1;
    }

    std::cout << "数据文件: " << path << "\n";
    std::cout << "内参: fx=" << general(ds.K.fx, 6) << " fy=" << general(ds.K.fy, 6)
              << " cx=" << general(ds.K.cx, 6) << " cy=" << general(ds.K.cy, 6) << "\n";
    std::cout << "外参约定: Pc = R * Pw + t  (世界 -> 相机)\n\n";

    std::cout << std::left << std::setw(16) << "id" << std::setw(32) << "world (Xw, Yw, Zw)"
              << std::setw(30) << "projected (u, v)" << std::setw(26) << "observed (u, v)"
              << std::setw(13) << "error(px)" << "status\n";
    std::cout << std::string(118, '-') << "\n";

    int         nProjected = 0;
    int         nSkipped   = 0;
    int         nWithErr   = 0;
    double      sumErr     = 0.0;
    double      maxErr     = 0.0;
    std::string maxErrId;

    for (const SamplePoint& p : ds.points) {
        const Vec3                              Pc  = reproj::worldToCamera(p.world, ds.ext);
        const std::optional<ReprojectionResult> res = reproj::projectPoint(Pc, ds.K);

        const std::string worldStr = "(" + general(p.world.x, 10) + ", " + general(p.world.y, 10) +
                                     ", " + general(p.world.z, 10) + ")";

        std::string projStr = "-";
        std::string obsStr  = "-";
        std::string errStr  = "-";
        std::string status;

        if (res) {
            ++nProjected;
            status = "OK";

            if (std::fabs(res->u) > 1.0e6 || std::fabs(res->v) > 1.0e6) {
                // Zc 极接近 0 时透视除法会算出天文数字，直接打印没有可读性
                projStr = "(u=" + scientific(res->u, 4) + ", v=" + scientific(res->v, 4) + ")";
                status += " [数值爆炸: Zc=" + scientific(res->Zc, 3) + "]";
            } else {
                projStr = "(" + fixed(res->u, 4) + ", " + fixed(res->v, 4) + ")";
            }

            if (p.hasObs) {
                obsStr      = "(" + fixed(p.uObs, 4) + ", " + fixed(p.vObs, 4) + ")";
                const double e = reproj::pixelDistance(res->u, res->v, p.uObs, p.vObs);
                errStr      = scientific(e, 5);
                sumErr += e;
                ++nWithErr;
                if (e > maxErr) {
                    maxErr   = e;
                    maxErrId = p.id;
                }
            }
        } else {
            ++nSkipped;
            status = "SKIPPED: Zc=" + general(Pc.z, 6) + " <= 0 (点在相机背后或光心平面)";
        }

        std::cout << std::left << std::setw(16) << p.id << std::setw(32) << worldStr
                  << std::setw(30) << projStr << std::setw(26) << obsStr << std::setw(13) << errStr
                  << status << "\n";
    }

    std::cout << std::string(118, '-') << "\n";
    std::cout << "统计: 成功重投影 " << nProjected << " 个，因非正深度跳过 " << nSkipped << " 个\n";
    if (nWithErr > 0) {
        std::cout << "平均像素误差: " << general(sumErr / nWithErr, 6) << " px（基于 " << nWithErr
                  << " 个带观测值的点）\n";
        std::cout << "最大像素误差: " << general(maxErr, 6) << " px (点 " << maxErrId << ")\n";
    } else {
        std::cout << "没有带观测值的点，无法统计误差。\n";
    }
    return 0;
}
