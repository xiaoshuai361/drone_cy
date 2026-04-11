/*
 * pcd_to_2d_map_node.cpp
 * PCD → 2D占据栅格地图转换 (C++版)
 * 
 * 用法: rosrun mak4_sim_cpp pcd_to_2d_map --pcd /path/to/map.pcd --height 1.0
 */
#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <sys/stat.h>
#include <clocale>

struct Point3 { float x, y, z; };

std::vector<Point3> readPCD(const std::string &path)
{
    std::vector<Point3> pts;
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) { printf("[PCD→2D] 无法打开: %s\n", path.c_str()); return pts; }

    std::string line;
    int npoints = 0, xi = -1, yi = -1, zi = -1;
    std::string data_type = "ascii";
    int n_fields = 0;
    std::vector<int> field_sizes;

    while (std::getline(f, line)) {
        if (line.rfind("FIELDS", 0) == 0) {
            std::istringstream iss(line);
            std::string tok; iss >> tok; // skip "FIELDS"
            int idx = 0;
            while (iss >> tok) {
                if (tok == "x") xi = idx;
                else if (tok == "y") yi = idx;
                else if (tok == "z") zi = idx;
                idx++;
            }
            n_fields = idx;
        } else if (line.rfind("SIZE", 0) == 0) {
            std::istringstream iss(line);
            std::string tok; iss >> tok;
            int s;
            while (iss >> s) field_sizes.push_back(s);
        } else if (line.rfind("POINTS", 0) == 0) {
            npoints = std::stoi(line.substr(7));
        } else if (line.rfind("DATA", 0) == 0) {
            data_type = line.substr(5);
            // trim
            while (!data_type.empty() && (data_type.back() == '\r' || data_type.back() == ' '))
                data_type.pop_back();
            break;
        }
    }

    if (npoints == 0 || xi < 0) { printf("[PCD→2D] PCD格式错误\n"); return pts; }
    printf("[PCD→2D] 点数: %d, 格式: %s, 字段数: %d\n", npoints, data_type.c_str(), n_fields);

    pts.reserve(npoints);

    if (data_type == "binary") {
        int point_size = 0;
        if ((int)field_sizes.size() >= n_fields)
            for (int s : field_sizes) point_size += s;
        else
            point_size = n_fields * 4;

        std::vector<char> buf(npoints * point_size);
        f.read(buf.data(), buf.size());

        for (int i = 0; i < npoints; ++i) {
            float x, y, z;
            memcpy(&x, buf.data() + i * point_size + xi * 4, 4);
            memcpy(&y, buf.data() + i * point_size + yi * 4, 4);
            memcpy(&z, buf.data() + i * point_size + zi * 4, 4);
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
                pts.push_back({x, y, z});
        }
    } else {
        // ASCII
        for (int i = 0; i < npoints; ++i) {
            if (!std::getline(f, line)) break;
            std::istringstream iss(line);
            std::vector<float> vals;
            float v;
            while (iss >> v) vals.push_back(v);
            if ((int)vals.size() > std::max({xi, yi, zi})) {
                float x = vals[xi], y = vals[yi], z = vals[zi];
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
                    pts.push_back({x, y, z});
            }
        }
    }
    printf("[PCD→2D] 有效点: %zu\n", pts.size());
    return pts;
}

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    std::string home = getenv("HOME") ? getenv("HOME") : "/home/user";
    std::string pcd_path = home + "/catkin_ws/maps/greenhouse_map.pcd";
    double height = 1.0, tolerance = 0.3, resolution = 0.05, inflate = 0.2;
    std::string output_dir = home + "/catkin_ws/maps";
    std::string map_name = "greenhouse_2d";

    // 简单参数解析
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--pcd" && i+1 < argc) pcd_path = argv[++i];
        else if (arg == "--height" && i+1 < argc) height = atof(argv[++i]);
        else if (arg == "--tolerance" && i+1 < argc) tolerance = atof(argv[++i]);
        else if (arg == "--resolution" && i+1 < argc) resolution = atof(argv[++i]);
        else if (arg == "--inflate" && i+1 < argc) inflate = atof(argv[++i]);
        else if (arg == "--output" && i+1 < argc) output_dir = argv[++i];
        else if (arg == "--name" && i+1 < argc) map_name = argv[++i];
    }

    printf("============================================================\n");
    printf("  PCD → 2D 占据栅格地图转换工具 (C++版)\n");
    printf("============================================================\n");
    printf("PCD: %s\n", pcd_path.c_str());
    printf("高度: %.2fm, 切片: [%.2f, %.2f]m\n", height, height-tolerance, height+tolerance);
    printf("分辨率: %.3fm, 膨胀: %.2fm\n", resolution, inflate);

    // 读取PCD
    auto points = readPCD(pcd_path);
    if (points.empty()) { printf("[PCD→2D] 失败\n"); return 1; }

    float xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
    for (auto &p : points) {
        xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z);
    }
    printf("[PCD→2D] 范围: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]\n", xmin,xmax,ymin,ymax,zmin,zmax);

    // Z轴直通滤波
    double z_lo = height - tolerance, z_hi = height + tolerance;
    std::vector<Point3> slice;
    for (auto &p : points)
        if (p.z >= z_lo && p.z <= z_hi) slice.push_back(p);
    printf("[PCD→2D] Z轴切片 [%.2f,%.2f]: %zu → %zu\n", z_lo, z_hi, points.size(), slice.size());
    if (slice.empty()) { printf("[PCD→2D] 切片范围内无点!\n"); return 1; }

    // 计算2D范围
    float sx_min = 1e9, sx_max = -1e9, sy_min = 1e9, sy_max = -1e9;
    for (auto &p : slice) {
        sx_min = std::min(sx_min, p.x); sx_max = std::max(sx_max, p.x);
        sy_min = std::min(sy_min, p.y); sy_max = std::max(sy_max, p.y);
    }
    float margin = 1.0f;
    sx_min -= margin; sx_max += margin; sy_min -= margin; sy_max += margin;
    int width = (int)ceil((sx_max - sx_min) / resolution);
    int height_px = (int)ceil((sy_max - sy_min) / resolution);
    printf("[PCD→2D] 栅格: %dx%d (%.1fm x %.1fm)\n", width, height_px, sx_max-sx_min, sy_max-sy_min);

    // 创建栅格 (254=free)
    std::vector<uint8_t> grid(width * height_px, 254);

    // 投影点到栅格
    for (auto &p : slice) {
        int cx = (int)((p.x - sx_min) / resolution);
        int ry = (int)((p.y - sy_min) / resolution);
        if (cx >= 0 && cx < width && ry >= 0 && ry < height_px)
            grid[ry * width + cx] = 0;
    }

    // 膨胀
    if (inflate > 0) {
        int ic = (int)ceil(inflate / resolution);
        auto inflated = grid;
        for (int y = 0; y < height_px; ++y)
            for (int x = 0; x < width; ++x)
                if (grid[y * width + x] == 0)
                    for (int dy = -ic; dy <= ic; ++dy)
                        for (int dx = -ic; dx <= ic; ++dx)
                            if (dx*dx + dy*dy <= ic*ic) {
                                int nx = x+dx, ny = y+dy;
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height_px)
                                    inflated[ny * width + nx] = 0;
                            }
        grid = inflated;
    }

    int occ = 0, free = 0;
    for (auto v : grid) { if (v == 0) occ++; if (v == 254) free++; }
    printf("[PCD→2D] 占据: %d, 自由: %d\n", occ, free);

    // 保存PGM
    std::string mkdirCmd = "mkdir -p " + output_dir;
    system(mkdirCmd.c_str());
    std::string pgm_path = output_dir + "/" + map_name + ".pgm";
    std::string yaml_path = output_dir + "/" + map_name + ".yaml";

    {
        std::ofstream pgm(pgm_path, std::ios::binary);
        char header[128];
        snprintf(header, sizeof(header), "P5\n%d %d\n255\n", width, height_px);
        pgm.write(header, strlen(header));
        // 翻转Y轴
        for (int y = height_px - 1; y >= 0; --y)
            pgm.write(reinterpret_cast<char*>(grid.data() + y * width), width);
    }

    // 保存YAML
    {
        std::ofstream yf(yaml_path);
        yf << "image: " << map_name << ".pgm" << std::endl;
        char buf[256];
        snprintf(buf, sizeof(buf), "resolution: %.4f\norigin: [%.4f, %.4f, 0.0000]\n",
                 resolution, (double)sx_min, (double)sy_min);
        yf << buf;
        yf << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    }

    printf("[PCD→2D] ✓ PGM: %s (%dx%d)\n", pgm_path.c_str(), width, height_px);
    printf("[PCD→2D] ✓ YAML: %s\n", yaml_path.c_str());
    return 0;
}
