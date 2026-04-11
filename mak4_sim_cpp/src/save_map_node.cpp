/*
 * save_map_node.cpp
 * 地图保存: OctoMap (.bt) + Fast-LIO2 PCD (.pcd)
 * 运行一次后退出
 */
#include <ros/ros.h>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <clocale>

static bool fileExists(const std::string &path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

static long fileSize(const std::string &path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) return st.st_size;
    return 0;
}

static bool nodeRunning(const std::string &nodeName) {
    std::string cmd = "rosnode list 2>/dev/null | grep -q '" + nodeName + "'";
    return system(cmd.c_str()) == 0;
}

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "save_map");
    ros::NodeHandle nh("~");

    std::string home = getenv("HOME") ? getenv("HOME") : "/home/user";
    std::string save_dir = home + "/catkin_ws/maps";
    if (argc > 1) save_dir = argv[1];

    // mkdir -p
    std::string mkdirCmd = "mkdir -p " + save_dir;
    system(mkdirCmd.c_str());

    std::string bt_path = save_dir + "/greenhouse_map.bt";
    std::string pcd_path = save_dir + "/greenhouse_map.pcd";

    ROS_INFO("[SaveMap] 保存目录: %s", save_dir.c_str());

    // 检查前提
    if (!nodeRunning("/octomap_server")) {
        ROS_ERROR("[SaveMap] OctoMap Server 未运行! 请在 slam_mapping.launch 运行时执行");
        return 1;
    }

    // 1. 保存 OctoMap
    ROS_INFO("[SaveMap] 保存 OctoMap...");
    std::string octoCmd = "rosrun octomap_server octomap_saver -f " + bt_path + " 2>/dev/null";
    int ret = system(octoCmd.c_str());
    if (ret == 0 && fileExists(bt_path)) {
        double kb = fileSize(bt_path) / 1024.0;
        if (kb < 1.0) ROS_WARN("[SaveMap] OctoMap 文件太小 (%.1f KB), 地图可能为空!", kb);
        else ROS_INFO("[SaveMap] ✓ OctoMap: %s (%.1f KB)", bt_path.c_str(), kb);
    } else {
        ROS_WARN("[SaveMap] OctoMap 保存失败");
    }

    // 2. 停止 Fast-LIO2 触发 PCD 写入
    if (nodeRunning("/laserMapping")) {
        ROS_INFO("[SaveMap] 停止 Fast-LIO2 触发PCD写入...");
        system("rosnode kill /laserMapping 2>/dev/null");
        for (int i = 0; i < 15; ++i) {
            ros::Duration(1).sleep();
            std::string pcdSrc = home + "/catkin_ws/src/FAST_LIO/PCD/scans.pcd";
            if (fileExists(pcdSrc) && fileSize(pcdSrc) > 100) break;
        }
    }

    // 3. 复制 PCD
    std::string pcdSources[] = {
        home + "/catkin_ws/src/FAST_LIO/PCD/scans.pcd",
        home + "/.ros/scans.pcd",
        "/tmp/scans.pcd"
    };
    bool pcdFound = false;
    for (auto &src : pcdSources) {
        if (fileExists(src) && fileSize(src) > 100) {
            std::string cp = "cp " + src + " " + pcd_path;
            system(cp.c_str());
            double mb = fileSize(pcd_path) / (1024.0 * 1024.0);
            ROS_INFO("[SaveMap] ✓ PCD: %s (%.1f MB, from %s)", pcd_path.c_str(), mb, src.c_str());
            pcdFound = true;
            break;
        }
    }
    if (!pcdFound) ROS_WARN("[SaveMap] 未找到 PCD 文件");

    // 汇总
    ROS_INFO("=======================================================");
    ROS_INFO("[SaveMap] 地图保存完成!");
    bool btOk = fileExists(bt_path) && fileSize(bt_path) > 200;
    bool pcdOk = fileExists(pcd_path) && fileSize(pcd_path) > 100;
    ROS_INFO("  OctoMap: %s %s", bt_path.c_str(), btOk ? "(OK)" : "(失败)");
    ROS_INFO("  PCD:     %s %s", pcd_path.c_str(), pcdOk ? "(OK)" : "(未找到)");
    if (btOk) ROS_INFO("  下一步: roslaunch mak4_sim_cpp navigation.launch");
    ROS_INFO("=======================================================");

    return 0;
}
