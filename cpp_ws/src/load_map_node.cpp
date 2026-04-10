/*
 * load_map_node.cpp
 * 地图加载提示节点 (C++版)
 * 实际加载由 navigation.launch 完成
 */
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <clocale>

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "load_map");
    ros::NodeHandle nh;

    std::string home = getenv("HOME") ? getenv("HOME") : "/home/user";
    std::string bt_path = home + "/catkin_ws/maps/greenhouse_map.bt";
    if (argc > 1) bt_path = argv[1];

    struct stat st;
    if (stat(bt_path.c_str(), &st) != 0) {
        ROS_ERROR("[LoadMap] 地图不存在: %s", bt_path.c_str());
        ROS_ERROR("[LoadMap] 请先运行 save_map 保存地图");
        return 1;
    }

    ROS_INFO("[LoadMap] 地图: %s (%.1f KB)", bt_path.c_str(), st.st_size / 1024.0);
    ROS_INFO("[LoadMap] 请使用: roslaunch mak4_sim_cpp navigation.launch map_file:=%s", bt_path.c_str());

    return 0;
}
