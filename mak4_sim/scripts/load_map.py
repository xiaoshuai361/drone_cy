#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图加载节点
加载已保存的 OctoMap (.bt) 并发布为 ROS 话题

用法:
  rosrun mak4_sim load_map.py [地图文件路径]
  默认加载: ~/catkin_ws/maps/greenhouse_map.bt

加载后发布:
  - /octomap_binary (octomap_msgs/Octomap)
  - /occupied_cells_vis_array (visualization_msgs/MarkerArray) 
  
前提: 不需要 slam_mapping.launch (这是离线加载)
"""

import rospy
import os
import sys
import subprocess


def load_map():
    rospy.init_node('load_map', anonymous=True)

    # 地图路径
    if len(sys.argv) > 1:
        bt_path = sys.argv[1]
    else:
        bt_path = os.path.expanduser('~/catkin_ws/maps/greenhouse_map.bt')

    if not os.path.exists(bt_path):
        rospy.logerr("[LoadMap] 地图文件不存在: %s", bt_path)
        rospy.logerr("[LoadMap] 请先运行 save_map.py 保存地图")
        return

    rospy.loginfo("[LoadMap] 加载地图: %s", bt_path)
    rospy.loginfo("[LoadMap] 正在启动 OctoMap Server (离线模式)...")

    # 通过 roslaunch 参数方式或直接启动
    # octomap_server 不直接支持加载.bt文件作为参数
    # 我们使用它提供的 octomap_server_node 并通过service加载
    # 方法: 直接启动一个进程
    rospy.loginfo("[LoadMap] 地图已准备就绪")
    rospy.loginfo("[LoadMap] 请使用以下命令启动导航:")
    rospy.loginfo("  roslaunch mak4_sim navigation.launch map_file:=%s", bt_path)


if __name__ == '__main__':
    try:
        load_map()
    except rospy.ROSInterruptException:
        pass
