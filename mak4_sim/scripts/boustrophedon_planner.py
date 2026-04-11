#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D Boustrophedon (牛耕式) 全覆盖路径规划器
用于大棚内系统性扫描建图

功能:
  - 在指定3D区域内生成牛耕式覆盖路径
  - 支持多层高度覆盖
  - 发布路径到 /coverage_path 供导航使用
  - 可视化路径 markers

订阅:
  - /octomap_binary (octomap_msgs/Octomap): 3D占据地图
  - /mavros/local_position/pose: 当前位姿

发布:
  - /coverage_path (nav_msgs/Path): 覆盖路径
  - /coverage_markers (visualization_msgs/MarkerArray): 可视化
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class BoustrophedonPlanner:
    def __init__(self):
        rospy.init_node('boustrophedon_planner', anonymous=True)

        # --- 区域参数 (大棚内部) ---
        self.x_min = rospy.get_param('~x_min', -9.0)
        self.x_max = rospy.get_param('~x_max', 9.0)
        self.y_min = rospy.get_param('~y_min', -3.5)
        self.y_max = rospy.get_param('~y_max', 3.5)
        self.z_layers = rospy.get_param('~z_layers', [1.5, 2.5, 3.5])
        self.line_spacing = rospy.get_param('~line_spacing', 1.5)  # 行间距 (m)
        self.step_size = rospy.get_param('~step_size', 0.5)  # 沿行步进 (m)

        # --- 发布器 ---
        self.path_pub = rospy.Publisher('/coverage_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/coverage_markers', MarkerArray, queue_size=1, latch=True)

        # --- 当前位姿 ---
        self.current_pose = None
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        rospy.loginfo("[Boustrophedon] 等待位姿数据...")
        rospy.sleep(2.0)

        # --- 生成并发布路径 ---
        path = self.generate_path()
        self.publish_path(path)
        self.publish_markers(path)
        rospy.loginfo("[Boustrophedon] 覆盖路径已发布, 共 %d 个路点", len(path))

    def _pose_cb(self, msg):
        self.current_pose = msg

    def generate_path(self):
        """
        生成3D牛耕式覆盖路径
        在每个高度层, 沿X方向蛇形扫描:
          - 奇数行: y_min -> y_max
          - 偶数行: y_max -> y_min
        然后上升到下一层
        """
        waypoints = []

        # 计算扫描行的Y坐标
        y_lines = np.arange(self.y_min, self.y_max + self.line_spacing, self.line_spacing)

        for layer_idx, z in enumerate(self.z_layers):
            # 交替层的起始方向
            lines = y_lines if layer_idx % 2 == 0 else y_lines[::-1]

            for i, y in enumerate(lines):
                # 蛇形: 奇数行反向
                if i % 2 == 0:
                    x_range = np.arange(self.x_min, self.x_max + self.step_size, self.step_size)
                else:
                    x_range = np.arange(self.x_max, self.x_min - self.step_size, -self.step_size)

                for x in x_range:
                    waypoints.append((float(x), float(y), float(z)))

        return waypoints

    def publish_path(self, waypoints):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for x, y, z in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def publish_markers(self, waypoints):
        ma = MarkerArray()

        # 路径线
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "coverage_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color = ColorRGBA(0, 1, 0, 0.8)
        line_marker.pose.orientation.w = 1.0

        for x, y, z in waypoints:
            line_marker.points.append(Point(x, y, z))

        ma.markers.append(line_marker)

        # 起点/终点球
        for idx, (wp, color) in enumerate([(waypoints[0], ColorRGBA(0, 1, 0, 1)),
                                            (waypoints[-1], ColorRGBA(1, 0, 0, 1))]):
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "coverage_endpoints"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(wp[0], wp[1], wp[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color = color
            ma.markers.append(sphere)

        self.marker_pub.publish(ma)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = BoustrophedonPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
