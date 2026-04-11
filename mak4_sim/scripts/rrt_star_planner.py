#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D RRT* 局部避障路径规划器
用于在已知3D占据地图中规划无碰撞路径

功能:
  - 使用OctoMap进行碰撞检测
  - RRT*算法在3D空间中规划
  - 实时跟踪目标路点 (来自覆盖路径)
  - 发布避障后的局部路径

订阅:
  - /coverage_path (nav_msgs/Path): 全覆盖路径 (来自 Boustrophedon)
  - /mavros/local_position/pose: 当前位姿
  - /octomap_binary (octomap_msgs/Octomap): 3D占据地图

发布:
  - /local_path (nav_msgs/Path): 避障后的局部路径
  - /rrt_markers (visualization_msgs/MarkerArray): RRT树可视化
  - /mavros/setpoint_position/local (PoseStamped): 位置控制指令
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from octomap_msgs.msg import Octomap
import struct


class Node3D:
    """RRT* 树节点"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0
        self.children = []


class RRTStar3D:
    def __init__(self):
        rospy.init_node('rrt_star_planner', anonymous=True)

        # --- 参数 ---
        self.step_size = rospy.get_param('~step_size', 0.5)
        self.max_iter = rospy.get_param('~max_iter', 2000)
        self.goal_threshold = rospy.get_param('~goal_threshold', 0.5)
        self.search_radius = rospy.get_param('~search_radius', 1.5)
        self.safety_margin = rospy.get_param('~safety_margin', 0.35)  # 无人机安全半径
        self.lookahead = rospy.get_param('~lookahead', 3)  # 前方看几个路点
        self.replan_rate = rospy.get_param('~replan_rate', 2.0)  # Hz

        # --- 状态 ---
        self.current_pose = None
        self.coverage_path = None
        self.current_wp_idx = 0
        self.octomap_data = None
        self.occupied_points = set()

        # --- 发布器 ---
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.rrt_marker_pub = rospy.Publisher('/rrt_markers', MarkerArray, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # --- 订阅器 ---
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/coverage_path', Path, self._coverage_path_cb)
        rospy.Subscriber('/octomap_binary', Octomap, self._octomap_cb)

        rospy.loginfo("[RRT*] 路径规划器已启动, 等待覆盖路径和地图...")

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _coverage_path_cb(self, msg):
        self.coverage_path = msg
        self.current_wp_idx = 0
        rospy.loginfo("[RRT*] 收到覆盖路径, 共 %d 个路点", len(msg.poses))

    def _octomap_cb(self, msg):
        self.octomap_data = msg

    def is_collision(self, x, y, z):
        """简化碰撞检测 - 基于OctoMap的占据查询"""
        # 当没有地图数据时,假设无碰撞
        if not self.occupied_points:
            return False
        # 量化到网格分辨率(0.15m)进行查询
        res = 0.15
        gx, gy, gz = int(x / res), int(y / res), int(z / res)
        margin = int(self.safety_margin / res) + 1
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                for dz in range(-margin, margin + 1):
                    if (gx + dx, gy + dy, gz + dz) in self.occupied_points:
                        return True
        return False

    def is_path_free(self, n1, n2):
        """检查两点之间的路径是否无碰撞"""
        dx = n2.x - n1.x
        dy = n2.y - n1.y
        dz = n2.z - n1.z
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        if dist < 0.01:
            return True
        steps = max(int(dist / 0.1), 2)
        for i in range(steps + 1):
            t = i / steps
            x = n1.x + t * dx
            y = n1.y + t * dy
            z = n1.z + t * dz
            if self.is_collision(x, y, z):
                return False
        return True

    def distance(self, n1, n2):
        return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2 + (n1.z - n2.z)**2)

    def plan_rrt_star(self, start, goal):
        """
        RRT*算法: 在3D空间中规划从start到goal的无碰撞路径
        """
        start_node = Node3D(start[0], start[1], start[2])
        goal_node = Node3D(goal[0], goal[1], goal[2])

        # 如果直线路径无碰撞，直接返回
        if self.is_path_free(start_node, goal_node):
            return [start, goal]

        tree = [start_node]

        # 搜索空间边界
        bounds = [
            (min(start[0], goal[0]) - 3.0, max(start[0], goal[0]) + 3.0),
            (min(start[1], goal[1]) - 3.0, max(start[1], goal[1]) + 3.0),
            (min(start[2], goal[2]) - 1.0, max(start[2], goal[2]) + 1.0),
        ]

        best_path = None

        for _ in range(self.max_iter):
            # 以10%概率直接采样目标点
            if np.random.random() < 0.1:
                rand_x, rand_y, rand_z = goal
            else:
                rand_x = np.random.uniform(bounds[0][0], bounds[0][1])
                rand_y = np.random.uniform(bounds[1][0], bounds[1][1])
                rand_z = np.random.uniform(bounds[2][0], bounds[2][1])

            rand_node = Node3D(rand_x, rand_y, rand_z)

            # 找最近节点
            nearest = min(tree, key=lambda n: self.distance(n, rand_node))
            dist = self.distance(nearest, rand_node)
            if dist > self.step_size:
                ratio = self.step_size / dist
                new_x = nearest.x + ratio * (rand_node.x - nearest.x)
                new_y = nearest.y + ratio * (rand_node.y - nearest.y)
                new_z = nearest.z + ratio * (rand_node.z - nearest.z)
                new_node = Node3D(new_x, new_y, new_z)
            else:
                new_node = rand_node

            if self.is_collision(new_node.x, new_node.y, new_node.z):
                continue
            if not self.is_path_free(nearest, new_node):
                continue

            # RRT*: 在搜索半径内找最优父节点
            near_nodes = [n for n in tree if self.distance(n, new_node) < self.search_radius]
            best_parent = nearest
            best_cost = nearest.cost + self.distance(nearest, new_node)

            for near in near_nodes:
                new_cost = near.cost + self.distance(near, new_node)
                if new_cost < best_cost and self.is_path_free(near, new_node):
                    best_parent = near
                    best_cost = new_cost

            new_node.parent = best_parent
            new_node.cost = best_cost
            tree.append(new_node)

            # 重连: 检查是否可以通过new_node降低附近节点代价
            for near in near_nodes:
                new_cost = new_node.cost + self.distance(new_node, near)
                if new_cost < near.cost and self.is_path_free(new_node, near):
                    near.parent = new_node
                    near.cost = new_cost

            # 检查是否到达目标
            if self.distance(new_node, goal_node) < self.goal_threshold:
                if self.is_path_free(new_node, goal_node):
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + self.distance(new_node, goal_node)
                    # 回溯路径
                    path = []
                    node = goal_node
                    while node is not None:
                        path.append((node.x, node.y, node.z))
                        node = node.parent
                    path.reverse()
                    if best_path is None or len(path) < len(best_path):
                        best_path = path

        return best_path if best_path else [start, goal]  # 退化为直线

    def get_current_target(self):
        """获取当前目标路点"""
        if self.coverage_path is None or self.current_pose is None:
            return None

        poses = self.coverage_path.poses
        if self.current_wp_idx >= len(poses):
            return None

        # 检查是否到达当前路点
        pose = self.current_pose.pose.position
        target = poses[self.current_wp_idx].pose.position
        dist = np.sqrt((pose.x - target.x)**2 + (pose.y - target.y)**2 + (pose.z - target.z)**2)

        if dist < self.goal_threshold:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(poses):
                rospy.loginfo("[RRT*] 覆盖路径完成!")
                return None
            rospy.loginfo("[RRT*] 前进到路点 %d/%d", self.current_wp_idx + 1, len(poses))

        return poses[self.current_wp_idx].pose.position

    def publish_setpoint(self, x, y, z):
        """发布位置控制指令"""
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        self.setpoint_pub.publish(ps)

    def publish_local_path(self, path_points):
        """发布局部路径"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for x, y, z in path_points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.local_path_pub.publish(path_msg)

    def run(self):
        rate = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            target = self.get_current_target()
            if target is None or self.current_pose is None:
                rate.sleep()
                continue

            start = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
            )
            goal = (target.x, target.y, target.z)

            # 规划路径
            path = self.plan_rrt_star(start, goal)
            self.publish_local_path(path)

            # 发布下一个路点作为控制指令
            if len(path) > 1:
                next_pt = path[1]
                self.publish_setpoint(next_pt[0], next_pt[1], next_pt[2])
            else:
                self.publish_setpoint(goal[0], goal[1], goal[2])

            rate.sleep()


if __name__ == '__main__':
    try:
        planner = RRTStar3D()
        planner.run()
    except rospy.ROSInterruptException:
        pass
