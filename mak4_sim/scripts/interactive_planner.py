#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D 交互式路径规划 + 自动飞行导航

工作流程:
  1. 订阅 OctoMap 可视化标记 → 获取3D障碍物信息
  2. 订阅 RViz "Publish Point" → 获取3D目标点
  3. 用 RRT* 规划3D无碰撞路径
  4. 自动控制无人机沿路径飞行

RViz操作:
  - 在工具栏选择 "Publish Point"
  - 点击3D视图中的目标位置
  - 无人机自动规划路径并飞行
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class Node3D:
    """RRT* 树节点"""
    __slots__ = ['x', 'y', 'z', 'parent', 'cost']

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0


class InteractivePlanner:
    def __init__(self):
        rospy.init_node('interactive_planner', anonymous=True)

        # RRT* 参数
        self.step_size = rospy.get_param('~step_size', 0.8)
        self.max_iter = rospy.get_param('~max_iter', 3000)
        self.goal_threshold = rospy.get_param('~goal_threshold', 0.5)
        self.search_radius = rospy.get_param('~search_radius', 2.0)
        self.safety_margin = rospy.get_param('~safety_margin', 0.4)
        self.flight_speed = rospy.get_param('~flight_speed', 0.5)
        self.waypoint_threshold = rospy.get_param('~waypoint_threshold', 0.4)

        # 状态
        self.current_pose = None
        self.current_state = State()
        self.occupied_voxels = set()
        self.resolution = 0.15
        self.is_flying = False
        self.current_path = None
        self.current_wp_idx = 0
        self.map_ready = False

        # 发布器
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/rrt_markers', MarkerArray, queue_size=1)
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 订阅器
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/clicked_point', PointStamped, self._clicked_point_cb)
        # 订阅 OctoMap 可视化标记获取障碍物信息 (比解析二进制OctoMap简单可靠)
        rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, self._occupied_cb)

        # 服务
        rospy.wait_for_service('/mavros/cmd/arming', timeout=15)
        rospy.wait_for_service('/mavros/set_mode', timeout=15)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo("=" * 55)
        rospy.loginfo("[规划器] 3D交互式路径规划器已启动!")
        rospy.loginfo("[规划器] 在RViz中使用 'Publish Point' 工具点击目标位置")
        rospy.loginfo("[规划器] 等待OctoMap数据...")
        rospy.loginfo("=" * 55)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _occupied_cb(self, msg):
        """从 /occupied_cells_vis_array 提取所有占据体素坐标"""
        new_voxels = set()
        res = self.resolution
        for marker in msg.markers:
            sx = marker.scale.x if marker.scale.x > 0 else res
            # 获取体素分辨率 (取markers的scale)
            if not self.map_ready:
                self.resolution = sx
                res = sx
            for pt in marker.points:
                gx = int(round(pt.x / res))
                gy = int(round(pt.y / res))
                gz = int(round(pt.z / res))
                new_voxels.add((gx, gy, gz))

        self.occupied_voxels = new_voxels
        if not self.map_ready and len(new_voxels) > 0:
            self.map_ready = True
            rospy.loginfo("[规划器] OctoMap已加载! 占据体素数: %d (分辨率: %.3fm)",
                          len(new_voxels), self.resolution)

    def _clicked_point_cb(self, msg):
        """处理 RViz 中的点击目标"""
        goal = [msg.point.x, msg.point.y, msg.point.z]
        rospy.loginfo("[规划器] 收到目标点: (%.2f, %.2f, %.2f)", *goal)

        if not self.map_ready:
            rospy.logwarn("[规划器] OctoMap数据尚未加载, 将使用无障碍规划")

        # 确保目标高度合理
        if goal[2] < 0.5:
            goal[2] = max(1.5, self.current_pose.pose.position.z if self.current_pose else 2.0)
            rospy.loginfo("[规划器] 目标高度调整为: %.2f m", goal[2])

        if self.current_pose is None:
            rospy.logwarn("[规划器] 无人机位姿未知")
            return

        start = (
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z,
        )
        goal = tuple(goal)

        self.publish_goal_marker(goal)

        rospy.loginfo("[规划器] RRT* 规划中... (体素数: %d)", len(self.occupied_voxels))
        path = self.plan_rrt_star(start, goal)

        if path and len(path) > 1:
            rospy.loginfo("[规划器] 路径成功! %d 个航点", len(path))
            self.publish_path(path)
            self.current_path = path
            self.current_wp_idx = 0
            self.is_flying = True
            self.ensure_offboard()
        else:
            rospy.logwarn("[规划器] 规划失败!")

    def ensure_offboard(self):
        if self.current_state.mode != "OFFBOARD":
            if self.current_pose:
                target = PoseStamped()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = "map"
                target.pose = self.current_pose.pose
                for _ in range(20):
                    self.setpoint_pub.publish(target)
                    rospy.sleep(0.05)
            try:
                self.set_mode_client(custom_mode="OFFBOARD")
                rospy.loginfo("[规划器] OFFBOARD")
            except rospy.ServiceException as e:
                rospy.logwarn("[规划器] OFFBOARD失败: %s", e)

        if not self.current_state.armed:
            try:
                self.arming_client(True)
                rospy.loginfo("[规划器] 已解锁")
            except rospy.ServiceException as e:
                rospy.logwarn("[规划器] 解锁失败: %s", e)

    # ==================== 碰撞检测 ====================

    def is_collision(self, x, y, z):
        """基于OctoMap占据体素的碰撞检测"""
        if not self.occupied_voxels:
            return False
        res = self.resolution
        gx = int(round(x / res))
        gy = int(round(y / res))
        gz = int(round(z / res))
        margin = max(1, int(self.safety_margin / res))
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                for dz in range(-margin, margin + 1):
                    if (gx + dx, gy + dy, gz + dz) in self.occupied_voxels:
                        return True
        return False

    def is_path_free(self, n1, n2):
        dx = n2.x - n1.x
        dy = n2.y - n1.y
        dz = n2.z - n1.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 0.01:
            return True
        steps = max(int(dist / (self.resolution * 0.5)), 2)
        for i in range(steps + 1):
            t = i / steps
            if self.is_collision(n1.x + t*dx, n1.y + t*dy, n1.z + t*dz):
                return False
        return True

    def node_dist(self, n1, n2):
        return math.sqrt((n1.x-n2.x)**2 + (n1.y-n2.y)**2 + (n1.z-n2.z)**2)

    # ==================== RRT* ====================

    def plan_rrt_star(self, start, goal):
        start_node = Node3D(*start)
        goal_node = Node3D(*goal)

        # 直线可行直接返回
        if self.is_path_free(start_node, goal_node):
            rospy.loginfo("[规划器] 直线路径可行")
            return [start, goal]

        tree = [start_node]

        bounds = [
            (min(start[0], goal[0]) - 5.0, max(start[0], goal[0]) + 5.0),
            (min(start[1], goal[1]) - 5.0, max(start[1], goal[1]) + 5.0),
            (min(start[2], goal[2]) - 2.0, max(start[2], goal[2]) + 2.0),
        ]
        bounds[2] = (max(0.3, bounds[2][0]), bounds[2][1])

        best_path = None
        best_cost = float('inf')

        for iteration in range(self.max_iter):
            if np.random.random() < 0.15:
                rand = Node3D(*goal)
            else:
                rand = Node3D(
                    np.random.uniform(*bounds[0]),
                    np.random.uniform(*bounds[1]),
                    np.random.uniform(*bounds[2]),
                )

            nearest = min(tree, key=lambda n: self.node_dist(n, rand))
            dist = self.node_dist(nearest, rand)

            if dist > self.step_size:
                ratio = self.step_size / dist
                new_node = Node3D(
                    nearest.x + ratio * (rand.x - nearest.x),
                    nearest.y + ratio * (rand.y - nearest.y),
                    nearest.z + ratio * (rand.z - nearest.z),
                )
            else:
                new_node = rand

            if self.is_collision(new_node.x, new_node.y, new_node.z):
                continue
            if not self.is_path_free(nearest, new_node):
                continue

            near_nodes = [n for n in tree if self.node_dist(n, new_node) < self.search_radius]
            best_parent = nearest
            bc = nearest.cost + self.node_dist(nearest, new_node)
            for near in near_nodes:
                nc = near.cost + self.node_dist(near, new_node)
                if nc < bc and self.is_path_free(near, new_node):
                    best_parent = near
                    bc = nc

            new_node.parent = best_parent
            new_node.cost = bc
            tree.append(new_node)

            for near in near_nodes:
                nc = new_node.cost + self.node_dist(new_node, near)
                if nc < near.cost and self.is_path_free(new_node, near):
                    near.parent = new_node
                    near.cost = nc

            if self.node_dist(new_node, goal_node) < self.goal_threshold:
                if self.is_path_free(new_node, goal_node):
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + self.node_dist(new_node, goal_node)
                    path = self._extract_path(goal_node)
                    if goal_node.cost < best_cost:
                        best_path = path
                        best_cost = goal_node.cost

            if (iteration + 1) % 500 == 0:
                rospy.loginfo("[规划器] RRT* iter %d/%d, 节点: %d",
                              iteration+1, self.max_iter, len(tree))

        if best_path:
            return self.smooth_path(best_path)
        else:
            rospy.logwarn("[规划器] RRT* 找不到路径, 退化为直线")
            return [start, goal]

    def _extract_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y, node.z))
            node = node.parent
        path.reverse()
        return path

    def smooth_path(self, path):
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                n1 = Node3D(*smoothed[-1])
                n2 = Node3D(*path[j])
                if self.is_path_free(n1, n2):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    # ==================== 可视化 ====================

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "camera_init"
        for x, y, z in path_points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    def publish_goal_marker(self, goal):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "camera_init"
        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = goal[0]
        m.pose.position.y = goal[1]
        m.pose.position.z = goal[2]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color = ColorRGBA(1.0, 0.2, 0.2, 0.8)
        m.lifetime = rospy.Duration(0)
        ma.markers.append(m)

        mt = Marker()
        mt.header = m.header
        mt.ns = "goal_text"
        mt.id = 1
        mt.type = Marker.TEXT_VIEW_FACING
        mt.action = Marker.ADD
        mt.pose.position.x = goal[0]
        mt.pose.position.y = goal[1]
        mt.pose.position.z = goal[2] + 0.8
        mt.pose.orientation.w = 1.0
        mt.scale.z = 0.3
        mt.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        mt.text = "Target (%.1f,%.1f,%.1f)" % goal
        mt.lifetime = rospy.Duration(0)
        ma.markers.append(mt)
        self.marker_pub.publish(ma)

    # ==================== 飞行 ====================

    def fly_path(self):
        if not self.is_flying or self.current_path is None:
            return
        if self.current_pose is None:
            return

        if self.current_wp_idx >= len(self.current_path):
            rospy.loginfo("[规划器] 到达目标!")
            self.is_flying = False
            self.current_path = None
            return

        target = self.current_path[self.current_wp_idx]
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z
        dist = math.sqrt((cx-target[0])**2 + (cy-target[1])**2 + (cz-target[2])**2)

        if dist < self.waypoint_threshold:
            self.current_wp_idx += 1
            if self.current_wp_idx < len(self.current_path):
                rospy.loginfo("[规划器] 航点 %d/%d",
                              self.current_wp_idx, len(self.current_path))
            return

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = target[0]
        ps.pose.position.y = target[1]
        ps.pose.position.z = target[2]

        dx = target[0] - cx
        dy = target[1] - cy
        yaw = math.atan2(dy, dx)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(ps)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.fly_path()
            if not self.is_flying and self.current_state.mode == "OFFBOARD" and self.current_pose:
                ps = PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = "map"
                ps.pose = self.current_pose.pose
                self.setpoint_pub.publish(ps)
            rate.sleep()


if __name__ == '__main__':
    try:
        planner = InteractivePlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
