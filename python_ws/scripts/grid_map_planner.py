#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D栅格地图巡检规划器 (基于PCD切片)

核心方案 (第十七次新增):
  使用 pcd_to_2d_map.py 生成的2D栅格地图进行路径规划
  无需OctoMap, 直接在2D栅格上做A*避障

数据流:
  FAST-LIO2 (.pcd) → pcd_to_2d_map.py → .pgm栅格 → 本规划器A* → 自动飞行

终端命令:
  alt H          - 设定飞行高度 (默认1.0m)
  start          - 开始执行巡检
  clear          - 清除航点
  stop           - 紧急悬停
  status         - 查看状态
  home           - 返航
  undo           - 撤销最后航点
  speed S        - 设置飞行速度 (m/s, 如: speed 0.3)

工作流:
  1. 先用 slam_mapping.launch 建图并 save_map.py 保存
  2. 运行 pcd_to_2d_map.py --height H 生成2D栅格
  3. roslaunch mak4_sim navigation_2d.launch
  4. 在RViz点击航点 → 输入 start
"""

import rospy
import math
import heapq
import threading
import sys
import select
import os
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class GridMapPlanner:
    def __init__(self):
        rospy.init_node('grid_map_planner', anonymous=True)

        # ---------- 参数 ----------
        self.safety_margin = rospy.get_param('~safety_margin', 0.3)
        self.waypoint_threshold = rospy.get_param('~waypoint_threshold', 0.5)
        self.default_altitude = rospy.get_param('~default_altitude', 1.0)
        self.flight_speed = rospy.get_param('~flight_speed', 6.9)
        self.map_yaml = rospy.get_param('~map_yaml', '')
        
        # ---------- 栅格地图 ----------
        self.grid = None
        self.grid_resolution = 0.05
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0
        self.grid_width = 0
        self.grid_height = 0
        self.map_ready = False

        # ---------- 状态 ----------
        self.current_pose = None
        self.current_state = State()
        self.patrol_altitude = self.default_altitude
        self.patrol_waypoints_2d = []
        self.planned_segments = []
        self.mode = 'SETTING'
        self.current_seg_idx = 0
        self.current_pt_idx = 0
        self.home_xy = None
        self.pre_arm_start = None
        self.takeoff_target_alt = self.default_altitude

        # ---------- ROS 通信 ----------
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/patrol/planned_route_path', Path, queue_size=1)
        self.active_path_pub = rospy.Publisher('/patrol/active_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/patrol/waypoints', MarkerArray, queue_size=1)
        self.route_marker_pub = rospy.Publisher('/patrol/planned_route', MarkerArray, queue_size=1)
        self.drone_marker_pub = rospy.Publisher('/patrol/drone_position', MarkerArray, queue_size=1)
        self.alt_plane_pub = rospy.Publisher('/patrol/altitude_plane', MarkerArray, queue_size=1, latch=True)
        self.gridmap_pub = rospy.Publisher('/map_2d', OccupancyGrid, queue_size=1, latch=True)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/clicked_point', PointStamped, self._click_cb)

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self._drone_viz_counter = 0
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        # 加载地图
        self._load_map()
        
        rospy.loginfo("[2D巡检] 已启动! 飞行高度=%.1fm, 速度=%.1fm/s", 
                      self.patrol_altitude, self.flight_speed)
        rospy.loginfo("[2D巡检] 输入 'alt H' 设定高度, 在RViz点击航点, 输入 'start' 开始")

    def _load_map(self):
        """加载PGM栅格地图"""
        if not self.map_yaml:
            rospy.logwarn("[2D巡检] 未指定地图文件!")
            return
        
        import yaml
        yaml_path = self.map_yaml
        if not os.path.exists(yaml_path):
            rospy.logerr("[2D巡检] 地图YAML不存在: %s", yaml_path)
            return
        
        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)
        
        pgm_name = map_info['image']
        if not os.path.isabs(pgm_name):
            pgm_name = os.path.join(os.path.dirname(yaml_path), pgm_name)
        
        self.grid_resolution = float(map_info['resolution'])
        origin = map_info['origin']
        self.grid_origin_x = float(origin[0])
        self.grid_origin_y = float(origin[1])
        
        # 读取PGM
        with open(pgm_name, 'rb') as f:
            # 跳过PGM头
            magic = f.readline().decode('ascii').strip()
            assert magic == 'P5', "不是P5格式PGM: %s" % magic
            
            # 跳过注释行
            line = f.readline().decode('ascii').strip()
            while line.startswith('#'):
                line = f.readline().decode('ascii').strip()
            
            w, h = map(int, line.split())
            max_val = int(f.readline().decode('ascii').strip())
            
            raw = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
        
        # PGM是从top到bottom存储, ROS map_server格式是y轴翻转的
        self.grid = np.flipud(raw)
        self.grid_width = w
        self.grid_height = h
        self.map_ready = True
        
        occupied = np.sum(self.grid < 50)
        free = np.sum(self.grid > 200)
        rospy.loginfo("[2D巡检] ✓ 地图已加载: %dx%d, 分辨率=%.3fm, 占据=%d, 自由=%d",
                      w, h, self.grid_resolution, occupied, free)
        
        # ★ 预膨胀障碍物: 大幅加速A* (避免每次扩展节点都做圆形碰撞检查)
        self._inflate_grid()
        
        # 发布为ROS OccupancyGrid供RViz显示
        self._publish_occupancy_grid()

    def _inflate_grid(self):
        """预膨胀障碍物: 生成布尔数组, A*直接查表无需逐点碰撞检测
        
        将安全距离内的所有栅格标记为不可通行, 使A*每次扩展只需一次数组查询
        → 速度提升10~50倍 (消除逐点碰撞检查的Python循环开销)
        """
        from scipy import ndimage
        margin_cells = max(1, int(self.safety_margin / self.grid_resolution))
        
        # 二值化: True=障碍物, False=自由
        obstacle_mask = (self.grid < 50)
        
        # 创建圆形膨胀核
        kernel_size = 2 * margin_cells + 1
        y, x = np.ogrid[-margin_cells:margin_cells+1, -margin_cells:margin_cells+1]
        kernel = (x*x + y*y <= margin_cells*margin_cells).astype(np.uint8)
        
        # 膨胀: 障碍物向外扩展 safety_margin
        inflated = ndimage.binary_dilation(obstacle_mask, structure=kernel)
        
        # 存储: True=可通行, False=不可通行(障碍物或安全距离内)
        self.free_grid = (~inflated) & (self.grid > 200)
        
        free_count = np.sum(self.free_grid)
        total = self.grid_width * self.grid_height
        rospy.loginfo("[2D巡检] ✓ 障碍物膨胀完成: 安全距离=%d格(%.2fm), 可通行=%d/%d (%.1f%%)",
                      margin_cells, self.safety_margin, free_count, total, 
                      100.0 * free_count / total if total > 0 else 0)

    def _publish_occupancy_grid(self):
        """发布OccupancyGrid到RViz"""
        if self.grid is None:
            return
        
        og = OccupancyGrid()
        og.header.stamp = rospy.Time.now()
        og.header.frame_id = "camera_init"
        og.info.resolution = self.grid_resolution
        og.info.width = self.grid_width
        og.info.height = self.grid_height
        og.info.origin.position.x = self.grid_origin_x
        og.info.origin.position.y = self.grid_origin_y
        og.info.origin.orientation.w = 1.0
        
        # 转换: PGM 0=occupied→ROS 100, PGM 254=free→ROS 0
        data = np.zeros(self.grid_height * self.grid_width, dtype=np.int8)
        flat = self.grid.flatten()
        data[flat < 50] = 100      # occupied
        data[flat > 200] = 0       # free
        data[(flat >= 50) & (flat <= 200)] = -1  # unknown
        og.data = data.tolist()
        
        self.gridmap_pub.publish(og)

    # ==================== 回调 ====================

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _state_cb(self, msg):
        self.current_state = msg

    def _click_cb(self, msg):
        if self.mode != 'SETTING':
            rospy.logwarn("[2D巡检] 当前不在设定模式")
            return
        
        x, y = msg.point.x, msg.point.y
        self.patrol_waypoints_2d.append((x, y))
        rospy.loginfo("[2D巡检] ✓ 航点 #%d: (%.1f, %.1f) 飞行高度=%.1fm",
                      len(self.patrol_waypoints_2d), x, y, self.patrol_altitude)
        self._publish_waypoint_markers()

    # ==================== A* 路径规划 ====================

    def _world_to_grid(self, wx, wy):
        """世界坐标 → 栅格坐标"""
        gx = int((wx - self.grid_origin_x) / self.grid_resolution)
        gy = int((wy - self.grid_origin_y) / self.grid_resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        """栅格坐标 → 世界坐标"""
        wx = gx * self.grid_resolution + self.grid_origin_x + self.grid_resolution / 2
        wy = gy * self.grid_resolution + self.grid_origin_y + self.grid_resolution / 2
        return wx, wy

    def _is_free(self, gx, gy):
        """检查栅格坐标是否为自由空间"""
        if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
            return False
        return self.grid[gy, gx] > 200

    def _is_inflated_free(self, gx, gy):
        """检查膨胀后的自由空间 (用于A*, O(1)查表)"""
        if gx < 0 or gx >= self.grid_width or gy < 0 or gy >= self.grid_height:
            return False
        return self.free_grid[gy, gx]

    def _is_collision_free(self, gx, gy, margin_cells):
        """检查带安全距离的碰撞检测 (使用预膨胀网格, O(1))"""
        if hasattr(self, 'free_grid') and self.free_grid is not None:
            return self._is_inflated_free(gx, gy)
        # 回退: 逐点检查 (慢)
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                if dx*dx + dy*dy <= margin_cells*margin_cells:
                    if not self._is_free(gx + dx, gy + dy):
                        return False
        return True

    def _astar(self, start_xy, goal_xy):
        """A*路径规划 (在2D栅格上)"""
        if not self.map_ready:
            rospy.logwarn("[2D巡检] 地图未加载!")
            return None
        
        sx, sy = self._world_to_grid(start_xy[0], start_xy[1])
        gx, gy = self._world_to_grid(goal_xy[0], goal_xy[1])
        margin_cells = max(1, int(self.safety_margin / self.grid_resolution))
        
        # 检查起终点 (使用膨胀后的网格确保安全)
        if not self._is_inflated_free(sx, sy):
            rospy.logwarn("[2D巡检] 起点 (%.1f,%.1f) 在障碍区域! 寻找最近安全点...", 
                          start_xy[0], start_xy[1])
            sx, sy = self._find_nearest_free(sx, sy)
            if sx is None:
                return None
        
        if not self._is_inflated_free(gx, gy):
            rospy.logwarn("[2D巡检] 终点 (%.1f,%.1f) 在障碍区域! 寻找最近安全点...",
                          goal_xy[0], goal_xy[1])
            gx, gy = self._find_nearest_free(gx, gy)
            if gx is None:
                return None
        
        # A*
        # 8方向: (dx, dy, cost)
        directions = [
            (1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0),
            (1,1,1.414), (-1,1,1.414), (1,-1,1.414), (-1,-1,1.414)
        ]
        
        open_set = [(0, sx, sy)]
        came_from = {}
        g_score = {(sx, sy): 0}
        visited = set()
        max_iter = 500000
        import time
        t0 = time.time()
        
        for _ in range(max_iter):
            if not open_set:
                break
            
            _, cx, cy = heapq.heappop(open_set)
            
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))
            
            if abs(cx - gx) <= 1 and abs(cy - gy) <= 1:
                # 回溯路径
                path = []
                node = (cx, cy)
                while node in came_from:
                    wx, wy = self._grid_to_world(node[0], node[1])
                    path.append((wx, wy))
                    node = came_from[node]
                path.reverse()
                # 添加终点
                wx, wy = self._grid_to_world(gx, gy)
                path.append((wx, wy))
                rospy.loginfo("[2D巡检] A*完成: %.2fs, 扩展%d节点, 路径%d点",
                              time.time() - t0, len(visited), len(path))
                return path
            
            for dx, dy, cost in directions:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in visited:
                    continue
                if not self._is_collision_free(nx, ny, margin_cells):
                    continue
                
                new_g = g_score[(cx, cy)] + cost
                if new_g < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = new_g
                    h = math.sqrt((nx - gx)**2 + (ny - gy)**2)
                    heapq.heappush(open_set, (new_g + h, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)
        
        rospy.logwarn("[2D巡检] A*未找到路径! (起点→终点距离太远或被障碍物阻挡)")
        return None

    def _find_nearest_free(self, gx, gy, max_radius=100):
        """找最近的安全空间栅格 (使用膨胀网格)"""
        for r in range(1, max_radius):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        if self._is_inflated_free(gx + dx, gy + dy):
                            return gx + dx, gy + dy
        return None, None

    def _simplify_path(self, path):
        """路径简化: 去除共线点"""
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            dx1 = path[i][0] - simplified[-1][0]
            dy1 = path[i][1] - simplified[-1][1]
            dx2 = path[i+1][0] - path[i][0]
            dy2 = path[i+1][1] - path[i][1]
            cross = abs(dx1 * dy2 - dy1 * dx2)
            if cross > 0.01:
                simplified.append(path[i])
        simplified.append(path[-1])
        return simplified

    def _plan_all(self):
        """规划所有航段"""
        if not self.map_ready:
            rospy.logerr("[2D巡检] 地图未加载!")
            return False
        
        if not self.patrol_waypoints_2d:
            rospy.logwarn("[2D巡检] 没有航点!")
            return False
        
        cx, cy = 0, 0
        if self.current_pose:
            cx = self.current_pose.pose.position.x
            cy = self.current_pose.pose.position.y
        
        self.planned_segments = []
        alt = self.patrol_altitude
        
        # 依次规划: 起点→wp1→wp2→...
        prev = (cx, cy)
        for i, wp in enumerate(self.patrol_waypoints_2d):
            rospy.loginfo("[2D巡检] 规划航段 %d: (%.1f,%.1f) → (%.1f,%.1f)", 
                          i+1, prev[0], prev[1], wp[0], wp[1])
            
            path_2d = self._astar(prev, wp)
            if path_2d is None:
                # A*失败, 使用直线
                rospy.logwarn("[2D巡检] A*失败, 使用直线!")
                path_2d = [prev, wp]
            
            path_2d = self._simplify_path(path_2d)
            
            # 转为3D路径 (添加高度)
            path_3d = [(p[0], p[1], alt) for p in path_2d]
            self.planned_segments.append(path_3d)
            
            rospy.loginfo("[2D巡检]   → %d 个路径点", len(path_3d))
            prev = wp
        
        rospy.loginfo("[2D巡检] ✓ 规划完成! %d 个航段", len(self.planned_segments))
        self._publish_route_markers()
        return True

    # ==================== 飞行控制 ====================

    def _send_setpoint(self, x, y, z, yaw=None):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        if yaw is not None:
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            ps.pose.orientation.w = 1.0
        self.setpoint_pub.publish(ps)

    def _get_current_yaw(self):
        q = self.current_pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _fly_step(self):
        if self.current_pose is None:
            return
        
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z

        if self.mode == 'PRE_ARM':
            cur_yaw = self._get_current_yaw()
            self._send_setpoint(cx, cy, self.takeoff_target_alt, cur_yaw)
            elapsed = (rospy.Time.now() - self.pre_arm_start).to_sec()
            if elapsed >= 2.5:
                try:
                    self.set_mode_client(custom_mode="OFFBOARD")
                    rospy.loginfo("[2D巡检] ✓ OFFBOARD 已设置")
                except rospy.ServiceException as e:
                    rospy.logwarn("[2D巡检] OFFBOARD失败: %s", e)
                    return
                rospy.sleep(0.3)
                try:
                    self.arming_client(True)
                    rospy.loginfo("[2D巡检] ✓ 已解锁")
                except rospy.ServiceException as e:
                    rospy.logwarn("[2D巡检] 解锁失败: %s", e)
                    return
                self.mode = 'TAKING_OFF'
                rospy.loginfo("[2D巡检] 起飞中... 目标高度 %.1fm", self.takeoff_target_alt)

        elif self.mode == 'TAKING_OFF':
            cur_yaw = self._get_current_yaw()
            self._send_setpoint(cx, cy, self.takeoff_target_alt, cur_yaw)
            if abs(cz - self.takeoff_target_alt) < 0.3:
                rospy.loginfo("[2D巡检] ✓ 已到达飞行高度 %.1fm!", self.takeoff_target_alt)
                self.mode = 'EXECUTING'

        elif self.mode == 'EXECUTING':
            self._execute_step(cx, cy, cz)

        elif self.mode == 'DONE':
            self._send_setpoint(cx, cy, cz)

    def _execute_step(self, cx, cy, cz):
        if self.current_seg_idx >= len(self.planned_segments):
            rospy.loginfo("[2D巡检] ====== 巡检完成! ======")
            self.mode = 'DONE'
            return

        seg = self.planned_segments[self.current_seg_idx]
        if self.current_pt_idx >= len(seg):
            self.current_seg_idx += 1
            self.current_pt_idx = 0
            if self.current_seg_idx < len(self.planned_segments):
                rospy.loginfo("[2D巡检] 航段 %d/%d 完成",
                              self.current_seg_idx, len(self.planned_segments))
            return

        target = seg[self.current_pt_idx]
        dx = target[0] - cx
        dy = target[1] - cy
        dz = target[2] - cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < self.waypoint_threshold:
            self.current_pt_idx += 1
            if self.current_pt_idx >= len(seg) and self.current_seg_idx < len(self.patrol_waypoints_2d):
                wp = self.patrol_waypoints_2d[self.current_seg_idx]
                rospy.loginfo("[2D巡检] ★ 到达航点 #%d: (%.1f, %.1f)",
                              self.current_seg_idx + 1, wp[0], wp[1])
            return

        yaw = math.atan2(dy, dx)
        # 限速: 每步最多移动 flight_speed * dt (20Hz → 0.05s)
        max_step = self.flight_speed * 0.05
        if dist > max_step:
            ratio = max_step / dist
            nx = cx + dx * ratio
            ny = cy + dy * ratio
            nz = cz + dz * ratio
            self._send_setpoint(nx, ny, nz, yaw)
        else:
            self._send_setpoint(target[0], target[1], target[2], yaw)

    # ==================== 可视化 ====================

    def _publish_waypoint_markers(self):
        ma = MarkerArray()
        for i, wp in enumerate(self.patrol_waypoints_2d):
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "camera_init"
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wp[0]
            m.pose.position.y = wp[1]
            m.pose.position.z = self.patrol_altitude
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.1
            m.color = ColorRGBA(1.0, 0.5, 0.0, 0.8)
            ma.markers.append(m)
            
            # 文字标签
            t = Marker()
            t.header = m.header
            t.ns = "wp_labels"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp[0]
            t.pose.position.y = wp[1]
            t.pose.position.z = self.patrol_altitude + 0.3
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            t.text = "#%d" % (i + 1)
            ma.markers.append(t)
        
        self.marker_pub.publish(ma)

    def _publish_route_markers(self):
        ma = MarkerArray()
        for si, seg in enumerate(self.planned_segments):
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "camera_init"
            m.ns = "route_%d" % si
            m.id = si
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            if si <= self.current_seg_idx:
                m.color = ColorRGBA(0.2, 1.0, 0.2, 0.8)
            else:
                m.color = ColorRGBA(1.0, 1.0, 0.2, 0.5)
            m.pose.orientation.w = 1.0
            for pt in seg:
                m.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))
            ma.markers.append(m)
        self.route_marker_pub.publish(ma)

    def _publish_drone_marker(self):
        if self.current_pose is None:
            return
        p = self.current_pose.pose.position
        ma = MarkerArray()
        
        sphere = Marker()
        sphere.header.stamp = rospy.Time.now()
        sphere.header.frame_id = "camera_init"
        sphere.ns = "drone"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = p
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
        sphere.color = ColorRGBA(1.0, 0.2, 0.2, 0.9)
        sphere.lifetime = rospy.Duration(0.5)
        ma.markers.append(sphere)

        # 方向箭头
        if self.current_pose:
            arrow = Marker()
            arrow.header = sphere.header
            arrow.ns = "drone_arrow"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = self.current_pose.pose
            arrow.scale.x = 0.5
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color = ColorRGBA(1.0, 0.2, 0.2, 0.9)
            arrow.lifetime = rospy.Duration(0.5)
            ma.markers.append(arrow)

            # 高度文字
            txt = Marker()
            txt.header = sphere.header
            txt.ns = "drone_alt"
            txt.id = 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = p.x
            txt.pose.position.y = p.y
            txt.pose.position.z = p.z + 0.4
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color = ColorRGBA(1.0, 1.0, 1.0, 0.9)
            txt.text = "UAV h=%.1fm" % p.z
            txt.lifetime = rospy.Duration(0.5)
            ma.markers.append(txt)
        
        self.drone_marker_pub.publish(ma)

    def _publish_altitude_plane(self):
        """发布巡检高度半透明平面 + 高度文字标注"""
        ma = MarkerArray()

        # 半透明蓝色平面
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "camera_init"
        m.ns = "alt_plane"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = self.patrol_altitude
        m.pose.orientation.w = 1.0
        m.scale.x = 40.0
        m.scale.y = 40.0
        m.scale.z = 0.02
        m.color = ColorRGBA(0.3, 0.6, 1.0, 0.25)
        ma.markers.append(m)

        # 高度文字标注 (4个角 + 中心)
        positions = [(0, 0), (-15, -8), (15, -8), (-15, 8), (15, 8)]
        for i, (px, py) in enumerate(positions):
            txt = Marker()
            txt.header.stamp = rospy.Time.now()
            txt.header.frame_id = "camera_init"
            txt.ns = "alt_text"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = px
            txt.pose.position.y = py
            txt.pose.position.z = self.patrol_altitude + 0.15
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.5 if i == 0 else 0.35
            txt.color = ColorRGBA(0.0, 0.8, 1.0, 1.0)
            txt.text = "=== %.1fm ===" % self.patrol_altitude if i == 0 else "h=%.1fm" % self.patrol_altitude
            ma.markers.append(txt)

        # 平面边框线 (4条边)
        border = Marker()
        border.header.stamp = rospy.Time.now()
        border.header.frame_id = "camera_init"
        border.ns = "alt_border"
        border.id = 0
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.scale.x = 0.05
        border.color = ColorRGBA(0.0, 0.7, 1.0, 0.6)
        border.pose.orientation.w = 1.0
        z = self.patrol_altitude
        for px, py in [(-20, -12), (20, -12), (20, 12), (-20, 12), (-20, -12)]:
            border.points.append(Point(x=px, y=py, z=z))
        ma.markers.append(border)

        self.alt_plane_pub.publish(ma)

    # ==================== 终端输入 ====================

    def _input_loop(self):
        while not rospy.is_shutdown():
            try:
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    line = sys.stdin.readline().strip().lower()
                    if not line:
                        continue
                    self._handle_command(line)
            except Exception:
                rospy.sleep(0.5)

    def _handle_command(self, cmd):
        parts = cmd.split()
        if not parts:
            return
        
        action = parts[0]
        
        if action == 'alt' and len(parts) >= 2:
            try:
                self.patrol_altitude = float(parts[1])
                self.takeoff_target_alt = self.patrol_altitude
                rospy.loginfo("[2D巡检] 飞行高度设为 %.1fm", self.patrol_altitude)
                rospy.loginfo("[2D巡检] ★ RViz中的蓝色半透明平面即为飞行高度")
                self._publish_altitude_plane()
            except ValueError:
                rospy.logwarn("[2D巡检] 无效高度")
        
        elif action == 'speed' and len(parts) >= 2:
            try:
                self.flight_speed = float(parts[1])
                rospy.loginfo("[2D巡检] 飞行速度设为 %.1fm/s", self.flight_speed)
            except ValueError:
                rospy.logwarn("[2D巡检] 无效速度")
        
        elif action == 'start':
            if len(self.patrol_waypoints_2d) == 0:
                rospy.logwarn("[2D巡检] 请先点击航点!")
                return
            if not self._plan_all():
                return
            self.current_seg_idx = 0
            self.current_pt_idx = 0
            if self.current_pose:
                self.home_xy = (self.current_pose.pose.position.x,
                                self.current_pose.pose.position.y)
            self.pre_arm_start = rospy.Time.now()
            self.mode = 'PRE_ARM'
            rospy.loginfo("[2D巡检] 准备起飞...")
        
        elif action == 'clear':
            self.patrol_waypoints_2d = []
            self.planned_segments = []
            self.mode = 'SETTING'
            self.current_seg_idx = 0
            self.current_pt_idx = 0
            # 清除markers
            ma = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            ma.markers.append(m)
            self.marker_pub.publish(ma)
            self.route_marker_pub.publish(ma)
            rospy.loginfo("[2D巡检] 已清除所有航点")
        
        elif action == 'stop':
            self.mode = 'DONE'
            rospy.loginfo("[2D巡检] 紧急悬停!")
        
        elif action == 'home':
            if self.home_xy and self.current_pose:
                hx, hy = self.home_xy
                rospy.loginfo("[2D巡检] 返航到 (%.1f, %.1f)", hx, hy)
                self.patrol_waypoints_2d = [(hx, hy)]
                self._plan_all()
                self.current_seg_idx = 0
                self.current_pt_idx = 0
                self.mode = 'EXECUTING'
        
        elif action == 'undo':
            if self.patrol_waypoints_2d:
                removed = self.patrol_waypoints_2d.pop()
                rospy.loginfo("[2D巡检] 撤销航点 (%.1f, %.1f)", removed[0], removed[1])
                self._publish_waypoint_markers()
        
        elif action == 'status':
            rospy.loginfo("[2D巡检] 模式=%s, 航点=%d, 高度=%.1fm, 速度=%.1fm/s, 地图=%s",
                          self.mode, len(self.patrol_waypoints_2d), self.patrol_altitude,
                          self.flight_speed, "已加载" if self.map_ready else "未加载")
        
        else:
            rospy.loginfo("[2D巡检] 可用命令: alt H | speed S | start | clear | stop | home | undo | status")

    # ==================== 主循环 ====================

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._fly_step()
            self._drone_viz_counter += 1
            if self._drone_viz_counter >= 4:
                self._drone_viz_counter = 0
                self._publish_drone_marker()
            rate.sleep()


if __name__ == '__main__':
    try:
        planner = GridMapPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
