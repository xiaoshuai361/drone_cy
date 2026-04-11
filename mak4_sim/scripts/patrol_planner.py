#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D分层航线巡检规划器

核心思路:
  指定巡检高度 → 俯视图点击航点(只取x,y) → A*平面避障 → 自动起飞执行
  支持多层巡检: 同一组航点在不同高度各飞一遍

终端命令:
  alt H          - 设定单层巡检高度 (如: alt 2.0)
  alt H1 H2 ...  - 设定多层巡检高度 (如: alt 1.5 2.5 3.5)
  start          - 开始执行巡检 (自动起飞)
  clear          - 清除航点重新设定
  stop           - 紧急停止悬停
  status         - 查看状态
  home           - 返航到起飞点
  undo           - 撤销最后一个航点

工作流:
  1. 输入 'alt 2.0' 设定巡检高度
  2. 在RViz中用 Publish Point 点击多个位置 (只需看x,y, 高度自动)
  3. 输入 'start' → 无人机自动起飞到巡检高度 → 逐点飞行 → 完成
  多层: alt 1.5 2.5 → 先飞1.5m层 → 回起点 → 飞2.5m层 → 完成
"""

import rospy
import math
import heapq
import threading
import sys
import select
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class PatrolPlanner:
    def __init__(self):
        rospy.init_node('patrol_planner', anonymous=True)

        # ---------- 参数 ----------
        self.safety_margin = rospy.get_param('~safety_margin', 0.4)
        self.waypoint_threshold = rospy.get_param('~waypoint_threshold', 0.5)
        self.default_altitude = rospy.get_param('~default_altitude', 1.5)
        self.flight_speed = rospy.get_param('~flight_speed', 6.9)

        # ---------- 状态 ----------
        self.current_pose = None
        self.current_state = State()
        self.occupied_voxels = set()
        self.resolution = 0.15
        self.map_ready = False

        # 航线配置
        self.patrol_altitudes = [self.default_altitude]
        self.patrol_waypoints_2d = []   # [(x, y), ...] 只存2D坐标

        # 执行状态机
        # SETTING → PRE_ARM → TAKING_OFF → EXECUTING → LAYER_RETURN → (下一层TAKING_OFF) → DONE
        self.mode = 'SETTING'
        self.current_layer_idx = 0
        self.planned_segments = []      # 当前层的路径段 [[(x,y,z),...], ...]
        self.current_seg_idx = 0
        self.current_pt_idx = 0
        self.home_xy = None
        self.pre_arm_start = None
        self.takeoff_target_alt = 0.0

        # ---------- 发布器 ----------
        self.waypoint_marker_pub = rospy.Publisher('/patrol/waypoints', MarkerArray, queue_size=1, latch=True)
        self.route_pub = rospy.Publisher('/patrol/planned_route', MarkerArray, queue_size=1)
        self.active_path_pub = rospy.Publisher('/patrol/active_path', Path, queue_size=1)
        self.alt_plane_pub = rospy.Publisher('/patrol/altitude_plane', MarkerArray, queue_size=1, latch=True)
        self.drone_marker_pub = rospy.Publisher('/patrol/drone_position', MarkerArray, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self._drone_viz_counter = 0

        # ---------- 订阅器 ----------
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/clicked_point', PointStamped, self._clicked_point_cb)
        rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, self._occupied_cb)

        # ---------- 服务 ----------
        rospy.wait_for_service('/mavros/cmd/arming', timeout=15)
        rospy.wait_for_service('/mavros/set_mode', timeout=15)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # ---------- 终端输入线程 ----------
        self.cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self.cmd_thread.start()

        rospy.loginfo("=" * 60)
        rospy.loginfo("[巡检] 2D分层航线巡检规划器已启动!")
        rospy.loginfo("[巡检] 当前巡检高度: %.1f m", self.patrol_altitudes[0])
        rospy.loginfo("[巡检] 操作步骤:")
        rospy.loginfo("  1. 输入 'alt 高度' 设定巡检高度 (如: alt 2.0)")
        rospy.loginfo("     多层: alt 1.5 2.5 3.5")
        rospy.loginfo("  2. 在RViz中用 Publish Point 点击巡检航点 (只需x,y)")
        rospy.loginfo("  3. 输入 'start' 自动起飞+执行巡检")
        rospy.loginfo("  其他: clear/stop/status/home/undo")
        rospy.loginfo("=" * 60)

        self._publish_altitude_plane()

    # ==================== 回调 ====================

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg
        if self.home_xy is None:
            self.home_xy = (msg.pose.position.x, msg.pose.position.y)

    def _occupied_cb(self, msg):
        new_voxels = set()
        res = self.resolution
        for marker in msg.markers:
            sx = marker.scale.x if marker.scale.x > 0 else res
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
            rospy.loginfo("[巡检] OctoMap已加载! 占据体素: %d", len(new_voxels))

    def _clicked_point_cb(self, msg):
        if self.mode != 'SETTING':
            rospy.logwarn("[巡检] 当前模式 %s, 无法添加航点. 输入 'clear' 重置.", self.mode)
            return

        # ★ 只取x,y! 高度由巡检高度决定
        x, y = msg.point.x, msg.point.y
        self.patrol_waypoints_2d.append((x, y))
        n = len(self.patrol_waypoints_2d)
        alt_str = '/'.join(['%.1f' % a for a in self.patrol_altitudes])
        rospy.loginfo("[巡检] 航点 #%d: (%.1f, %.1f) 巡检高度=%s m | 共 %d 个航点",
                      n, x, y, alt_str, n)
        rospy.loginfo("[巡检] 继续点击添加, 或 'start' 开始, 'undo' 撤销")
        self._publish_waypoint_markers()

    # ==================== 终端命令 ====================

    def _cmd_loop(self):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            try:
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    line = sys.stdin.readline().strip()
                    if not line:
                        continue
                    parts = line.split()
                    cmd = parts[0].lower()

                    if cmd == 'alt':
                        self._set_altitude(parts[1:])
                    elif cmd == 'start':
                        self._start_patrol()
                    elif cmd == 'clear':
                        self._clear_waypoints()
                    elif cmd == 'stop':
                        self._emergency_stop()
                    elif cmd == 'status':
                        self._print_status()
                    elif cmd == 'home':
                        self._go_home()
                    elif cmd == 'undo':
                        self._undo_waypoint()
                    else:
                        rospy.loginfo("[巡检] 未知命令: '%s'. 可用: alt/start/clear/stop/status/home/undo", cmd)
            except Exception:
                rospy.sleep(0.5)

    def _set_altitude(self, args):
        if not args:
            rospy.loginfo("[巡检] 用法: alt 高度 [高度2 高度3 ...]")
            rospy.loginfo("[巡检] 例如: alt 2.0    或    alt 1.5 2.5 3.5")
            return
        try:
            alts = [float(a) for a in args]
            for a in alts:
                if a < 0.3 or a > 15.0:
                    rospy.logwarn("[巡检] 高度 %.1f 超出安全范围 (0.3~15.0m)", a)
                    return
            self.patrol_altitudes = sorted(alts)
            alt_str = ', '.join(['%.1f' % a for a in self.patrol_altitudes])
            rospy.loginfo("[巡检] ✓ 巡检高度: [%s] m", alt_str)
            rospy.loginfo("[巡检] ★ RViz中的蓝色半透明平面即为飞行高度")
            if len(alts) > 1:
                rospy.loginfo("[巡检]   多层模式: 共 %d 层, 同一航点在每层各飞一遍", len(alts))
            self._publish_altitude_plane()
            self._publish_waypoint_markers()
        except ValueError:
            rospy.logwarn("[巡检] 高度格式错误! 请输入数字")

    def _undo_waypoint(self):
        if not self.patrol_waypoints_2d:
            rospy.loginfo("[巡检] 没有航点可撤销")
            return
        wp = self.patrol_waypoints_2d.pop()
        rospy.loginfo("[巡检] 已撤销航点 (%.1f, %.1f), 剩余 %d 个",
                      wp[0], wp[1], len(self.patrol_waypoints_2d))
        self._publish_waypoint_markers()

    def _start_patrol(self):
        if not self.patrol_waypoints_2d:
            rospy.logwarn("[巡检] 需要至少1个航点! 请在RViz中点击.")
            return
        if self.current_pose is None:
            rospy.logwarn("[巡检] 无人机位姿未知!")
            return

        n_wp = len(self.patrol_waypoints_2d)
        n_layers = len(self.patrol_altitudes)
        rospy.loginfo("[巡检] 开始规划: %d个航点 × %d层高度", n_wp, n_layers)

        # 规划第一层
        self.current_layer_idx = 0
        if not self._plan_layer(self.current_layer_idx):
            return

        # 进入预武装阶段: 先持续发送目标高度setpoint 2.5秒, 再切OFFBOARD+ARM
        # (PX4 要求: 进入OFFBOARD前必须已接收setpoint ≥2秒)
        self.takeoff_target_alt = self.patrol_altitudes[0]
        self.mode = 'PRE_ARM'
        self.pre_arm_start = rospy.Time.now()
        rospy.loginfo("[巡检] 准备起飞... (预发送setpoint → OFFBOARD → 解锁 → 起飞)")

    def _plan_layer(self, layer_idx):
        """规划某一层的所有航段路径 (2D A* 避障)"""
        alt = self.patrol_altitudes[layer_idx]
        rospy.loginfo("[巡检] === 规划第 %d 层 (高度=%.1fm) ===", layer_idx + 1, alt)

        # 从3D OctoMap提取该高度的2D障碍物切片
        obstacles_2d = self._extract_2d_obstacles(alt)
        rospy.loginfo("[巡检] 高度 %.1fm 的2D障碍物: %d 个格子", alt, len(obstacles_2d))

        # 起点 = 当前位置
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y

        all_xy = [(cx, cy)] + list(self.patrol_waypoints_2d)
        self.planned_segments = []

        for i in range(len(all_xy) - 1):
            a, b = all_xy[i], all_xy[i + 1]
            rospy.loginfo("[巡检]   航段 %d/%d: (%.1f,%.1f) → (%.1f,%.1f)",
                          i + 1, len(all_xy) - 1, a[0], a[1], b[0], b[1])

            path_2d = self._plan_astar(a, b, obstacles_2d)
            if path_2d is None:
                rospy.logwarn("[巡检]   A*未找到路径, 使用直线")
                path_2d = [a, b]

            # 2D路径 → 3D路径 (加上巡检高度)
            seg_3d = [(p[0], p[1], alt) for p in path_2d]
            self.planned_segments.append(seg_3d)
            rospy.loginfo("[巡检]   航段 %d → %d 个路径点", i + 1, len(seg_3d))

        self.current_seg_idx = 0
        self.current_pt_idx = 0
        rospy.loginfo("[巡检] 第 %d 层规划完成! %d 个航段", layer_idx + 1, len(self.planned_segments))
        self._publish_route_markers()
        return True

    def _clear_waypoints(self):
        self.patrol_waypoints_2d = []
        self.planned_segments = []
        self.mode = 'SETTING'
        self.current_layer_idx = 0
        self.current_seg_idx = 0
        self.current_pt_idx = 0
        # 清除标记
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.waypoint_marker_pub.publish(ma)
        self.route_pub.publish(ma)
        self.active_path_pub.publish(Path())
        rospy.loginfo("[巡检] 已清除所有航点. 重新设定航线.")

    def _emergency_stop(self):
        self.mode = 'DONE'
        if self.current_pose:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = "map"
            ps.pose = self.current_pose.pose
            for _ in range(50):
                self.setpoint_pub.publish(ps)
                rospy.sleep(0.05)
        rospy.logwarn("[巡检] ★ 紧急停止! 无人机悬停.")
        rospy.loginfo("[巡检] 输入 'clear' 重新设定, 'home' 返航")

    def _go_home(self):
        if self.home_xy is None or self.current_pose is None:
            rospy.logwarn("[巡检] 位姿未知")
            return
        rospy.loginfo("[巡检] 返航中...")
        self.patrol_waypoints_2d = [self.home_xy]
        self.current_layer_idx = 0
        alt = self.current_pose.pose.position.z
        if alt < 1.0:
            alt = self.patrol_altitudes[0]
        self.patrol_altitudes = [alt]
        self._plan_layer(0)
        self.mode = 'EXECUTING'

    def _print_status(self):
        rospy.loginfo("[巡检] ---- 状态 ----")
        rospy.loginfo("  模式: %s", self.mode)
        alt_str = ', '.join(['%.1f' % a for a in self.patrol_altitudes])
        rospy.loginfo("  巡检高度: [%s] m (第%d/%d层)",
                      alt_str, self.current_layer_idx + 1, len(self.patrol_altitudes))
        rospy.loginfo("  航点: %d 个", len(self.patrol_waypoints_2d))
        if self.current_pose:
            p = self.current_pose.pose.position
            rospy.loginfo("  位置: (%.1f, %.1f, %.1f)", p.x, p.y, p.z)
        if self.mode == 'EXECUTING' and self.planned_segments:
            rospy.loginfo("  航段: %d/%d", self.current_seg_idx + 1, len(self.planned_segments))
        rospy.loginfo("  OctoMap: %s (%d体素)", "已加载" if self.map_ready else "未加载",
                      len(self.occupied_voxels))
        rospy.loginfo("  飞控: mode=%s armed=%s", self.current_state.mode, self.current_state.armed)

    # ==================== 2D A* 路径规划 ====================

    def _extract_2d_obstacles(self, altitude):
        """从3D OctoMap提取指定高度的2D障碍物切片"""
        res = self.resolution
        gz_center = int(round(altitude / res))
        # ±1m高度范围内的体素都视为该层障碍物
        margin_z = max(2, int(1.0 / res))
        # 额外安全膨胀
        margin_xy = max(1, int(self.safety_margin / res))

        obstacles = set()
        for (vx, vy, vz) in self.occupied_voxels:
            if abs(vz - gz_center) <= margin_z:
                for dx in range(-margin_xy, margin_xy + 1):
                    for dy in range(-margin_xy, margin_xy + 1):
                        obstacles.add((vx + dx, vy + dy))
        return obstacles

    def _plan_astar(self, start_xy, goal_xy, obstacles_2d):
        """2D A* 路径规划 (8方向)"""
        res = self.resolution
        sx = int(round(start_xy[0] / res))
        sy = int(round(start_xy[1] / res))
        gx = int(round(goal_xy[0] / res))
        gy = int(round(goal_xy[1] / res))

        # 起/终点在障碍物中则找最近空闲点
        if (sx, sy) in obstacles_2d:
            sx, sy = self._find_nearest_free(sx, sy, obstacles_2d)
        if (gx, gy) in obstacles_2d:
            gx, gy = self._find_nearest_free(gx, gy, obstacles_2d)

        # 直线可通? 直接返回
        if self._is_line_free_2d(sx, sy, gx, gy, obstacles_2d):
            return [start_xy, goal_xy]

        # A* 搜索 (8方向)
        DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
        COSTS = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

        counter = 0
        h0 = math.hypot(gx - sx, gy - sy)
        open_set = [(h0, counter, sx, sy)]
        came_from = {}
        g_score = {(sx, sy): 0.0}
        closed = set()

        max_nodes = 80000

        while open_set and len(closed) < max_nodes:
            _, _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gx and cy == gy:
                # 回溯路径
                path_grid = []
                node = (gx, gy)
                while node in came_from:
                    path_grid.append(node)
                    node = came_from[node]
                path_grid.append((sx, sy))
                path_grid.reverse()
                # 转世界坐标 + 简化
                path_world = [(p[0] * res, p[1] * res) for p in path_grid]
                return self._simplify_path_2d(path_world, obstacles_2d)

            for (dx, dy), cost in zip(DIRS, COSTS):
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in obstacles_2d or (nx, ny) in closed:
                    continue
                ng = g_score[(cx, cy)] + cost
                if ng < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = ng
                    h = math.hypot(gx - nx, gy - ny)
                    counter += 1
                    heapq.heappush(open_set, (ng + h, counter, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

        return None  # 无路径

    def _is_line_free_2d(self, x0, y0, x1, y1, obstacles):
        """Bresenham 2D直线碰撞检查"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy)
        if steps == 0:
            return True
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            if (x, y) in obstacles:
                return False
        return True

    def _find_nearest_free(self, cx, cy, obstacles, max_r=30):
        """螺旋搜索最近的非障碍物格子"""
        for r in range(1, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        if (cx + dx, cy + dy) not in obstacles:
                            return cx + dx, cy + dy
        return cx, cy

    def _simplify_path_2d(self, path, obstacles_2d):
        """路径简化: 去除A*中间多余的格子步进点"""
        if len(path) <= 2:
            return path
        res = self.resolution
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                x0 = int(round(simplified[-1][0] / res))
                y0 = int(round(simplified[-1][1] / res))
                x1 = int(round(path[j][0] / res))
                y1 = int(round(path[j][1] / res))
                if self._is_line_free_2d(x0, y0, x1, y1, obstacles_2d):
                    break
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified

    # ==================== 飞行控制 (状态机) ====================

    def _send_setpoint(self, x, y, z, yaw=None):
        """发送位置目标"""
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
        """20Hz 主循环状态机"""
        if self.current_pose is None:
            return

        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z

        if self.mode == 'PRE_ARM':
            # ★ PX4起飞关键: 先持续发送目标高度setpoint ≥2.5秒, 再切OFFBOARD+ARM
            cur_yaw = self._get_current_yaw()
            self._send_setpoint(cx, cy, self.takeoff_target_alt, cur_yaw)
            elapsed = (rospy.Time.now() - self.pre_arm_start).to_sec()
            if elapsed >= 2.5:
                try:
                    self.set_mode_client(custom_mode="OFFBOARD")
                    rospy.loginfo("[巡检] ✓ OFFBOARD 已设置")
                except rospy.ServiceException as e:
                    rospy.logwarn("[巡检] OFFBOARD失败: %s", e)
                    return
                rospy.sleep(0.3)
                try:
                    self.arming_client(True)
                    rospy.loginfo("[巡检] ✓ 已解锁")
                except rospy.ServiceException as e:
                    rospy.logwarn("[巡检] 解锁失败: %s", e)
                    return
                self.mode = 'TAKING_OFF'
                rospy.loginfo("[巡检] 起飞中... 目标高度 %.1fm", self.takeoff_target_alt)

        elif self.mode == 'TAKING_OFF':
            # 垂直爬升到巡检高度
            cur_yaw = self._get_current_yaw()
            self._send_setpoint(cx, cy, self.takeoff_target_alt, cur_yaw)
            if abs(cz - self.takeoff_target_alt) < 0.3:
                rospy.loginfo("[巡检] ✓ 已到达巡检高度 %.1fm!", self.takeoff_target_alt)
                self.mode = 'EXECUTING'
                rospy.loginfo("[巡检] ====== 开始巡检! (第%d/%d层, 高度%.1fm) ======",
                              self.current_layer_idx + 1, len(self.patrol_altitudes),
                              self.takeoff_target_alt)

        elif self.mode == 'EXECUTING':
            self._execute_step(cx, cy, cz)

        elif self.mode == 'LAYER_RETURN':
            # 返回起点, 准备切换下一层
            hx, hy = self.home_xy if self.home_xy else (0, 0)
            alt = self.patrol_altitudes[min(self.current_layer_idx, len(self.patrol_altitudes) - 1)]
            dist = math.hypot(cx - hx, cy - hy)
            yaw = math.atan2(hy - cy, hx - cx) if dist > 0.5 else 0
            self._send_setpoint(hx, hy, alt, yaw)

            if dist < 0.8:
                self.current_layer_idx += 1
                if self.current_layer_idx < len(self.patrol_altitudes):
                    next_alt = self.patrol_altitudes[self.current_layer_idx]
                    rospy.loginfo("[巡检] ★ 第 %d 层完成! 切换到第 %d 层 (高度=%.1fm)",
                                  self.current_layer_idx, self.current_layer_idx + 1, next_alt)
                    self.takeoff_target_alt = next_alt
                    self._plan_layer(self.current_layer_idx)
                    self.mode = 'TAKING_OFF'
                else:
                    rospy.loginfo("[巡检] ====== 所有 %d 层巡检完成! ======",
                                  len(self.patrol_altitudes))
                    rospy.loginfo("[巡检] 输入 'home' 返航, 'clear' 重新规划, 'stop' 悬停")
                    self.mode = 'DONE'

        elif self.mode == 'DONE':
            # 悬停
            self._send_setpoint(cx, cy, cz)

    def _execute_step(self, cx, cy, cz):
        """执行巡检: 逐段逐点跟踪 (限速)"""
        if self.current_seg_idx >= len(self.planned_segments):
            # 当前层全部航段完成
            if len(self.patrol_altitudes) > 1 and self.current_layer_idx < len(self.patrol_altitudes) - 1:
                rospy.loginfo("[巡检] 第 %d 层航点全部完成! 返回起点准备下一层...",
                              self.current_layer_idx + 1)
                self.mode = 'LAYER_RETURN'
            else:
                rospy.loginfo("[巡检] ====== 巡检完成! ======")
                rospy.loginfo("[巡检] 输入 'home' 返航, 'clear' 重新规划")
                self.mode = 'DONE'
            return

        seg = self.planned_segments[self.current_seg_idx]
        if self.current_pt_idx >= len(seg):
            self.current_seg_idx += 1
            self.current_pt_idx = 0
            if self.current_seg_idx < len(self.planned_segments):
                rospy.loginfo("[巡检] 航段 %d/%d 完成",
                              self.current_seg_idx, len(self.planned_segments))
                self._publish_route_markers()
            return

        target = seg[self.current_pt_idx]
        dx = target[0] - cx
        dy = target[1] - cy
        dz = target[2] - cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < self.waypoint_threshold:
            self.current_pt_idx += 1
            # 到达用户航点时提示
            if self.current_pt_idx >= len(seg) and self.current_seg_idx < len(self.patrol_waypoints_2d):
                wp = self.patrol_waypoints_2d[self.current_seg_idx]
                rospy.loginfo("[巡检] ★ 到达航点 #%d: (%.1f, %.1f) 高度=%.1fm",
                              self.current_seg_idx + 1, wp[0], wp[1],
                              self.patrol_altitudes[self.current_layer_idx])
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
        self._publish_active_path()

    # ==================== 可视化 ====================

    def _publish_altitude_plane(self):
        """发布巡检高度半透明平面 + 高度文字标注 + 地面参考平面 (可点击)"""
        ma = MarkerArray()

        for layer_idx, alt in enumerate(self.patrol_altitudes):
            # 半透明平面
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "camera_init"
            m.ns = "alt_plane"
            m.id = layer_idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = 0
            m.pose.position.y = 0
            m.pose.position.z = alt
            m.pose.orientation.w = 1.0
            m.scale.x = 60.0
            m.scale.y = 30.0
            m.scale.z = 0.02
            m.color = ColorRGBA(0.2, 0.6, 1.0, 0.25)
            m.lifetime = rospy.Duration(0)
            ma.markers.append(m)

            # 高度文字标注 (中心 + 4角)
            positions = [(0, 0), (-20, -8), (20, -8), (-20, 8), (20, 8)]
            for i, (px, py) in enumerate(positions):
                txt = Marker()
                txt.header.stamp = rospy.Time.now()
                txt.header.frame_id = "camera_init"
                txt.ns = "alt_text_%d" % layer_idx
                txt.id = i
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = px
                txt.pose.position.y = py
                txt.pose.position.z = alt + 0.15
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.5 if i == 0 else 0.35
                txt.color = ColorRGBA(0.0, 0.8, 1.0, 1.0)
                if i == 0:
                    txt.text = "=== Layer%d: %.1fm ===" % (layer_idx + 1, alt)
                else:
                    txt.text = "h=%.1fm" % alt
                txt.lifetime = rospy.Duration(0)
                ma.markers.append(txt)

            # 平面边框
            border = Marker()
            border.header.stamp = rospy.Time.now()
            border.header.frame_id = "camera_init"
            border.ns = "alt_border_%d" % layer_idx
            border.id = 0
            border.type = Marker.LINE_STRIP
            border.action = Marker.ADD
            border.scale.x = 0.05
            border.color = ColorRGBA(0.0, 0.7, 1.0, 0.6)
            border.pose.orientation.w = 1.0
            border.lifetime = rospy.Duration(0)
            for px, py in [(-25, -12), (25, -12), (25, 12), (-25, 12), (-25, -12)]:
                border.points.append(Point(x=px, y=py, z=alt))
            ma.markers.append(border)

        self.alt_plane_pub.publish(ma)

    def _publish_waypoint_markers(self):
        """发布航点标记 (圆柱+编号+连线)"""
        ma = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        delete.ns = "patrol_wp"
        ma.markers.append(delete)
        delete2 = Marker()
        delete2.action = Marker.DELETEALL
        delete2.ns = "patrol_line"
        ma.markers.append(delete2)

        alt = self.patrol_altitudes[0]

        for i, (wx, wy) in enumerate(self.patrol_waypoints_2d):
            # 圆柱标记(在巡检高度)
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "camera_init"
            m.ns = "patrol_wp"
            m.id = i * 3
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = alt
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.5
            m.scale.z = 0.08
            m.color = ColorRGBA(1.0, 0.5, 0.0, 0.9)  # 橙色
            m.lifetime = rospy.Duration(0)
            ma.markers.append(m)

            # 编号文字
            mt = Marker()
            mt.header = m.header
            mt.ns = "patrol_wp"
            mt.id = i * 3 + 1
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = wx
            mt.pose.position.y = wy
            mt.pose.position.z = alt + 0.5
            mt.pose.orientation.w = 1.0
            mt.scale.z = 0.35
            mt.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            mt.text = "#%d (%.1f,%.1f)" % (i + 1, wx, wy)
            mt.lifetime = rospy.Duration(0)
            ma.markers.append(mt)

            # 从地面到高度的参考线
            mc = Marker()
            mc.header = m.header
            mc.ns = "patrol_wp"
            mc.id = i * 3 + 2
            mc.type = Marker.LINE_LIST
            mc.action = Marker.ADD
            mc.scale.x = 0.03
            mc.color = ColorRGBA(1.0, 0.5, 0.0, 0.3)
            mc.pose.orientation.w = 1.0
            mc.points.append(Point(x=wx, y=wy, z=0))
            mc.points.append(Point(x=wx, y=wy, z=alt))
            mc.lifetime = rospy.Duration(0)
            ma.markers.append(mc)

        # 航点间连线 (在巡检高度)
        if len(self.patrol_waypoints_2d) > 1:
            line = Marker()
            line.header.stamp = rospy.Time.now()
            line.header.frame_id = "camera_init"
            line.ns = "patrol_line"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = ColorRGBA(1.0, 0.8, 0.0, 0.6)
            line.pose.orientation.w = 1.0
            for wx, wy in self.patrol_waypoints_2d:
                line.points.append(Point(x=wx, y=wy, z=alt))
            line.lifetime = rospy.Duration(0)
            ma.markers.append(line)

        self.waypoint_marker_pub.publish(ma)

    def _publish_route_markers(self):
        """发布避障后的路径: 绿色=当前, 黄色=待飞, 已飞=隐藏"""
        ma = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        delete.ns = "patrol_route"
        ma.markers.append(delete)

        for seg_idx, seg in enumerate(self.planned_segments):
            if seg_idx < self.current_seg_idx:
                continue

            line = Marker()
            line.header.stamp = rospy.Time.now()
            line.header.frame_id = "camera_init"
            line.ns = "patrol_route"
            line.id = seg_idx
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.lifetime = rospy.Duration(0)

            if seg_idx == self.current_seg_idx:
                line.color = ColorRGBA(0.0, 1.0, 0.0, 0.9)
                line.scale.x = 0.1
            else:
                line.color = ColorRGBA(1.0, 0.8, 0.0, 0.6)
                line.scale.x = 0.06

            for pt in seg:
                line.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))
            ma.markers.append(line)

        self.route_pub.publish(ma)

    def _publish_active_path(self):
        """发布当前航段剩余路径"""
        if self.current_seg_idx >= len(self.planned_segments):
            return
        seg = self.planned_segments[self.current_seg_idx]
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "camera_init"
        for pt in seg[self.current_pt_idx:]:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            ps.pose.position.z = pt[2]
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.active_path_pub.publish(path_msg)

    # ==================== 无人机位置可视化 ====================

    def _publish_drone_marker(self):
        """发布无人机当前位置标记 (红色球体+朝向箭头+高度文字)"""
        if self.current_pose is None:
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        ma = MarkerArray()

        # 球体
        sphere = Marker()
        sphere.header.stamp = rospy.Time.now()
        sphere.header.frame_id = "map"
        sphere.ns = "drone_pos"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = p
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
        sphere.color = ColorRGBA(1.0, 0.0, 0.0, 0.9)
        sphere.lifetime = rospy.Duration(0.5)
        ma.markers.append(sphere)

        # 朝向箭头
        arrow = Marker()
        arrow.header = sphere.header
        arrow.ns = "drone_pos"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position = p
        arrow.pose.orientation = q
        arrow.scale.x = 1.0
        arrow.scale.y = 0.15
        arrow.scale.z = 0.15
        arrow.color = ColorRGBA(1.0, 0.2, 0.2, 0.9)
        arrow.lifetime = rospy.Duration(0.5)
        ma.markers.append(arrow)

        # 高度文字
        txt = Marker()
        txt.header = sphere.header
        txt.ns = "drone_pos"
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = p.x
        txt.pose.position.y = p.y
        txt.pose.position.z = p.z + 0.6
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.3
        txt.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        txt.text = "UAV h=%.1fm" % p.z
        txt.lifetime = rospy.Duration(0.5)
        ma.markers.append(txt)

        # 从地面到无人机的垂直参考线
        vline = Marker()
        vline.header = sphere.header
        vline.ns = "drone_pos"
        vline.id = 3
        vline.type = Marker.LINE_LIST
        vline.action = Marker.ADD
        vline.scale.x = 0.03
        vline.color = ColorRGBA(1.0, 0.3, 0.3, 0.4)
        vline.pose.orientation.w = 1.0
        vline.points.append(Point(x=p.x, y=p.y, z=0))
        vline.points.append(Point(x=p.x, y=p.y, z=p.z))
        vline.lifetime = rospy.Duration(0.5)
        ma.markers.append(vline)

        self.drone_marker_pub.publish(ma)

    # ==================== 主循环 ====================

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._fly_step()
            # 每4帧发布一次无人机位置标记 (~5Hz)
            self._drone_viz_counter += 1
            if self._drone_viz_counter >= 4:
                self._drone_viz_counter = 0
                self._publish_drone_marker()
            rate.sleep()


if __name__ == '__main__':
    try:
        planner = PatrolPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
