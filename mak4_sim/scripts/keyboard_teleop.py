#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAK4 无人机键盘遥控节点 (低延迟版)
持续速度模式: 按下方向键设置速度, 持续发送直到按空格悬停

键盘控制:
  W/S = 前进/后退    A/D = 左移/右移
  Q/E = 左转/右转    R/F = 上升/下降
  T   = 一键起飞(OFFBOARD+ARM+2m高度)
  L   = 降落
  空格= 零速悬停     1/2 = 减速/加速
  ESC = 退出

运行: rosrun mak4_sim keyboard_teleop.py
"""

import rospy
import sys
import select
import termios
import tty
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode


HELP_MSG = """
==============================================
   MAK4 无人机键盘遥控 (持续速度模式)
==============================================
  T = 起飞(OFFBOARD+ARM)   L = 降落

     W = 前进       R = 上升
  A=左  S=后退  D=右  F = 下降
  Q = 左转       E = 右转

  空格 = 悬停    1/2 = 减速/加速
  ESC  = 退出
----------------------------------------------
 >> 按下方向键后速度持续, 按空格停止 <<
==============================================
"""


class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop', anonymous=True)

        self.linear_speed = 0.5
        self.vertical_speed = 0.3
        self.yaw_rate = 0.5
        self.speed_step = 0.1

        # 持续发送的速度分量
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yr = 0.0

        # 起飞后的位置保持目标 (解决起飞后立即落地问题)
        self.hold_position = None
        self.use_position_hold = False
        self.hold_altitude = None  # 定高目标 (vz=0时锁定Z轴位置)

        self.current_state = State()
        self.current_pose = PoseStamped()

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.pos_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
        rospy.wait_for_service('/mavros/set_mode', timeout=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.old_settings = termios.tcgetattr(sys.stdin)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def get_key(self):
        """非阻塞读取 - 超时20ms"""
        if select.select([sys.stdin], [], [], 0.02)[0]:
            return sys.stdin.read(1)
        return ''

    def send_command(self):
        """发送飞行指令: 水平用速度, 垂直用位置保持 (定高防漂移)"""
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # 从位姿四元数提取偏航角
        q = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 将机体坐标速度 (vx=前, vy=左) 旋转到世界坐标 (ENU)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        msg.velocity.x = self.vx * cos_yaw - self.vy * sin_yaw
        msg.velocity.y = self.vx * sin_yaw + self.vy * cos_yaw
        msg.yaw_rate = self.yr

        if self.hold_altitude is not None and self.vz == 0.0:
            # ★ 定高模式: 水平速度 + Z轴位置锁定 (解决高度漂移)
            msg.position.z = self.hold_altitude
            msg.type_mask = (
                PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW)
        else:
            # 全速度模式 (手动升降中)
            msg.velocity.z = self.vz
            msg.type_mask = (
                PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_PZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW)

        self.raw_pub.publish(msg)

    def takeoff(self, height=1.5):
        rospy.loginfo("[Teleop] 起飞序列... 目标高度=%.1fm", height)
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = self.current_pose.pose.position.x
        target.pose.position.y = self.current_pose.pose.position.y
        target.pose.position.z = height
        target.pose.orientation = self.current_pose.pose.orientation

        rate = rospy.Rate(30)
        # 预发送setpoint 2.5秒 (PX4 OFFBOARD要求)
        for _ in range(75):
            target.header.stamp = rospy.Time.now()
            self.pos_pub.publish(target)
            rate.sleep()

        if self.current_state.mode != "OFFBOARD":
            try:
                self.set_mode_client(custom_mode="OFFBOARD")
                rospy.loginfo("[Teleop] OFFBOARD 已设置")
            except rospy.ServiceException as e:
                rospy.logwarn("[Teleop] OFFBOARD失败: %s", e)

        rospy.sleep(0.3)

        if not self.current_state.armed:
            try:
                self.arming_client(True)
                rospy.loginfo("[Teleop] 已解锁")
            except rospy.ServiceException as e:
                rospy.logwarn("[Teleop] 解锁失败: %s", e)

        # 持续发送位置目标直到到达高度 (最多15秒)
        for i in range(450):
            target.header.stamp = rospy.Time.now()
            self.pos_pub.publish(target)
            rate.sleep()
            if abs(self.current_pose.pose.position.z - height) < 0.3:
                rospy.loginfo("[Teleop] 已到达目标高度!")
                break
            if i % 30 == 0 and i > 0:
                rospy.loginfo("[Teleop] 爬升中... 当前高度: %.1fm / %.1fm",
                              self.current_pose.pose.position.z, height)

        # ★ 起飞完成后: 设置位置保持模式 (防止切到速度模式后掉高度)
        self.hold_position = PoseStamped()
        self.hold_position.header.frame_id = "map"
        self.hold_position.pose.position.x = self.current_pose.pose.position.x
        self.hold_position.pose.position.y = self.current_pose.pose.position.y
        self.hold_position.pose.position.z = height
        self.hold_position.pose.orientation = self.current_pose.pose.orientation
        self.use_position_hold = True
        self.hold_altitude = None  # 位置保持模式接管高度
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yr = 0.0

        rospy.loginfo("[Teleop] 起飞完成! 高度: %.1fm (位置保持模式)", self.current_pose.pose.position.z)
        rospy.loginfo("[Teleop] 按方向键开始飞行, 空格=悬停")

    def land(self):
        rospy.loginfo("[Teleop] 降落...")
        try:
            self.set_mode_client(custom_mode="AUTO.LAND")
            rospy.loginfo("[Teleop] AUTO.LAND")
        except rospy.ServiceException as e:
            rospy.logwarn("[Teleop] 降落失败: %s", e)

    def run(self):
        print(HELP_MSG)
        tty.setraw(sys.stdin.fileno())
        rate = rospy.Rate(50)  # 50Hz 高频发送

        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                if key:
                    k = key.lower()
                    if k == 'w':
                        self.vx = self.linear_speed
                        self.vy = 0.0
                        self.vz = 0.0
                        self.yr = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 's':
                        self.vx = -self.linear_speed
                        self.vy = 0.0
                        self.vz = 0.0
                        self.yr = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 'a':
                        self.vy = self.linear_speed
                        self.vx = 0.0
                        self.vz = 0.0
                        self.yr = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 'd':
                        self.vy = -self.linear_speed
                        self.vx = 0.0
                        self.vz = 0.0
                        self.yr = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 'q':
                        self.yr = self.yaw_rate
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 'e':
                        self.yr = -self.yaw_rate
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        if self.hold_altitude is None:
                            self.hold_altitude = self.current_pose.pose.position.z
                        self.use_position_hold = False
                    elif k == 'r':
                        self.vz = self.vertical_speed
                        self.vx = 0.0
                        self.vy = 0.0
                        self.yr = 0.0
                        self.hold_altitude = None
                        self.use_position_hold = False
                    elif k == 'f':
                        self.vz = -self.vertical_speed
                        self.vx = 0.0
                        self.vy = 0.0
                        self.yr = 0.0
                        self.hold_altitude = None
                        self.use_position_hold = False
                    elif k == ' ':
                        # 空格: 悬停 = 锁定当前位置
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        self.yr = 0.0
                        self.hold_altitude = None
                        if self.current_pose:
                            self.hold_position = PoseStamped()
                            self.hold_position.header.frame_id = "map"
                            self.hold_position.pose = self.current_pose.pose
                            self.use_position_hold = True
                    elif k == '1':
                        self.linear_speed = max(0.1, self.linear_speed - self.speed_step)
                        self.vertical_speed = max(0.1, self.vertical_speed - self.speed_step)
                    elif k == '2':
                        self.linear_speed = min(2.0, self.linear_speed + self.speed_step)
                        self.vertical_speed = min(1.0, self.vertical_speed + self.speed_step)
                    elif k == 't':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                        self.takeoff(1.5)
                        tty.setraw(sys.stdin.fileno())
                        continue
                    elif k == 'l':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                        self.land()
                        tty.setraw(sys.stdin.fileno())
                        continue
                    elif key == '\x1b' or key == '\x03':
                        break

                # 每帧都发送当前速度 (保持OFFBOARD不超时)
                if self.current_state.mode == "OFFBOARD":
                    if self.use_position_hold and self.hold_position is not None:
                        # 位置保持模式: 起飞后/空格后锁定位置
                        self.hold_position.header.stamp = rospy.Time.now()
                        self.pos_pub.publish(self.hold_position)
                    else:
                        self.send_command()

                rate.sleep()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n[Teleop] 退出")


if __name__ == '__main__':
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
