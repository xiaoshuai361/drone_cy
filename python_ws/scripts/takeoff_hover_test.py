#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAK4 无人机起飞悬停测试脚本
通过 MAVROS 与 PX4 SITL 通信，实现自动解锁、切换OFFBOARD模式、起飞并悬停
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest


class MAK4TakeoffTest:
    def __init__(self):
        rospy.init_node('mak4_takeoff_test', anonymous=True)

        # 目标高度 (米)
        self.target_altitude = 2.5
        self.current_state = State()
        self.current_pose = PoseStamped()

        # 订阅飞控状态
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        # 发布目标位置
        self.local_pos_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 服务客户端
        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        rospy.wait_for_service('/mavros/set_mode', timeout=30)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)  # 20Hz

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def wait_for_connection(self):
        """等待与飞控建立连接"""
        rospy.loginfo("等待飞控连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("飞控已连接!")

    def send_setpoints_before_offboard(self):
        """在切换到OFFBOARD模式前需要先发送一些setpoint"""
        rospy.loginfo("预发送 setpoint（OFFBOARD 模式要求）...")
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.target_altitude

        for i in range(100):
            if rospy.is_shutdown():
                return
            pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

    def set_offboard_mode(self):
        """切换到 OFFBOARD 模式"""
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'OFFBOARD'

        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and \
               (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                resp = self.set_mode_client(mode_req)
                if resp.mode_sent:
                    rospy.loginfo("OFFBOARD 模式已请求")
                last_req = rospy.Time.now()

            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo("已切换到 OFFBOARD 模式!")
                return True

            # 持续发送目标位置
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = self.target_altitude
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
        return False

    def arm(self):
        """解锁电机"""
        arm_req = CommandBoolRequest()
        arm_req.value = True

        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.current_state.armed and \
               (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                resp = self.arming_client(arm_req)
                if resp.success:
                    rospy.loginfo("解锁命令已发送")
                last_req = rospy.Time.now()

            if self.current_state.armed:
                rospy.loginfo("无人机已解锁!")
                return True

            # 持续发送目标位置
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = self.target_altitude
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
        return False

    def hover(self, duration=30):
        """在目标高度悬停指定时间（秒）"""
        rospy.loginfo("起飞到 %.1f 米并悬停 %d 秒..." % (self.target_altitude, duration))
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > duration:
                rospy.loginfo("悬停完成!")
                break

            current_alt = self.current_pose.pose.position.z
            if elapsed > 5 and abs(current_alt - self.target_altitude) < 0.3:
                if int(elapsed) % 5 == 0:
                    rospy.loginfo("悬停中... 当前高度: %.2f m | 目标: %.1f m | 已过 %.0f 秒" %
                                  (current_alt, self.target_altitude, elapsed))

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = self.target_altitude
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

    def land(self):
        """切换到自动降落模式"""
        rospy.loginfo("开始降落...")
        mode_req = SetModeRequest()
        mode_req.custom_mode = 'AUTO.LAND'
        resp = self.set_mode_client(mode_req)
        if resp.mode_sent:
            rospy.loginfo("降落模式已切换!")

        # 等待降落完成
        while not rospy.is_shutdown():
            if self.current_pose.pose.position.z < 0.15:
                rospy.loginfo("已降落!")
                break
            if not self.current_state.armed:
                rospy.loginfo("电机已锁定，降落完成!")
                break
            self.rate.sleep()

    def run(self):
        """执行完整的起飞悬停降落测试"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("MAK4 无人机起飞悬停测试")
        rospy.loginfo("=" * 50)

        # 1. 等待飞控连接
        self.wait_for_connection()

        # 2. 预发送 setpoint
        self.send_setpoints_before_offboard()

        # 3. 切换到 OFFBOARD 模式
        if not self.set_offboard_mode():
            return

        # 4. 解锁
        if not self.arm():
            return

        # 5. 悬停 30 秒
        self.hover(duration=30)

        # 6. 降落
        self.land()

        rospy.loginfo("=" * 50)
        rospy.loginfo("测试完成!")
        rospy.loginfo("=" * 50)


if __name__ == '__main__':
    try:
        test = MAK4TakeoffTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
