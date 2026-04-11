#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM → MAVROS 视觉定位桥接

将 Fast-LIO2 的高精度里程计输出转发给 PX4 EKF2 作为外部视觉定位源
/Odometry (nav_msgs/Odometry) → /mavros/vision_pose/pose (geometry_msgs/PoseStamped)

★ 第十三次新增: 解决无人机悬停漂移问题
   原因: Gazebo 光流仿真在大棚内(无纹理地面)不可靠 → 位置估计漂移
   方案: 用 Fast-LIO2 的 SLAM 位置(精度~cm级)替代光流, 回传给 PX4 EKF2
         PX4 用 SLAM 位置做飞行控制 → 悬停稳如磐石

   工作流: Fast-LIO2(LiDAR+IMU) → 高精度位姿 → MAVROS → PX4 EKF2
           PX4 EKF2 融合 SLAM 位置 → 精确速度/位置控制
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class SlamToMavros:
    def __init__(self):
        rospy.init_node('slam_to_mavros', anonymous=True)
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=2)
        rospy.Subscriber('/Odometry', Odometry, self.callback, queue_size=2)
        self.count = 0
        rospy.loginfo("[Vision Bridge] Fast-LIO2 → PX4 EKF2 视觉定位已启动")
        rospy.loginfo("[Vision Bridge] /Odometry → /mavros/vision_pose/pose")

    def callback(self, msg):
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = "map"
        ps.pose = msg.pose.pose
        self.pub.publish(ps)

        self.count += 1
        if self.count % 100 == 0:
            p = msg.pose.pose.position
            rospy.loginfo_throttle(30, "[Vision Bridge] 已转发 %d 帧, 位置: (%.2f, %.2f, %.2f)",
                                   self.count, p.x, p.y, p.z)


if __name__ == '__main__':
    try:
        SlamToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
