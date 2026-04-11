#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图保存节点
保存 OctoMap (.bt) 和 Fast-LIO2 点云地图 (.pcd)

重要: 必须在 slam_mapping.launch 仍在运行时执行本脚本!
      本脚本会先保存OctoMap, 再自动停止Fast-LIO2以触发PCD写入.

用法:
  rosrun mak4_sim save_map.py [保存目录]
  默认: ~/catkin_ws/maps/

保存的文件:
  - greenhouse_map.bt  : OctoMap 二进制地图
  - greenhouse_map.pcd : Fast-LIO2 全局点云
"""

import rospy
import os
import sys
import subprocess
import shutil
import time
import rosnode


def check_node_running(node_name):
    """检查ROS节点是否正在运行"""
    try:
        nodes = rosnode.get_node_names()
        return node_name in nodes
    except Exception:
        return False


def save_map():
    rospy.init_node('save_map', anonymous=True)

    save_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/catkin_ws/maps')
    os.makedirs(save_dir, exist_ok=True)

    bt_path = os.path.join(save_dir, 'greenhouse_map.bt')
    pcd_path = os.path.join(save_dir, 'greenhouse_map.pcd')

    rospy.loginfo("[SaveMap] 保存目录: %s", save_dir)

    # ========== 检查前提条件 ==========
    octomap_running = check_node_running('/octomap_server')
    fastlio_running = check_node_running('/laserMapping')

    if not octomap_running:
        rospy.logerr("=" * 55)
        rospy.logerr("[SaveMap] 错误: OctoMap Server 未运行!")
        rospy.logerr("[SaveMap] 请在 slam_mapping.launch 运行时执行本脚本")
        rospy.logerr("[SaveMap] 正确流程:")
        rospy.logerr("  1. 保持 slam_mapping.launch 运行")
        rospy.logerr("  2. 新终端运行: rosrun mak4_sim save_map.py")
        rospy.logerr("  3. 脚本会自动保存地图并停止建图节点")
        rospy.logerr("=" * 55)
        return

    # ========== 1. 保存 OctoMap ==========
    rospy.loginfo("[SaveMap] 正在保存 OctoMap (服务器运行中)...")
    try:
        ret = subprocess.run(
            ['rosrun', 'octomap_server', 'octomap_saver', '-f', bt_path],
            timeout=30, capture_output=True, text=True
        )
        if ret.returncode == 0 and os.path.exists(bt_path):
            size_kb = os.path.getsize(bt_path) / 1024
            if size_kb < 1.0:
                rospy.logwarn("[SaveMap] OctoMap 文件太小 (%.1f KB), 地图可能为空!", size_kb)
                rospy.logwarn("[SaveMap] 请确认建图时飞行了足够距离, NaN过滤正常工作")
            else:
                rospy.loginfo("[SaveMap] OctoMap 保存成功: %s (%.1f KB)", bt_path, size_kb)
        else:
            rospy.logwarn("[SaveMap] OctoMap 保存失败: %s", ret.stderr.strip() if ret.stderr else "未知错误")
    except subprocess.TimeoutExpired:
        rospy.logwarn("[SaveMap] OctoMap 保存超时 - OctoMap服务器可能无响应")
    except Exception as e:
        rospy.logwarn("[SaveMap] OctoMap 异常: %s", e)

    # ========== 2. 停止 Fast-LIO2 以触发 PCD 保存 ==========
    if fastlio_running:
        rospy.loginfo("[SaveMap] 正在停止 Fast-LIO2 节点以触发PCD写入...")
        try:
            subprocess.run(['rosnode', 'kill', '/laserMapping'],
                           timeout=10, capture_output=True, text=True)
            # 等待 PCD 文件生成
            rospy.loginfo("[SaveMap] 等待PCD文件生成...")
            for i in range(15):  # 最多等15秒
                time.sleep(1)
                pcd_src = os.path.expanduser('~/catkin_ws/src/FAST_LIO/PCD/scans.pcd')
                if os.path.isfile(pcd_src) and os.path.getsize(pcd_src) > 100:
                    break
        except Exception as e:
            rospy.logwarn("[SaveMap] 停止Fast-LIO2失败: %s", e)

    # ========== 3. 查找并复制 PCD ==========
    fastlio_pcd_paths = [
        os.path.expanduser('~/catkin_ws/src/FAST_LIO/PCD/scans.pcd'),
        os.path.expanduser('~/catkin_ws/src/FAST_LIO/PCD/'),
        os.path.expanduser('~/.ros/scans.pcd'),
        '/tmp/scans.pcd',
    ]

    pcd_found = False
    for src in fastlio_pcd_paths:
        if os.path.isdir(src):
            pcds = sorted(
                [os.path.join(src, f) for f in os.listdir(src) if f.endswith('.pcd')],
                key=os.path.getmtime, reverse=True
            )
            if pcds:
                shutil.copy2(pcds[0], pcd_path)
                size_mb = os.path.getsize(pcd_path) / (1024*1024)
                rospy.loginfo("[SaveMap] PCD 保存: %s (来自 %s, %.1f MB)", pcd_path, pcds[0], size_mb)
                pcd_found = True
                break
        elif os.path.isfile(src) and os.path.getsize(src) > 100:
            shutil.copy2(src, pcd_path)
            size_mb = os.path.getsize(pcd_path) / (1024*1024)
            rospy.loginfo("[SaveMap] PCD 保存: %s (来自 %s, %.1f MB)", pcd_path, src, size_mb)
            pcd_found = True
            break

    if not pcd_found:
        rospy.logwarn("[SaveMap] 未找到 PCD 文件")
        rospy.loginfo("[SaveMap] Fast-LIO2 PCD路径: ~/catkin_ws/src/FAST_LIO/PCD/scans.pcd")

    # ========== 输出汇总 ==========
    rospy.loginfo("=" * 55)
    rospy.loginfo("[SaveMap] 地图保存完成!")
    bt_ok = os.path.exists(bt_path) and os.path.getsize(bt_path) > 200
    pcd_ok = os.path.exists(pcd_path) and os.path.getsize(pcd_path) > 100
    rospy.loginfo("  OctoMap: %s %s", bt_path,
                  "(OK %.1fKB)" % (os.path.getsize(bt_path)/1024) if bt_ok else "(空或失败)")
    rospy.loginfo("  PCD:     %s %s", pcd_path,
                  "(OK %.1fMB)" % (os.path.getsize(pcd_path)/(1024*1024)) if pcd_ok else "(未找到)")
    if bt_ok:
        rospy.loginfo("")
        rospy.loginfo("  下一步: roslaunch mak4_sim navigation.launch")
        rospy.loginfo("  在RViz工具栏选择 'Publish Point', 点击地图设置目标点")
    else:
        rospy.logwarn("  地图为空! 请重新建图并确认NaN过滤无报错")
    rospy.loginfo("=" * 55)


if __name__ == '__main__':
    try:
        save_map()
    except rospy.ROSInterruptException:
        pass
