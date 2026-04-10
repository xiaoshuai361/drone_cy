#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置 PX4 EKF2 参数 (通过 MAVROS)
启用下视测距仪 + 光流融合, 提高室内定高/定位精度

★ 测距仪: 提供精确高度 (取代气压计, 消除高度漂移)
★ 光流: 提供室内水平位置 (GPS拒止环境下)
★ 与 3D LiDAR SLAM 独立运行, 互不冲突

第十四次修复: 移除视觉定位(SLAM→PX4), 回归光流+测距仪方案
第十九次修复: 参数名更新为 PX4 v1.14.3 实际参数名
  旧名称 → 新名称:
    EKF2_HGT_MODE → EKF2_HGT_REF  (高度参考源)
    EKF2_RNG_AID  → EKF2_RNG_CTRL  (测距仪控制)
    EKF2_AID_MASK(bit1=光流) → EKF2_OF_CTRL (光流独立开关)
  原因: PX4 v1.14.3 重构了 EKF2 参数命名体系
        旧参数名在运行时报 "Unknown parameter" 导致所有配置失败
        → 无人机无法起飞 (无高度/位置参考)
"""
import rospy
from mavros_msgs.srv import ParamSet, ParamGet
from mavros_msgs.msg import ParamValue


def set_int_param(srv, name, value, retries=3):
    """设置整型 PX4 参数 (带重试)"""
    for attempt in range(retries):
        pv = ParamValue(integer=value, real=0.0)
        try:
            resp = srv(param_id=name, value=pv)
            if resp.success:
                rospy.loginfo("[EKF2] %s = %d", name, value)
                return True
            else:
                if attempt < retries - 1:
                    rospy.sleep(1.0)
                else:
                    rospy.logwarn("[EKF2] %s 设置失败 (参数可能不存在)", name)
                    return False
        except Exception as e:
            if attempt < retries - 1:
                rospy.sleep(1.0)
            else:
                rospy.logwarn("[EKF2] %s 错误: %s", name, e)
                return False
    return False


def set_float_param(srv, name, value, retries=3):
    """设置浮点型 PX4 参数 (带重试)"""
    for attempt in range(retries):
        pv = ParamValue(integer=0, real=value)
        try:
            resp = srv(param_id=name, value=pv)
            if resp.success:
                rospy.loginfo("[EKF2] %s = %.2f", name, value)
                return True
            else:
                if attempt < retries - 1:
                    rospy.sleep(1.0)
                else:
                    rospy.logwarn("[EKF2] %s 设置失败 (参数可能不存在)", name)
                    return False
        except Exception as e:
            if attempt < retries - 1:
                rospy.sleep(1.0)
            else:
                rospy.logwarn("[EKF2] %s 错误: %s", name, e)
                return False
    return False


def try_set_int(srv, names, value):
    """尝试多个候选参数名 (兼容不同PX4版本)"""
    for name in names:
        if set_int_param(srv, name, value, retries=2):
            return True
    rospy.logerr("[EKF2] 所有候选参数均失败: %s", names)
    return False


def try_set_float(srv, names, value):
    """尝试多个候选参数名 (兼容不同PX4版本)"""
    for name in names:
        if set_float_param(srv, name, value, retries=2):
            return True
    rospy.logerr("[EKF2] 所有候选参数均失败: %s", names)
    return False


def wait_for_params_ready(get_srv, timeout=30):
    """等待PX4参数系统就绪 (用MAV_SYS_ID探测, 所有PX4版本都有)"""
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < timeout:
        try:
            resp = get_srv(param_id='MAV_SYS_ID')
            if resp.success:
                rospy.loginfo("[EKF2] PX4参数系统就绪 (MAV_SYS_ID=%d)", resp.value.integer)
                return True
        except Exception:
            pass
        rospy.sleep(1.0)
    rospy.logwarn("[EKF2] 等待PX4参数超时, 仍尝试配置...")
    return False


def main():
    rospy.init_node('configure_ekf2', anonymous=True)
    rospy.loginfo("[EKF2] 等待 MAVROS 参数服务...")

    try:
        rospy.wait_for_service('/mavros/param/set', timeout=60)
        rospy.wait_for_service('/mavros/param/get', timeout=10)
    except rospy.ROSException:
        rospy.logerr("[EKF2] MAVROS 服务超时, 跳过 EKF2 配置")
        return

    set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
    get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)

    # ★ 等待PX4参数系统完全就绪 (而非仅MAVROS服务可用)
    rospy.loginfo("[EKF2] 等待PX4参数系统就绪...")
    if not wait_for_params_ready(get_param, timeout=30):
        rospy.logwarn("[EKF2] 未确认参数就绪, 仍尝试配置...")

    rospy.loginfo("[EKF2] 开始配置 EKF2 参数...")

    # === 高度源: 使用测距仪 ===
    # PX4 v1.14.3: EKF2_HGT_REF (新), 旧版: EKF2_HGT_MODE
    # 值: 0=气压计, 1=GPS, 2=测距仪, 3=视觉
    try_set_int(set_param, ['EKF2_HGT_REF', 'EKF2_HGT_MODE'], 2)

    # === 测距仪控制: 始终启用 ===
    # PX4 v1.14.3: EKF2_RNG_CTRL (新), 旧版: EKF2_RNG_AID
    # EKF2_RNG_CTRL: 0=禁用, 1=条件(仅起降), 2=始终启用
    try_set_int(set_param, ['EKF2_RNG_CTRL', 'EKF2_RNG_AID'], 2)

    # === 光流定位控制 ===
    # PX4 v1.14.3: EKF2_OF_CTRL (新独立开关), 旧版: EKF2_AID_MASK bit1
    # EKF2_OF_CTRL: 0=禁用, 1=启用
    set_int_param(set_param, 'EKF2_OF_CTRL', 1)
    # 旧版兼容: 同时设置 AID_MASK (某些PX4版本仍看此参数)
    set_int_param(set_param, 'EKF2_AID_MASK', 2, retries=1)

    # === 测距仪参数 ===
    set_float_param(set_param, 'EKF2_RNG_A_HMAX', 8.0)   # 测距仪最大有效高度
    set_float_param(set_param, 'EKF2_RNG_NOISE', 0.05)    # 测距仪噪声 (m)
    set_float_param(set_param, 'EKF2_RNG_A_VMAX', 2.0)    # 测距仪最大垂直速度

    rospy.loginfo("[EKF2] ✓ 配置完成!")
    rospy.loginfo("[EKF2]   高度: 测距仪 (EKF2_HGT_REF=2, EKF2_RNG_CTRL=2)")
    rospy.loginfo("[EKF2]   水平: 光流 (EKF2_OF_CTRL=1)")
    rospy.loginfo("[EKF2]   与真机 MTF-01P 光流测距一体模块匹配")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
