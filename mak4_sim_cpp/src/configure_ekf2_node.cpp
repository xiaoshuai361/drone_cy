/*
 * configure_ekf2_node.cpp
 * 配置 PX4 EKF2 参数 (通过 MAVROS)
 * 启用下视测距仪 + 光流融合, 提高室内定高/定位精度
 *
 * C++版本: 第二十次新增
 */
#include <ros/ros.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <clocale>

static ros::ServiceClient set_client, get_client;

bool setIntParam(const std::string &name, int value, int retries = 3)
{
    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.integer = value;
    srv.request.value.real = 0.0;
    for (int i = 0; i < retries; ++i) {
        if (set_client.call(srv) && srv.response.success) {
            ROS_INFO("[EKF2] %s = %d", name.c_str(), value);
            return true;
        }
        if (i < retries - 1) ros::Duration(1.0).sleep();
    }
    ROS_WARN("[EKF2] %s 设置失败", name.c_str());
    return false;
}

bool setFloatParam(const std::string &name, double value, int retries = 3)
{
    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.integer = 0;
    srv.request.value.real = value;
    for (int i = 0; i < retries; ++i) {
        if (set_client.call(srv) && srv.response.success) {
            ROS_INFO("[EKF2] %s = %.2f", name.c_str(), value);
            return true;
        }
        if (i < retries - 1) ros::Duration(1.0).sleep();
    }
    ROS_WARN("[EKF2] %s 设置失败", name.c_str());
    return false;
}

bool trySetInt(const std::vector<std::string> &names, int value)
{
    for (const auto &name : names) {
        if (setIntParam(name, value, 2)) return true;
    }
    ROS_ERROR("[EKF2] 所有候选参数均失败");
    return false;
}

void waitForParamsReady(int timeout = 30)
{
    ros::Time start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < timeout) {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = "MAV_SYS_ID";
        if (get_client.call(srv) && srv.response.success) {
            ROS_INFO("[EKF2] PX4参数系统就绪 (MAV_SYS_ID=%ld)", srv.response.value.integer);
            return;
        }
        ros::Duration(1.0).sleep();
    }
    ROS_WARN("[EKF2] 等待PX4参数超时, 仍尝试配置...");
}

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "configure_ekf2");
    ros::NodeHandle nh;

    ROS_INFO("[EKF2] 等待 MAVROS 参数服务...");
    if (!ros::service::waitForService("/mavros/param/set", ros::Duration(60))) {
        ROS_ERROR("[EKF2] MAVROS 服务超时");
        return 1;
    }
    ros::service::waitForService("/mavros/param/get", ros::Duration(10));

    set_client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    get_client = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

    ROS_INFO("[EKF2] 等待PX4参数系统就绪...");
    waitForParamsReady();

    ROS_INFO("[EKF2] 开始配置 EKF2 参数...");

    // 高度源: 测距仪
    trySetInt({"EKF2_HGT_REF", "EKF2_HGT_MODE"}, 2);
    // 测距仪控制: 始终启用
    trySetInt({"EKF2_RNG_CTRL", "EKF2_RNG_AID"}, 2);
    // 光流定位
    setIntParam("EKF2_OF_CTRL", 1);
    setIntParam("EKF2_AID_MASK", 2, 1);
    // 测距仪参数
    setFloatParam("EKF2_RNG_A_HMAX", 8.0);
    setFloatParam("EKF2_RNG_NOISE", 0.05);
    setFloatParam("EKF2_RNG_A_VMAX", 2.0);

    ROS_INFO("[EKF2] ✓ 配置完成!");
    ROS_INFO("[EKF2]   高度: 测距仪, 水平: 光流");
    return 0;
}
