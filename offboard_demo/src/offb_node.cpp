/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 全局变量，保存飞控当前状态
mavros_msgs::State current_state;

// 回调函数：每当收到飞控状态消息时更新 current_state
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点，节点名为 "offb_node"
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅飞控状态话题 mavros/state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 发布本地位置设定点到 mavros/setpoint_position/local
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 请求解锁的服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 请求切换飞行模式的服务客户端
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 发布频率必须大于 2Hz，否则 PX4 会因超时退出 Offboard 模式
    ros::Rate rate(20.0);

    // 等待飞控连接（等待心跳包）
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    // 设定目标位置：x=0, y=0, z=2（ENU坐标系，2米高度）
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 在切换到 Offboard 模式之前，必须先发送一些设定点
    // 否则模式切换会被 PX4 拒绝
    ROS_INFO("Sending initial setpoints...");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 准备 Offboard 模式请求
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 准备解锁请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 主循环
    while (ros::ok())
    {
        // 如果当前不是 Offboard 模式，尝试切换（每5秒尝试一次）
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            // 如果已经是 Offboard 模式但还没解锁，尝试解锁
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 持续发布位置设定点（必须保持 >2Hz）
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
