/*
 * takeoff_hover_test_node.cpp
 * 自动起飞悬停测试
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <cmath>
#include <clocale>

static mavros_msgs::State state;
static geometry_msgs::PoseStamped pose;

void stateCb(const mavros_msgs::State::ConstPtr &msg) { state = *msg; }
void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) { pose = *msg; }

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "takeoff_hover_test");
    ros::NodeHandle nh;

    double target_alt = 2.5;
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, stateCb);
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, poseCb);

    ros::service::waitForService("/mavros/cmd/arming", ros::Duration(30));
    ros::service::waitForService("/mavros/set_mode", ros::Duration(30));
    ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate rate(20);
    ROS_INFO("等待飞控连接...");
    while (ros::ok() && !state.connected) { ros::spinOnce(); rate.sleep(); }
    ROS_INFO("飞控已连接!");

    // 预发送setpoint
    geometry_msgs::PoseStamped sp;
    sp.header.frame_id = "map";
    sp.pose.position.z = target_alt;
    sp.pose.orientation.w = 1.0;
    ROS_INFO("预发送 setpoint...");
    for (int i = 0; i < 100 && ros::ok(); ++i) {
        sp.header.stamp = ros::Time::now();
        pos_pub.publish(sp);
        ros::spinOnce();
        rate.sleep();
    }

    // OFFBOARD + ARM
    mavros_msgs::SetMode sm;
    sm.request.custom_mode = "OFFBOARD";
    ros::Time last_req = ros::Time::now();

    while (ros::ok()) {
        if (state.mode != "OFFBOARD" && (ros::Time::now() - last_req).toSec() > 2.0) {
            mode_client.call(sm);
            last_req = ros::Time::now();
        }
        if (state.mode == "OFFBOARD") { ROS_INFO("OFFBOARD!"); break; }
        sp.header.stamp = ros::Time::now();
        pos_pub.publish(sp);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool ab;
    ab.request.value = true;
    last_req = ros::Time::now();
    while (ros::ok()) {
        if (!state.armed && (ros::Time::now() - last_req).toSec() > 2.0) {
            arm_client.call(ab);
            last_req = ros::Time::now();
        }
        if (state.armed) { ROS_INFO("已解锁!"); break; }
        sp.header.stamp = ros::Time::now();
        pos_pub.publish(sp);
        ros::spinOnce();
        rate.sleep();
    }

    // 悬停30秒
    ROS_INFO("起飞到 %.1fm...", target_alt);
    ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < 30.0) {
        sp.header.stamp = ros::Time::now();
        pos_pub.publish(sp);
        if ((int)(ros::Time::now() - start).toSec() % 5 == 0) {
            ROS_INFO_THROTTLE(5, "悬停中... h=%.2fm", pose.pose.position.z);
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 降落
    ROS_INFO("降落...");
    sm.request.custom_mode = "AUTO.LAND";
    mode_client.call(sm);
    while (ros::ok() && state.armed) { ros::spinOnce(); rate.sleep(); }
    ROS_INFO("测试完成!");
    return 0;
}
