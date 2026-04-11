/*
 * slam_to_mavros_node.cpp
 * Fast-LIO2 → MAVROS 视觉定位桥接
 * /Odometry → /mavros/vision_pose/pose
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <clocale>

static ros::Publisher pub;
static int count = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = msg->header.stamp;
    ps.header.frame_id = "map";
    ps.pose = msg->pose.pose;
    pub.publish(ps);

    if (++count % 100 == 0) {
        ROS_INFO_THROTTLE(30, "[Vision Bridge] 已转发 %d 帧, 位置: (%.2f, %.2f, %.2f)",
                          count, msg->pose.pose.position.x,
                          msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
}

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "slam_to_mavros");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 2);
    ros::Subscriber sub = nh.subscribe("/Odometry", 2, odomCallback);

    ROS_INFO("[Vision Bridge] Fast-LIO2 → PX4 EKF2 已启动");
    ros::spin();
    return 0;
}
