/*
 * nan_filter_node.cpp
 * 点云NaN过滤 + 体素降采样节点
 * /cloud_registered_body → /cloud_filtered
 *
 * C++版本: 使用PCL进行高效过滤和降采样
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <clocale>

static ros::Publisher pub;
static int frame_count = 0;
static int drop_count = 0;
static const double VOXEL_SIZE = 0.15;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (msg->width * msg->height == 0) {
        pub.publish(*msg);
        return;
    }

    // 转PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    int before = cloud->size();

    // 移除NaN点
    pcl::PointCloud<pcl::PointXYZ>::Ptr clean(new pcl::PointCloud<pcl::PointXYZ>);
    clean->reserve(cloud->size());
    for (const auto &pt : *cloud) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) &&
            std::abs(pt.x) < 1e6 && std::abs(pt.y) < 1e6 && std::abs(pt.z) < 1e6) {
            clean->push_back(pt);
        }
    }

    if (clean->empty()) return;

    // 体素降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(clean);
    vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    vg.filter(*downsampled);

    // 转回ROS消息
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*downsampled, out);
    out.header = msg->header;
    pub.publish(out);

    frame_count++;
    drop_count += (before - (int)downsampled->size());
    if (frame_count % 100 == 0) {
        ROS_INFO_THROTTLE(15, "[NaN Filter] 已处理 %d 帧, 本帧: %d→%d→%zu, 累计过滤 %d",
                          frame_count, before, (int)clean->size(), downsampled->size(), drop_count);
    }
}

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "nan_filter");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 2);
    ros::Subscriber sub = nh.subscribe("/cloud_registered_body", 2, cloudCallback);

    ROS_INFO("[NaN Filter C++] 已启动 - /cloud_registered_body → /cloud_filtered (体素=%.2fm)", VOXEL_SIZE);
    ros::spin();
    return 0;
}
