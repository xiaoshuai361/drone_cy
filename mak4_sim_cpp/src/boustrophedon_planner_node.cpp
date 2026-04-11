/*
 * boustrophedon_planner_node.cpp
 * 3D 牛耕式全覆盖路径规划器 (C++版)
 * 在指定3D区域内生成蛇形覆盖路径, 支持多层高度
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <cmath>
#include <clocale>

class BoustrophedonPlanner
{
public:
    BoustrophedonPlanner() : nh_("~")
    {
        nh_.param("x_min", x_min_, -9.0);
        nh_.param("x_max", x_max_, 9.0);
        nh_.param("y_min", y_min_, -3.5);
        nh_.param("y_max", y_max_, 3.5);
        nh_.param("line_spacing", line_spacing_, 1.5);
        nh_.param("step_size", step_size_, 0.5);

        // z_layers 参数: 默认 [1.5, 2.5, 3.5]
        std::vector<double> default_layers = {1.5, 2.5, 3.5};
        nh_.param("z_layers", z_layers_, default_layers);

        path_pub_ = nh_.advertise<nav_msgs::Path>("/coverage_path", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/coverage_markers", 1, true);

        nh_.subscribe("/mavros/local_position/pose", 10, &BoustrophedonPlanner::poseCb, this);

        ROS_INFO("[Boustrophedon] 等待位姿...");
        ros::Duration(2.0).sleep();

        auto path = generatePath();
        publishPath(path);
        publishMarkers(path);
        ROS_INFO("[Boustrophedon] 覆盖路径已发布, 共 %zu 个路点", path.size());
    }

    void run() { ros::spin(); }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_, marker_pub_;
    double x_min_, x_max_, y_min_, y_max_, line_spacing_, step_size_;
    std::vector<double> z_layers_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &) {}

    std::vector<std::tuple<double,double,double>> generatePath()
    {
        std::vector<std::tuple<double,double,double>> wps;
        // Y轴扫描线
        std::vector<double> y_lines;
        for (double y = y_min_; y <= y_max_ + line_spacing_ * 0.5; y += line_spacing_)
            y_lines.push_back(y);

        for (size_t li = 0; li < z_layers_.size(); ++li) {
            double z = z_layers_[li];
            auto lines = y_lines;
            if (li % 2 == 1) std::reverse(lines.begin(), lines.end());

            for (size_t i = 0; i < lines.size(); ++i) {
                double y = lines[i];
                if (i % 2 == 0) {
                    for (double x = x_min_; x <= x_max_ + step_size_ * 0.5; x += step_size_)
                        wps.emplace_back(x, y, z);
                } else {
                    for (double x = x_max_; x >= x_min_ - step_size_ * 0.5; x -= step_size_)
                        wps.emplace_back(x, y, z);
                }
            }
        }
        return wps;
    }

    void publishPath(const std::vector<std::tuple<double,double,double>> &wps)
    {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        for (auto &[x,y,z] : wps) {
            geometry_msgs::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_.publish(msg);
    }

    void publishMarkers(const std::vector<std::tuple<double,double,double>> &wps)
    {
        visualization_msgs::MarkerArray ma;
        // 路径线
        visualization_msgs::Marker line;
        line.header.frame_id = "map"; line.header.stamp = ros::Time::now();
        line.ns = "coverage_path"; line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.05; line.pose.orientation.w = 1.0;
        line.color.g = 1; line.color.a = 0.8;
        for (auto &[x,y,z] : wps) {
            geometry_msgs::Point p; p.x = x; p.y = y; p.z = z;
            line.points.push_back(p);
        }
        ma.markers.push_back(line);

        // 起终点
        auto addSphere = [&](const std::tuple<double,double,double> &wp, int id, float r, float g, float b) {
            visualization_msgs::Marker s;
            s.header = line.header; s.ns = "coverage_endpoints"; s.id = id;
            s.type = visualization_msgs::Marker::SPHERE;
            s.action = visualization_msgs::Marker::ADD;
            auto [x,y,z] = wp;
            s.pose.position.x = x; s.pose.position.y = y; s.pose.position.z = z;
            s.pose.orientation.w = 1.0;
            s.scale.x = s.scale.y = s.scale.z = 0.3;
            s.color.r = r; s.color.g = g; s.color.b = b; s.color.a = 1;
            ma.markers.push_back(s);
        };
        if (!wps.empty()) {
            addSphere(wps.front(), 0, 0, 1, 0);
            addSphere(wps.back(), 1, 1, 0, 0);
        }
        marker_pub_.publish(ma);
    }
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "boustrophedon_planner");
    BoustrophedonPlanner planner;
    planner.run();
    return 0;
}
