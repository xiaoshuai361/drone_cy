/*
 * rrt_star_planner_node.cpp
 * 3D RRT* 局部避障路径规划器 (C++版)
 * 订阅覆盖路径, 实时重规划 + 发送setpoint
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <octomap_msgs/Octomap.h>

#include <cmath>
#include <vector>
#include <random>
#include <unordered_set>
#include <mutex>
#include <algorithm>
#include <clocale>

struct Node3D {
    double x, y, z;
    int parent;
    double cost;
    Node3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_), parent(-1), cost(0) {}
};

struct TripleHash {
    size_t operator()(const std::tuple<int,int,int> &t) const {
        return std::hash<int>()(std::get<0>(t)) ^ (std::hash<int>()(std::get<1>(t)) << 16)
               ^ (std::hash<int>()(std::get<2>(t)) << 8);
    }
};

class RRTStarPlanner
{
public:
    RRTStarPlanner() : nh_("~"), current_wp_idx_(0), gen_(std::random_device{}())
    {
        nh_.param("step_size", step_size_, 0.5);
        nh_.param("max_iter", max_iter_, 2000);
        nh_.param("goal_threshold", goal_threshold_, 0.5);
        nh_.param("search_radius", search_radius_, 1.5);
        nh_.param("safety_margin", safety_margin_, 0.35);
        nh_.param("replan_rate", replan_rate_, 2.0);
        nh_.param("resolution", resolution_, 0.15);

        local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &RRTStarPlanner::poseCb, this);
        path_sub_ = nh_.subscribe("/coverage_path", 1, &RRTStarPlanner::pathCb, this);
        octo_sub_ = nh_.subscribe("/occupied_cells_vis_array", 1, &RRTStarPlanner::occupiedCb, this);

        ROS_INFO("[RRT*] 路径规划器已启动, 等待覆盖路径...");
    }

    void run()
    {
        ros::Rate rate(replan_rate_);
        while (ros::ok()) {
            ros::spinOnce();
            planStep();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher local_path_pub_, setpoint_pub_;
    ros::Subscriber pose_sub_, path_sub_, octo_sub_;

    double step_size_, goal_threshold_, search_radius_, safety_margin_, replan_rate_, resolution_;
    int max_iter_;

    geometry_msgs::PoseStamped current_pose_;
    bool has_pose_ = false;
    nav_msgs::Path coverage_path_;
    bool has_path_ = false;
    int current_wp_idx_;

    std::unordered_set<std::tuple<int,int,int>, TripleHash> occupied_;
    std::mutex occ_mutex_;
    std::mt19937 gen_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose_ = *msg; has_pose_ = true;
    }
    void pathCb(const nav_msgs::Path::ConstPtr &msg) {
        coverage_path_ = *msg; has_path_ = true; current_wp_idx_ = 0;
        ROS_INFO("[RRT*] 收到覆盖路径: %zu 个路点", msg->poses.size());
    }
    void occupiedCb(const visualization_msgs::MarkerArray::ConstPtr &msg) {
        std::unordered_set<std::tuple<int,int,int>, TripleHash> nv;
        double res = resolution_;
        for (auto &m : msg->markers) {
            if (m.scale.x > 0) res = m.scale.x;
            for (auto &p : m.points)
                nv.emplace((int)round(p.x/res), (int)round(p.y/res), (int)round(p.z/res));
        }
        std::lock_guard<std::mutex> lock(occ_mutex_);
        occupied_ = std::move(nv);
    }

    bool isCollision(double x, double y, double z) {
        double res = resolution_;
        int gx = (int)round(x/res), gy = (int)round(y/res), gz = (int)round(z/res);
        int m = std::max(1, (int)(safety_margin_/res)) + 1;
        std::lock_guard<std::mutex> lock(occ_mutex_);
        for (int dx = -m; dx <= m; ++dx)
            for (int dy = -m; dy <= m; ++dy)
                for (int dz = -m; dz <= m; ++dz)
                    if (occupied_.count({gx+dx, gy+dy, gz+dz})) return true;
        return false;
    }

    bool isPathFree(const Node3D &a, const Node3D &b) {
        double dx = b.x-a.x, dy = b.y-a.y, dz = b.z-a.z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < 0.01) return true;
        int steps = std::max((int)(dist / 0.1), 2);
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            if (isCollision(a.x+t*dx, a.y+t*dy, a.z+t*dz)) return false;
        }
        return true;
    }

    double dist3(const Node3D &a, const Node3D &b) {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
    }

    std::vector<std::tuple<double,double,double>> planRRTStar(
        std::tuple<double,double,double> start, std::tuple<double,double,double> goal)
    {
        auto [sx,sy,sz] = start;
        auto [gx,gy,gz] = goal;
        Node3D startN(sx,sy,sz), goalN(gx,gy,gz);

        if (isPathFree(startN, goalN)) return {start, goal};

        std::vector<Node3D> tree = {startN};

        double bx[2] = {std::min(sx,gx)-3, std::max(sx,gx)+3};
        double by[2] = {std::min(sy,gy)-3, std::max(sy,gy)+3};
        double bz[2] = {std::max(0.3, std::min(sz,gz)-1), std::max(sz,gz)+1};

        std::vector<std::tuple<double,double,double>> best_path;
        std::uniform_real_distribution<> ux(bx[0],bx[1]), uy(by[0],by[1]), uz(bz[0],bz[1]);
        std::uniform_real_distribution<> u01(0,1);

        for (int iter = 0; iter < max_iter_; ++iter) {
            double rx, ry, rz;
            if (u01(gen_) < 0.1) { rx=gx; ry=gy; rz=gz; }
            else { rx=ux(gen_); ry=uy(gen_); rz=uz(gen_); }

            Node3D randN(rx,ry,rz);
            int nearest_idx = 0;
            double nearest_dist = 1e18;
            for (int i = 0; i < (int)tree.size(); ++i) {
                double d = dist3(tree[i], randN);
                if (d < nearest_dist) { nearest_dist = d; nearest_idx = i; }
            }

            Node3D newN = randN;
            if (nearest_dist > step_size_) {
                double r = step_size_ / nearest_dist;
                newN = Node3D(
                    tree[nearest_idx].x + r*(rx-tree[nearest_idx].x),
                    tree[nearest_idx].y + r*(ry-tree[nearest_idx].y),
                    tree[nearest_idx].z + r*(rz-tree[nearest_idx].z));
            }

            if (isCollision(newN.x, newN.y, newN.z)) continue;
            if (!isPathFree(tree[nearest_idx], newN)) continue;

            // Find best parent
            int best_par = nearest_idx;
            double best_cost = tree[nearest_idx].cost + dist3(tree[nearest_idx], newN);
            for (int i = 0; i < (int)tree.size(); ++i) {
                if (dist3(tree[i], newN) < search_radius_) {
                    double nc = tree[i].cost + dist3(tree[i], newN);
                    if (nc < best_cost && isPathFree(tree[i], newN)) {
                        best_par = i; best_cost = nc;
                    }
                }
            }
            newN.parent = best_par; newN.cost = best_cost;
            tree.push_back(newN);

            // Rewire
            int new_idx = tree.size() - 1;
            for (int i = 0; i < (int)tree.size()-1; ++i) {
                if (dist3(tree[i], newN) < search_radius_) {
                    double nc = newN.cost + dist3(newN, tree[i]);
                    if (nc < tree[i].cost && isPathFree(newN, tree[i])) {
                        tree[i].parent = new_idx; tree[i].cost = nc;
                    }
                }
            }

            // Check goal
            if (dist3(newN, goalN) < goal_threshold_ && isPathFree(newN, goalN)) {
                std::vector<std::tuple<double,double,double>> path;
                path.push_back(goal);
                int idx = new_idx;
                while (idx >= 0) {
                    path.emplace_back(tree[idx].x, tree[idx].y, tree[idx].z);
                    idx = tree[idx].parent;
                }
                std::reverse(path.begin(), path.end());
                if (best_path.empty() || path.size() < best_path.size())
                    best_path = path;
            }
        }
        return best_path.empty() ? std::vector<std::tuple<double,double,double>>{start, goal} : best_path;
    }

    void planStep()
    {
        if (!has_pose_ || !has_path_) return;
        if (current_wp_idx_ >= (int)coverage_path_.poses.size()) return;

        auto &cp = current_pose_.pose.position;
        auto &tp = coverage_path_.poses[current_wp_idx_].pose.position;
        double d = sqrt(pow(cp.x-tp.x,2)+pow(cp.y-tp.y,2)+pow(cp.z-tp.z,2));
        if (d < goal_threshold_) {
            current_wp_idx_++;
            if (current_wp_idx_ >= (int)coverage_path_.poses.size()) {
                ROS_INFO("[RRT*] 覆盖路径完成!");
                return;
            }
            ROS_INFO("[RRT*] 路点 %d/%zu", current_wp_idx_+1, coverage_path_.poses.size());
        }

        auto &t = coverage_path_.poses[current_wp_idx_].pose.position;
        auto start = std::make_tuple(cp.x, cp.y, cp.z);
        auto goal = std::make_tuple(t.x, t.y, t.z);

        auto path = planRRTStar(start, goal);

        // Publish local path
        nav_msgs::Path pm;
        pm.header.stamp = ros::Time::now(); pm.header.frame_id = "map";
        for (auto &[x,y,z] : path) {
            geometry_msgs::PoseStamped ps;
            ps.header = pm.header;
            ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
            ps.pose.orientation.w = 1.0;
            pm.poses.push_back(ps);
        }
        local_path_pub_.publish(pm);

        // Send setpoint
        geometry_msgs::PoseStamped sp;
        sp.header.stamp = ros::Time::now(); sp.header.frame_id = "map";
        if (path.size() > 1) {
            auto [x,y,z] = path[1];
            sp.pose.position.x = x; sp.pose.position.y = y; sp.pose.position.z = z;
        } else {
            sp.pose.position = t;
        }
        sp.pose.orientation.w = 1.0;
        setpoint_pub_.publish(sp);
    }
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "rrt_star_planner");
    RRTStarPlanner planner;
    planner.run();
    return 0;
}
