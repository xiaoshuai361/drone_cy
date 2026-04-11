/*
 * interactive_planner_node.cpp
 * 3D 交互式路径规划 + 自动飞行 (C++版)
 * RViz "Publish Point" → RRT* → 自动飞行
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

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
    Node3D() : x(0),y(0),z(0),parent(-1),cost(0) {}
    Node3D(double x_, double y_, double z_) : x(x_),y(y_),z(z_),parent(-1),cost(0) {}
};

struct TripleHash {
    size_t operator()(const std::tuple<int,int,int> &t) const {
        return std::hash<int>()(std::get<0>(t)) ^ (std::hash<int>()(std::get<1>(t)) << 16)
               ^ (std::hash<int>()(std::get<2>(t)) << 8);
    }
};

class InteractivePlanner
{
public:
    InteractivePlanner() : nh_("~"), is_flying_(false), current_wp_idx_(0),
                            map_ready_(false), resolution_(0.15), gen_(std::random_device{}())
    {
        nh_.param("step_size", step_size_, 0.8);
        nh_.param("max_iter", max_iter_, 3000);
        nh_.param("goal_threshold", goal_threshold_, 0.5);
        nh_.param("search_radius", search_radius_, 2.0);
        nh_.param("safety_margin", safety_margin_, 0.4);
        nh_.param("flight_speed", flight_speed_, 0.5);
        nh_.param("waypoint_threshold", waypoint_threshold_, 0.4);

        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rrt_markers", 1);
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        nh_.subscribe("/mavros/local_position/pose", 10, &InteractivePlanner::poseCb, this);
        nh_.subscribe("/mavros/state", 10, &InteractivePlanner::stateCb, this);
        nh_.subscribe("/clicked_point", 10, &InteractivePlanner::clickCb, this);
        nh_.subscribe("/occupied_cells_vis_array", 1, &InteractivePlanner::occupiedCb, this);

        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("=======================================================");
        ROS_INFO("[规划器C++] 3D交互式路径规划器已启动!");
        ROS_INFO("[规划器C++] RViz中使用 'Publish Point' 点击目标位置");
        ROS_INFO("=======================================================");
    }

    void run()
    {
        ros::Rate rate(20);
        while (ros::ok()) {
            flyPath();
            if (!is_flying_ && current_state_.mode == "OFFBOARD" && has_pose_) {
                geometry_msgs::PoseStamped ps;
                ps.header.stamp = ros::Time::now(); ps.header.frame_id = "map";
                ps.pose = current_pose_.pose;
                setpoint_pub_.publish(ps);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_, marker_pub_, setpoint_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    double step_size_, goal_threshold_, search_radius_, safety_margin_;
    double flight_speed_, waypoint_threshold_, resolution_;
    int max_iter_;

    geometry_msgs::PoseStamped current_pose_;
    mavros_msgs::State current_state_;
    bool has_pose_ = false, is_flying_, map_ready_;
    std::vector<std::tuple<double,double,double>> current_path_;
    int current_wp_idx_;

    std::unordered_set<std::tuple<int,int,int>, TripleHash> occupied_;
    std::mutex occ_mutex_;
    std::mt19937 gen_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose_ = *msg; has_pose_ = true;
    }
    void stateCb(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

    void occupiedCb(const visualization_msgs::MarkerArray::ConstPtr &msg) {
        std::unordered_set<std::tuple<int,int,int>, TripleHash> nv;
        double res = resolution_;
        for (auto &m : msg->markers) {
            double sx = m.scale.x > 0 ? m.scale.x : res;
            if (!map_ready_) { resolution_ = sx; res = sx; }
            for (auto &p : m.points)
                nv.emplace((int)round(p.x/res), (int)round(p.y/res), (int)round(p.z/res));
        }
        {
            std::lock_guard<std::mutex> lock(occ_mutex_);
            occupied_ = std::move(nv);
        }
        if (!map_ready_ && !occupied_.empty()) {
            map_ready_ = true;
            ROS_INFO("[规划器C++] OctoMap已加载! 体素: %zu", occupied_.size());
        }
    }

    void clickCb(const geometry_msgs::PointStamped::ConstPtr &msg) {
        double gx = msg->point.x, gy = msg->point.y, gz = msg->point.z;
        ROS_INFO("[规划器C++] 目标: (%.2f, %.2f, %.2f)", gx, gy, gz);

        if (gz < 0.5) {
            gz = has_pose_ ? std::max(1.5, current_pose_.pose.position.z) : 2.0;
            ROS_INFO("[规划器C++] 高度调整为: %.2f", gz);
        }
        if (!has_pose_) { ROS_WARN("[规划器C++] 位姿未知"); return; }

        auto &p = current_pose_.pose.position;
        auto start = std::make_tuple(p.x, p.y, p.z);
        auto goal = std::make_tuple(gx, gy, gz);

        publishGoalMarker(gx, gy, gz);

        ROS_INFO("[规划器C++] RRT* 规划中... (体素: %zu)", occupied_.size());
        auto path = planRRTStar(start, goal);

        if (path.size() > 1) {
            ROS_INFO("[规划器C++] ✓ 路径 %zu 个航点", path.size());
            publishPath(path);
            current_path_ = path;
            current_wp_idx_ = 0;
            is_flying_ = true;
            ensureOffboard();
        } else {
            ROS_WARN("[规划器C++] 规划失败!");
        }
    }

    void ensureOffboard() {
        if (current_state_.mode != "OFFBOARD") {
            if (has_pose_) {
                geometry_msgs::PoseStamped ps;
                ps.header.stamp = ros::Time::now(); ps.header.frame_id = "map";
                ps.pose = current_pose_.pose;
                for (int i = 0; i < 20; ++i) { setpoint_pub_.publish(ps); ros::Duration(0.05).sleep(); }
            }
            mavros_msgs::SetMode sm; sm.request.custom_mode = "OFFBOARD";
            if (mode_client_.call(sm)) ROS_INFO("[规划器C++] OFFBOARD");
        }
        if (!current_state_.armed) {
            mavros_msgs::CommandBool ab; ab.request.value = true;
            if (arm_client_.call(ab)) ROS_INFO("[规划器C++] 已解锁");
        }
    }

    // ==================== 碰撞检测 ====================
    bool isCollision(double x, double y, double z) {
        double res = resolution_;
        int gx = (int)round(x/res), gy = (int)round(y/res), gz = (int)round(z/res);
        int m = std::max(1, (int)(safety_margin_/res));
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
        int steps = std::max((int)(dist / (resolution_*0.5)), 2);
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            if (isCollision(a.x+t*dx, a.y+t*dy, a.z+t*dz)) return false;
        }
        return true;
    }

    double dist3(const Node3D &a, const Node3D &b) {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
    }

    // ==================== RRT* ====================
    std::vector<std::tuple<double,double,double>> planRRTStar(
        std::tuple<double,double,double> start, std::tuple<double,double,double> goal)
    {
        auto [sx,sy,sz] = start;
        auto [gx,gy,gz] = goal;
        Node3D startN(sx,sy,sz), goalN(gx,gy,gz);

        if (isPathFree(startN, goalN)) {
            ROS_INFO("[规划器C++] 直线可行");
            return {start, goal};
        }

        std::vector<Node3D> tree = {startN};
        double bx[2] = {std::min(sx,gx)-5, std::max(sx,gx)+5};
        double by[2] = {std::min(sy,gy)-5, std::max(sy,gy)+5};
        double bz[2] = {std::max(0.3, std::min(sz,gz)-2), std::max(sz,gz)+2};

        std::vector<std::tuple<double,double,double>> best_path;
        double best_cost = 1e18;
        std::uniform_real_distribution<> ux(bx[0],bx[1]), uy(by[0],by[1]), uz(bz[0],bz[1]);
        std::uniform_real_distribution<> u01(0,1);

        for (int iter = 0; iter < max_iter_; ++iter) {
            double rx, ry, rz;
            if (u01(gen_) < 0.15) { rx=gx; ry=gy; rz=gz; }
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

            int best_par = nearest_idx;
            double bc = tree[nearest_idx].cost + dist3(tree[nearest_idx], newN);
            for (int i = 0; i < (int)tree.size(); ++i) {
                if (dist3(tree[i], newN) < search_radius_) {
                    double nc = tree[i].cost + dist3(tree[i], newN);
                    if (nc < bc && isPathFree(tree[i], newN)) { best_par = i; bc = nc; }
                }
            }
            newN.parent = best_par; newN.cost = bc;
            tree.push_back(newN);

            int new_idx = tree.size() - 1;
            for (int i = 0; i < (int)tree.size()-1; ++i) {
                if (dist3(tree[i], newN) < search_radius_) {
                    double nc = newN.cost + dist3(newN, tree[i]);
                    if (nc < tree[i].cost && isPathFree(newN, tree[i])) {
                        tree[i].parent = new_idx; tree[i].cost = nc;
                    }
                }
            }

            if (dist3(newN, goalN) < goal_threshold_ && isPathFree(newN, goalN)) {
                double gc = newN.cost + dist3(newN, goalN);
                if (gc < best_cost) {
                    best_cost = gc;
                    best_path.clear();
                    best_path.push_back(goal);
                    int idx = new_idx;
                    while (idx >= 0) {
                        best_path.emplace_back(tree[idx].x, tree[idx].y, tree[idx].z);
                        idx = tree[idx].parent;
                    }
                    std::reverse(best_path.begin(), best_path.end());
                }
            }

            if ((iter+1) % 500 == 0)
                ROS_INFO("[规划器C++] RRT* iter %d/%d, 节点: %zu", iter+1, max_iter_, tree.size());
        }

        if (best_path.empty()) {
            ROS_WARN("[规划器C++] RRT* 无路径, 退化直线");
            return {start, goal};
        }
        return smoothPath(best_path);
    }

    std::vector<std::tuple<double,double,double>> smoothPath(
        const std::vector<std::tuple<double,double,double>> &path)
    {
        if (path.size() <= 2) return path;
        std::vector<std::tuple<double,double,double>> s = {path[0]};
        size_t i = 0;
        while (i < path.size() - 1) {
            size_t j = path.size() - 1;
            while (j > i + 1) {
                auto [x0,y0,z0] = s.back();
                auto [x1,y1,z1] = path[j];
                Node3D n1(x0,y0,z0), n2(x1,y1,z1);
                if (isPathFree(n1, n2)) break;
                --j;
            }
            s.push_back(path[j]);
            i = j;
        }
        return s;
    }

    // ==================== 可视化 ====================
    void publishPath(const std::vector<std::tuple<double,double,double>> &path) {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now(); msg.header.frame_id = "camera_init";
        for (auto &[x,y,z] : path) {
            geometry_msgs::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_.publish(msg);
    }

    void publishGoalMarker(double gx, double gy, double gz) {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now(); m.header.frame_id = "camera_init";
        m.ns = "goal"; m.id = 0;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = gx; m.pose.position.y = gy; m.pose.position.z = gz;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.5;
        m.color.r = 1; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.8;
        ma.markers.push_back(m);

        visualization_msgs::Marker t;
        t.header = m.header; t.ns = "goal_text"; t.id = 1;
        t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::Marker::ADD;
        t.pose.position.x = gx; t.pose.position.y = gy; t.pose.position.z = gz + 0.8;
        t.pose.orientation.w = 1.0; t.scale.z = 0.3;
        t.color.r = 1; t.color.g = 1; t.color.a = 1;
        char buf[64]; snprintf(buf, 64, "Target (%.1f,%.1f,%.1f)", gx, gy, gz);
        t.text = buf;
        ma.markers.push_back(t);
        marker_pub_.publish(ma);
    }

    // ==================== 飞行 ====================
    void flyPath() {
        if (!is_flying_ || current_path_.empty() || !has_pose_) return;
        if (current_wp_idx_ >= (int)current_path_.size()) {
            ROS_INFO("[规划器C++] ✓ 到达目标!");
            is_flying_ = false;
            current_path_.clear();
            return;
        }
        auto [tx,ty,tz] = current_path_[current_wp_idx_];
        auto &p = current_pose_.pose.position;
        double dist = sqrt(pow(p.x-tx,2)+pow(p.y-ty,2)+pow(p.z-tz,2));

        if (dist < waypoint_threshold_) {
            current_wp_idx_++;
            if (current_wp_idx_ < (int)current_path_.size())
                ROS_INFO("[规划器C++] 航点 %d/%zu", current_wp_idx_, current_path_.size());
            return;
        }

        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now(); ps.header.frame_id = "map";
        ps.pose.position.x = tx; ps.pose.position.y = ty; ps.pose.position.z = tz;
        double yaw = atan2(ty-p.y, tx-p.x);
        ps.pose.orientation.z = sin(yaw/2.0);
        ps.pose.orientation.w = cos(yaw/2.0);
        setpoint_pub_.publish(ps);
    }
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "interactive_planner");
    InteractivePlanner planner;
    planner.run();
    return 0;
}
