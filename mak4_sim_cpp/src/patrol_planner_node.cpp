/*
 * patrol_planner_node.cpp
 * OctoMap 2D分层航线巡检规划器 (C++高性能版)
 *
 * 核心: OctoMap → 2D障碍切片 → A*路径规划 → 多层自动巡检
 * 终端命令: alt H [H2 H3] | start | clear | stop | status | home | undo | speed S
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <cmath>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>
#include <thread>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <clocale>

struct PairHash {
    size_t operator()(const std::pair<int,int> &p) const {
        return std::hash<long long>()(((long long)p.first << 32) | (unsigned int)p.second);
    }
};
struct TripleHash {
    size_t operator()(const std::tuple<int,int,int> &t) const {
        auto h1 = std::hash<int>()(std::get<0>(t));
        auto h2 = std::hash<int>()(std::get<1>(t));
        auto h3 = std::hash<int>()(std::get<2>(t));
        return h1 ^ (h2 << 16) ^ (h3 << 8);
    }
};

class PatrolPlanner
{
public:
    PatrolPlanner() : nh_("~"), mode_("SETTING"), current_layer_idx_(0),
                      current_seg_idx_(0), current_pt_idx_(0),
                      map_ready_(false), drone_viz_counter_(0),
                      resolution_(0.15)
    {
        nh_.param("safety_margin", safety_margin_, 0.4);
        nh_.param("waypoint_threshold", waypoint_threshold_, 0.5);
        nh_.param("default_altitude", default_altitude_, 1.5);
        nh_.param("flight_speed", flight_speed_, 6.9);

        patrol_altitudes_.push_back(default_altitude_);
        takeoff_target_alt_ = default_altitude_;

        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/waypoints", 1, true);
        route_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/planned_route", 1);
        active_path_pub_ = nh_.advertise<nav_msgs::Path>("/patrol/active_path", 1);
        alt_plane_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/altitude_plane", 1, true);
        drone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/drone_position", 1);

        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &PatrolPlanner::poseCb, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10, &PatrolPlanner::stateCb, this);
        click_sub_ = nh_.subscribe("/clicked_point", 10, &PatrolPlanner::clickCb, this);
        octomap_sub_ = nh_.subscribe("/occupied_cells_vis_array", 1, &PatrolPlanner::occupiedCb, this);

        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        input_thread_ = std::thread(&PatrolPlanner::inputLoop, this);
        input_thread_.detach();

        ROS_INFO("============================================================");
        ROS_INFO("[巡检C++] 2D分层航线巡检规划器已启动!");
        ROS_INFO("[巡检C++] 当前巡检高度: %.1f m", patrol_altitudes_[0]);
        ROS_INFO("[巡检C++] 操作: alt H | start | clear | stop | status | home | undo");
        ROS_INFO("============================================================");

        publishAltitudePlane();
    }

    void run()
    {
        ros::Rate rate(20);
        while (ros::ok()) {
            flyStep();
            if (++drone_viz_counter_ >= 4) {
                drone_viz_counter_ = 0;
                publishDroneMarker();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_, waypoint_marker_pub_, route_pub_;
    ros::Publisher active_path_pub_, alt_plane_pub_, drone_marker_pub_;
    ros::Subscriber pose_sub_, state_sub_, click_sub_, octomap_sub_;
    ros::ServiceClient arm_client_, mode_client_;

    // OctoMap
    std::unordered_set<std::tuple<int,int,int>, TripleHash> occupied_voxels_;
    std::mutex voxel_mutex_;
    double resolution_;
    bool map_ready_;

    // 状态
    geometry_msgs::PoseStamped current_pose_;
    mavros_msgs::State current_state_;
    bool has_pose_ = false;
    double safety_margin_, waypoint_threshold_, default_altitude_, flight_speed_;

    // 航线
    std::vector<double> patrol_altitudes_;
    std::vector<std::pair<double,double>> patrol_waypoints_2d_;
    std::vector<std::vector<std::tuple<double,double,double>>> planned_segments_;
    std::string mode_;
    int current_layer_idx_, current_seg_idx_, current_pt_idx_;
    std::pair<double,double> home_xy_;
    bool has_home_ = false;
    double takeoff_target_alt_;
    ros::Time pre_arm_start_;
    int drone_viz_counter_;
    std::thread input_thread_;
    double replan_check_interval_ = 1.0;
    ros::Time last_replan_check_;

    // ==================== 回调 ====================
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose_ = *msg;
        has_pose_ = true;
        if (!has_home_) {
            home_xy_ = {msg->pose.position.x, msg->pose.position.y};
            has_home_ = true;
        }
    }
    void stateCb(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

    void occupiedCb(const visualization_msgs::MarkerArray::ConstPtr &msg) {
        std::unordered_set<std::tuple<int,int,int>, TripleHash> new_voxels;
        double res = resolution_;
        for (auto &marker : msg->markers) {
            double sx = marker.scale.x > 0 ? marker.scale.x : res;
            if (!map_ready_) { resolution_ = sx; res = sx; }
            for (auto &pt : marker.points) {
                int gx = (int)round(pt.x / res);
                int gy = (int)round(pt.y / res);
                int gz = (int)round(pt.z / res);
                new_voxels.emplace(gx, gy, gz);
            }
        }
        {
            std::lock_guard<std::mutex> lock(voxel_mutex_);
            occupied_voxels_ = std::move(new_voxels);
        }
        if (!map_ready_ && !occupied_voxels_.empty()) {
            map_ready_ = true;
            ROS_INFO("[巡检C++] OctoMap已加载! 占据体素: %zu", occupied_voxels_.size());
        }
    }

    void clickCb(const geometry_msgs::PointStamped::ConstPtr &msg) {
        if (mode_ != "SETTING") {
            ROS_WARN("[巡检C++] 不在设定模式, 输入 'clear' 重置");
            return;
        }
        double x = msg->point.x, y = msg->point.y;
        patrol_waypoints_2d_.emplace_back(x, y);
        std::string alt_str;
        for (size_t i = 0; i < patrol_altitudes_.size(); ++i) {
            if (i > 0) alt_str += "/";
            char buf[16]; snprintf(buf, 16, "%.1f", patrol_altitudes_[i]);
            alt_str += buf;
        }
        ROS_INFO("[巡检C++] 航点 #%zu: (%.1f, %.1f) 高度=%s m",
                 patrol_waypoints_2d_.size(), x, y, alt_str.c_str());
        publishWaypointMarkers();
    }

    // ==================== 2D障碍提取 ====================
    std::unordered_set<std::pair<int,int>, PairHash> extract2dObstacles(double altitude) {
        double res = resolution_;
        int gz_center = (int)round(altitude / res);
        int margin_z = std::max(2, (int)(1.0 / res));
        int margin_xy = std::max(1, (int)(safety_margin_ / res));

        std::unordered_set<std::pair<int,int>, PairHash> obstacles;
        std::lock_guard<std::mutex> lock(voxel_mutex_);
        for (auto &[vx, vy, vz] : occupied_voxels_) {
            if (abs(vz - gz_center) <= margin_z) {
                for (int dx = -margin_xy; dx <= margin_xy; ++dx)
                    for (int dy = -margin_xy; dy <= margin_xy; ++dy)
                        obstacles.emplace(vx + dx, vy + dy);
            }
        }
        return obstacles;
    }

    // ==================== A* ====================
    bool isLineFree2d(int x0, int y0, int x1, int y1,
                      const std::unordered_set<std::pair<int,int>, PairHash> &obs) {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int steps = std::max(dx, dy);
        if (steps == 0) return true;
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            int x = (int)round(x0 + t * (x1 - x0));
            int y = (int)round(y0 + t * (y1 - y0));
            if (obs.count({x, y})) return false;
        }
        return true;
    }

    std::pair<int,int> findNearestFree(int cx, int cy,
                                        const std::unordered_set<std::pair<int,int>, PairHash> &obs,
                                        int max_r = 30) {
        for (int r = 1; r <= max_r; ++r)
            for (int dx = -r; dx <= r; ++dx)
                for (int dy = -r; dy <= r; ++dy)
                    if ((abs(dx) == r || abs(dy) == r) && !obs.count({cx+dx, cy+dy}))
                        return {cx+dx, cy+dy};
        return {cx, cy};
    }

    std::vector<std::pair<double,double>> planAstar(
        std::pair<double,double> start_xy, std::pair<double,double> goal_xy,
        const std::unordered_set<std::pair<int,int>, PairHash> &obstacles)
    {
        double res = resolution_;
        int sx = (int)round(start_xy.first / res), sy = (int)round(start_xy.second / res);
        int gx = (int)round(goal_xy.first / res), gy = (int)round(goal_xy.second / res);

        if (obstacles.count({sx, sy})) {
            auto [nx, ny] = findNearestFree(sx, sy, obstacles);
            sx = nx; sy = ny;
        }
        if (obstacles.count({gx, gy})) {
            auto [nx, ny] = findNearestFree(gx, gy, obstacles);
            gx = nx; gy = ny;
        }

        // 直线可通
        if (isLineFree2d(sx, sy, gx, gy, obstacles))
            return {start_xy, goal_xy};

        static const int DX[] = {-1,1,0,0,-1,-1,1,1};
        static const int DY[] = {0,0,-1,1,-1,1,-1,1};
        static const double COST[] = {1,1,1,1,1.414,1.414,1.414,1.414};

        using PQE = std::tuple<double,int,int,int>; // f, counter, x, y
        std::priority_queue<PQE, std::vector<PQE>, std::greater<PQE>> open;
        std::unordered_map<long long, double> g_score;
        std::unordered_map<long long, long long> came_from;
        std::unordered_set<long long> closed;

        auto key = [](int x, int y) -> long long { return ((long long)x << 32) | (unsigned int)y; };

        int counter = 0;
        double h0 = hypot(gx - sx, gy - sy);
        open.push({h0, counter++, sx, sy});
        g_score[key(sx,sy)] = 0;

        int max_nodes = 80000;

        while (!open.empty() && (int)closed.size() < max_nodes) {
            auto [f, cnt, cx, cy] = open.top(); open.pop();
            long long ck = key(cx, cy);
            if (closed.count(ck)) continue;
            closed.insert(ck);

            if (cx == gx && cy == gy) {
                // 回溯
                std::vector<std::pair<double,double>> path_world;
                long long node = ck;
                while (came_from.count(node)) {
                    int nx = (int)(node >> 32), ny = (int)(node & 0xFFFFFFFF);
                    path_world.push_back({nx * res, ny * res});
                    node = came_from[node];
                }
                path_world.push_back({sx * res, sy * res});
                std::reverse(path_world.begin(), path_world.end());
                return simplifyPath(path_world, obstacles);
            }

            for (int d = 0; d < 8; ++d) {
                int nx = cx + DX[d], ny = cy + DY[d];
                long long nk = key(nx, ny);
                if (closed.count(nk) || obstacles.count({nx, ny})) continue;
                double ng = g_score[ck] + COST[d];
                auto it = g_score.find(nk);
                if (it == g_score.end() || ng < it->second) {
                    g_score[nk] = ng;
                    double h = hypot(gx - nx, gy - ny);
                    open.push({ng + h, counter++, nx, ny});
                    came_from[nk] = ck;
                }
            }
        }
        return {}; // 无路径
    }

    std::vector<std::pair<double,double>> simplifyPath(
        const std::vector<std::pair<double,double>> &path,
        const std::unordered_set<std::pair<int,int>, PairHash> &obs)
    {
        if (path.size() <= 2) return path;
        double res = resolution_;
        std::vector<std::pair<double,double>> simplified = {path[0]};
        size_t i = 0;
        while (i < path.size() - 1) {
            size_t j = path.size() - 1;
            while (j > i + 1) {
                int x0 = (int)round(simplified.back().first / res);
                int y0 = (int)round(simplified.back().second / res);
                int x1 = (int)round(path[j].first / res);
                int y1 = (int)round(path[j].second / res);
                if (isLineFree2d(x0, y0, x1, y1, obs)) break;
                --j;
            }
            simplified.push_back(path[j]);
            i = j;
        }
        return simplified;
    }

    bool planLayer(int layer_idx) {
        double alt = patrol_altitudes_[layer_idx];
        ROS_INFO("[巡检C++] === 规划第 %d 层 (高度=%.1fm) ===", layer_idx + 1, alt);

        auto obstacles = extract2dObstacles(alt);
        ROS_INFO("[巡检C++] 高度 %.1fm 的2D障碍物: %zu 个格子", alt, obstacles.size());

        double cx = has_pose_ ? current_pose_.pose.position.x : 0;
        double cy = has_pose_ ? current_pose_.pose.position.y : 0;

        std::vector<std::pair<double,double>> all_xy = {{cx, cy}};
        for (auto &wp : patrol_waypoints_2d_) all_xy.push_back(wp);

        planned_segments_.clear();
        for (size_t i = 0; i + 1 < all_xy.size(); ++i) {
            auto a = all_xy[i], b = all_xy[i+1];
            ROS_INFO("[巡检C++]   航段 %zu/%zu: (%.1f,%.1f) → (%.1f,%.1f)",
                     i+1, all_xy.size()-1, a.first, a.second, b.first, b.second);
            auto path_2d = planAstar(a, b, obstacles);
            if (path_2d.empty()) {
                ROS_WARN("[巡检C++]   A*未找到路径, 使用直线");
                path_2d = {a, b};
            }
            std::vector<std::tuple<double,double,double>> seg;
            for (auto &p : path_2d) seg.emplace_back(p.first, p.second, alt);
            planned_segments_.push_back(seg);
            ROS_INFO("[巡检C++]   航段 %zu → %zu 个路径点", i+1, seg.size());
        }
        current_seg_idx_ = 0;
        current_pt_idx_ = 0;
        ROS_INFO("[巡检C++] 第 %d 层规划完成! %zu 个航段", layer_idx+1, planned_segments_.size());
        publishRouteMarkers();
        return true;
    }

    double getCurrentYaw() {
        auto &q = current_pose_.pose.orientation;
        return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    }

    // ==================== 飞行控制 ====================
    void sendSetpoint(double x, double y, double z, double yaw = NAN) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = "map";
        ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
        if (!std::isnan(yaw)) {
            ps.pose.orientation.z = sin(yaw / 2.0);
            ps.pose.orientation.w = cos(yaw / 2.0);
        } else {
            ps.pose.orientation.w = 1.0;
        }
        setpoint_pub_.publish(ps);
    }

    void flyStep() {
        if (!has_pose_) return;
        double cx = current_pose_.pose.position.x;
        double cy = current_pose_.pose.position.y;
        double cz = current_pose_.pose.position.z;

        if (mode_ == "PRE_ARM") {
            double cur_yaw = getCurrentYaw();
            sendSetpoint(cx, cy, takeoff_target_alt_, cur_yaw);
            if ((ros::Time::now() - pre_arm_start_).toSec() >= 2.5) {
                mavros_msgs::SetMode sm; sm.request.custom_mode = "OFFBOARD";
                if (mode_client_.call(sm)) ROS_INFO("[巡检C++] ✓ OFFBOARD");
                ros::Duration(0.3).sleep();
                mavros_msgs::CommandBool ab; ab.request.value = true;
                if (arm_client_.call(ab)) ROS_INFO("[巡检C++] ✓ 已解锁");
                mode_ = "TAKING_OFF";
                ROS_INFO("[巡检C++] 起飞中... 目标 %.1fm", takeoff_target_alt_);
            }
        } else if (mode_ == "TAKING_OFF") {
            double cur_yaw = getCurrentYaw();
            sendSetpoint(cx, cy, takeoff_target_alt_, cur_yaw);
            if (fabs(cz - takeoff_target_alt_) < 0.3) {
                ROS_INFO("[巡检C++] ✓ 已到达 %.1fm!", takeoff_target_alt_);
                mode_ = "EXECUTING";
                ROS_INFO("[巡检C++] ====== 开始巡检! (第%d/%zu层, 高度%.1fm) ======",
                         current_layer_idx_ + 1, patrol_altitudes_.size(), takeoff_target_alt_);
            }
        } else if (mode_ == "EXECUTING") {
            executeStep(cx, cy, cz);
        } else if (mode_ == "LAYER_RETURN") {
            double hx = has_home_ ? home_xy_.first : 0;
            double hy = has_home_ ? home_xy_.second : 0;
            double alt = patrol_altitudes_[std::min(current_layer_idx_, (int)patrol_altitudes_.size()-1)];
            double dist = hypot(cx - hx, cy - hy);
            double yaw = dist > 0.5 ? atan2(hy - cy, hx - cx) : 0;
            sendSetpoint(hx, hy, alt, yaw);
            if (dist < 0.8) {
                current_layer_idx_++;
                if (current_layer_idx_ < (int)patrol_altitudes_.size()) {
                    double next_alt = patrol_altitudes_[current_layer_idx_];
                    ROS_INFO("[巡检C++] ★ 第 %d 层完成! → 第 %d 层 (%.1fm)",
                             current_layer_idx_, current_layer_idx_ + 1, next_alt);
                    takeoff_target_alt_ = next_alt;
                    planLayer(current_layer_idx_);
                    mode_ = "TAKING_OFF";
                } else {
                    ROS_INFO("[巡检C++] ====== 所有 %zu 层巡检完成! ======", patrol_altitudes_.size());
                    mode_ = "DONE";
                }
            }
        } else if (mode_ == "DONE") {
            sendSetpoint(cx, cy, cz);
        }
    }

    void executeStep(double cx, double cy, double cz) {
        if (current_seg_idx_ >= (int)planned_segments_.size()) {
            if ((int)patrol_altitudes_.size() > 1 && current_layer_idx_ < (int)patrol_altitudes_.size() - 1) {
                ROS_INFO("[巡检C++] 第 %d 层完成! 返回起点...", current_layer_idx_ + 1);
                mode_ = "LAYER_RETURN";
            } else {
                ROS_INFO("[巡检C++] ====== 巡检完成! ======");
                mode_ = "DONE";
            }
            return;
        }

        // 动态避障: 定期检查路径碰撞
        ros::Time now = ros::Time::now();
        if ((now - last_replan_check_).toSec() >= replan_check_interval_) {
            last_replan_check_ = now;
            double alt = patrol_altitudes_[std::min(current_layer_idx_, (int)patrol_altitudes_.size()-1)];
            auto obstacles = extract2dObstacles(alt);
            auto &seg = planned_segments_[current_seg_idx_];
            bool collision = false;
            double res = resolution_;
            for (int i = current_pt_idx_; i < (int)seg.size(); ++i) {
                auto [wx, wy, wz] = seg[i];
                int gx = (int)round(wx / res), gy = (int)round(wy / res);
                if (obstacles.count({gx, gy})) { collision = true; break; }
            }
            if (collision) {
                ROS_WARN("[巡检C++] 检测到动态障碍! 重规划航段 %d...", current_seg_idx_ + 1);
                auto &end_pt = seg.back();
                auto [tx, ty, tz] = end_pt;
                auto path_2d = planAstar({cx, cy}, {tx, ty}, obstacles);
                if (path_2d.empty()) {
                    ROS_WARN("[巡检C++] 重规划失败, 使用直线");
                    path_2d = {{cx, cy}, {tx, ty}};
                }
                std::vector<std::tuple<double,double,double>> new_seg;
                for (auto &p : path_2d) new_seg.emplace_back(p.first, p.second, alt);
                planned_segments_[current_seg_idx_] = new_seg;
                current_pt_idx_ = 0;
                ROS_INFO("[巡检C++] 重规划完成! 新路径 %zu 点", new_seg.size());
                publishRouteMarkers();
            }
        }

        auto &seg = planned_segments_[current_seg_idx_];
        if (current_pt_idx_ >= (int)seg.size()) {
            current_seg_idx_++; current_pt_idx_ = 0;
            if (current_seg_idx_ < (int)planned_segments_.size()) {
                ROS_INFO("[巡检C++] 航段 %d/%zu 完成", current_seg_idx_, planned_segments_.size());
                publishRouteMarkers();
            }
            return;
        }
        auto [tx, ty, tz] = seg[current_pt_idx_];
        double dx = tx-cx, dy = ty-cy, dz = tz-cz;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < waypoint_threshold_) {
            current_pt_idx_++;
            if (current_pt_idx_ >= (int)seg.size() && current_seg_idx_ < (int)patrol_waypoints_2d_.size()) {
                auto &wp = patrol_waypoints_2d_[current_seg_idx_];
                ROS_INFO("[巡检C++] ★ 到达航点 #%d: (%.1f, %.1f)", current_seg_idx_+1, wp.first, wp.second);
            }
            return;
        }
        double yaw = atan2(dy, dx);
        double max_step = flight_speed_ * 0.05;
        if (dist > max_step) {
            double r = max_step / dist;
            sendSetpoint(cx+dx*r, cy+dy*r, cz+dz*r, yaw);
        } else {
            sendSetpoint(tx, ty, tz, yaw);
        }
        publishActivePath();
    }

    // ==================== 可视化 ====================
    void publishAltitudePlane() {
        visualization_msgs::MarkerArray ma;

        for (size_t li = 0; li < patrol_altitudes_.size(); ++li) {
            double alt = patrol_altitudes_[li];
            // 半透明平面
            visualization_msgs::Marker m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = "camera_init";
            m.ns = "alt_plane"; m.id = li;
            m.type = visualization_msgs::Marker::CUBE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.z = alt; m.pose.orientation.w = 1.0;
            m.scale.x = 60; m.scale.y = 30; m.scale.z = 0.02;
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 0.25;
            ma.markers.push_back(m);

            // 高度文字 (中心+4角)
            double positions[][2] = {{0,0},{-20,-8},{20,-8},{-20,8},{20,8}};
            for (int i = 0; i < 5; ++i) {
                visualization_msgs::Marker t;
                t.header = m.header;
                char ns[32]; snprintf(ns, 32, "alt_text_%zu", li);
                t.ns = ns; t.id = i;
                t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                t.action = visualization_msgs::Marker::ADD;
                t.pose.position.x = positions[i][0];
                t.pose.position.y = positions[i][1];
                t.pose.position.z = alt + 0.15;
                t.pose.orientation.w = 1.0;
                t.scale.z = i == 0 ? 0.5 : 0.35;
                t.color.g = 0.8; t.color.b = 1.0; t.color.a = 1.0;
                char buf[64];
                if (i == 0) snprintf(buf, 64, "=== Layer%zu: %.1fm ===", li+1, alt);
                else snprintf(buf, 64, "h=%.1fm", alt);
                t.text = buf;
                ma.markers.push_back(t);
            }

            // 边框
            visualization_msgs::Marker b;
            b.header = m.header;
            char bns[32]; snprintf(bns, 32, "alt_border_%zu", li);
            b.ns = bns; b.id = 0;
            b.type = visualization_msgs::Marker::LINE_STRIP;
            b.action = visualization_msgs::Marker::ADD;
            b.scale.x = 0.05; b.pose.orientation.w = 1.0;
            b.color.g = 0.7; b.color.b = 1.0; b.color.a = 0.6;
            double corners[][2] = {{-25,-12},{25,-12},{25,12},{-25,12},{-25,-12}};
            for (auto &c : corners) {
                geometry_msgs::Point p; p.x = c[0]; p.y = c[1]; p.z = alt;
                b.points.push_back(p);
            }
            ma.markers.push_back(b);
        }
        alt_plane_pub_.publish(ma);
    }

    void publishWaypointMarkers() {
        visualization_msgs::MarkerArray ma;
        // DELETEALL
        visualization_msgs::Marker del; del.action = visualization_msgs::Marker::DELETEALL;
        del.ns = "patrol_wp"; ma.markers.push_back(del);
        visualization_msgs::Marker del2; del2.action = visualization_msgs::Marker::DELETEALL;
        del2.ns = "patrol_line"; ma.markers.push_back(del2);

        double alt = patrol_altitudes_[0];
        for (size_t i = 0; i < patrol_waypoints_2d_.size(); ++i) {
            double wx = patrol_waypoints_2d_[i].first, wy = patrol_waypoints_2d_[i].second;
            // 圆柱
            visualization_msgs::Marker m;
            m.header.stamp = ros::Time::now(); m.header.frame_id = "camera_init";
            m.ns = "patrol_wp"; m.id = i * 3;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = wx; m.pose.position.y = wy; m.pose.position.z = alt;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 0.5; m.scale.z = 0.08;
            m.color.r = 1; m.color.g = 0.5; m.color.a = 0.9;
            ma.markers.push_back(m);

            // 编号
            visualization_msgs::Marker t;
            t.header = m.header; t.ns = "patrol_wp"; t.id = i * 3 + 1;
            t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::Marker::ADD;
            t.pose.position.x = wx; t.pose.position.y = wy; t.pose.position.z = alt + 0.5;
            t.pose.orientation.w = 1.0; t.scale.z = 0.35;
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
            char buf[64]; snprintf(buf, 64, "#%zu (%.1f,%.1f)", i+1, wx, wy);
            t.text = buf;
            ma.markers.push_back(t);

            // 垂直参考线
            visualization_msgs::Marker mc;
            mc.header = m.header; mc.ns = "patrol_wp"; mc.id = i * 3 + 2;
            mc.type = visualization_msgs::Marker::LINE_LIST;
            mc.action = visualization_msgs::Marker::ADD;
            mc.scale.x = 0.03; mc.pose.orientation.w = 1.0;
            mc.color.r = 1; mc.color.g = 0.5; mc.color.a = 0.3;
            geometry_msgs::Point p0, p1;
            p0.x = wx; p0.y = wy; p0.z = 0;
            p1.x = wx; p1.y = wy; p1.z = alt;
            mc.points.push_back(p0); mc.points.push_back(p1);
            ma.markers.push_back(mc);
        }

        // 航点连线
        if (patrol_waypoints_2d_.size() > 1) {
            visualization_msgs::Marker line;
            line.header.stamp = ros::Time::now(); line.header.frame_id = "camera_init";
            line.ns = "patrol_line"; line.id = 0;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = 0.08; line.pose.orientation.w = 1.0;
            line.color.r = 1; line.color.g = 0.8; line.color.a = 0.6;
            for (auto &[wx,wy] : patrol_waypoints_2d_) {
                geometry_msgs::Point p; p.x = wx; p.y = wy; p.z = alt;
                line.points.push_back(p);
            }
            ma.markers.push_back(line);
        }
        waypoint_marker_pub_.publish(ma);
    }

    void publishRouteMarkers() {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker del; del.action = visualization_msgs::Marker::DELETEALL;
        del.ns = "patrol_route"; ma.markers.push_back(del);

        for (size_t si = 0; si < planned_segments_.size(); ++si) {
            if ((int)si < current_seg_idx_) continue;
            visualization_msgs::Marker line;
            line.header.stamp = ros::Time::now(); line.header.frame_id = "camera_init";
            line.ns = "patrol_route"; line.id = si;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            if ((int)si == current_seg_idx_) {
                line.color.g = 1.0; line.color.a = 0.9; line.scale.x = 0.1;
            } else {
                line.color.r = 1; line.color.g = 0.8; line.color.a = 0.6; line.scale.x = 0.06;
            }
            for (auto &[px,py,pz] : planned_segments_[si]) {
                geometry_msgs::Point p; p.x = px; p.y = py; p.z = pz;
                line.points.push_back(p);
            }
            ma.markers.push_back(line);
        }
        route_pub_.publish(ma);
    }

    void publishActivePath() {
        if (current_seg_idx_ >= (int)planned_segments_.size()) return;
        auto &seg = planned_segments_[current_seg_idx_];
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now(); path.header.frame_id = "camera_init";
        for (int i = current_pt_idx_; i < (int)seg.size(); ++i) {
            auto [px,py,pz] = seg[i];
            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = px; ps.pose.position.y = py; ps.pose.position.z = pz;
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(ps);
        }
        active_path_pub_.publish(path);
    }

    void publishDroneMarker() {
        if (!has_pose_) return;
        auto &p = current_pose_.pose.position;
        auto &q = current_pose_.pose.orientation;
        visualization_msgs::MarkerArray ma;

        // 球体
        visualization_msgs::Marker s;
        s.header.stamp = ros::Time::now(); s.header.frame_id = "map";
        s.ns = "drone_pos"; s.id = 0;
        s.type = visualization_msgs::Marker::SPHERE;
        s.action = visualization_msgs::Marker::ADD;
        s.pose.position = p; s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 0.5;
        s.color.r = 1; s.color.a = 0.9;
        s.lifetime = ros::Duration(0.5);
        ma.markers.push_back(s);

        // 箭头
        visualization_msgs::Marker a;
        a.header = s.header; a.ns = "drone_pos"; a.id = 1;
        a.type = visualization_msgs::Marker::ARROW;
        a.action = visualization_msgs::Marker::ADD;
        a.pose.position = p; a.pose.orientation = q;
        a.scale.x = 1.0; a.scale.y = a.scale.z = 0.15;
        a.color.r = 1; a.color.g = 0.2; a.color.b = 0.2; a.color.a = 0.9;
        a.lifetime = ros::Duration(0.5);
        ma.markers.push_back(a);

        // 高度文字
        visualization_msgs::Marker t;
        t.header = s.header; t.ns = "drone_pos"; t.id = 2;
        t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::Marker::ADD;
        t.pose.position.x = p.x; t.pose.position.y = p.y; t.pose.position.z = p.z + 0.6;
        t.pose.orientation.w = 1.0; t.scale.z = 0.3;
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
        char buf[32]; snprintf(buf, 32, "UAV h=%.1fm", p.z);
        t.text = buf;
        t.lifetime = ros::Duration(0.5);
        ma.markers.push_back(t);

        // 垂直线
        visualization_msgs::Marker v;
        v.header = s.header; v.ns = "drone_pos"; v.id = 3;
        v.type = visualization_msgs::Marker::LINE_LIST;
        v.action = visualization_msgs::Marker::ADD;
        v.scale.x = 0.03; v.pose.orientation.w = 1.0;
        v.color.r = 1; v.color.g = 0.3; v.color.b = 0.3; v.color.a = 0.4;
        geometry_msgs::Point p0, p1;
        p0.x = p.x; p0.y = p.y; p0.z = 0;
        p1.x = p.x; p1.y = p.y; p1.z = p.z;
        v.points.push_back(p0); v.points.push_back(p1);
        v.lifetime = ros::Duration(0.5);
        ma.markers.push_back(v);

        drone_marker_pub_.publish(ma);
    }

    // ==================== 终端输入 ====================
    void inputLoop() {
        std::string line;
        while (ros::ok()) {
            if (!std::getline(std::cin, line)) { ros::Duration(0.5).sleep(); continue; }
            if (line.empty()) continue;
            std::istringstream iss(line);
            std::string cmd; iss >> cmd;
            for (auto &c : cmd) c = tolower(c);

            if (cmd == "alt") {
                std::vector<double> alts;
                double v;
                while (iss >> v) alts.push_back(v);
                if (alts.empty()) {
                    ROS_INFO("[巡检C++] 用法: alt H 或 alt H1 H2 H3");
                    continue;
                }
                bool valid = true;
                for (auto a : alts) if (a < 0.3 || a > 15.0) { ROS_WARN("[巡检C++] 高度 %.1f 超范围!", a); valid = false; }
                if (!valid) continue;
                std::sort(alts.begin(), alts.end());
                patrol_altitudes_ = alts;
                std::string s;
                for (auto a : alts) { char b[16]; snprintf(b, 16, "%.1f", a); if (!s.empty()) s += ", "; s += b; }
                ROS_INFO("[巡检C++] ✓ 巡检高度: [%s] m", s.c_str());
                ROS_INFO("[巡检C++] ★ RViz蓝色半透明平面即为飞行高度");
                if (alts.size() > 1) ROS_INFO("[巡检C++]   多层模式: %zu 层", alts.size());
                publishAltitudePlane();
                publishWaypointMarkers();
            } else if (cmd == "speed") {
                double s; if (iss >> s) { flight_speed_ = s; ROS_INFO("[巡检C++] 速度=%.1fm/s", s); }
            } else if (cmd == "start") {
                if (patrol_waypoints_2d_.empty()) { ROS_WARN("[巡检C++] 请先点击航点!"); continue; }
                if (!has_pose_) { ROS_WARN("[巡检C++] 位姿未知!"); continue; }
                current_layer_idx_ = 0;
                if (!planLayer(0)) continue;
                takeoff_target_alt_ = patrol_altitudes_[0];
                pre_arm_start_ = ros::Time::now();
                mode_ = "PRE_ARM";
                ROS_INFO("[巡检C++] 准备起飞...");
            } else if (cmd == "clear") {
                patrol_waypoints_2d_.clear(); planned_segments_.clear();
                mode_ = "SETTING"; current_layer_idx_ = 0;
                current_seg_idx_ = 0; current_pt_idx_ = 0;
                visualization_msgs::MarkerArray del_ma;
                visualization_msgs::Marker d; d.action = visualization_msgs::Marker::DELETEALL;
                d.ns = "patrol_wp"; del_ma.markers.push_back(d);
                d.ns = "patrol_line"; del_ma.markers.push_back(d);
                waypoint_marker_pub_.publish(del_ma);
                ROS_INFO("[巡检C++] 已清除所有航点");
            } else if (cmd == "stop") {
                mode_ = "DONE";
                ROS_WARN("[巡检C++] ★ 紧急停止! 悬停.");
            } else if (cmd == "status") {
                std::string s;
                for (auto a : patrol_altitudes_) { char b[16]; snprintf(b, 16, "%.1f", a); if (!s.empty()) s += ","; s += b; }
                ROS_INFO("[巡检C++] mode=%s, wp=%zu, alt=[%s], layer=%d/%zu, v=%.1f, map=%s(%zu)",
                         mode_.c_str(), patrol_waypoints_2d_.size(), s.c_str(),
                         current_layer_idx_+1, patrol_altitudes_.size(), flight_speed_,
                         map_ready_ ? "OK" : "NO", occupied_voxels_.size());
            } else if (cmd == "undo") {
                if (!patrol_waypoints_2d_.empty()) {
                    patrol_waypoints_2d_.pop_back();
                    publishWaypointMarkers();
                    ROS_INFO("[巡检C++] 已撤销, 剩余 %zu 个航点", patrol_waypoints_2d_.size());
                }
            } else if (cmd == "home") {
                if (!has_home_ || !has_pose_) { ROS_WARN("[巡检C++] 位姿未知"); continue; }
                patrol_waypoints_2d_ = {home_xy_};
                double alt = current_pose_.pose.position.z;
                if (alt < 1.0) alt = patrol_altitudes_[0];
                patrol_altitudes_ = {alt};
                current_layer_idx_ = 0;
                planLayer(0);
                mode_ = "EXECUTING";
                ROS_INFO("[巡检C++] 返航中...");
            } else {
                ROS_INFO("[巡检C++] 命令: alt H | speed S | start | clear | stop | home | undo | status");
            }
        }
    }
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "patrol_planner");
    PatrolPlanner planner;
    planner.run();
    return 0;
}
