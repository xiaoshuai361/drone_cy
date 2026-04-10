/*
 * grid_map_planner_node.cpp
 * 2D栅格地图巡检规划器 (C++高性能版)
 *
 * 核心: 加载PGM栅格 → 预膨胀障碍物 → A*路径规划 → 自动飞行
 * 终端命令: alt H | start | clear | stop | status | home | undo | speed S
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
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
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <clocale>
#include <mutex>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

struct PairHash {
    size_t operator()(const std::pair<int,int> &p) const {
        return std::hash<long long>()(((long long)p.first << 32) | (unsigned int)p.second);
    }
};

class GridMapPlanner
{
public:
    GridMapPlanner() : nh_("~"), mode_("SETTING"), current_seg_idx_(0), current_pt_idx_(0),
                       map_ready_(false), drone_viz_counter_(0)
    {
        nh_.param("safety_margin", safety_margin_, 0.3);
        nh_.param("waypoint_threshold", waypoint_threshold_, 0.5);
        nh_.param("default_altitude", default_altitude_, 1.0);
        nh_.param("flight_speed", flight_speed_, 6.9);
        nh_.param<std::string>("map_yaml", map_yaml_, "");

        patrol_altitude_ = default_altitude_;
        takeoff_target_alt_ = default_altitude_;

        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/patrol/planned_route_path", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/waypoints", 1);
        route_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/planned_route", 1);
        drone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/drone_position", 1);
        alt_plane_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/patrol/altitude_plane", 1, true);
        gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_2d", 1, true);
        local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap_2d", 1);

        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &GridMapPlanner::poseCb, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10, &GridMapPlanner::stateCb, this);
        click_sub_ = nh_.subscribe("/clicked_point", 10, &GridMapPlanner::clickCb, this);
        cloud_sub_ = nh_.subscribe("/cloud_filtered", 1, &GridMapPlanner::cloudCb, this);

        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        nh_.param("local_costmap_radius", local_costmap_radius_, 5.0);
        nh_.param("obstacle_ttl", obstacle_ttl_, 3.0);
        nh_.param("replan_check_interval", replan_check_interval_, 0.5);

        loadMap();

        // 终端输入线程
        input_thread_ = std::thread(&GridMapPlanner::inputLoop, this);
        input_thread_.detach();

        ROS_INFO("[2D巡检C++] 已启动! 高度=%.1fm, 速度=%.1fm/s", patrol_altitude_, flight_speed_);
    }

    void run()
    {
        ros::Rate rate(20);
        int costmap_counter = 0;
        while (ros::ok()) {
            flyStep();
            if (++drone_viz_counter_ >= 4) {
                drone_viz_counter_ = 0;
                publishDroneMarker();
            }
            // 每10帧发布一次局部代价地图
            if (++costmap_counter >= 10) {
                costmap_counter = 0;
                publishLocalCostmap();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_, path_pub_, marker_pub_, route_marker_pub_;
    ros::Publisher drone_marker_pub_, alt_plane_pub_, gridmap_pub_;
    ros::Publisher local_costmap_pub_;
    ros::Subscriber pose_sub_, state_sub_, click_sub_, cloud_sub_;
    ros::ServiceClient arm_client_, mode_client_;

    // 地图
    std::vector<uint8_t> grid_;
    std::vector<bool> free_grid_;
    double grid_resolution_, grid_origin_x_, grid_origin_y_;
    int grid_width_, grid_height_;
    bool map_ready_;
    std::string map_yaml_;
    double safety_margin_, waypoint_threshold_, default_altitude_, flight_speed_;

    // 局部代价地图 (动态障碍)
    std::unordered_map<long long, ros::Time> dynamic_obstacles_;  // grid_key → last_seen_time
    std::mutex dynamic_obs_mutex_;
    double local_costmap_radius_;   // 检测范围 (m)
    double obstacle_ttl_;           // 动态障碍存活时间 (s)
    double replan_check_interval_;  // 重规划检查间隔 (s)
    ros::Time last_replan_check_;

    // 状态
    geometry_msgs::PoseStamped current_pose_;
    mavros_msgs::State current_state_;
    bool has_pose_ = false;
    double patrol_altitude_, takeoff_target_alt_;
    std::vector<std::pair<double,double>> waypoints_2d_;
    std::vector<std::vector<std::tuple<double,double,double>>> planned_segments_;
    std::string mode_;
    int current_seg_idx_, current_pt_idx_;
    std::pair<double,double> home_xy_;
    bool has_home_ = false;
    ros::Time pre_arm_start_;
    int drone_viz_counter_;
    std::thread input_thread_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose_ = *msg;
        has_pose_ = true;
    }
    void stateCb(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }
    void clickCb(const geometry_msgs::PointStamped::ConstPtr &msg) {
        if (mode_ != "SETTING") { ROS_WARN("[2D巡检] 不在设定模式"); return; }
        double x = msg->point.x, y = msg->point.y;
        waypoints_2d_.emplace_back(x, y);
        ROS_INFO("[2D巡检] ✓ 航点 #%zu: (%.1f, %.1f) 高度=%.1fm",
                 waypoints_2d_.size(), x, y, patrol_altitude_);
        publishWaypointMarkers();
    }

    // ==================== 局部代价地图 (动态障碍) ====================
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        if (!map_ready_ || !has_pose_) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        double alt = patrol_altitude_;
        double alt_tol = 1.0;  // 高度容差, 只看飞行高度附近的点
        int margin = std::max(1, (int)(safety_margin_ / grid_resolution_));
        ros::Time now = ros::Time::now();

        std::lock_guard<std::mutex> lock(dynamic_obs_mutex_);
        for (auto &pt : cloud->points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if (fabs(pt.z - alt) > alt_tol) continue;  // 不在飞行平面

            int gx = (int)((pt.x - grid_origin_x_) / grid_resolution_);
            int gy = (int)((pt.y - grid_origin_y_) / grid_resolution_);

            // 膨胀: 将障碍点周围 margin 范围的栅格标记
            for (int dy = -margin; dy <= margin; ++dy) {
                for (int dx = -margin; dx <= margin; ++dx) {
                    if (dx*dx + dy*dy > margin*margin) continue;
                    int nx = gx + dx, ny = gy + dy;
                    if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_) continue;
                    // 只标记静态地图中本来是自由的格子 (新障碍)
                    long long key = ((long long)nx << 32) | (unsigned int)ny;
                    dynamic_obstacles_[key] = now;
                }
            }
        }

        // 清除过期的动态障碍
        for (auto it = dynamic_obstacles_.begin(); it != dynamic_obstacles_.end(); ) {
            if ((now - it->second).toSec() > obstacle_ttl_)
                it = dynamic_obstacles_.erase(it);
            else
                ++it;
        }
    }

    bool isDynamicObstacle(int gx, int gy) {
        long long key = ((long long)gx << 32) | (unsigned int)gy;
        std::lock_guard<std::mutex> lock(dynamic_obs_mutex_);
        return dynamic_obstacles_.count(key) > 0;
    }

    // 综合检查: 静态地图 + 动态障碍
    bool isCellFree(int gx, int gy) {
        if (!isInflatedFree(gx, gy)) return false;
        return !isDynamicObstacle(gx, gy);
    }

    // 检测当前路径是否有碰撞
    bool checkPathCollision() {
        if (current_seg_idx_ >= (int)planned_segments_.size()) return false;
        auto &seg = planned_segments_[current_seg_idx_];
        // 检查当前航段剩余的路径点
        for (int i = current_pt_idx_; i < (int)seg.size(); ++i) {
            auto [wx, wy, wz] = seg[i];
            auto [gx, gy] = worldToGrid(wx, wy);
            if (isDynamicObstacle(gx, gy)) return true;
        }
        return false;
    }

    // 重新规划当前航段
    bool replanCurrentSegment() {
        if (current_seg_idx_ >= (int)planned_segments_.size()) return false;
        double cx = current_pose_.pose.position.x;
        double cy = current_pose_.pose.position.y;

        // 目标: 当前航段的终点
        auto &seg = planned_segments_[current_seg_idx_];
        auto [tx, ty, tz] = seg.back();
        auto start = std::make_pair(cx, cy);
        auto goal = std::make_pair(tx, ty);

        ROS_WARN("[2D巡检] 检测到动态障碍! 重新规划航段 %d...", current_seg_idx_ + 1);
        auto path = astarDynamic(start, goal);
        if (path.empty()) {
            ROS_WARN("[2D巡检] 重规划失败! 使用直线");
            path = {start, goal};
        }
        path = simplifyPath(path);
        std::vector<std::tuple<double,double,double>> new_seg;
        for (auto &p : path) new_seg.emplace_back(p.first, p.second, patrol_altitude_);
        planned_segments_[current_seg_idx_] = new_seg;
        current_pt_idx_ = 0;
        ROS_INFO("[2D巡检] 重规划完成! 新路径 %zu 点", new_seg.size());
        publishRouteMarkers();
        return true;
    }

    void publishLocalCostmap() {
        if (!map_ready_) return;
        nav_msgs::OccupancyGrid og;
        og.header.stamp = ros::Time::now();
        og.header.frame_id = "camera_init";
        og.info.resolution = grid_resolution_;
        og.info.width = grid_width_;
        og.info.height = grid_height_;
        og.info.origin.position.x = grid_origin_x_;
        og.info.origin.position.y = grid_origin_y_;
        og.info.origin.orientation.w = 1.0;
        og.data.resize(grid_width_ * grid_height_, 0);

        // 静态层
        for (int i = 0; i < (int)grid_.size(); ++i) {
            if (grid_[i] < 50) og.data[i] = 100;
            else if (!free_grid_[i]) og.data[i] = 50;  // 膨胀区
            else og.data[i] = 0;
        }

        // 动态层叠加
        {
            std::lock_guard<std::mutex> lock(dynamic_obs_mutex_);
            for (auto &[key, t] : dynamic_obstacles_) {
                int gx = (int)(key >> 32);
                int gy = (int)(key & 0xFFFFFFFF);
                if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_)
                    og.data[gy * grid_width_ + gx] = 80;  // 动态障碍
            }
        }
        local_costmap_pub_.publish(og);
    }

    // ==================== 地图加载 ====================
    void loadMap()
    {
        if (map_yaml_.empty()) { ROS_WARN("[2D巡检] 未指定地图!"); return; }

        YAML::Node config = YAML::LoadFile(map_yaml_);
        std::string pgm_name = config["image"].as<std::string>();
        grid_resolution_ = config["resolution"].as<double>();
        auto origin = config["origin"];
        grid_origin_x_ = origin[0].as<double>();
        grid_origin_y_ = origin[1].as<double>();

        // 绝对路径
        if (pgm_name[0] != '/') {
            std::string dir = map_yaml_.substr(0, map_yaml_.rfind('/'));
            pgm_name = dir + "/" + pgm_name;
        }

        // 读PGM P5
        std::ifstream pgm(pgm_name, std::ios::binary);
        if (!pgm.is_open()) { ROS_ERROR("[2D巡检] 无法打开PGM: %s", pgm_name.c_str()); return; }

        std::string magic;
        pgm >> magic;
        if (magic != "P5") { ROS_ERROR("[2D巡检] 非P5格式!"); return; }

        // 跳过注释
        char c;
        pgm.get(c);
        while (pgm.peek() == '#') {
            std::string comment;
            std::getline(pgm, comment);
        }

        int w, h, maxval;
        pgm >> w >> h >> maxval;
        pgm.get(c); // newline

        grid_width_ = w;
        grid_height_ = h;
        std::vector<uint8_t> raw(w * h);
        pgm.read(reinterpret_cast<char*>(raw.data()), w * h);
        pgm.close();

        // 翻转Y轴
        grid_.resize(w * h);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                grid_[y * w + x] = raw[(h - 1 - y) * w + x];

        map_ready_ = true;
        int occupied = 0, free = 0;
        for (auto v : grid_) { if (v < 50) occupied++; if (v > 200) free++; }
        ROS_INFO("[2D巡检] ✓ 地图: %dx%d, 分辨率=%.3f, 占据=%d, 自由=%d",
                 w, h, grid_resolution_, occupied, free);

        inflateGrid();
        publishOccupancyGrid();
    }

    void inflateGrid()
    {
        int margin = std::max(1, (int)(safety_margin_ / grid_resolution_));
        free_grid_.assign(grid_width_ * grid_height_, false);

        // 标记自由
        for (int y = 0; y < grid_height_; ++y)
            for (int x = 0; x < grid_width_; ++x)
                if (grid_[y * grid_width_ + x] > 200)
                    free_grid_[y * grid_width_ + x] = true;

        // 膨胀: 在障碍物margin范围内标记为不自由
        std::vector<bool> obs(grid_width_ * grid_height_, false);
        for (int y = 0; y < grid_height_; ++y)
            for (int x = 0; x < grid_width_; ++x)
                if (grid_[y * grid_width_ + x] < 50)
                    obs[y * grid_width_ + x] = true;

        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                if (!obs[y * grid_width_ + x]) continue;
                for (int dy = -margin; dy <= margin; ++dy) {
                    for (int dx = -margin; dx <= margin; ++dx) {
                        if (dx*dx + dy*dy > margin*margin) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_)
                            free_grid_[ny * grid_width_ + nx] = false;
                    }
                }
            }
        }

        int free_count = 0;
        for (auto v : free_grid_) if (v) free_count++;
        ROS_INFO("[2D巡检] ✓ 膨胀完成: 安全=%d格, 可通行=%d/%d",
                 margin, free_count, grid_width_ * grid_height_);
    }

    void publishOccupancyGrid()
    {
        nav_msgs::OccupancyGrid og;
        og.header.stamp = ros::Time::now();
        og.header.frame_id = "camera_init";
        og.info.resolution = grid_resolution_;
        og.info.width = grid_width_;
        og.info.height = grid_height_;
        og.info.origin.position.x = grid_origin_x_;
        og.info.origin.position.y = grid_origin_y_;
        og.info.origin.orientation.w = 1.0;

        og.data.resize(grid_width_ * grid_height_);
        for (int i = 0; i < (int)grid_.size(); ++i) {
            if (grid_[i] < 50) og.data[i] = 100;
            else if (grid_[i] > 200) og.data[i] = 0;
            else og.data[i] = -1;
        }
        gridmap_pub_.publish(og);
    }

    // ==================== A* ====================
    inline bool isInflatedFree(int gx, int gy) {
        if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) return false;
        return free_grid_[gy * grid_width_ + gx];
    }

    std::pair<int,int> worldToGrid(double wx, double wy) {
        return {(int)((wx - grid_origin_x_) / grid_resolution_),
                (int)((wy - grid_origin_y_) / grid_resolution_)};
    }

    std::pair<double,double> gridToWorld(int gx, int gy) {
        return {gx * grid_resolution_ + grid_origin_x_ + grid_resolution_ / 2,
                gy * grid_resolution_ + grid_origin_y_ + grid_resolution_ / 2};
    }

    std::pair<int,int> findNearestFree(int gx, int gy, int max_r = 100) {
        for (int r = 1; r < max_r; ++r)
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx)
                    if ((abs(dx) == r || abs(dy) == r) && isInflatedFree(gx+dx, gy+dy))
                        return {gx+dx, gy+dy};
        return {-1, -1};
    }

    std::vector<std::pair<double,double>> astar(std::pair<double,double> start_xy,
                                                 std::pair<double,double> goal_xy)
    {
        if (!map_ready_) return {};
        auto [sx, sy] = worldToGrid(start_xy.first, start_xy.second);
        auto [gx, gy] = worldToGrid(goal_xy.first, goal_xy.second);

        if (!isInflatedFree(sx, sy)) {
            auto [nx, ny] = findNearestFree(sx, sy);
            if (nx < 0) return {};
            sx = nx; sy = ny;
        }
        if (!isInflatedFree(gx, gy)) {
            auto [nx, ny] = findNearestFree(gx, gy);
            if (nx < 0) return {};
            gx = nx; gy = ny;
        }

        // 8-方向
        static const int DX[] = {1,-1,0,0,1,-1,1,-1};
        static const int DY[] = {0,0,1,-1,1,1,-1,-1};
        static const double COST[] = {1,1,1,1,1.414,1.414,1.414,1.414};

        using PQE = std::tuple<double,int,int,int>; // f, counter, x, y
        std::priority_queue<PQE, std::vector<PQE>, std::greater<PQE>> open;
        std::unordered_map<long long, double> g_score;
        std::unordered_map<long long, long long> came_from;
        std::unordered_set<long long> visited;

        auto key = [](int x, int y) -> long long { return ((long long)x << 32) | (unsigned int)y; };

        int counter = 0;
        double h0 = sqrt((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy));
        open.push({h0, counter++, sx, sy});
        g_score[key(sx,sy)] = 0;

        auto t0 = std::chrono::steady_clock::now();
        int max_iter = 500000, iter = 0;

        while (!open.empty() && iter++ < max_iter) {
            auto [f, cnt, cx, cy] = open.top(); open.pop();
            long long ck = key(cx, cy);
            if (visited.count(ck)) continue;
            visited.insert(ck);

            if (abs(cx-gx) <= 1 && abs(cy-gy) <= 1) {
                // 回溯
                std::vector<std::pair<double,double>> path;
                long long node = ck;
                while (came_from.count(node)) {
                    int nx = (int)(node >> 32), ny = (int)(node & 0xFFFFFFFF);
                    auto [wx, wy] = gridToWorld(nx, ny);
                    path.push_back({wx, wy});
                    node = came_from[node];
                }
                std::reverse(path.begin(), path.end());
                auto [wx, wy] = gridToWorld(gx, gy);
                path.push_back({wx, wy});

                auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                ROS_INFO("[2D巡检] A*完成: %.3fs, 扩展%zu节点, 路径%zu点",
                         dt, visited.size(), path.size());
                return path;
            }

            for (int d = 0; d < 8; ++d) {
                int nx = cx + DX[d], ny = cy + DY[d];
                long long nk = key(nx, ny);
                if (visited.count(nk) || !isInflatedFree(nx, ny)) continue;
                double ng = g_score[ck] + COST[d];
                auto it = g_score.find(nk);
                if (it == g_score.end() || ng < it->second) {
                    g_score[nk] = ng;
                    double h = sqrt((nx-gx)*(nx-gx) + (ny-gy)*(ny-gy));
                    open.push({ng + h, counter++, nx, ny});
                    came_from[nk] = ck;
                }
            }
        }
        ROS_WARN("[2D巡检] A*未找到路径!");
        return {};
    }

    // A* 包含动态障碍 (用于重规划)
    std::vector<std::pair<double,double>> astarDynamic(std::pair<double,double> start_xy,
                                                        std::pair<double,double> goal_xy)
    {
        if (!map_ready_) return {};
        auto [sx, sy] = worldToGrid(start_xy.first, start_xy.second);
        auto [gx, gy] = worldToGrid(goal_xy.first, goal_xy.second);

        auto isFree = [this](int x, int y) -> bool {
            return isInflatedFree(x, y) && !isDynamicObstacle(x, y);
        };

        if (!isFree(sx, sy)) {
            for (int r = 1; r < 100; ++r)
                for (int dy = -r; dy <= r; ++dy)
                    for (int dx = -r; dx <= r; ++dx)
                        if ((abs(dx)==r||abs(dy)==r) && isFree(sx+dx,sy+dy))
                            { sx+=dx; sy+=dy; goto found_start; }
            return {};
            found_start:;
        }
        if (!isFree(gx, gy)) {
            for (int r = 1; r < 100; ++r)
                for (int dy = -r; dy <= r; ++dy)
                    for (int dx = -r; dx <= r; ++dx)
                        if ((abs(dx)==r||abs(dy)==r) && isFree(gx+dx,gy+dy))
                            { gx+=dx; gy+=dy; goto found_goal; }
            return {};
            found_goal:;
        }

        static const int DX[] = {1,-1,0,0,1,-1,1,-1};
        static const int DY[] = {0,0,1,-1,1,1,-1,-1};
        static const double COST[] = {1,1,1,1,1.414,1.414,1.414,1.414};

        using PQE = std::tuple<double,int,int,int>;
        std::priority_queue<PQE, std::vector<PQE>, std::greater<PQE>> open;
        std::unordered_map<long long, double> g_score;
        std::unordered_map<long long, long long> came_from;
        std::unordered_set<long long> visited;

        auto key = [](int x, int y) -> long long { return ((long long)x << 32) | (unsigned int)y; };

        int counter = 0;
        double h0 = sqrt((gx-sx)*(gx-sx)+(gy-sy)*(gy-sy));
        open.push({h0, counter++, sx, sy});
        g_score[key(sx,sy)] = 0;

        int max_iter = 500000, iter = 0;
        while (!open.empty() && iter++ < max_iter) {
            auto [f, cnt, cx, cy] = open.top(); open.pop();
            long long ck = key(cx,cy);
            if (visited.count(ck)) continue;
            visited.insert(ck);

            if (abs(cx-gx)<=1 && abs(cy-gy)<=1) {
                std::vector<std::pair<double,double>> path;
                long long node = ck;
                while (came_from.count(node)) {
                    int nx=(int)(node>>32), ny=(int)(node&0xFFFFFFFF);
                    auto [wx,wy] = gridToWorld(nx,ny);
                    path.push_back({wx,wy});
                    node = came_from[node];
                }
                std::reverse(path.begin(), path.end());
                auto [wx,wy] = gridToWorld(gx,gy);
                path.push_back({wx,wy});
                return path;
            }

            for (int d = 0; d < 8; ++d) {
                int nx=cx+DX[d], ny=cy+DY[d];
                long long nk = key(nx,ny);
                if (visited.count(nk) || !isFree(nx,ny)) continue;
                double ng = g_score[ck]+COST[d];
                auto it = g_score.find(nk);
                if (it==g_score.end() || ng<it->second) {
                    g_score[nk] = ng;
                    double h = sqrt((nx-gx)*(nx-gx)+(ny-gy)*(ny-gy));
                    open.push({ng+h, counter++, nx, ny});
                    came_from[nk] = ck;
                }
            }
        }
        return {};
    }

    std::vector<std::pair<double,double>> simplifyPath(const std::vector<std::pair<double,double>> &path) {
        if (path.size() <= 2) return path;
        std::vector<std::pair<double,double>> s = {path[0]};
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            double dx1 = path[i].first - s.back().first, dy1 = path[i].second - s.back().second;
            double dx2 = path[i+1].first - path[i].first, dy2 = path[i+1].second - path[i].second;
            if (fabs(dx1*dy2 - dy1*dx2) > 0.01) s.push_back(path[i]);
        }
        s.push_back(path.back());
        return s;
    }

    bool planAll() {
        if (!map_ready_ || waypoints_2d_.empty()) return false;
        double cx = 0, cy = 0;
        if (has_pose_) { cx = current_pose_.pose.position.x; cy = current_pose_.pose.position.y; }
        planned_segments_.clear();
        auto prev = std::make_pair(cx, cy);
        for (size_t i = 0; i < waypoints_2d_.size(); ++i) {
            auto wp = waypoints_2d_[i];
            ROS_INFO("[2D巡检] 规划航段 %zu: (%.1f,%.1f)→(%.1f,%.1f)",
                     i+1, prev.first, prev.second, wp.first, wp.second);
            auto path = astar(prev, wp);
            if (path.empty()) {
                ROS_WARN("[2D巡检] A*失败, 使用直线!");
                path = {prev, wp};
            }
            path = simplifyPath(path);
            std::vector<std::tuple<double,double,double>> seg;
            for (auto &p : path) seg.emplace_back(p.first, p.second, patrol_altitude_);
            planned_segments_.push_back(seg);
            ROS_INFO("[2D巡检]   → %zu 个路径点", seg.size());
            prev = wp;
        }
        ROS_INFO("[2D巡检] ✓ 规划完成! %zu 个航段", planned_segments_.size());
        publishRouteMarkers();
        return true;
    }

    // ==================== 飞行控制 ====================
    double getCurrentYaw() {
        auto &q = current_pose_.pose.orientation;
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return atan2(siny, cosy);
    }

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
                if (mode_client_.call(sm)) ROS_INFO("[2D巡检] OFFBOARD OK");
                ros::Duration(0.3).sleep();
                mavros_msgs::CommandBool ab; ab.request.value = true;
                if (arm_client_.call(ab)) ROS_INFO("[2D巡检] ARM OK");
                mode_ = "TAKING_OFF";
                ROS_INFO("[2D巡检] Takeoff -> %.1fm", takeoff_target_alt_);
            }
        } else if (mode_ == "TAKING_OFF") {
            double cur_yaw = getCurrentYaw();
            sendSetpoint(cx, cy, takeoff_target_alt_, cur_yaw);
            if (fabs(cz - takeoff_target_alt_) < 0.3) {
                ROS_INFO("[2D巡检] Reached %.1fm!", takeoff_target_alt_);
                mode_ = "EXECUTING";
            }
        } else if (mode_ == "EXECUTING") {
            executeStep(cx, cy, cz);
        } else if (mode_ == "DONE") {
            sendSetpoint(cx, cy, cz);
        }
    }

    void executeStep(double cx, double cy, double cz) {
        if (current_seg_idx_ >= (int)planned_segments_.size()) {
            ROS_INFO("[2D巡检] ====== 巡检完成! ======");
            mode_ = "DONE"; return;
        }

        // 定期检查动态障碍碰撞 → 重规划
        ros::Time now = ros::Time::now();
        if ((now - last_replan_check_).toSec() >= replan_check_interval_) {
            last_replan_check_ = now;
            if (checkPathCollision()) {
                replanCurrentSegment();
            }
        }

        auto &seg = planned_segments_[current_seg_idx_];
        if (current_pt_idx_ >= (int)seg.size()) {
            current_seg_idx_++; current_pt_idx_ = 0; return;
        }
        auto [tx, ty, tz] = seg[current_pt_idx_];
        double dx = tx-cx, dy = ty-cy, dz = tz-cz;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < waypoint_threshold_) { current_pt_idx_++; return; }
        double yaw = atan2(dy, dx);
        double max_step = flight_speed_ * 0.05;
        if (dist > max_step) {
            double r = max_step / dist;
            sendSetpoint(cx+dx*r, cy+dy*r, cz+dz*r, yaw);
        } else {
            sendSetpoint(tx, ty, tz, yaw);
        }
    }

    // ==================== 可视化 ====================
    void publishWaypointMarkers() {
        visualization_msgs::MarkerArray ma;
        for (size_t i = 0; i < waypoints_2d_.size(); ++i) {
            visualization_msgs::Marker m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = "camera_init";
            m.ns = "waypoints"; m.id = i;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = waypoints_2d_[i].first;
            m.pose.position.y = waypoints_2d_[i].second;
            m.pose.position.z = patrol_altitude_;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 0.3; m.scale.z = 0.1;
            m.color.r = 1; m.color.g = 0.5; m.color.a = 0.8;
            ma.markers.push_back(m);

            visualization_msgs::Marker t;
            t.header = m.header; t.ns = "wp_labels"; t.id = i;
            t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::Marker::ADD;
            t.pose.position.x = waypoints_2d_[i].first;
            t.pose.position.y = waypoints_2d_[i].second;
            t.pose.position.z = patrol_altitude_ + 0.3;
            t.pose.orientation.w = 1.0; t.scale.z = 0.3;
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
            t.text = "#" + std::to_string(i + 1);
            ma.markers.push_back(t);
        }
        marker_pub_.publish(ma);
    }

    void publishRouteMarkers() {
        visualization_msgs::MarkerArray ma;
        for (size_t si = 0; si < planned_segments_.size(); ++si) {
            visualization_msgs::Marker m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = "camera_init";
            m.ns = "route_" + std::to_string(si); m.id = si;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.scale.x = 0.05; m.pose.orientation.w = 1.0;
            m.color.g = 1; m.color.a = 0.8;
            for (auto &[px,py,pz] : planned_segments_[si]) {
                geometry_msgs::Point p; p.x = px; p.y = py; p.z = pz;
                m.points.push_back(p);
            }
            ma.markers.push_back(m);
        }
        route_marker_pub_.publish(ma);
    }

    void publishDroneMarker() {
        if (!has_pose_) return;
        auto &p = current_pose_.pose.position;
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker s;
        s.header.stamp = ros::Time::now(); s.header.frame_id = "camera_init";
        s.ns = "drone"; s.id = 0;
        s.type = visualization_msgs::Marker::SPHERE;
        s.action = visualization_msgs::Marker::ADD;
        s.pose.position = p; s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 0.3;
        s.color.r = 1; s.color.g = 0.2; s.color.b = 0.2; s.color.a = 0.9;
        s.lifetime = ros::Duration(0.5);
        ma.markers.push_back(s);

        visualization_msgs::Marker txt;
        txt.header = s.header; txt.ns = "drone_alt"; txt.id = 2;
        txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::Marker::ADD;
        txt.pose.position.x = p.x; txt.pose.position.y = p.y;
        txt.pose.position.z = p.z + 0.4;
        txt.pose.orientation.w = 1.0; txt.scale.z = 0.25;
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0;
        txt.text = "UAV h=" + std::to_string(p.z).substr(0,4) + "m";
        txt.lifetime = ros::Duration(0.5);
        ma.markers.push_back(txt);
        drone_marker_pub_.publish(ma);
    }

    void publishAltitudePlane() {
        visualization_msgs::MarkerArray ma;

        // 平面
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now(); m.header.frame_id = "camera_init";
        m.ns = "alt_plane"; m.id = 0;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.z = patrol_altitude_;
        m.pose.orientation.w = 1.0;
        m.scale.x = 40; m.scale.y = 40; m.scale.z = 0.02;
        m.color.r = 0.3; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 0.25;
        ma.markers.push_back(m);
        // 文字
        visualization_msgs::Marker t;
        t.header = m.header; t.ns = "alt_text"; t.id = 0;
        t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::Marker::ADD;
        t.pose.position.z = patrol_altitude_ + 0.15;
        t.pose.orientation.w = 1.0; t.scale.z = 0.5;
        t.color.g = 0.8; t.color.b = 1.0; t.color.a = 1.0;
        char buf[64]; snprintf(buf, sizeof(buf), "=== %.1fm ===", patrol_altitude_);
        t.text = buf;
        ma.markers.push_back(t);
        alt_plane_pub_.publish(ma);
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
                double h; if (iss >> h) {
                    patrol_altitude_ = h;
                    takeoff_target_alt_ = h;
                    ROS_INFO("[2D巡检] 高度=%.1fm", h);
                    ROS_INFO("[2D巡检] ★ RViz蓝色半透明平面即为飞行高度");
                    publishAltitudePlane();
                }
            } else if (cmd == "speed") {
                double s; if (iss >> s) { flight_speed_ = s; ROS_INFO("[2D巡检] 速度=%.1fm/s", s); }
            } else if (cmd == "start") {
                if (waypoints_2d_.empty()) { ROS_WARN("[2D巡检] 请先点击航点!"); continue; }
                if (!planAll()) continue;
                current_seg_idx_ = 0; current_pt_idx_ = 0;
                if (has_pose_) { home_xy_ = {current_pose_.pose.position.x, current_pose_.pose.position.y}; has_home_ = true; }
                pre_arm_start_ = ros::Time::now();
                mode_ = "PRE_ARM";
                ROS_INFO("[2D巡检] 准备起飞...");
            } else if (cmd == "clear") {
                waypoints_2d_.clear(); planned_segments_.clear();
                mode_ = "SETTING"; current_seg_idx_ = 0; current_pt_idx_ = 0;
                ROS_INFO("[2D巡检] 已清除");
            } else if (cmd == "stop") {
                mode_ = "DONE"; ROS_INFO("[2D巡检] 紧急悬停!");
            } else if (cmd == "status") {
                ROS_INFO("[2D巡检] mode=%s, wp=%zu, h=%.1f, v=%.1f, map=%s",
                         mode_.c_str(), waypoints_2d_.size(), patrol_altitude_,
                         flight_speed_, map_ready_ ? "OK" : "NO");
            } else if (cmd == "undo") {
                if (!waypoints_2d_.empty()) { waypoints_2d_.pop_back(); publishWaypointMarkers(); }
            } else if (cmd == "home" && has_home_) {
                waypoints_2d_ = {home_xy_};
                planAll(); current_seg_idx_ = 0; current_pt_idx_ = 0;
                mode_ = "EXECUTING";
            } else {
                ROS_INFO("[2D巡检] 命令: alt H | speed S | start | clear | stop | home | undo | status");
            }
        }
    }
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "grid_map_planner");
    GridMapPlanner planner;
    planner.run();
    return 0;
}
