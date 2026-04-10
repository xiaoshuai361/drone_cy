/*
 * keyboard_teleop_node.cpp
 * MAK4 无人机键盘遥控节点 (C++高性能版)
 *
 * 键盘操作:
 *   W/S = 前进/后退    A/D = 左移/右移
 *   Q/E = 左转/右转    R/F = 上升/下降
 *   T   = 一键起飞     L   = 降落
 *   空格= 悬停         1/2 = 减速/加速
 *   ESC = 退出
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <signal.h>
#include <clocale>

static struct termios old_settings;
static bool settings_saved = false;

void restoreTerminal(int)
{
    if (settings_saved)
        tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings);
    printf("\n[Teleop] 退出\n");
    ros::shutdown();
    exit(0);
}

class KeyboardTeleop
{
public:
    KeyboardTeleop() : nh_("~")
    {
        linear_speed_ = 0.5;
        vertical_speed_ = 0.3;
        yaw_rate_val_ = 0.5;
        vx_ = vy_ = vz_ = yr_ = 0.0;
        use_position_hold_ = false;
        hold_altitude_valid_ = false;
        hold_altitude_ = 0.0;

        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        state_sub_ = nh_.subscribe("/mavros/state", 10, &KeyboardTeleop::stateCb, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &KeyboardTeleop::poseCb, this);

        ros::service::waitForService("/mavros/cmd/arming", ros::Duration(10));
        ros::service::waitForService("/mavros/set_mode", ros::Duration(10));
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    void stateCb(const mavros_msgs::State::ConstPtr &msg) { state_ = *msg; }
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) { pose_ = *msg; }

    char getKey()
    {
        fd_set fds;
        struct timeval tv;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 20000; // 20ms
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
            return getchar();
        return 0;
    }

    void sendCommand()
    {
        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        double q_w = pose_.pose.orientation.w, q_x = pose_.pose.orientation.x;
        double q_y = pose_.pose.orientation.y, q_z = pose_.pose.orientation.z;
        double siny = 2.0 * (q_w * q_z + q_x * q_y);
        double cosy = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
        double yaw = atan2(siny, cosy);

        msg.velocity.x = vx_ * cos(yaw) - vy_ * sin(yaw);
        msg.velocity.y = vx_ * sin(yaw) + vy_ * cos(yaw);
        msg.yaw_rate = yr_;

        if (hold_altitude_valid_ && vz_ == 0.0) {
            // ★ 定高模式: 水平速度 + Z轴位置锁定
            msg.position.z = hold_altitude_;
            msg.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
        } else {
            // 全速度模式 (手动升降中)
            msg.velocity.z = vz_;
            msg.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
        }

        raw_pub_.publish(msg);
    }

    void takeoff(double height = 1.5)
    {
        ROS_INFO("[Teleop] 起飞序列... 目标高度=%.1fm", height);
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.pose.position.x = pose_.pose.position.x;
        target.pose.position.y = pose_.pose.position.y;
        target.pose.position.z = height;
        target.pose.orientation = pose_.pose.orientation;

        ros::Rate rate(30);
        for (int i = 0; i < 75; ++i) {
            target.header.stamp = ros::Time::now();
            pos_pub_.publish(target);
            rate.sleep();
        }

        if (state_.mode != "OFFBOARD") {
            mavros_msgs::SetMode sm;
            sm.request.custom_mode = "OFFBOARD";
            if (mode_client_.call(sm) && sm.response.mode_sent)
                ROS_INFO("[Teleop] OFFBOARD 已设置");
        }
        ros::Duration(0.3).sleep();

        if (!state_.armed) {
            mavros_msgs::CommandBool ab;
            ab.request.value = true;
            if (arm_client_.call(ab) && ab.response.success)
                ROS_INFO("[Teleop] 已解锁");
        }

        for (int i = 0; i < 450; ++i) {
            target.header.stamp = ros::Time::now();
            pos_pub_.publish(target);
            ros::spinOnce();
            rate.sleep();
            if (fabs(pose_.pose.position.z - height) < 0.3) {
                ROS_INFO("[Teleop] 已到达目标高度!");
                break;
            }
            if (i % 30 == 0 && i > 0)
                ROS_INFO("[Teleop] 爬升中... %.1fm / %.1fm", pose_.pose.position.z, height);
        }

        // 设置位置保持
        hold_pos_ = target;
        hold_pos_.pose.position.x = pose_.pose.position.x;
        hold_pos_.pose.position.y = pose_.pose.position.y;
        hold_pos_.pose.position.z = height;
        hold_pos_.pose.orientation = pose_.pose.orientation;
        use_position_hold_ = true;
        hold_altitude_valid_ = false;
        vx_ = vy_ = vz_ = yr_ = 0.0;

        ROS_INFO("[Teleop] 起飞完成! 高度: %.1fm (位置保持)", pose_.pose.position.z);
    }

    void land()
    {
        ROS_INFO("[Teleop] 降落...");
        mavros_msgs::SetMode sm;
        sm.request.custom_mode = "AUTO.LAND";
        if (mode_client_.call(sm)) ROS_INFO("[Teleop] AUTO.LAND");
    }

    void run()
    {
        printf("\n==============================================\n");
        printf("   MAK4 无人机键盘遥控 (C++版)\n");
        printf("==============================================\n");
        printf("  T = 起飞    L = 降落\n");
        printf("  W=前进 S=后退 A=左移 D=右移\n");
        printf("  Q=左转 E=右转 R=上升 F=下降\n");
        printf("  空格=悬停  1/2=减速/加速  ESC=退出\n");
        printf("==============================================\n\n");

        // 设置终端raw模式
        tcgetattr(STDIN_FILENO, &old_settings);
        settings_saved = true;
        struct termios raw = old_settings;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        signal(SIGINT, restoreTerminal);
        signal(SIGTERM, restoreTerminal);

        ros::Rate rate(50);
        while (ros::ok()) {
            char key = getKey();
            if (key) {
                char k = tolower(key);
                switch (k) {
                case 'w': vx_ = linear_speed_; vy_ = 0; vz_ = 0; yr_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 's': vx_ = -linear_speed_; vy_ = 0; vz_ = 0; yr_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 'a': vy_ = linear_speed_; vx_ = 0; vz_ = 0; yr_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 'd': vy_ = -linear_speed_; vx_ = 0; vz_ = 0; yr_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 'q': yr_ = yaw_rate_val_; vx_ = 0; vy_ = 0; vz_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 'e': yr_ = -yaw_rate_val_; vx_ = 0; vy_ = 0; vz_ = 0;
                    if (!hold_altitude_valid_) { hold_altitude_ = pose_.pose.position.z; hold_altitude_valid_ = true; }
                    use_position_hold_ = false; break;
                case 'r': vz_ = vertical_speed_; vx_ = 0; vy_ = 0; yr_ = 0;
                    hold_altitude_valid_ = false; use_position_hold_ = false; break;
                case 'f': vz_ = -vertical_speed_; vx_ = 0; vy_ = 0; yr_ = 0;
                    hold_altitude_valid_ = false; use_position_hold_ = false; break;
                case ' ':
                    vx_ = vy_ = vz_ = yr_ = 0;
                    hold_altitude_valid_ = false;
                    hold_pos_.header.frame_id = "map";
                    hold_pos_.pose = pose_.pose;
                    use_position_hold_ = true;
                    break;
                case '1':
                    linear_speed_ = std::max(0.1, linear_speed_ - 0.1);
                    vertical_speed_ = std::max(0.1, vertical_speed_ - 0.1);
                    break;
                case '2':
                    linear_speed_ = std::min(2.0, linear_speed_ + 0.1);
                    vertical_speed_ = std::min(1.0, vertical_speed_ + 0.1);
                    break;
                case 't':
                    tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings);
                    takeoff(1.5);
                    raw = old_settings;
                    raw.c_lflag &= ~(ICANON | ECHO);
                    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
                    continue;
                case 'l':
                    tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings);
                    land();
                    raw = old_settings;
                    raw.c_lflag &= ~(ICANON | ECHO);
                    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
                    continue;
                case 27: case 3: // ESC or Ctrl+C
                    restoreTerminal(0);
                    return;
                }
            }

            if (state_.mode == "OFFBOARD") {
                if (use_position_hold_) {
                    hold_pos_.header.stamp = ros::Time::now();
                    pos_pub_.publish(hold_pos_);
                } else {
                    sendCommand();
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
        tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_, raw_pub_, pos_pub_;
    ros::Subscriber state_sub_, pose_sub_;
    ros::ServiceClient arm_client_, mode_client_;

    mavros_msgs::State state_;
    geometry_msgs::PoseStamped pose_, hold_pos_;

    double linear_speed_, vertical_speed_, yaw_rate_val_;
    double vx_, vy_, vz_, yr_;
    double hold_altitude_;
    bool use_position_hold_;
    bool hold_altitude_valid_;
};

int main(int argc, char **argv)
{
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "keyboard_teleop");
    KeyboardTeleop teleop;
    teleop.run();
    return 0;
}
