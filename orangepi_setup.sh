#!/bin/bash
# =============================================================================
#   MAK4 仿真环境配置脚本 —— 香橙派 5 Pro (RK3588, ARM64, Ubuntu 20.04)
#   前提: 已通过 fishros.com 安装 ROS Noetic, 网络畅通, 已换国内源
#
#   用法:
#     chmod +x orangepi_setup.sh
#     可整体执行: bash orangepi_setup.sh
#     也可按 "==== 第X步 ====" 分段逐步执行 (推荐, 便于排查错误)
#
#   代码来源 (全部从 GitHub clone, 无需 U 盘):
#     用户代码:   https://github.com/xiaoshuai361/drone_cy.git
#     FAST_LIO:   https://github.com/hku-mars/FAST_LIO.git
#     livox驱动:  https://github.com/Livox-SDK/livox_ros_driver.git
#     PX4 v1.14.3: https://github.com/PX4/PX4-Autopilot.git (tag v1.14.3)
# =============================================================================
set -e  # 遇到错误立即停止 (整体运行时有效; 分段时请自行注意报错)

# ─────────────────────────────────────────────────────────────────────────────
# 第1步: 更新系统 & 安装基础工具
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第1步: 更新系统 & 基础工具 ===="
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential cmake cmake-curses-gui ninja-build \
    git wget curl vim htop net-tools \
    python3-pip python3-dev python3-setuptools python3-wheel \
    python-is-python3 \
    lsb-release gnupg2 \
    openssh-server \
    pkg-config libfuse2 \
    astyle cppcheck file gdb \
    libxml2-dev libxml2-utils

# 设置时区
sudo timedatectl set-timezone Asia/Shanghai

echo "==== 第1步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第2步: 安装 Gazebo 11 (若 fishros 已装 ros-noetic-desktop-full 则会跳过)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第2步: 安装 Gazebo 11 ===="
sudo apt install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-dev \
    ros-noetic-gazebo-msgs \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-urdf \
    ros-noetic-rviz

echo "==== 第2步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第3步: 安装 MAVROS + GeographicLib 数据集
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第3步: 安装 MAVROS ===="
sudo apt install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    ros-noetic-mavlink \
    ros-noetic-libmavconn \
    ros-noetic-geographic-msgs \
    geographiclib-tools

# 安装 GeographicLib 数据集 (MAVROS 必须, 约 60 MB, 需要访问外网)
# 如果外网慢, 可手动下载后离线安装 (见脚本末尾说明)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

echo "==== 第3步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第4步: 安装 PCL + OctoMap + Velodyne + 其他 ROS 感知包
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第4步: 安装感知相关 ROS 包 ===="
sudo apt install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-msgs \
    ros-noetic-perception-pcl \
    libpcl-dev \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-server \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-octomap-mapping \
    liboctomap-dev \
    ros-noetic-velodyne-description \
    ros-noetic-velodyne-driver \
    ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-velodyne-laserscan \
    ros-noetic-velodyne-msgs \
    ros-noetic-velodyne-pointcloud \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-tf2-eigen \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-visualization-msgs \
    libyaml-cpp-dev \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv

echo "==== 第4步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第5步: 安装 Python 依赖 (ROS 节点 + PX4 构建工具)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第5步: 安装 Python 依赖 ===="
pip3 install --user \
    numpy scipy \
    PyYAML \
    empy==3.3.4 \
    jinja2 \
    pyserial \
    pymavlink \
    future \
    cerberus \
    coverage \
    argcomplete \
    packaging \
    toml \
    sympy \
    lxml \
    matplotlib \
    pandas \
    psutil \
    pyulog \
    kconfiglib \
    nunavut \
    pyros-genmsg

echo "==== 第5步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第6步: 安装 PX4 SITL 编译依赖 (Gazebo-classic 仿真)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第6步: 安装 PX4 编译依赖 ===="
sudo apt install -y \
    ant \
    genromfs \
    xmlstarlet \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libopenni2-dev \
    libprotobuf-dev protobuf-compiler \
    libeigen3-dev \
    libsdl2-dev

# 将当前用户加入 dialout 组 (串口权限, 需重新登录生效)
sudo usermod -aG dialout $USER

echo "==== 第6步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第7步: 安装 rosdep / catkin 工具
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第7步: 安装 rosdep & catkin 工具 ===="
sudo apt install -y \
    python3-rosdep \
    python3-wstool \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-catkin-tools \
    python3-osrf-pycommon

# 初始化 rosdep (若已初始化会报错, 可忽略)
sudo rosdep init 2>/dev/null || true
rosdep update

echo "==== 第7步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第8步: 从 GitHub 克隆所有代码仓库  (网络不好时此步耗时较长)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第8步: 克隆代码仓库 ===="

# 创建工作空间目录
mkdir -p ~/catkin_ws/src ~/catkin_ws/maps
mkdir -p ~/catkin_ws_cpp/src

# --- 8.1 克隆用户自定义代码 ---
git clone https://github.com/xiaoshuai361/drone_cy.git ~/mak4_drone_sim

# 将 python_ws 和 cpp_ws 复制到对应工作空间的 src 目录
cp -r ~/mak4_drone_sim/python_ws ~/catkin_ws/src/mak4_sim
cp -r ~/mak4_drone_sim/cpp_ws    ~/catkin_ws_cpp/src/mak4_sim_cpp

# --- 8.2 克隆 FAST_LIO (两个工作空间各需要一份) ---
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO && git submodule update --init && cd ..

cd ~/catkin_ws_cpp/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO && git submodule update --init && cd ..

# --- 8.3 克隆 livox_ros_driver (两个工作空间各需要一份) ---
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git

cd ~/catkin_ws_cpp/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git

# --- 8.4 克隆 PX4-Autopilot v1.14.3 ---
git clone https://github.com/PX4/PX4-Autopilot.git \
    --branch v1.14.3 ~/PX4-Autopilot
cd ~/PX4-Autopilot
git submodule update --init --recursive
# 注意: PX4 子模块很多, 此步需要较长时间且需要访问 GitHub

echo "==== 第8步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第9步: 编译 PX4 SITL (ARM64 原生编译, 约 20~40 分钟)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第9步: 编译 PX4 SITL ===="
cd ~/PX4-Autopilot

# 编译 PX4 SITL + Gazebo classic (iris 模型)
# -j3: 限制并发数避免 OOM (Orange Pi 5 Pro 内存 16GB 可用 -j4)
DONT_RUN=1 make px4_sitl_default gazebo-classic -j3

echo "==== 第9步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第10步: 编译 catkin_ws (Python 工作空间: mak4_sim + FAST_LIO + livox_ros_driver)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第10步: 编译 catkin_ws (Python 工作空间) ===="
cd ~/catkin_ws

# ARM64 上并发数限制为 2~3, 避免 OOM
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release

echo "==== 第10步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第11步: 编译 catkin_ws_cpp (C++ 工作空间: mak4_sim_cpp)
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第11步: 编译 catkin_ws_cpp (C++ 工作空间) ===="
cd ~/catkin_ws_cpp

catkin_make -j2 -DCMAKE_BUILD_TYPE=Release

echo "==== 第11步完成 ===="

# ─────────────────────────────────────────────────────────────────────────────
# 第12步: 配置 ~/.bashrc
# ─────────────────────────────────────────────────────────────────────────────
echo "==== 第12步: 写入 ~/.bashrc ===="

# 检查是否已配置, 避免重复写入
if grep -q "MAK4_ENV_CONFIGURED" ~/.bashrc; then
    echo "[跳过] ~/.bashrc 已配置过 MAK4 环境"
else
    cat >> ~/.bashrc << 'BASHRC_EOF'

# ===== MAK4 仿真环境 (由 orangepi_setup.sh 写入) =====
# MAK4_ENV_CONFIGURED

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export PATH=$HOME/.local/bin:$PATH
export PX4_HOME=$HOME/PX4-Autopilot

setup_px4_env() {
  source $PX4_HOME/Tools/simulation/gazebo-classic/setup_gazebo.bash \
    $PX4_HOME $PX4_HOME/build/px4_sitl_default 2>/dev/null
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_HOME:$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic
}

setup_px4_env

alias use_cpp='source ~/catkin_ws_cpp/devel/setup.bash && setup_px4_env && echo "[OK] C++工作空间 + PX4 已加载"'
alias use_py='source ~/catkin_ws/devel/setup.bash && setup_px4_env && echo "[OK] Python工作空间 + PX4 已加载"'

# ===== MAK4 END =====
BASHRC_EOF
    echo "[完成] ~/.bashrc 已写入 MAK4 环境配置"
fi

echo "==== 第12步完成 ===="

echo ""
echo "============================================================"
echo "  全部配置完成! 请执行以下操作:"
echo "  1) source ~/.bashrc  (或重新开终端)"
echo "  2) 参考说明文件了解 GPU Ray 问题的解决方案"
echo "  3) 仿真生成的地图将存放在 ~/catkin_ws/maps/"
echo "============================================================"
