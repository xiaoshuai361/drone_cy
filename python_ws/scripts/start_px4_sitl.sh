#!/bin/bash
# 启动 PX4 SITL 的辅助脚本
PX4_DIR=$1
PX4_BUILD_DIR=$2

# 设置 PX4 环境
cd "$PX4_DIR" || exit 1

# 设置必要的环境变量
source "$PX4_BUILD_DIR/etc/init.d-posix/rcS" "$PX4_BUILD_DIR" "$PX4_BUILD_DIR/etc" 2>/dev/null || true
source Tools/simulation/gazebo-classic/setup_gazebo.bash "$PX4_DIR" "$PX4_BUILD_DIR" 2>/dev/null || true

# 将 Gazebo 模型路径和插件路径导出
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PX4_BUILD_DIR}/build_gazebo-classic
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PX4_BUILD_DIR}/build_gazebo-classic

# 启动 PX4 SITL
exec "$PX4_BUILD_DIR/bin/px4" "$PX4_BUILD_DIR/etc" -s etc/init.d-posix/rcS -t "$PX4_DIR/test_data"
