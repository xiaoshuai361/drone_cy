#!/bin/bash
# ============================================================
# MAK4 无人机仿真环境 - 一键验证脚本
# 功能：启动 PX4 SITL + Gazebo + MAVROS，然后自动起飞悬停
# ============================================================
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  MAK4 7寸四旋翼 PX4 SITL 仿真验证${NC}"
echo -e "${GREEN}============================================${NC}"

# 1. 检查环境
echo -e "\n${YELLOW}[1/4] 检查环境...${NC}"

if ! rosversion -d >/dev/null 2>&1; then
    echo -e "${RED}错误: ROS 未安装或未 source${NC}"
    exit 1
fi
echo "  ROS: $(rosversion -d)"

if ! command -v gazebo &>/dev/null; then
    echo -e "${RED}错误: Gazebo 未安装${NC}"
    exit 1
fi
echo "  Gazebo: $(gazebo --version 2>&1 | head -1)"

if ! rospack find mavros &>/dev/null; then
    echo -e "${RED}错误: MAVROS 未安装${NC}"
    exit 1
fi
echo "  MAVROS: OK"

if [ ! -f "$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]; then
    echo -e "${RED}错误: PX4 SITL 未编译，请先执行:${NC}"
    echo "  cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo-classic"
    exit 1
fi
echo "  PX4 SITL: OK"

# 2. 启动仿真环境 (PX4 + Gazebo + MAVROS)
echo -e "\n${YELLOW}[2/4] 启动仿真环境 (PX4 SITL + Gazebo + MAVROS)...${NC}"
echo "  这可能需要一些时间，请等待 Gazebo 窗口出现..."

roslaunch mak4_sim mak4_sitl.launch gui:=true &
LAUNCH_PID=$!

# 等待 MAVROS 连接就绪
echo -e "\n${YELLOW}[3/4] 等待飞控连接...${NC}"
TIMEOUT=120
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if rostopic echo /mavros/state -n1 2>/dev/null | grep -q "connected: True"; then
        echo -e "  ${GREEN}飞控已连接!${NC}"
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        echo "  等待中... ($ELAPSED/$TIMEOUT 秒)"
    fi
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo -e "${RED}错误: 飞控连接超时${NC}"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# 3. 执行起飞悬停测试
echo -e "\n${YELLOW}[4/4] 执行起飞悬停测试...${NC}"
sleep 5
rosrun mak4_sim takeoff_hover_test.py

# 清理
echo -e "\n${GREEN}仿真验证完成! 按 Ctrl+C 关闭所有进程${NC}"
wait $LAUNCH_PID
