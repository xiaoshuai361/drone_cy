# MAK4 无人机仿真 & 自主巡检系统

PX4 SITL + ROS Noetic + Gazebo 温室/森林仿真，含 SLAM 建图、路径规划、自主巡检全套功能。  
提供 **Python 版** 和 **C++ 高性能版** 两套完整实现。

## 功能

- 🏗 Gazebo 仿真环境 (20m×8m 温室 / 60m×60m 森林)
- 🗺 FAST-LIO2 3D SLAM 建图 → OctoMap / 2D栅格
- 🛫 键盘遥控飞行 (混合控制: 速度XY + 位置Z)
- 📍 交互式航点巡检 (RViz 点击 → A* 路径规划 → 自动飞行)
- 🔄 动态避障 (实时点云 → 局部代价地图 → 自动重规划)
- 🌲 RRT* 3D 局部避障
- 📐 弓字形全覆盖路径规划

## 目录结构

```
mak4_drone_sim/
├── python_ws/          # Python 版功能包 (mak4_sim)
│   ├── scripts/        # Python 节点
│   ├── launch/         # Launch 文件
│   ├── worlds/         # Gazebo 世界文件
│   ├── models/         # Gazebo 模型
│   ├── config/         # RViz 配置
│   └── docs/           # HTML 文档
├── cpp_ws/             # C++ 高性能版功能包 (mak4_sim_cpp)
│   ├── src/            # C++ 源码
│   ├── launch/         # Launch 文件
│   ├── worlds/         # Gazebo 世界文件
│   └── models/         # Gazebo 模型
├── docs/               # 指令清单、真实飞行指南等
└── README.md
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- PX4 v1.14.3
- Gazebo 11
- FAST-LIO2, livox_ros_driver

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/xiaoshuai361/drone_cy.git

# 2. 复制功能包到 catkin 工作空间
cp -r drone_cy/python_ws ~/catkin_ws/src/mak4_sim
cp -r drone_cy/cpp_ws ~/catkin_ws/src/mak4_sim_cpp

# 3. 编译
cd ~/catkin_ws && catkin_make

# 4. 启动仿真 (Python版)
roslaunch mak4_sim mak4_greenhouse.launch

# 5. 启动导航 (C++版)
roslaunch mak4_sim_cpp navigation_2d.launch
```

## 文档

- 📖 详细操作指南: `docs/指令清单.txt`
- 🚁 真实飞行测试: `docs/真实启动.txt`
- 🌐 HTML 学习文档: `python_ws/docs/ros_framework_guide.html`

## 许可

本项目仅用于学习和研究目的。
