# RM_SIM

## 项目简介

本项目是为了26赛季的视觉组算法仿真而创建，包括但不限于麦克纳姆轮，全向轮，机械臂的仿真

## 环境要求

- Ubuntu 24.04
- Gazebo Sim Harmonic
- ROS 2 Jazzy

## 雷达仿真模式

本项目支持两种雷达仿真方案，可通过配置文件 `config/sim_config.yaml` 切换：

### 1. RGL模式 (需要NVIDIA显卡)

使用 [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) 进行高精度 Livox Mid360 仿真。

```yaml
lidar_mode: "rgl"
```

**要求**：
- NVIDIA显卡 + 驱动
- 编译安装RGLGazeboPlugin

### 2. gpu_lidar模式 (通用)

使用Gazebo内置的 `gpu_lidar` 传感器进行仿真，无需NVIDIA显卡。

```yaml
lidar_mode: "gpu_lidar"
```

### 3. 自动检测 (默认)

自动检测系统是否有NVIDIA显卡，有则使用RGL，否则使用gpu_lidar。

```yaml
lidar_mode: "auto"
```

## 快速开始

```bash
# 构建
colcon build

# 运行仿真
source install/setup.bash
ros2 launch rm_sim_26 rmuc_2025_sim.launch.py
```

当前仅维护 `rmuc_2025_sim.launch.py` 这一启动入口。

## 仓库结构

- `config/`: 仿真模式与桥接配置
- `launch/`: Gazebo / ROS 2 启动文件
- `models/`: 机器人与场地模型资源
- `plugin/`: Livox Mid360 雷达pattern文件
- `worlds/`: Gazebo world文件

## RGL模式准备

如需启用 `lidar_mode: "rgl"`，先初始化并编译RGL子模块：

```bash
git submodule update --init --recursive
```

## TODO

- ✅ 给机器人加入云台结构
- ✅ 机器人能够发布TF
- ✅ 加入比赛场地模型
- ✅ 实现Mid360的仿真
- ✅ 支持无NVIDIA显卡的仿真方案
- [ ] 修正每次启动的视角
- [ ] 修改车身模型以符合实际机器人

## 致谢

致敬[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)开发者的贡献
