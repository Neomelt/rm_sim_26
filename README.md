# RM_SIM

## 项目简介

本项目是为了26赛季的视觉组算法仿真而创建，包括但不限于麦克纳姆轮，全向轮，机械臂的仿真

## 环境要求

ubuntu24.04
gazebo sim Harmonic

## 依赖拉取

本项目使用子模块自动拉取插件依赖，首次克隆后执行：

git submodule update --init --recursive

## TODO

- ✅ 给机器人加入云台结构
- ✅ 机器人能够发布TF
- ✅ 加入比赛场地模型
- ✅ 实现Mid360的仿真
- [] 修正每次启动的视角
- [] 修改车身模型以符合实际机器人

## 致谢

致敬[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)开发者的贡献
