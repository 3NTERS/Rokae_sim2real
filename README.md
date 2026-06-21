# Rokae_sim2real

这是一个面向 Rokae 机械臂的 ROS 2 / 仿真到真实部署相关工程仓库，包含标定、控制、调试和使用说明等内容。

## 项目简介

该仓库当前主要聚焦于：
- 机械臂的 ROS 2 相关控制与交互
- 相机与机械臂之间的手眼标定流程
- 仿真与真实系统之间的联调与验证
- 相关调试记录与使用说明

## 仓库结构

- `rokae_ros2_sim2real/`：主工程目录，包含实际使用的 ROS 2 工作空间与相关说明
  - `Readme.md`：项目的主要使用说明与标定流程
  - `ros_note.md`：常用命令与调试笔记

## 快速开始

1. 进入项目目录：
   ```bash
   cd rokae_ros2_sim2real
   ```
2. 编译工作空间：
   ```bash
   colcon build
   ```
3. 加载环境变量：
   ```bash
   source install/setup.bash
   ```
4. 根据需要启动相关功能，例如标定或发布控制命令。

## 主要文档

- [rokae_ros2_sim2real/Readme.md](rokae_ros2_sim2real/Readme.md)
- [rokae_ros2_sim2real/ros_note.md](rokae_ros2_sim2real/ros_note.md)

## 说明

如果你是第一次使用这个仓库，建议先阅读 `rokae_ros2_sim2real/Readme.md`，其中包含了标定流程与常见问题说明。

如果你在使用过程中遇到问题，可以结合 `ros_note.md` 中的命令与步骤进行排查。
