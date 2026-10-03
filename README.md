# 规划四旋翼巡检项目

四旋翼仿真与自主巡检工作区，包含 ROS 无人机控制、路径规划、激光雷达里程计、PX4 SITL 仿真及巡检验证工具。

## 目录

- `src/`：ROS 工作空间源码，包括控制、规划和传感器驱动。
- `external/PX4-Autopilot/`：PX4 仿真源码及本项目的仿真配置修改。
- `external/px4_python_deps/`：仿真所需 Python 依赖。
- `start/`：启动、验证和分析工具。
- `DRONE_SIM_PROGRESS.md`：仿真与项目进度记录。

构建目录、运行日志、实验录包和检查点属于本地生成数据，由 `.gitignore` 排除。
