# 巡检无人机仿真

在工作空间根目录运行：

```bash
src/drone_stack/scripts/start_simulation.sh
```

该脚本启动 ROS 定位/规划/控制栈、PX4/Gazebo 和 Qt/RViz。`--no-gui` 可只启动后台仿真。按 Ctrl-C 会停止整组进程，包含后台 gzserver；禁止重复启动已有仿真。日志按运行时间保存在 `start/logs/`，最新一次为 `start/logs/latest/`。

默认使用 `ROS_MASTER_URI=http://127.0.0.1:11311` 和 `ROS_HOSTNAME=127.0.0.1`，避免系统 `.bashrc` 的 `d.local` 与节点地址混用。跨机器使用时，通过 `DRONE_ROS_MASTER_URI` 和 `DRONE_ROS_HOSTNAME` 显式指定地址。

## 数据链路

- Gazebo 雷达 → `/drone/sim/lidar/points` → Livox 适配 → `/livox/lidar` → Faster-LIO。
- Gazebo 雷达 IMU → `/drone/sim/lidar/imu_raw` → 启动稳定等待 → `/livox/imu` → Faster-LIO。
- Faster-LIO `/Odometry` → 雷达到机体中心外参和初始世界方向变换 → `/drone/lio/odom`、`/mavros/odometry/out`。
- PX4 `/mavros/local_position/odom` → EGO 与飞行管理器；桥接点云 `/drone/cloud_fcu_world` 使用同一数值坐标。
- `/drone/fcu/odom` 是用于 RViz 的 FCU 本地位姿，重标记为与上述点云一致的 `odom` 帧，避免 MAVROS `map` 标签造成显示层 TF 错误。

ROS 端数据为 ENU/FLU；MAVROS 负责 MAVLink 所需的坐标转换。PX4 参数 `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`，只融合外部位置、高度与航向，外部速度不参与融合。IMU 仍参与 EKF 状态传播和速度估计。

解锁阶段持续发送零速度目标；收到 PX4 状态中的实际 ARM 后才进入 ARMED。显式起飞指令使用 0.3 m/s 的位置目标爬升斜坡。悬停和导航阶段持续发送 30 Hz 目标。

## 仿真验证

验证工具只针对 `inspection_demo` 场景，会要求 Gazebo `inspection_quad` 真值话题存在。打开另一终端：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311
python3 src/drone_stack/scripts/validate_simulation.py takeoff
python3 src/drone_stack/scripts/validate_simulation.py hold_check
python3 src/drone_stack/scripts/validate_simulation.py dry
python3 src/drone_stack/scripts/validate_simulation.py flight
python3 src/drone_stack/scripts/validate_simulation.py land
```

先验证起飞与悬停，再检查 dry 报告中的地图地板和占据体素碰撞计数，检查通过后执行 flight。报告写入 `/tmp/drone_validation_*.json`；已保存的验证结果位于 `start/validation/`。

两个相机现在显示 `/drone/front/image_processed`、`/drone/down/image_processed`，包含仿真红色目标像素框。实际巡检缺陷检测模型尚未接入；此颜色检测仅验证处理画面链路。模拟雷达为规则射线与瞬时扫描，视场和安装倾角按 MID360 配置，不模拟其真实非重复扫描图案与全部硬件误差。

## 最新仿真验收

见 [SIM_ACCEPTANCE.md](SIM_ACCEPTANCE.md) 和工作空间 DRONE_SIM_PROGRESS.md。实验原始 bag、配置快照、逐项报告位于 start/experiments；有限场景通过不代表不存在所有缺陷。

仿真 MAVROS 使用 PASSTHROUGH，共享 Gazebo/PX4 lockstep 时钟。真实硬件必须使用正常 MAVLink 时间同步，不能照搬仿真配置。SITL 等待 65 秒仿真时间后才允许解锁，避免 PX4 同步收敛前使用消息到达时刻融合。

场景切换：`DRONE_SIM_WORLD=/绝对路径/inspection_corridor.world src/drone_stack/scripts/start_simulation.sh`；另有 inspection_3d.world 和 inspection_blocked.world。新场景尚待逐项验收。

自动冷启动工具 run_cold_start_tests.py 管理启动、原始遥测和退出；--fault-case 可选择雷达/IMU中断、撤销授权、退出OFFBOARD或低电量。故障项目必须实际执行通过后才算验收完成。


## 地图时间戳离线回归（2026-10-03）

EGO 地图消息保留实际生成地图的输入点云/深度图采集时间；重发不会改变时间。
导航目标检查和运行中的地图年龄保护均为2秒，过期停止规划、持续发送悬停目标。
以下检查不需要 ROS master，不能替代仿真飞行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make map_message_offline_test -j2 -l2
mkdir -p /tmp/drone_map_fixtures
devel/lib/plan_env/map_message_offline_test /tmp/drone_map_fixtures
DRONE_MAP_FIXTURE_DIR=/tmp/drone_map_fixtures python3 src/drone_stack/scripts/test_flight_safety.py
python3 src/drone_stack/scripts/analyse_bag_timing.py <telemetry.bag> <audit.json>
```

ROS/PX4/Gazebo 运行需要本地网络套接字；受限环境的启动失败必须与飞行失败区分。
具体进度和下一次冷启动命令见根目录 DRONE_SIM_PROGRESS.md。


## 执行保护与第一阶段自动验收

执行保护使用 `/grid_map/occupancy_inflate_safety` 完整高度体素；显示仍用原 occupancy_inflate。
命令必须在飞行范围内、采集时间有效、距当前机体不超过0.75 m，且整条跟踪线段不能穿过膨胀体素。
轨迹过期切悬停仍继续目标流；虚拟天花板2.6 m，目标高度上限2.5 m。
LIO桥位置/角度跳变上限分别为0.5+3*dt m和0.3+4*dt rad，拒绝异常位姿并锁定无效，落地后检查并重启桥接。
这些保护已离线验证，实际飞行效果仍待回归。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/drone_stack/scripts/test_sim_contracts.py
python3 src/drone_stack/scripts/test_regression_runner.py
python3 src/drone_stack/scripts/run_core_acceptance.py start/experiments/<新目录>
```

core入口执行三次导航冷启动和五种基础故障实验；每轮保存ULog并审计融合标志。
相同源码/参数/二进制可--resume，修改后必须新目录。该第一阶段通过不代表复杂场景、GUI和压力验收通过。
本地TCP/UDP套接字不可用时记录blocked_environment，未飞行，不能判定实机就绪。

## 仿真雷达安装位置

MID360 仿真传感器与独立 IMU 共用 `[0.27, 0, 0.10] m` 机体系平移，绕机体 Y 轴前倾30°。此位置用于把传感器放在机身前方，避免向下射线被机身盒体遮挡；真实硬件平移外参仍需要测量。模型生成器、雷达 IMU、静态 TF 和 LIO 位姿桥外参必须一致。

## 静态障碍记忆

无人机点云地图启用 `grid_map/retain_cloud_obstacles=true`。某帧缺少一个点并不能证明原位置为空，因此保留已观测静态占据，供盲区返程规划和执行保护使用；安全地图发布当前局部窗口所有记忆体素。没有自由空间射线清除模型，移动物体离开后会留下保守占据，重启地图清空。原点云输入新鲜度保护保持不变。

静态点云记忆在管理器首次确认 READY 时清除初始化阶段的地图后启用，避免 PX4 本地原点收敛期间的占据残留。后续 HOLD/READY 不再清空已扫描障碍；没有自由空间射线证据时仍保留静态占据。
