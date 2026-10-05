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
轨迹过期切悬停仍继续目标流；当前可调虚拟天花板默认2.5 m（ENU Z），目标与轨迹保留0.20 m高度裕量。
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

## 完整曲线执行门控

管理器先验证 `/planning/bspline` 的整条三次曲线，再接受该曲线 ID 的 `/planning/pos_cmd`。逐段转换为 Bezier 曲线并递归收缩控制点包络，检查整个连续曲线是否越过飞行体积或触碰膨胀占据体素；原有当前目标、跟踪距离、速度和地图时间保护仍逐次执行。非法输入、计算预算耗尽或几何不安全都会取消导航并持续悬停目标流。

曲线与 `/clock` 可能经不同连接先后到达。稍早到达的曲线只完成几何检查并等待激活，时钟到达曲线起始时刻且地图仍新鲜才开放执行；不能用未来数据提前控制。新目标代次会清除待激活曲线。

完整去畸变点云与桥接配套机体位姿使用相同消息时间，由 EGO ExactTime 配对；桥接位姿来自采集时刻附近的 FCU 样本。地图按对应雷达原点到返回点之间的射线记录自由体素，返回后空间保持未知。当前机身0.47×0.47×0.11m所占体素标记为自身空间，已知膨胀占据始终优先；首次READY同时清除初始化占据和自由记录。

严格自由空间模式可通过 `require_observed_free=true` 启用：目标可以先排队，EGO可以生成未来候选路线；整曲线仍检查已知碰撞与体积边界，接下来3秒的执行前瞻及当前目标线段必须处于已知自由/自身空间。前瞻每0.1秒随指令重检，验证在控制锁外，未知尾段不提前执行。无顶场景的向上无返回不能证明自由空间，这个严格模式目前会阻止部分抬高目标；仿真默认关闭，仍记录自由地图。默认新增点面法向位置观测约束检查，弱方向通过高不确定度协方差使桥接停止EV并锁定，随后保护降落。阈值和飞行尚在验证，当前完整回归未通过。实机外参、采集时序和现场能力另见 `HARDWARE_ACCEPTANCE.md`。

### 连续曲线碰撞检查（2026-10-03）

EGO优化、时间细化与最终发布前采用逐个三次Bezier控制凸包递归检查完整曲线，闭合体素面也判相交；另加5cm规划储备。占据或未能在有界预算内证明安全时重新优化/规划，不发布候选曲线。管理器仍独立检查完整占据碰撞和执行体积。修复原上游前2/3稀疏采样漏检；核心第3轮实际失败曲线已保存为离线回归fixture。当前新版本完整飞行回归尚在进行。


## 连续朝向、虚拟天花板与多点队列（2026-10-05）

- **朝向**：起步先悬停对准。移动中以实际获准执行的 EGO B 样条的有向 XY 切线为目标，按最多30°/s连续调整机头；不使用位置跟踪误差或终点直线决定朝向。切线与实际机头或当前指令朝向超出±60°时仍先悬停对准，低水平速度/纯垂直路段保持上一朝向。
- **最高高度**：Qt右侧“最高Z（ENU）设置”，范围0.8～2.5 m。它限制机体中心在当前 ROS ENU 坐标中的绝对 Z，和“点选目标绝对 Z”使用同一基准，不是距地高度或相对起飞高度。目标、起飞终点、全局参考和获准执行的整段 EGO 曲线不得高于设置值减0.20 m。EGO地图中的虚拟天花板同步调整，超高0.05 m触发降落保护；这不能保证有惯性/估计误差的实际飞机绝不瞬间越界。
- **修改条件**：未解锁且已确认地面，或健康 OFFBOARD HOLD 且没有排队目标。空中不能把上限降到当前高度加0.20 m以下；导航中先按“悬停”取消队列，再修改高度。启动参数为 `roslaunch drone_stack stack.launch max_flight_height:=1.5`，根参数 `/drone/max_flight_height_m`；Qt修改在本次运行有效，重新启动仍采用启动参数。
- **连续点选**：起飞进入HOLD后点击“点选XY目标”，可连续左键点击，滚轮缩放；再次点“结束连续点选”仅结束鼠标点选，已有任务继续。第一个点立即启动，其余点以红色实心圆和剩余顺序编号排队，最多100点（含当前）。相对目标按钮也加入同一队列。只显示并控制当前目标的黄色全局路线和紫色 EGO 曲线，绿色轨迹在整次队列任务中累积，最后一项完成后清空。
- **顺序执行**：上一点进入15 cm到达容差、速度不高于0.15 m/s并稳定1 s后完成；下一点重新检查最新地图并开始规划，没有后台预先搜索后续点。已知膨胀障碍内的点拒绝加入，未知区域仍可加入。若排队期间下一点变成已知障碍，取消剩余任务并在HOLD报告原因。
- **任务取消**：悬停、取消目标、降落、撤销授权触发的保护、健康异常取消导航均清空剩余队列；不会在恢复后自行继续原任务。短暂健康警告沿用原保护时序。

接口：`/drone/queue_goal`（`drone_stack/QueueGoal`），`/drone/goal_queue`（latched `nav_msgs/Path`，仅保存目标点，不是规划路线），`/drone/set_max_flight_height`（`drone_stack/SetMaxFlightHeight`），`/drone/max_flight_height`（latched `std_msgs/Float64`）。原 `/drone/local_goal` 保留单目标接口，调用会替换/清空待执行队列。

## 终点接近与收敛（2026-10-05）

- 沿当前全局曲线的剩余弧长进入接近阶段，默认0.8 m；高速时按制动距离扩大。实际EGO控制曲线重新规划减速，末段目标速度0.2 m/s，保留当前起始速度/加速度边界。末端精确到目标，期望速度与加速度为零；时间调整后再次检查物理可行性和完整碰撞。
- 终点短轨迹最小长度降至0.02 m，消除原20 cm规划下限与15 cm到达容差之间的空档。不满足可行性或安全条件的短轨迹仍拒绝。
- 到达条件统一覆盖规划、等待、转向与运动：XYZ三维球半径15 cm、速度不高于0.15 m/s、连续稳定1 s。确认期间取消旧规划回调影响，持续发送测得的到达位置/实际朝向悬停目标；离开容差后重新规划。
- 合格的零速度终点允许最多3 s位置收敛。每个当前机体到终点的小跟踪段仍检查障碍、飞行范围及启用时的已观测自由空间；超过范围或视野不能直接修正。收敛超时再申请新规划，安全保护仍生效。
- `/drone/navigation_terminal_stage`为独立状态话题：IDLE、CRUISE、APPROACH、CONVERGING、ARRIVAL_CONFIRM、COMPLETED。共享参数为`/drone/terminal_approach_distance_m`、`terminal_speed_mps`、`terminal_deceleration_mps2`、`terminal_settle_time_s`（后三项同一`/drone/`命名空间）。紫线仍来自真实批准控制样条。

本轮完成源码与启动检查后仍需手动飞行验证；绕障急弯、视野限制、规划检查超预算等仍可能要求悬停或取消，不能据此宣称所有终点问题已经通过飞行验收。


## 地图保存、加载与导航模式

Qt右侧新增“建图与预建地图导航”：

1. **开始新建图**：地面未解锁、READY且健康时清除上一张障碍/自由地图。随后按原流程起飞，手动扫描或依次导航采集；实时EGO地图始终更新。
2. **保存地图**：扫描后降落上锁，在READY状态保存`.dmap`文件，默认目录`start/maps/`。保存全部记忆范围内的原始占据证据与已观测自由体素，不存膨胀层或虚拟天花板。文件写入临时文件再替换目标文件。
3. **加载地图导航**：地面READY且健康、没有目标队列时加载已有地图；校验版本、ENU坐标系、分辨率、网格范围与全部记录，失败保持现有地图。加载后按当前机体膨胀参数重建安全层，同时供黄色全局规划、EGO实际控制和管理器碰撞/自由空间检查使用；雷达继续更新、确认新障碍并沿自由射线清除旧占据。
4. **返回在线建图导航**：清除预建地图，从当前扫描重新建立地图；不会保留预建地图的未知区域占据。

地图坐标变换为`p_current = Rz(yaw) * p_saved + [dx,dy,dz]`，输入在Qt中显示为“地图→当前ENU”。相同仿真场景、相同起飞位置及朝向使用零偏移。换起点或换坐标原点时必须正确对齐；本功能不做自动重定位，也不把预建地图送给Faster-LIO匹配。占据体素在旋转/非网格平移后按变换后的体素包络保守重栅格化；历史自由空间仅在零yaw、整格平移时导入，其他变换需由实时雷达重新证明自由。

存图和加载只开放落地未解锁状态。文件包括原始地图、历史自由信息与网格元数据，加载不绕过实时定位健康门、OFFBOARD目标流或避障保护。预建地图不会改善雷达当下的位置观测约束不足问题。

接口：`/drone/map_archive`（`plan_env/MapArchive`，action为`new/save/load/online`，path绝对路径及ENU变换），latched `/drone/map_mode`（MAPPING/PRIOR_NAV）、`/drone/map_status`。地图模式显示在Qt右侧，体素仍来自参与真实规划的同一EGO地图。

全局A*上限15000节点，25ms CPU分批执行；同一目标的PLANNING/SPACE_WAIT转换保留搜索状态，目标变化、飞行范围变化或起点显著移动仍使旧搜索失效。管理器完整全局路径等待45秒墙钟、ACK总窗口50秒墙钟，连续30Hz悬停指令保持；没有安全路线时不发布显示专用路径。

### 管理器曲线校验墙钟预算（2026-10-05）

完整EGO曲线与执行前瞻的默认墙钟上限调整为2.0 s；线程CPU预算仍为0.05 s、递归节点上限仍为10000。检查在控制锁外进行，目标流继续；轨迹时间、地图新鲜度、碰撞检查和任务代次仍按原保护判定。全局参考平滑调用显式设置的0.15/0.5 s预算保持。Python已运行进程需重新启动才能使用新的默认值。
