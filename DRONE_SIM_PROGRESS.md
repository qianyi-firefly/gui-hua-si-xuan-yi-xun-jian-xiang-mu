# 巡检四旋翼仿真进度

更新时间：2026-10-03 14:58 CST。ROS模型生成/超时清理修复及低电量短复测通过；新core_live_17与extended_live_05监督器已启动。完整扩展未通过前不能实机验收。

## 最新运行状态（优先于下方历史记录）

- 最新主会话18277 / 监督器54549：core_live_17新版本完整核心运行中，通过后执行extended_live_05全部17项。生产指纹ec4b6f871ac03d373f0ab5f383a90eafc7420f0ced97be7c1c7a4570ac750def。唯一仿真，源码与工具冻结。
- ros_spawn_low_battery_short会话53154整轮通过，飞前只有20s悬停；低电量3.160s保护、反升0.002569m、漂移0.049623m、最终上锁，ULog通过，不代替完整60s回归。
- ROS生成分支成功/明确失败/挂起三个实际shell分支检查通过（模拟ROS命令，测试超时缩短）；失败后TERM、2s后KILL并wait，防止世界进程令清理无限等待。prepare最终幂等与bash语法通过，24项配置与54项保护通过，ros_model_spawn_fix证据已保存。
- 新实机交接文件src/drone_stack/HARDWARE_ACCEPTANCE.md，记录实机外参、时钟同步、真实点云/IMU格式、电池补偿和动力控制等必需差异，仍标未验证。

- 主会话22339已失败退出，监督器53479因生产变化退出；extended_live_05未启动。low_battery_attempt_01世界在跑但模型缺失，gz model发现/生成请求无响应，PX4未启动。无飞行，ground_safety和land均失败，证据保留。已TERM停止拥有该世界的启动器，ROS/Gazebo已退出。
- 生产prepare_px4_sitl对inspection_quad/ROS1使用timeout60s的gazebo_ros spawn_model服务调用，失败停止世界并退出；其它模型保持上游路径。验收timeout若无无人机记录模型名而不再次抛ValueError。prepare两次幂等和bash语法通过，24模型/54保护复核中；当前ros_spawn_low_battery_short仅20s悬停，是诊断不是完整回归。
- live_16已通过：雷达0.524s/无反升/漂移0.012413m；雷达IMU0.768s/无反升/漂移0.026012m；授权撤销0.014s/无反升/漂移0.019281m；OFFBOARD原生0.608s/无反升/漂移0.027960m。整轮仍失败，不能宣称完整核心通过。

- 当前主会话22339 / 监督器53479；生产d205216d...未变。core_live_16三轮整轮导航和ULog通过，三轮包络通过，最大跟踪0.176860/0.180261/0.193264m。前后60s Z范围：0.087548/0.107748、0.089866/0.109454、0.087303/0.109898m。当前fault_lidar_attempt_01，17项扩展仍待核心结束。不可修改源码或工具/启动第二套仿真。

- ready_gate_scenes_01两场景完整通过，会话47676结束，无残留仿真。走廊28111真值样本、零包络重叠、最小间隙0.213581m、跟踪最大0.181671m；三维零重叠、最小间隙0.140671m、跟踪最大0.228032m，融合/降落上锁通过。修复前后地图切片已保存cloud_memory_ready_fix/ready_gate_occupancy_comparison.png并查看。
- 最新唯一飞行是core_live_16，生产指纹d205216d4f5c080ab741ed31f336f924f35e1630d8e30e37b3a505022695d35d；监督器等待同版本核心全部通过后执行extended_live_05全部17项。源码与工具冻结，实机准备仍false。

- 新构建生产指纹d205216d4f5c080ab741ed31f336f924f35e1630d8e30e37b3a505022695d35d；构建和24项模型/54项保护/12项工具检查通过。真实EGO回调ready_gate通过，证明首次READY清初始化残留，之后HOLD/READY保留盲区旧障碍。实验工具默认无门控参数时仍可对照retain/replace。
- 当前唯一在线会话47676：ready_gate_scenes_01顺序corridor/three_d；走廊场景导航往返已通过，正在降落及整轮审计。禁止同时启动另一套仿真或修改源码/工具。完成两场景后执行core_live_16与extended_live_05全部17项。
- core_live_15三轮导航包络审计全部通过，最大跟踪0.192850/0.132558/0.174318m；optimization_comparison图表已生成。旧通过只代表旧版本。

- 2026-10-03恢复核对：原主会话13796/监督器35395均已结束，无残留ROS/PX4/Gazebo。core_live_15整轮通过，原生产指纹4e9644da...；extended_live_04/corridor首次目标无轨迹，5秒触发EGO-Planner trajectory timed out，地图在82.654s/FCU高度0.897m存在入口与中心虚假占据。失败bag/ULog/几何通过记录保留。
- 当前正在增加地图初始化门控：drone配置订阅flight_state，首个READY清除初始化阶段地图并开始永久静态记忆；之前按当帧替换，之后HOLD/READY不得清除。其它上游应用无门控参数时保持原行为。生产已变化，旧核心通过不能代表新配置全回归通过。

- **地图记忆修复后的三维短复测会话32143已整轮通过并退出。新完整核心`20261003_core_live_15`和监督器`run_sim_campaign.py ... extended_live_04`已启动（exec会话见最新调用）；生产版本4e9644da3bb8b8000095183bf91bc39d0ac1cdad2837c6b1631df21748cce489，当前核心首轮。禁止第二组仿真与源码变更；核心全过后监督器重跑全部17项扩展。**
- 新安装位置输入诊断对照图已保存head_lidar_fix/head_mount_ground.png与ground_visibility_comparison.json：飞行真值高度≥1.5m的68个扫描，地面点至少1428、中央値1773；LIO与真值高度诊断残差最大0.05708m。旧中央安装的同类扫描中央値12、最小0，最大残差2.414m；这是近似时间配对诊断，不代替验收。
- 新地图记忆三维复测`20261003_cloud_memory_three_d_01`整轮通过：四目标/各20s悬停/降落上锁/融合审计通过；44076个真值样本零包络重叠，最小分离轴间隙0.121477m（上方梁），最大跟踪偏差0.207185m，真值最大间隔0.052s。旧缺陷已实测修复，但新配置完整核心与全部扩展仍待结果。
- live_03三维复测前三目标/悬停通过，但低位返程遭轨迹跳变保护，且包络审计有1219个low_barrier重叠样本，不能只报“保护有效”。根因：EGO cloudCallback每帧resetBuffer清掉旧障碍；已经扫描的低障碍转到前倾雷达后方盲区后被遗忘。旧失败bag/ULog/GUI完整保留，最终清理降落通过。
- 修复EGO：可选retain_cloud_obstacles，drone配置true，缺少自由空间证据时保留静态占据；publish安全地图包含当前局部窗口所有已记忆障碍，不因当帧点云边界窄而遗漏。上游默认false，当前没有自由空间射线清除/动态障碍移除模型，重启地图清空。地图freshness仍来自实际新点云，未改变超时和膨胀/跟踪门槛。
- 实际EGO回调两模式对照（无飞行）均通过：retain保留旧障碍，replace删除旧障碍；旧时间戳和空点云不刷新地图。固定构建指纹一致；23项配置/模型与54项保护检查通过。完整编译完成。
- 扩展live_02：multi_goal/corridor整轮通过；three_d第二高位目标近上方梁体膨胀体素，触发保护HOLD但误被工具以goal_error<0.2认作到达，后悬停XY=0.205m/LIO误差0.113m超限，整轮失败保留。降落/融合/几何通过，不能把保护取消当成功。
- 工具修复：记录flight_error事件序号，到达需HOLD/OFFBOARD并无本目标的新保护原因，等待0.25s跨话题原因传递；12项检查通过。三维测试高位路线改[.5,0,.45]、[2.7,0,.45]后再低位穿梁，场景/障碍/保护门槛未改变。当前15项使用新工具快照，生产版本未变。
- live_14基础故障全部通过并有ULog：雷达0.554s、反升0.004723m、漂移0.018519m；雷达IMU0.720s、无反升、漂移0.008984m；授权撤销同一个仿真时间刻保护、漂移0.020513m；原生OFFBOARD失流0.544s、无反升、漂移0.034654m；低电量2.494s、反升0.007347m、漂移0.038644m。最终均上锁。核心status=core_passed_remaining_acceptance_pending。
- live_14三轮完整导航和ULog均通过；三轮前后60s Z峰峰值0.085992/0.103084、0.086698/0.107802、0.082995/0.108825m，LIO最大位移误差均≤0.015240m。前两轮全场景包络审计通过，最大跟踪偏差0.190024/0.183260m；第三轮包络审计也通过，最大跟踪偏差0.189507m；图表已完成。五种基础故障现已通过，扩展仍待结果。
- live_14 round_01完整导航/降落/ULog通过：前后60s悬停Z范围0.085992/0.103084m，LIO位移误差0.009681/0.014077m；全场景包络审计另行进行。
- 多高度短复测head_lidar_multi_goal_02：五目标/每次20s悬停/降落/融合均通过，悬停Z范围0.044282～0.086289m、LIO误差最大0.022353m；落地漂移0.038519m。但额外原始传感器密集录制时真值最大记录间隔0.112s超过0.1s，整轮严格保留失败。工具增加--diagnostic-sensors选项，标准验收用原轻量记录量，未放宽几何门槛。
- head_lidar_multi_goal_01因测试服务响应和相位话题不同连接，旧HOLD被误判取消；bag证明管理器从NAV直接收到工具LAND请求，不是飞行保护HOLD。工具新增等待NAV转换，10项检查通过。旧失败保留。
- 当前指纹/启动快照新增Faster-LIO源码与二进制，以完整记录算法版本；54项飞行保护和22项模型/坐标离线检查通过。第一次保护检查误用了不存在的临时fixture目录，失败保留，改为保存的实际C++fixture后54项全过且无跳过。
- 根因：旧雷达传感器位于机体中央上方0.10m，距机身顶面只有0.045m，向下射线大多打到自身；升高后余下浅角射线先打到墙而看不到地面，造成高度约束退化。原始点云真值投影复现地面点降到零，详见diagnostic_01/raw_geometry_observation.jsonl与原始传感器bag。
- 修复：仿真雷达/独立IMU/生成器/桥接/静态TF改为[0.27,0,0.10]m，前倾30°不变；实际硬件平移外参待测量。解析20160条射线中向下4638条，机身遮挡2752→0；22项模型/坐标检查通过，在线多高度尚待结果。
- 扩展失败：第二个抬高目标到达后，Gazebo真值高度从约1.78m持续升到4.72m，而LIO/FCU高度长期约1.44m。20s悬停高度范围3.004m、LIO位移误差2.619m，不通过。清理降落最终上锁，但水平漂移0.627m超限，未掩盖失败；完整bag/ULog已保存。需要查定位高度失真根因，不能宣称实机就绪。
- 同时发现Qt AT-SPI同一进程有空/非空两份登记；gui_snapshot已按唯一PID选择非空树，8项离线检查通过，待实际GUI检查。扩展工具加录原始传感器话题，工具版本已变化。
- 核心最后两项：真实OFFBOARD目标流丢失原生反应0.596s，反升0.002923m、漂移0.034458m；低电量3.308s，反升0.006063m、漂移0.033287m。最终均上锁，ULog通过；低电量完整60s前置悬停已完成。核心报告status=core_passed_remaining_acceptance_pending，扩展/实机准备尚未通过。
- 当前故障：雷达反应0.616s、反升0.004934m、漂移0.010080m；IMU反应0.688s、反升0.000308m、漂移0.011526m；授权撤销反应0.008s、反升0、漂移0.024485m。三项均最终上锁且ULog审计通过。
- 当前版本三轮导航均完成起飞、前后60s悬停、绕障到达、降落上锁和ULog审计；三轮场景包络离线审计完成。前两轮悬停Z峰峰值分别0.084284/0.122528m、0.107808/0.125404m；最大跟踪偏差0.175781/0.178481m。优化图表已保存于live_13/optimization_comparison。整个核心现已通过，扩展尚待实际结果。
- live_12核心最后低电量项失败并结束；前三轮导航和四种其它故障均通过，不能判完整核心通过。会话52646与监督器80726均已退出。
- 低电量失败已定位：SIM_BAT_MIN_PCT=0、SIM_BAT_DRAIN=1实际写入成功；电压跌至14.4V，但无电流/负载模型的模拟器叠加油门压降补偿，使报告剩余电量停在0.154778，仍高于BAT_LOW_THR=0.15，更未到临界0.07。battery_injection_diagnosis.json保存实际ULog参数/范围/告警及源码SHA，未回写旧失败。
- 仿真airframe新增BAT1_V_LOAD_DROP=0：模拟器给的是开路电压，不应再补偿压降。仅仿真参数，真实电池需标定补偿；BAT_LOW/CRIT/EMERGEN阈值和COM_LOW_BAT_ACT=2不变。故障工具增加参数写入返回值核验。
- `20261003_low_battery_corrected`短复测通过（飞前20s悬停，不是完整60s回归）：2.560s触发降落，反升0.010865m，漂移0.031947m，落地上锁；ULog融合审计通过，真实电量最小0.055820，battery_status和failsafe_flags告警码0/1/2。新配置完整回归仍待完成。
- 新变化只涉及仿真电池补偿与注入回读，定位/ENU坐标/外部速度禁止融合/高度协方差/规划保护不变。离线模型/LIO21项通过。

- **当前主会话52646 / 监督器80726不变，版本9711d411f8cd5c458b57fb39457b7a43b87df26b3c42cc548c9323359e15d096。live_12三轮完整导航和雷达/IMU中断、撤销授权、真实OFFBOARD目标流丢失均通过，正在low_battery。** 扩展监督器尚在等待，不存在第二组ROS/Gazebo。
- 三轮前/后60s悬停Z峰峰值：0.093664/0.127644、0.106864/0.107769、0.104988/0.123991m；三轮ULog和全场景包络几何审计通过。跟踪最大偏差0.178372/0.178184/0.196660m。
- 当前故障实测：雷达反应0.548s，无反升，漂移0.020431m；IMU反应0.702s、反升0.013190m、漂移0.006043m；撤销授权反应0.026s、无反升、漂移0.021658m。均落地上锁，ULog禁止融合源保持0。
- 真正目标流丢失：停止管理器后PX4最后记录输入122.672s，123.264s进入failsafe AUTO_LAND，原生0.592s（≤1.5s）通过；ROS模式观察0.660s，无反升、漂移0.035173m，落地上锁。offboard_native_audit.json是必需验收项。
- 优化图表与数值来源保存于live_12/optimization_comparison。旧位置单独控制最大偏差0.685187/0.713614m；当前位置+期望速度前馈三轮0.178372/0.178184/0.196660m。不是单变量因果实验；不据图表宣称其它故障/实机通过。
- **检查点已更新：start/checkpoints/drone_sim_checkpoint_2026-10-03_0526.tar.gz（3.1GiB），latest为指向它的符号链接。** 包含源码/工具依赖、历史实验和live_12三轮闭合bag/ULog，不包含当时正在运行的基础故障轮次。gzip CRC通过，1170个选定源码及闭合数据文件SHA256一致；tar仅因活动实验父目录mtime变化返回1，内容另行严格核对。verification_final.json与archive.sha256已保存。
- 历史20261002_aligned_pose/telemetry.bag.active仍以失败/未闭合证据保留在归档，未计为完成飞行。不能把它和当前正在录制的active bag混淆。归档快照早于后续工具小改动，当前工作区与最新进度文件仍是续作依据。
- 位姿/时间输入故障工具新增桥接ROS错误日志匹配，必须证明触发的是对应的跳变/时间倒退保护，不能以任意invalid=true替代该项覆盖。扩展bag新增/rosout。生产版本未变化。

- `live_11` round_02导航成功后EKF状态年龄2.014s触发保护落地。原始bag状态以约1Hz持续发送，位置有效标志正常，ULog全部空中外部位姿/高度/航向融合且禁止源0；不是Faster-LIO大幅定位错误。旧未来样本直接丢弃使低频状态流出现假超时。
- 已修复管理器：小幅超前的EKF状态只暂存（最多100ms），在ROS时间真正达到采集时刻后才提交；不能用暂存样本刷新健康，超过100ms未来不缓存。未来20ms原接收容忍和状态2s超时均不增加。54项管理器检查通过，包含暂存/不提前使用/超大未来拒绝；21项模型与LIO检查通过。
- **当前主进程exec会话52646：`20261003_core_live_12`，配置指纹9711d411f8cd5c458b57fb39457b7a43b87df26b3c42cc548c9323359e15d096。round_01、round_02完整导航+前后60s悬停+降落上锁+ULog通过，round_03起飞过程中。不要启动其它ROS/PX4/Gazebo或修改生产源码。** 首轮高度峰峰值0.093664/0.127644m，LIO位移最大误差0.027179/0.032838m。
- **当前扩展监督器exec会话80726**等待live_12同配置核心全部通过，然后运行`20261003_extended_live_01`共17种测试。旧监督器6018已因live_11失败退出。工具仍未进入扩展飞行；其源码SHA会在实际开始时固定，开始后禁止修改工具。
- 两轮几何审计exec会话92558正在离线处理已闭合bag；这是读取数据，不是第二组仿真。
- Qt可访问性验证：本机默认IsEnabled/ScreenReaderEnabled均false，当前默认启动的GUI树为空。临时状态开启无法激活旧Qt树，已恢复所有桌面状态；独立PyQt窗口设置QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1后，确实能跨进程读到真实按钮，测试窗口已自动关闭。扩展GUI启动已设置该环境变量，仍需实际故障按钮检查。
- 工具离线结果：扩展场景8项、UDP转发/丢弃/不重放2项、GUI严格断言5项、原生OFFBOARD审计5项均通过，证据在`20261003_extended_tools`。离线结果不得等同于在线故障通过。

## 此前阶段记录

- `20261003_core_live_09` 已结束：三次完整导航冷启动、雷达短暂中断、雷达IMU短暂中断、空中撤销授权均通过并有ULog融合审计。全部GPS/气压/外部速度禁止融合源保持关闭。
- 三轮飞前/到达后60s真值Z峰峰值：0.093479/0.124127、0.094435/0.121715、0.113947/0.128559m，均小于0.15m；三轮场景包络几何审计均通过。第三轮51945个真值样本，最小分离轴间隙0.239967m。
- 雷达/IMU短暂中断反应仿真时间0.476/0.524s，降落无反升，水平最大漂移0.020114/0.035862m，最终自动上锁。这些实验恢复传感器后继续降落，不代表永久定位丢失能力。
- 原offboard_loss测试失败原因：AUTO.LOITER需要全局位置；第一次改POSCTL也被拒绝，因为其需要遥控输入而本项目已关闭遥控。两个失败实验均安全清理降落，原始记录保留，未判通过。ULog command176的result=1证明请求被拒绝，MAVROS mode_sent不能当作飞控接受。
- 当前offboard_loss测试已改为停止唯一目标发布节点`/drone_flight_manager`，保留LIO和MAVROS，核验实际PX4 AUTO.LAND/AUTO.DESCEND及落地上锁，测试真正的目标流丢失，而不是模拟模式请求。
- `20261003_offboard_stream_loss`已结束，原在线门槛判失败：ROS模式观察延迟1.508s；实际无反升、漂移0.031713m、落地上锁。新增ULog独立审计确认PX4最后记录目标99.002s，99.610s进入failsafe AUTO_LAND，原生反应0.608s（≤1.5s）。保留原失败，不回写为整轮通过。
- 当前测试把ROS模式观察等待与原生反应分开：在线目标流丢失观察≤3s，run_cold_start_tests另外强制ULog原生反应≤1.5s；未放宽飞控反应标准。故障真值轨迹现在从注入瞬间记录，而不是保护阶段后才开始。
- 修复明显ROS时钟倒退和LIO采集时间倒退：桥接锁定无效，管理器fresh_pose拒绝，两个Python定时器reset=True保证不因时钟异常死亡；fallback下降dt限[0,0.1]防止负dt反升。52项管理器检查通过（带C++真实地图fixture、无跳过），21项模型/坐标/LIO检查通过。
- 系统MAVROS默认required=true会在节点退出时关闭整个ROS链路；stack.launch现使用标准PX4配置文件的独立required=false节点，禁止该连带退出。启动器DRONE_FCU_URL可选覆盖，默认URL不变；用于实验UDP代理，生产配置指纹新增GUI源码/二进制与包配置。
- `live_10`首轮导航与ULog通过，但在落地上锁后人为停止，以应用MAVROS启动修复；不是三轮或完整核心通过。
- **当前运行：`20261003_core_live_11`，exec会话51181，配置指纹a585d043791a191f6ca07159bba2cea73dab25bb06211c14d4907f5a2f212c80。round_01完整通过并有ULog与几何审计，round_02在悬停。不能并行启动其它ROS/PX4/Gazebo，不能修改生产文件。** 首轮几何51995样本、最小分离轴间隙0.225370m，最大跟踪偏差0.193042m。
- **后续自动监督器已运行：exec会话6018，`run_sim_campaign.py`等待live_11同一配置核心全部通过，才启动`20261003_extended_live_01`的17种扩展验收；核心失败则停止，不跳过失败。** 无通过结果前不得标记实机可用。
- `start/tools/extended_flight_probe.py`和`run_extended_scenarios.py`新增多高度连续目标、corridor、three_d、blocked、boundaries；planner/traj/LIO/bridge/manager/MAVROS永久退出；双向MAVLink丢包；位姿跳变/采集时间倒退/ROS时钟重置；GUI相机断流；10次往返加600s悬停压力验收。通过生产服务发ENU目标；每轮冷启动、bag、ULog、全场景包络检查、清理降落，失败即停止。工具8项、UDP代理2项、GUI断言5项离线检查通过，扩展飞行尚未执行。
- 独立NativeMonitor已实测UDP14550读取PX4 custom_mode=393216、base_mode=145、OFFBOARD/armed=true；旧pymavlink高层模式解释会对该base_mode返回UNKNOWN，实验工具按本机PX4v1.15源码枚举直接解码并保留raw字段。不是ENU/NED坐标转换。pymavlink2.4.49安装于start/tools/python_deps，安装日志已保存；未改系统包。
- 新GUI工具在真实故障发生时截图、读取Qt AT-SPI按钮enabled标志并严格检查；无可访问树不能判GUI通过。扩展启动设置QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1用于读取，尚待实测可访问性。扩展工具源码快照、SHA、依赖版本另行保存；执行期间也禁止更改工具源码。
- 已实际截图`live_09/fault_imu_gui.png`：LANDING时解锁/起飞/悬停/取消禁用，一键降落保留，两相机与体素可见，底部健康故障原因可见。截图时传感器已恢复，因此不能据此勾选LIO-invalid显示；雷达截图观察器错过故障并超时，未记录为成功。
- 下一步：目标流丢失复测→低电量→新版本完整核心回归→多高度/复杂场景→节点和通信丢失、位姿/时间异常、GUI故障与长期压力。实机准备仍为false。

## 不可改变的约束

MID360 前倾 30°，独立雷达 IMU，前视/下视相机；ROS ENU/FLU，MAVROS 负责协议坐标转换。
Faster-LIO 位姿变换到机体中心后发送 `/mavros/odometry/out`。PX4 `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`；外部速度、GPS、气压高度融合关闭。PX4 IMU 参与状态传播和速度估计，不能字面上从位置传播中移除。
真实检测模型、真机标定与飞行另行验证；全局任务/扫描仍按原约定待需求明确，不把局部规划称为全局寻路。

## 新增的实验和验收工具

- `src/drone_stack/SIM_ACCEPTANCE.md`：指标和待办。
- `validate_simulation.py`：起飞、悬停、轨迹检查、执行、全过程降落、双相机 TF/处理图像、地面/空中非法操作、雷达/IMU 中断、授权撤销、退出 OFFBOARD、低电量测试。
- `run_sim_regression.py`：执行单轮测试，保存输出/返回码，失败时降落；`run_cold_start_tests.py`：管理完整冷启动与原始 ROS bag。
- `test_flight_safety.py`：42 项独立状态/指令、轨迹和跨语言消息检查已通过（2026-10-03），不能等同于真实异常飞行验收。
- `analyse_sim_experiments.py`：汇总实验数据并生成对比、悬停和降落图表。
- 新实验位于 `start/experiments/`；旧基线仍在 `start/validation/`。不得覆盖失败实验或以旧基线代替当前验收。
- 统一启动器会将当次 config/launch/scripts/patches、EGO map 源码和头文件保存到 source_snapshot，并记录 EGO 二进制 SHA256；之后改文件不影响已有快照。

## 已修复/改进

1. Faster-LIO 初始化阈值支持 ROS 参数 `mapping/imu_init_samples`，仿真设 400，算法默认仍 20。
2. MAVROS 仿真用 PASSTHROUGH 并关闭主动 TIMESYNC：实测原 near-zero 负偏移样本溢出到约 2^63，估计达到数亿秒。真实硬件不能照搬透传；需要正常时钟同步。
3. PX4 在 500 次 timesync 交换前会把外部位姿采集时刻替换为到达时刻；仿真管理器等待到 65 秒仿真时间才允许 READY。
4. 修复未解锁心跳把 LOCALIZING 错改 READY；允许 20 ms 有界未来时间偏差；LIO 健康心跳超时 0.5 秒不能继续沿用旧的 valid=true。
5. LIO 仿真位姿协方差为位置 0.0025（5 cm 标准差）。最新将 PX4 `EKF2_EVP_NOISE` 下限从 0.10 改为 0.03 m，以使消息协方差实际生效；v3 首轮 60 秒悬停通过，但整轮因地图过期拒绝导航而未通过。
6. Gazebo FCU IMU 改为完整 250 Hz HIL 窗口的物理速度差比力，保留原噪声和随机偏置。修正源码完整保存在包的 patches/gazebo_imu，prepare_px4_sitl.py 幂等复制到固定 PX4 版本。
7. 落脚碰撞增加 kp=1000、kd=40、max_vel=0.1、min_depth=0.001、零回弹。改善地面接触冲击；修正后雷达 IMU 初始化模长 9.81009，原值可达 10.1983。
8. 添加地面上锁接口；撤销地面授权时请求上锁；非法起飞高度和目标飞行范围拒绝；到达目标并低速保持 1 秒后自动 HOLD。
9. 增加目标占据体素和地图新鲜度检查。仿真适配器提供 lidar/imu 故障注入服务，仅用于仿真。
10. 两相机增加光学 TF，消息头为 front_camera_optical/down_camera_optical；仿真红色目标处理节点发布 image_processed 和像素框。Qt 已改看处理画面，并按状态启用按钮、增加地面上锁和授权状态文字。
11. 生成 corridor/3d/blocked 三个测试场景，尚未完成飞行验收；DRONE_SIM_WORLD 可切换场景。
12. PX4 `COM_LOW_BAT_ACT=2`，临界电量降落，低电量故障测试尚待执行。

## 实验结果（不可混为全部通过）

| 实验 | 结果 |
| --- | --- |
| init400 | 早期 1 m 起飞真值升高 1.80 m；30 秒悬停 Z 峰峰值 0.5075 m，不合格 |
| passthrough | 时间大偏移消失；早期起飞未完成 HOLD，不合格 |
| aligned_pose | 起飞真值 0.9808 m；随后旧时间判据误触发降落，整轮不合格 |
| regression_01 | 90 秒悬停 Z 0.1981 m，LIO 位移误差 0.0470 m；降落曾反升并水平漂移，不合格；双相机处理和 TF 通过 |
| imu_interval | 90 秒悬停 Z 0.1584 m，LIO误差0.0327 m；降落反升2.03 m，不合格 |
| compliant_feet | 短回归通过：30 秒悬停 Z 0.0610 m，LIO误差0.0150 m；降落无反升，水平漂移0.0226 m，自动上锁 |
| cold_regression | 自动工具等待启动过短，未起飞，已修正初始等待到120秒 |
| cold_regression_v2/round_01 | 起飞/相机/非法指令通过；60 秒悬停 Z 0.1661 m 不合格；降落无反升，漂移0.0056 m，通过 |
| cold_regression_v3/round_01 | 60 秒悬停通过：真值 Z 峰峰值0.108343 m、LIO位移误差0.032543 m；轨迹体素检查1500采样零碰撞；flight 被 Need fresh obstacle map before navigation 拒绝，整轮失败，未执行后续两轮；降落通过 |

降落验收新增全过程指标：反升不超过0.15 m、水平漂移不超过0.30 m，最终落地上锁。不能只看最后位置。

## 2026-10-03 已完成的阶段

- 源码明确复现根因：`pcl::toROSMsg` 接收到没有 stamp 的临时体素云，生成的 `/grid_map/occupancy_inflate` 时间为0。管理器拒绝无时间戳地图的保护没有被删除。
- EGO MappingData 新增实际观测时间；成功处理 lidar cloud 后更新，点云的 ENU frame、非零时间、时间递增、有效点检查通过后才使用。深度路径在地图更新完成后提交图像采集时间。三个地图消息发布函数统一使用 makeMapMessage，保留纳秒时间。
- 重复显示旧地图不会刷新采集时间。缺少新数据/时间倒退后不能伪装成新地图；重新启动并重建地图才恢复。
- 管理器复用 fresh_map，导航中地图超时取消导航、转 HOLD，并在同一周期继续发布悬停目标，维持 OFFBOARD 数据流。
- 冷启动记录增加 `/clock`、`/drone/cloud_fcu_world`、`/grid_map/occupancy_inflate`、bspline、规划开关、目标、estimator_status、timesync_status、外部odometry。上一轮 bag 缺少地图和时钟，不能直接测量其历史地图时间戳。
- 完整 catkin_make 编译通过；C++ 离线消息目标编译通过，7 项检查通过；Python 管理器24项检查通过、无跳过。其中 C++ 生成实际 ROS 序列化点云，Python 反序列化交给 on_map：旧零时间被拒绝，新时间保留、占据目标拒绝、空闲目标接受。
- 旧 v3 bag 离线审计：解锁且 OFFBOARD 时记录的目标消息最大间隔约0.074秒；这不证明每条消息都被 PX4 接收，也不是新的飞行实验。
- 更新实验汇总图表 `start/experiments/EXPERIMENT_COMPARISON.md` 和 `height_comparison.png`。
- 报告、fixture、构建/检查日志、源码差异：`start/experiments/20261003_map_timestamp_fix/`。离线通过不得当作导航执行/三轮冷启动通过。

## 2026-10-03 执行保护阶段（当前源码）

- 已补齐执行命令检查：采集时间、ENU frame、有限数值、飞行范围、0.75 m跟踪跳变上限、当前机体到目标点的体素穿越。DDA同时检查角点和网格面相邻体素；500条随机线段与独立闭合包围盒算法对照通过。
- 轨迹有过期数据时0.5秒转HOLD，不再等待初始5秒规划超时；到达/超时/保护切HOLD的同周期继续发布目标。
- 新增未裁剪高度的 `/grid_map/occupancy_inflate_safety` 用于执行保护和dry检查。原 `/grid_map/occupancy_inflate` 保持显示裁剪，Qt仍显示该体素话题。避免使用显示云代替完整安全地图。
- 虚拟天花板2.6 m已加入点云建图路径，规划查询阻止在天花板上方寻路；ceil索引校验防止越界写入。执行目标仍限制z≤2.5 m。重复地图时间不变时跳过重复Python解析，C++订阅状态只查询一次，降低新增数据开销。
- LIO时间检查允许未来偏差仅20 ms；健康判据不再把未来位姿判为有效。位置跳变上限0.5+3*dt m、角度上限0.3+4*dt rad；跳变拒绝转发并锁定无效，需落地后重启桥接和检查原因，不能自动接受新跳变原点。
- Faster-LIO、桥接、管理器不再设required=true；其中一个退出时其它节点保持运行，使管理器保护或PX4 OFFBOARD-loss降落有机会执行。实际故障飞行效果尚待验证。
- 管理器增加 `/drone/manager_heartbeat`。Qt用墙钟检查PX4/LIO/管理器/相机消息，新鲜度失效禁用普通操作；管理器不可用时，一键降落可直接通过MAVROS请求PX4 AUTO.LAND。仅编译通过，真实GUI/故障显示尚待联调。
- 回归工具捕获启动/超时异常并保留suite_result，降落清理允许连接120秒+下降55秒；land验证不依赖LIO/管理器状态话题，管理器缺失时可请求MAVROS AUTO.LAND。
- 每轮冷启动保存flight.ulg并执行audit_px4_ulog.py，未获取ULog不能判通过；仅雷达/IMU故障实验允许外部位姿融合在故障后退出，其它融合源全程必须禁用。bag采用无损LZ4。
- 新run_core_acceptance.py管理第一阶段三次导航冷启动+五种基础故障，保留配置/二进制指纹，代码变化后不能复用旧通过结果。它明确不代表全部场景/GUI/压力验收。

证据位于 `start/experiments/20261003_execution_guards/`：最终编译通过；管理器42、模型/坐标/LIO17、工具6、C++13，共78项离线检查通过，无跳过。
旧v3的1223条LIO里程计重构回放无误拒绝（不是重跑原始点云/Faster-LIO）；旧ULog 57个空中样本外部位置/高度/航向均融合，外部速度/GPS/气压等禁用源计数为0。原ULog副本reference_v3.ulg已保存。两项历史审计均不是本轮新飞行。

## 当前运行状态与继续步骤

### 2026-10-03 网络恢复后的在线实验

- `20261003_core_live_01` 首轮实际起飞：地面/空中非法操作、起飞、双相机、60秒悬停、轨迹体素预检、降落通过。起飞真值相对高度1.038076m，LIO位移误差0.012278m；悬停真值Z峰峰值0.107089m，LIO位移最大误差0.016010m；降落无反升，水平漂移0.038722m，落地自动上锁。
- 安全地图实测时间戳非零。10秒在线采样目标流最大仿真间隔0.062s，地图年龄最大0.388s。
- 导航执行开始后被 `Invalid or stale trajectory acquisition time/frame` 切回HOLD，整轮失败。bag显示指令frame=odom、时间邻近/clock，但不能替代管理器内部时钟诊断；未直接放宽时间保护。
- 首轮ULog审计通过：105个空中样本均融合外部位置/高度/航向，外部速度/GPS/气压等禁止源全程计数0；原始bag、ULog、截图和报告保留。
- 在启动器设OPENBLAS_NUM_THREADS=1，避免Python小矩阵唤醒多线程，LIO桥接进程CPU从约128%降至约5.5%。轨迹时间错误增加age/stamp/frame诊断，验证工具在保护取消目标后快速保留失败并降落。
- live_02/live_03分别在起飞/悬停后触发定位健康保护并落地上锁。live_03内部诊断确认是飞控里程计相对管理器时钟超前22ms，LIO健康和EKF标志正常；已记录clock没有倒退。不能将此误报称为Faster-LIO定位误差。
- live_04导航时间误报为恰好-20ms；复现Duration.to_sec()得到-0.020000000000000018，改为从整数纳秒转换，新增20ms允许、20ms+1ns拒绝的回归检查。管理器现45项检查通过。
- live_05仍捕捉到-22ms，确认在init_node之后设置TCP_NODELAY无法重新协商已经建立的/clock连接。当前源码在init_node之前预注册callback-free /clock订阅，保持rospy内置回调管理时间；控制订阅/目标发布同时设立即发送和queue_size=1，地图接收缓冲4MiB。未来容忍20ms、轨迹最大年龄0.5s均未放宽。
- live_06导航再次遇到26ms超前。独立时钟观察器记录大多数FCU消息年龄-4至12ms，短时clock调度墙钟间隔可达0.1s；只修传输选项不能保证多个TCPROS话题同步到达。
- 当前管理器丢弃尚未到时的FCU/estimator样本，保留上一有效样本，不能用未来数据刷新健康；没有有效FCU数据0.7s仍降落。EGO小幅未来指令被丢弃，上一有效指令仍须通过时间/体素/范围检查；有效指令过期0.5s或时钟偏差持续0.5s转HOLD并保持同周期目标流，超过0.5s的大幅未来指令立即取消。未来20ms容忍未放宽。新增检查使管理器共48项通过，LIO/模型17项通过。
- live_07实现实际绕柱到达并自动HOLD：23.85墙钟秒，最小真值柱中心距离0.961346m，最终位置距目标约0.037m；起飞相对高度1.041772m，LIO位移误差0.015050m，飞前60s高度峰峰值0.114066m。到达后60s高度峰峰值0.151183m，略超0.15m，因此整轮失败；LIO位移误差仍仅0.026294m，不能归为LIO大幅误差。降落/自动上锁和融合审计通过。
- live_07数据估计LIO高度相对真值残差标准差7-8mm，最大约25mm，已在仿真lio.launch将报告的Z标准差50mm降至30mm（方差0.0025→0.0009）；XY/姿态协方差、EKF融合源、0.15m验收门槛保持。实机需要独立标定，不直接照搬仿真协方差。调参依据height_tuning_basis.json位于live_08。
- live_08飞前悬停Z峰峰值0.105920m通过，导航在柱体拐弯处触发体素穿越保护而失败。bag重构机体到规划目标线段确实穿过体素[2.65,0.75,0.95]，不删除保护。旧管理器只发规划位置，实际跟踪偏差最高0.69-0.71m，拐弯连接线会切入膨胀边界。
- 当前NAVIGATING发送规划位置+EGO期望速度前馈，使用PX4支持的position+velocity setpoint；速度有限值与模长≤0.75m/s检查，机体到目标0.75m跳变/体素检查保持。该速度仅是控制目标，`/mavros/odometry/out`速度仍未知，EKF2_EV_CTRL仍11，无外部速度融合。其它阶段保持原位置/地面零速度控制。管理器50项、模型/LIO17项检查通过。
- 正在运行 `20261003_core_live_09`（exec会话66580），进程由run_core_acceptance.py管理。三轮导航中的round_01、round_02已完整通过，并各有ULog融合审计通过。首轮两次60s高度峰峰值0.093479/0.124127m，LIO位移最大误差0.016399/0.041741m；全部空中109条flags外部速度/GPS/气压均0。第三轮在飞前悬停，尚未结束；三轮后自动继续五种基础故障。不要另外启动ROS master。
- 新独立几何验收工具位于start/tools/sim_world_geometry.py、audit_world_bag.py，使用场景全部静态碰撞box和模型/link/collision层级姿态，车辆含传感器保守包络半尺寸[0.35,0.40,0.21]m（含30mm余量），支持真值姿态旋转和批量SAT检查。8项检查通过，含500个随机姿态批量/单点一致性；证据start/experiments/20261003_world_geometry。它明确是有限采样包络检查，不能冒充Gazebo接触传感器或连续碰撞证明。
- live_09首两轮原始bag几何检查已通过，首轮最小分离轴间隙0.235428m；跟踪偏差最高0.177953/0.184794m（按最新接收FCU位置估计，未插值），明显优于位置单独控制。工具/世界SHA256保存在world_geometry_audit.json；第一轮timing_audit.json也保存。
- Qt综合操作台已实际打开，雷达点云/EGO体素/双相机处理画面可见；初始化截图在live_02，悬停截图在live_03，到达目标截图在live_07。live_01至live_08均未整体通过，融合审计均通过，原始数据完整保留。
- 修改后管理器42项离线检查通过，无跳过（使用上一阶段编译的地图消息fixture）。

- 以下网络受限记录属于01:33时的旧会话环境，03:09后已经恢复实际飞行，不能再作为当前阻塞原因。
- 用户已授权执行和测试，不需要再次询问。旧会话禁止本地TCP/UDP套接字，旧preflight返回PermissionError / Operation not permitted。
- 最终启动记录 `start/experiments/20261003_core_acceptance_attempt_v2/core_acceptance_result.json`：status=blocked_environment，cases=[]、passed=false、real_hardware_ready=false。
- 上次实际仿真仍是2026-10-02 v3，落地上锁；本轮所有启动和测试进程已结束，没有运行中的ROS/PX4/Gazebo。
- ROS本地通信可用后，先重启完整仿真，不能热加载旧节点混用新安全话题。使用新目录运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/d/robotproject/project0/devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOME=/tmp/drone_ros_home
python3 src/drone_stack/scripts/run_core_acceptance.py start/experiments/20261003_core_live_01
```

- 如失败，保留数据，修复后用新目录。只有同一源码/参数/二进制指纹可以--resume；已经创建的case目录会使用下一次attempt，不覆盖失败实验。
- 重点核对：安全体素非零采集时间和完整高度；dry→HOLD→flight实际绕柱、到达自动HOLD；连续目标流；跳变保护无误触发；新增安全地图的CPU负载和时间延迟；天花板限制不会造成误碰撞。
- 第一阶段通过后继续：多高度/连续目标、corridor/3d/blocked/边界、节点/通信失效、定位跳变/时间重置、界面故障状态、长期重复任务。当前flight验证仍使用demo柱体检查，需要扩展各场景几何检查后再自动套用其它世界，不能据同一柱体检查宣称通道/三维场景通过。
- 这些在线项目全部待验收，硬件标定/真机噪声/真实时钟同步/真实检测仍属于实机阶段。全局任务与扫描仍按原约定后续明确。
- 本轮完成的是源码与离线检查阶段；尚未满足进入实机验收条件。

## 启动与测试

```bash
cd /home/d/robotproject/project0
src/drone_stack/scripts/start_simulation.sh
# Ctrl-C 清理整组仿真；启动器拒绝已有 ROS master。
```

测试前 source /opt/ros/noetic/setup.bash、devel/setup.bash，unset ROS_IP，设置 ROS_HOSTNAME=127.0.0.1、ROS_MASTER_URI=http://127.0.0.1:11311。

```bash
python3 src/drone_stack/scripts/run_cold_start_tests.py start/experiments/<新目录> --rounds 3 --navigation
python3 src/drone_stack/scripts/run_cold_start_tests.py start/experiments/<新故障目录> --rounds 1 --hover-seconds 20 --fault-case fault_lidar
python3 src/drone_stack/scripts/analyse_sim_experiments.py start/experiments
```

最新检查点：`start/checkpoints/drone_sim_checkpoint_2026-10-03_0133.tar.gz`，完整drone_stack/EGO源码、Faster-LIO关键修改、Gazebo IMU补丁、全部实验数据及原始bag/参考ULog已保存。最新副本为 `start/checkpoints/drone_sim_checkpoint_latest.tar.gz`；历史检查点保留。完整PX4源码和运行依赖仍在原工作区，根目录.git元数据不完整，未创建commit。
