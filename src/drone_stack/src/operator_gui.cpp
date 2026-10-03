#include <drone_stack/LocalGoal.h>
#include <drone_stack/Takeoff.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>
#include <map>
#include <mutex>
#include <string>

class OperatorWindow : public QMainWindow {
public:
  explicit OperatorWindow(ros::NodeHandle& nh) : nh_(nh) {
    setWindowTitle(QStringLiteral("巡检四旋翼 · 综合操作台"));
    resize(1600, 960);
    setMinimumSize(1100, 700);
    setStyleSheet(QStringLiteral(
      "QMainWindow,QWidget#root,QWidget#side,QScrollArea{background:#101721;color:#e7edf5;}"
      "QLabel{color:#dce5f0;}"
      "QLabel#title{font-size:22pt;font-weight:700;color:#f3f7fb;}"
      "QLabel#card{background:#17212e;border:1px solid #405067;border-radius:7px;"
      "padding:8px;color:#dce5f0;font-size:11pt;}"
      "QGroupBox{background:#17212e;border:1px solid #2b3a4e;border-radius:7px;"
      "margin-top:20px;padding:14px 8px 8px;color:#dce5f0;font-size:13pt;}"
      "QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 5px;}"
      "QPushButton{background:#245d87;color:white;border:1px solid #347bb0;"
      "border-radius:5px;min-height:34px;padding:3px 9px;}"
      "QPushButton:disabled{background:#293442;color:#6f7b8a;}"
      "QPushButton#land{background:#8b4e1f;border-color:#b87132;}"
      "QDoubleSpinBox{background:#0d141d;color:white;border:1px solid #405067;"
      "min-height:30px;}QPlainTextEdit{background:#0d141d;color:#d4deea;}"));
    buildUi();

    auth_client_ = nh_.serviceClient<std_srvs::SetBool>("/drone/set_authorized");
    arm_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/arm");
    takeoff_client_ = nh_.serviceClient<drone_stack::Takeoff>("/drone/takeoff");
    hold_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/hold");
    cancel_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/cancel_goal");
    land_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/land");
    direct_land_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    goal_client_ = nh_.serviceClient<drone_stack::LocalGoal>("/drone/local_goal");
    state_sub_ = nh_.subscribe("/mavros/state", 5, &OperatorWindow::onState, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 5, &OperatorWindow::onOdom, this);
    battery_sub_ = nh_.subscribe("/mavros/battery", 5, &OperatorWindow::onBattery, this);
    lio_sub_ = nh_.subscribe("/drone/lio/valid", 5, &OperatorWindow::onLio, this);
    heartbeat_sub_ = nh_.subscribe<std_msgs::Header>("/drone/manager_heartbeat", 1, [this](const std_msgs::Header::ConstPtr&) {
      std::lock_guard<std::mutex> guard(mutex_); manager_receipt_ = ros::WallTime::now();
    });
    phase_sub_ = nh_.subscribe("/drone/flight_state", 5, &OperatorWindow::onPhase, this);
    auth_sub_ = nh_.subscribe("/drone/authorized", 5, &OperatorWindow::onAuth, this);
    error_sub_ = nh_.subscribe("/drone/flight_error", 5, &OperatorWindow::onError, this);
    front_sub_ = nh_.subscribe("/drone/front/image_processed", 1, &OperatorWindow::onFront, this);
    down_sub_ = nh_.subscribe("/drone/down/image_processed", 1, &OperatorWindow::onDown, this);
    clicked_sub_ = nh_.subscribe("/clicked_point", 3, &OperatorWindow::onClicked, this);
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(150);
  }

private:
  void buildUi() {
    auto* root = new QWidget(this);
    root->setObjectName("root");
    auto* main = new QVBoxLayout(root);
    main->setContentsMargins(12, 10, 12, 10);
    auto* title = new QLabel(QStringLiteral("巡检四旋翼 · 综合操作台"), root);
    title->setObjectName("title");
    main->addWidget(title);
    auto* cards = new QHBoxLayout();
    for (const QString& key : {"ROS", "PX4", "授权/ARM", "模式", "电池", "LIO", "雷达/EGO", "双相机", "告警"}) {
      auto* label = new QLabel(key + QStringLiteral("\n--"), root);
      label->setObjectName("card");
      label->setAlignment(Qt::AlignCenter);
      label->setMinimumHeight(60);
      cards->addWidget(label, 1);
      cards_[key] = label;
    }
    main->addLayout(cards);
    auto* horizontal = new QSplitter(Qt::Horizontal, root);
    auto* left = new QWidget(horizontal);
    auto* leftLayout = new QVBoxLayout(left);
    auto* viewTools = new QHBoxLayout();
    auto* view3d = new QPushButton(QStringLiteral("三维视图"), left);
    auto* view2d = new QPushButton(QStringLiteral("俯视图"), left);
    auto* pickGoal = new QPushButton(QStringLiteral("点选 XY 目标"), left);
    viewTools->addWidget(view3d);
    viewTools->addWidget(view2d);
    viewTools->addWidget(pickGoal);
    viewTools->addStretch();
    leftLayout->addLayout(viewTools);
    render_ = new rviz::RenderPanel(left);
    leftLayout->addWidget(render_, 1);
    manager_ = new rviz::VisualizationManager(render_);
    render_->initialize(manager_->getSceneManager(), manager_);
    render_->setAutoRender(true);
    manager_->initialize();
    manager_->getRootDisplayGroup()->setEnabled(true);
    manager_->setFixedFrame("odom");
    manager_->startUpdate();
    manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
    auto* view = manager_->getViewManager()->getCurrent();
    view->subProp("Distance")->setValue(14.0);
    view->subProp("Pitch")->setValue(0.75);
    view->subProp("Yaw")->setValue(0.8);
    static_cast<rviz::VectorProperty*>(view->subProp("Focal Point"))->setVector(Ogre::Vector3(1.5, 0, 1));
    auto* grid = manager_->createDisplay("rviz/Grid", "ENU 地面网格", true);
    grid->subProp("Plane Cell Count")->setValue(30);
    grid->subProp("Cell Size")->setValue(0.5);
    // Use the cloud registered into PX4's local ENU frame.  This keeps the
    // lidar map, EGO voxels and MAVROS odometry aligned in the same RViz view.
    addDisplay("rviz/PointCloud2", "MID360 点云", "/drone/cloud_fcu_world", "Points");
    addDisplay("rviz/PointCloud2", "EGO 体素", "/grid_map/occupancy_inflate", "Boxes");
    addDisplay("rviz/Odometry", "无人机位姿", "/drone/fcu/odom", "");
    addDisplay("rviz/Marker", "EGO 轨迹", "/ego_planner_node/optimal_list", "");
    auto* tool = manager_->getToolManager()->addTool("rviz/PublishPoint");
    connect(view3d, &QPushButton::clicked, this, [this] {
      manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
    });
    connect(view2d, &QPushButton::clicked, this, [this] {
      manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
    });
    connect(pickGoal, &QPushButton::clicked, this, [this, tool] {
      manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
      manager_->getToolManager()->setCurrentTool(tool);
      log(QStringLiteral("在俯视图点选 XY；Z 使用右侧目标高度"));
    });
    horizontal->addWidget(left);

    auto* sideScroll = new QScrollArea(horizontal);
    sideScroll->setWidgetResizable(true);
    sideScroll->setMinimumWidth(355);
    sideScroll->setMaximumWidth(480);
    auto* side = new QWidget(sideScroll);
    side->setObjectName("side");
    auto* sideLayout = new QVBoxLayout(side);
    front_image_ = cameraBox(sideLayout, side, QStringLiteral("前视处理画面 · 仿真目标"));
    down_image_ = cameraBox(sideLayout, side, QStringLiteral("下视处理画面 · 仿真目标"));
    auto* ops = new QGroupBox(QStringLiteral("飞行操作"), side);
    auto* opsLayout = new QGridLayout(ops);
    auto* auth = new QPushButton(QStringLiteral("操作授权"), ops);
    auto* arm = new QPushButton(QStringLiteral("PX4 解锁 ARM"), ops);
    auto* takeoff = new QPushButton(QStringLiteral("起飞"), ops);
    auto* hold = new QPushButton(QStringLiteral("悬停"), ops);
    auto* cancel = new QPushButton(QStringLiteral("取消目标"), ops);
    auto* land = new QPushButton(QStringLiteral("一键降落"), ops);
    auto* disarm = new QPushButton(QStringLiteral("地面上锁"), ops);
    arm_button_ = arm; takeoff_button_ = takeoff; hold_button_ = hold;
    cancel_button_ = cancel; land_button_ = land; disarm_button_ = disarm;
    auth_button_ = auth;
    land->setObjectName("land");
    takeoff_height_ = spin(1.2, ops);
    takeoff_height_->setRange(0.2, 2.0);
    opsLayout->addWidget(auth, 0, 0);
    opsLayout->addWidget(arm, 0, 1);
    opsLayout->addWidget(new QLabel(QStringLiteral("相对起飞高度 m")), 1, 0);
    opsLayout->addWidget(takeoff_height_, 1, 1);
    opsLayout->addWidget(takeoff, 2, 0, 1, 2);
    opsLayout->addWidget(hold, 3, 0);
    opsLayout->addWidget(cancel, 3, 1);
    opsLayout->addWidget(land, 4, 0, 1, 2);
    opsLayout->addWidget(disarm, 5, 0, 1, 2);
    sideLayout->addWidget(ops);
    auto* goals = new QGroupBox(QStringLiteral("局部三维目标 · ROS ENU"), side);
    auto* goalLayout = new QGridLayout(goals);
    relative_x_ = spin(1.0, goals);
    relative_y_ = spin(0.0, goals);
    relative_z_ = spin(0.0, goals);
    clicked_height_ = spin(1.2, goals);
    goalLayout->addWidget(new QLabel(QStringLiteral("前向 ΔX m")), 0, 0);
    goalLayout->addWidget(relative_x_, 0, 1);
    goalLayout->addWidget(new QLabel(QStringLiteral("左向 ΔY m")), 1, 0);
    goalLayout->addWidget(relative_y_, 1, 1);
    goalLayout->addWidget(new QLabel(QStringLiteral("上向 ΔZ m")), 2, 0);
    goalLayout->addWidget(relative_z_, 2, 1);
    auto* sendRelative = new QPushButton(QStringLiteral("发送相对目标"), goals);
    goal_button_ = sendRelative;
    goalLayout->addWidget(sendRelative, 3, 0, 1, 2);
    goalLayout->addWidget(new QLabel(QStringLiteral("点选目标绝对 Z m")), 4, 0);
    goalLayout->addWidget(clicked_height_, 4, 1);
    sideLayout->addWidget(goals);
    sideLayout->addStretch();
    sideScroll->setWidget(side);
    horizontal->addWidget(sideScroll);
    horizontal->setStretchFactor(0, 4);
    horizontal->setStretchFactor(1, 1);
    main->addWidget(horizontal, 1);
    events_ = new QPlainTextEdit(root);
    events_->setReadOnly(true);
    events_->setMaximumBlockCount(200);
    events_->setMaximumHeight(110);
    main->addWidget(events_);
    setCentralWidget(root);

    connect(auth, &QPushButton::clicked, this, [this] {
      std_srvs::SetBool srv;
      srv.request.data = !authorized_;
      if (!auth_client_.call(srv)) log(QStringLiteral("授权服务不可用"));
      else log(QString::fromStdString(srv.response.message));
    });
    connect(arm, &QPushButton::clicked, this, [this] { trigger(arm_client_, QStringLiteral("解锁")); });
    connect(takeoff, &QPushButton::clicked, this, [this] {
      drone_stack::Takeoff srv;
      srv.request.height_m = takeoff_height_->value();
      if (!takeoff_client_.call(srv)) log(QStringLiteral("起飞服务不可用"));
      else log(QString::fromStdString(srv.response.message));
    });
    connect(hold, &QPushButton::clicked, this, [this] { trigger(hold_client_, QStringLiteral("悬停")); });
    connect(cancel, &QPushButton::clicked, this, [this] { trigger(cancel_client_, QStringLiteral("取消目标")); });
    connect(land, &QPushButton::clicked, this, [this] {
      std_srvs::Trigger request;
      if (land_client_.call(request) && request.response.success) {
        log(QString::fromStdString(request.response.message));
        return;
      }
      mavros_msgs::SetMode fallback;
      fallback.request.custom_mode = "AUTO.LAND";
      const bool sent = direct_land_client_.call(fallback) && fallback.response.mode_sent;
      log(sent ? QStringLiteral("管理器不可用，已向 PX4 请求降落") : QStringLiteral("降落请求失败，请检查飞控连接"));
    });
    connect(disarm, &QPushButton::clicked, this, [this] {
      auto client = nh_.serviceClient<std_srvs::Trigger>("/drone/disarm");
      trigger(client, QStringLiteral("地面上锁"));
    });
    connect(sendRelative, &QPushButton::clicked, this, [this] { sendRelativeGoal(); });
  }

  static QDoubleSpinBox* spin(double value, QWidget* parent) {
    auto* out = new QDoubleSpinBox(parent);
    out->setDecimals(2);
    out->setRange(-1000000.0, 1000000.0);
    out->setValue(value);
    out->setSuffix(QStringLiteral(" m"));
    return out;
  }

  QLabel* cameraBox(QVBoxLayout* layout, QWidget* parent, const QString& name) {
    auto* box = new QGroupBox(name, parent);
    auto* body = new QVBoxLayout(box);
    auto* image = new QLabel(QStringLiteral("等待画面"), box);
    image->setAlignment(Qt::AlignCenter);
    image->setMinimumHeight(170);
    image->setStyleSheet("background:#080d13;color:#8fa0b5;border:1px solid #405067;");
    body->addWidget(image);
    layout->addWidget(box);
    return image;
  }

  void addDisplay(const QString& cls, const QString& title, const QString& topic, const QString& style) {
    auto* display = manager_->createDisplay(cls, title, true);
    if (!display) { log(QStringLiteral("RViz 无法加载：") + title); return; }
    // rviz/Marker names its topic property differently from PointCloud2 and
    // Odometry.  Accessing "Topic" here creates an RViz undefined-property
    // error and leaves the trajectory unsubscribed.
    const char* topic_property = cls == "rviz/Marker" ? "Marker Topic" : "Topic";
    if (auto* property = display->subProp(topic_property)) property->setValue(topic);
    if (!style.isEmpty()) if (auto* property = display->subProp("Style")) property->setValue(style);
    if (cls == "rviz/PointCloud2") {
      // Simulation intensity is constant, so use explicit colours rather
      // than automatically normalising an empty or zero-width intensity range.
      display->subProp("Color Transformer")->setValue("FlatColor");
      display->subProp("Color")->setValue(style == "Boxes" ? QColor(255, 170, 55) : QColor(75, 190, 255));
      if (style == "Boxes") {
        display->subProp("Size (m)")->setValue(0.1);
        display->subProp("Alpha")->setValue(0.4);
      }
    }
  }

  void trigger(ros::ServiceClient& client, const QString& name) {
    std_srvs::Trigger srv;
    if (!client.call(srv)) log(name + QStringLiteral("服务不可用"));
    else log(name + QStringLiteral(": ") + QString::fromStdString(srv.response.message));
  }

  void submit(const geometry_msgs::PoseStamped& goal) {
    drone_stack::LocalGoal srv;
    srv.request.goal = goal;
    if (!goal_client_.call(srv)) log(QStringLiteral("目标服务不可用"));
    else log(QString::fromStdString(srv.response.message));
  }

  void sendRelativeGoal() {
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!have_odom_ || (ros::Time::now() - odom_.header.stamp).toSec() > 0.35) {
        log(QStringLiteral("位姿过期，目标未发送")); return;
      }
      odom = odom_;
    }
    const auto& q = odom.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double f = relative_x_->value(), l = relative_y_->value();
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.header.stamp = ros::Time::now();
    goal.pose = odom.pose.pose;
    goal.pose.position.x += std::cos(yaw) * f - std::sin(yaw) * l;
    goal.pose.position.y += std::sin(yaw) * f + std::cos(yaw) * l;
    goal.pose.position.z += relative_z_->value();
    submit(goal);
  }

  void onClicked(const geometry_msgs::PointStamped::ConstPtr& point) {
    if (point->header.frame_id != "odom" || !std::isfinite(point->point.x) || !std::isfinite(point->point.y)) return;
    std::lock_guard<std::mutex> guard(mutex_);
    pending_click_ = *point;
    have_click_ = true;
  }

  void onState(const mavros_msgs::State::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); state_ = *msg; state_receipt_ = ros::WallTime::now(); }
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); odom_ = *msg; have_odom_ = true; }
  void onBattery(const sensor_msgs::BatteryState::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); battery_ = *msg; }
  void onLio(const std_msgs::Bool::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); lio_valid_ = msg->data; lio_receipt_ = ros::WallTime::now(); }
  void onPhase(const std_msgs::String::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); phase_ = msg->data; }
  void onAuth(const std_msgs::Bool::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); authorized_ = msg->data; }
  void onError(const std_msgs::String::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); pending_error_ = QString::fromStdString(msg->data); }
  void onFront(const sensor_msgs::Image::ConstPtr& msg) { onImage(msg, true); }
  void onDown(const sensor_msgs::Image::ConstPtr& msg) { onImage(msg, false); }

  void onImage(const sensor_msgs::Image::ConstPtr& msg, bool front) {
    QImage image;
    if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
      image = QImage(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888).copy();
      if (msg->encoding == "bgr8") image = image.rgbSwapped();
    } else if (msg->encoding == "mono8") {
      image = QImage(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_Grayscale8).copy();
    }
    std::lock_guard<std::mutex> guard(mutex_);
    if (front) { front_ = image; front_stamp_ = msg->header.stamp; front_receipt_ = ros::WallTime::now(); }
    else { down_ = image; down_stamp_ = msg->header.stamp; down_receipt_ = ros::WallTime::now(); }
  }

  void refresh() {
    nav_msgs::Odometry odom;
    QImage front, down;
    ros::Time frontStamp, downStamp;
    ros::WallTime stateReceipt, lioReceipt, managerReceipt, frontReceipt, downReceipt;
    geometry_msgs::PointStamped clicked;
    bool click = false, authorized, lio;
    mavros_msgs::State state;
    sensor_msgs::BatteryState battery;
    std::string phase;
    QString error;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      state = state_; battery = battery_; phase = phase_; odom = odom_;
      front = front_; down = down_; frontStamp = front_stamp_; downStamp = down_stamp_;
      authorized = authorized_; lio = lio_valid_;
      stateReceipt = state_receipt_; lioReceipt = lio_receipt_; managerReceipt = manager_receipt_;
      frontReceipt = front_receipt_; downReceipt = down_receipt_;
      click = have_click_; clicked = pending_click_; have_click_ = false;
      error = pending_error_; pending_error_.clear();
    }
    if (!error.isEmpty()) log(error);
    if (!error.isEmpty()) last_error_ = error;
    const auto now = ros::WallTime::now();
    const bool stateFresh = !stateReceipt.isZero() && (now-stateReceipt).toSec() < 2.0;
    const bool managerFresh = !managerReceipt.isZero() && (now-managerReceipt).toSec() < 0.75;
    lio = lio && !lioReceipt.isZero() && (now-lioReceipt).toSec() < 0.75;
    state.connected = state.connected && stateFresh;
    const bool flightReady = state.connected && lio && managerFresh;
    auth_button_->setEnabled(state.connected && managerFresh);
    goal_button_->setEnabled(flightReady && authorized && state.armed && (phase == "HOLD" || phase == "NAVIGATING"));
    arm_button_->setEnabled(flightReady && authorized && !state.armed && phase == "READY");
    takeoff_button_->setEnabled(flightReady && authorized && state.armed && phase == "ARMED");
    const bool canHold = flightReady && state.armed &&
        (phase == "TAKEOFF" || phase == "HOLD" || phase == "NAVIGATING");
    hold_button_->setEnabled(canHold); cancel_button_->setEnabled(canHold);
    land_button_->setEnabled(state.connected && state.armed);
    disarm_button_->setEnabled(state.connected && managerFresh && state.armed && phase == "ARMED");
    auth_button_->setText(authorized ? QStringLiteral("撤销授权") : QStringLiteral("操作授权"));
    if (click && goal_button_->isEnabled()) {
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = "odom";
      goal.header.stamp = ros::Time::now();
      goal.pose.position = clicked.point;
      goal.pose.position.z = clicked_height_->value();
      goal.pose.orientation.w = 1.0;
      submit(goal);
    }
    cards_["ROS"]->setText(QStringLiteral("ROS\n") + (ros::master::check() ? QStringLiteral("在线") : QStringLiteral("离线")));
    cards_["PX4"]->setText(QStringLiteral("PX4\n") + (state.connected ? QStringLiteral("已连接") : QStringLiteral("断连")));
    cards_["授权/ARM"]->setText(QStringLiteral("授权/ARM\n%1 / %2").arg(authorized ? QStringLiteral("是") : QStringLiteral("否"), state.armed ? QStringLiteral("已解锁") : QStringLiteral("锁定")));
    cards_["模式"]->setText(QStringLiteral("模式\n") + QString::fromStdString(state.mode));
    cards_["电池"]->setText(QStringLiteral("电池\n") + (std::isfinite(battery.percentage) ? QString::number(100 * battery.percentage, 'f', 0) + "%" : QStringLiteral("--")));
    cards_["LIO"]->setText(QStringLiteral("LIO\n") + (lio ? QStringLiteral("有效") : QStringLiteral("失效")));
    cards_["雷达/EGO"]->setText(QStringLiteral("雷达/EGO\n") + (managerFresh ? QString::fromStdString(phase) : QStringLiteral("管理器离线")));
    const bool frontFresh = !frontReceipt.isZero() && (now-frontReceipt).toSec() < 1.0 && !frontStamp.isZero() && (ros::Time::now()-frontStamp).toSec() >= -0.02 && (ros::Time::now()-frontStamp).toSec() < 1.0;
    const bool downFresh = !downReceipt.isZero() && (now-downReceipt).toSec() < 1.0 && !downStamp.isZero() && (ros::Time::now()-downStamp).toSec() >= -0.02 && (ros::Time::now()-downStamp).toSec() < 1.0;
    cards_["双相机"]->setText(QStringLiteral("双相机\n%1 / %2").arg(frontFresh ? QStringLiteral("前在线") : QStringLiteral("前断流"), downFresh ? QStringLiteral("下在线") : QStringLiteral("下断流")));
    cards_["告警"]->setText(QStringLiteral("告警\n") + (last_error_.isEmpty() ? QStringLiteral("无") : QStringLiteral("见日志")));
    cards_["告警"]->setToolTip(last_error_);
    if (frontFresh && !front.isNull()) front_image_->setPixmap(QPixmap::fromImage(front).scaled(front_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    else front_image_->setText(QStringLiteral("前视画面等待中"));
    if (downFresh && !down.isNull()) down_image_->setPixmap(QPixmap::fromImage(down).scaled(down_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    else down_image_->setText(QStringLiteral("下视画面等待中"));
  }

  void log(const QString& text) { if (events_) events_->appendPlainText(text); }

  ros::NodeHandle nh_;
  ros::Subscriber heartbeat_sub_, state_sub_, odom_sub_, battery_sub_, lio_sub_, phase_sub_, auth_sub_, error_sub_, front_sub_, down_sub_, clicked_sub_;
  ros::ServiceClient auth_client_, arm_client_, takeoff_client_, hold_client_, cancel_client_, land_client_, goal_client_;
  ros::ServiceClient direct_land_client_;
  std::mutex mutex_;
  mavros_msgs::State state_;
  nav_msgs::Odometry odom_;
  sensor_msgs::BatteryState battery_;
  geometry_msgs::PointStamped pending_click_;
  bool have_odom_ = false, have_click_ = false, authorized_ = false, lio_valid_ = false;
  std::string phase_;
  QString pending_error_;
  QString last_error_;
  QImage front_, down_;
  ros::Time front_stamp_, down_stamp_;
  ros::WallTime state_receipt_, lio_receipt_, manager_receipt_, front_receipt_, down_receipt_;
  rviz::RenderPanel* render_ = nullptr;
  rviz::VisualizationManager* manager_ = nullptr;
  std::map<QString, QLabel*> cards_;
  QLabel *front_image_ = nullptr, *down_image_ = nullptr;
  QPushButton *arm_button_ = nullptr, *takeoff_button_ = nullptr, *hold_button_ = nullptr,
      *cancel_button_ = nullptr, *land_button_ = nullptr, *disarm_button_ = nullptr,
      *auth_button_ = nullptr, *goal_button_ = nullptr;
  QDoubleSpinBox *takeoff_height_ = nullptr, *relative_x_ = nullptr, *relative_y_ = nullptr, *relative_z_ = nullptr, *clicked_height_ = nullptr;
  QPlainTextEdit* events_ = nullptr;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_operator_gui");
  QApplication app(argc, argv);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OperatorWindow window(nh);
  window.show();
  return app.exec();
}
