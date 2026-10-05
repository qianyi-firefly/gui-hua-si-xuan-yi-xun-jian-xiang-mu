#include <drone_stack/LocalGoal.h>
#include <drone_stack/SetNavigationSpeed.h>
#include <drone_stack/Takeoff.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <QCheckBox>
#include <QMouseEvent>
#include <QWheelEvent>
#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <vector>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
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
#include <QStringList>
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
      "QLabel,QCheckBox{color:#dce5f0;}"
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
    nh_.param("/use_sim_time", use_sim_time_, false);
    buildUi();

    auth_client_ = nh_.serviceClient<std_srvs::SetBool>("/drone/set_authorized");
    arm_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/arm");
    takeoff_client_ = nh_.serviceClient<drone_stack::Takeoff>("/drone/takeoff");
    hold_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/hold");
    cancel_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/cancel_goal");
    land_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/land");
    direct_land_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    goal_client_ = nh_.serviceClient<drone_stack::LocalGoal>("/drone/local_goal");
    speed_client_ = nh_.serviceClient<drone_stack::SetNavigationSpeed>("/drone/set_navigation_speed");
    state_sub_ = nh_.subscribe("/mavros/state", 5, &OperatorWindow::onState, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 5, &OperatorWindow::onOdom, this);
    battery_sub_ = nh_.subscribe("/mavros/battery", 5, &OperatorWindow::onBattery, this);
    lio_quality_sub_ = nh_.subscribe<std_msgs::String>("/drone/flight_health_snapshot", 1, [this](const std_msgs::String::ConstPtr& msg) {
      const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      if (!doc.isObject()) return;
      const auto obj = doc.object();
      if (!obj.value("valid").isBool() || !obj.value("quality").isString()) return;
      std::lock_guard<std::mutex> guard(mutex_);
      lio_quality_ = obj.value("quality").toString().toStdString();
      lio_valid_ = obj.value("valid").toBool();
      lio_receipt_ = ros::WallTime::now(); lio_ros_receipt_ = ros::Time::now();
    });
    heartbeat_sub_ = nh_.subscribe<std_msgs::Header>("/drone/manager_heartbeat", 1, [this](const std_msgs::Header::ConstPtr&) {
      std::lock_guard<std::mutex> guard(mutex_); manager_receipt_ = ros::WallTime::now(); manager_ros_receipt_ = ros::Time::now();
    });
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/drone/cloud_fcu_world", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      cloud_stamp_ = msg->header.stamp; cloud_receipt_ = ros::WallTime::now();
    });
    map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      map_stamp_ = msg->header.stamp; map_receipt_ = ros::WallTime::now();
    });
    phase_sub_ = nh_.subscribe("/drone/flight_state", 5, &OperatorWindow::onPhase, this);
    auth_sub_ = nh_.subscribe("/drone/authorized", 5, &OperatorWindow::onAuth, this);
    error_sub_ = nh_.subscribe("/drone/flight_error", 5, &OperatorWindow::onError, this);
    front_sub_ = nh_.subscribe("/drone/front/image_processed", 1, &OperatorWindow::onFront, this);
    down_sub_ = nh_.subscribe("/drone/down/image_processed", 1, &OperatorWindow::onDown, this);
    navigation_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/drone/gui/navigation_markers", 1, true);
    active_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone/gui/active_goal", 1, false);
    global_path_sub_ = nh_.subscribe<nav_msgs::Path>("/drone/global_path", 1, [this](const nav_msgs::Path::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); global_path_message_ = *msg; have_global_path_ = true;
    });
    global_marker_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/drone/global_path_markers", 1, [this](const visualization_msgs::MarkerArray::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); yellow_markers_ = *msg; have_yellow_markers_ = true;
      yellow_receipt_ = ros::WallTime::now();
      yellow_stamp_ = msg->markers.empty() ? ros::Time(0) : msg->markers.front().header.stamp;
    });
    local_path_sub_ = nh_.subscribe<nav_msgs::Path>("/drone/executed_local_path", 1, [this](const nav_msgs::Path::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); local_path_message_ = *msg; have_local_path_ = true;
      local_path_receipt_ = ros::WallTime::now();
    });
    global_status_sub_ = nh_.subscribe<std_msgs::String>("/drone/global_path_status", 1, [this](const std_msgs::String::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); global_status_ = QString::fromStdString(msg->data);
    });
    navigation_wait_sub_ = nh_.subscribe<std_msgs::String>("/drone/navigation_wait_reason", 1, [this](const std_msgs::String::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); navigation_wait_ = QString::fromStdString(msg->data);
    });
    clicked_sub_ = nh_.subscribe("/clicked_point", 3, &OperatorWindow::onClicked, this);
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(150);
  }

private:
  enum class CardColor { Green, Yellow, Red };

  void setCardColor(const QString& key, CardColor color) {
    auto* card = cards_.at(key);
    const int value = static_cast<int>(color);
    if (card->property("healthState").isValid() && card->property("healthState").toInt() == value) return;
    card->setProperty("healthState", value);
    const QString background = color == CardColor::Green ? "#205c3b" : color == CardColor::Yellow ? "#765b19" : "#7b2c36";
    const QString border = color == CardColor::Green ? "#53b880" : color == CardColor::Yellow ? "#e7bd45" : "#e16875";
    card->setStyleSheet(QStringLiteral("QLabel#card{background:%1;border:2px solid %2;border-radius:7px;padding:8px;color:#ffffff;font-size:11pt;}").arg(background, border));
  }

  void showTopDown() {
    two_d_view_ = true;
    manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
    auto* view = manager_->getViewManager()->getCurrent();
    view->subProp("Scale")->setValue(std::max(10.0, std::min(render_->width(), render_->height())/18.0));
    view->subProp("Angle")->setValue(0.0);
    nav_msgs::Odometry odom;
    { std::lock_guard<std::mutex> guard(mutex_); odom = odom_; }
    view->subProp("X")->setValue(odom.pose.pose.position.x);
    view->subProp("Y")->setValue(odom.pose.pose.position.y);
    manager_->getToolManager()->setCurrentTool(manager_->getToolManager()->getDefaultTool());
  }

  bool eventFilter(QObject* watched, QEvent* event) override {
    if (watched == render_ && two_d_view_) {
      if (event->type() == QEvent::Wheel) {
        auto* wheel = static_cast<QWheelEvent*>(event);
        auto* scale = manager_->getViewManager()->getCurrent()->subProp("Scale");
        const double steps = wheel->angleDelta().y()/120.0;
        scale->setValue(std::max(2.0, std::min(2000.0, scale->getValue().toDouble()*std::pow(1.2, steps))));
        event->accept(); return true;
      }
      if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonRelease) {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (picking_goal_ && mouse->button() == Qt::LeftButton) {
          if (event->type() == QEvent::MouseButtonRelease) {
            const auto ray = manager_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
                mouse->localPos().x()/std::max(1,render_->width()), mouse->localPos().y()/std::max(1,render_->height()));
            const auto hit = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0));
            if (hit.first) {
              const auto point = ray.getPoint(hit.second);
              std::lock_guard<std::mutex> guard(mutex_);
              if (!have_click_) {
                pending_click_.header.frame_id = "odom"; pending_click_.header.stamp = ros::Time::now();
                pending_click_.point.x = point.x; pending_click_.point.y = point.y;
                have_click_ = true; pick_button_->setEnabled(false);
              }
            }
          }
          return true;
        }
      }
    }
    return QMainWindow::eventFilter(watched, event);
  }

  void publishNavigation() {
    visualization_msgs::MarkerArray array;
    for (int id=0; id<3; ++id) {
      visualization_msgs::Marker m;
      m.header.frame_id = "odom"; m.header.stamp = ros::Time::now();
      m.ns = "operator_navigation"; m.id = id; m.pose.orientation.w = 1.0;
      m.action = have_target_marker_ ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
      m.color.a = 1.0;
      if (id == 0) {
        m.type = visualization_msgs::Marker::CYLINDER; m.pose.position = selected_goal_;
        m.pose.position.z += .08; m.scale.x = .30; m.scale.y = .30; m.scale.z = .04; m.color.r = 1.0;
      } else {
        m.type = visualization_msgs::Marker::LINE_STRIP; m.scale.x = .055;
        m.color.r = id == 1 ? 1.0 : .10; m.color.g = 1.0; m.color.b = id == 1 ? 0.0 : .15;
        m.points = id == 1 ? planned_path_ : actual_path_;
        // Yellow reference segments are owned by the map-aware route node,
        // which distinguishes observed solid lines from unknown dashed lines.
        if (id == 1 || m.points.size()<2) m.action = visualization_msgs::Marker::DELETE;
        for (auto& point : m.points) point.z += .06;
      }
      array.markers.push_back(m);
    }
    navigation_pub_.publish(array);
  }

  Ogre::ManualObject* createRouteLayer(const std::string& name, unsigned char queue) {
    const std::string materialName = "drone_layer_" + name;
    auto material = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setLightingEnabled(false);
    material->setDepthCheckEnabled(false); material->setDepthWriteEnabled(false);
    material->setCullingMode(Ogre::CULL_NONE);
    material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    auto* object = manager_->getSceneManager()->createManualObject(materialName);
    object->setDynamic(true); object->setRenderQueueGroup(queue);
    manager_->getSceneManager()->getRootSceneNode()->attachObject(object);
    return object;
  }

  void drawRouteLayer(Ogre::ManualObject* object, const std::vector<geometry_msgs::Point>& points,
                      bool pairs, const Ogre::ColourValue& color, double width) {
    object->clear(); if (points.size()<2) return;
    object->begin(object->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const auto vertex = [&](const Ogre::Vector3& p) { object->position(p); object->colour(color); };
    for (size_t i=0;i+1<points.size();i+=pairs?2:1) {
      const auto& a=points[i];const auto& b=points[i+1];
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<1e-8 && std::abs(b.z-a.z)<1e-8) continue;
      Ogre::Vector3 offset = length<1e-8 ? Ogre::Vector3(width/2,0,0) :
          Ogre::Vector3(-(b.y-a.y)*width/(2*length),(b.x-a.x)*width/(2*length),0);
      Ogre::Vector3 start(a.x,a.y,a.z),end(b.x,b.y,b.z);
      vertex(start+offset);vertex(start-offset);vertex(end-offset);
      vertex(start+offset);vertex(end-offset);vertex(end+offset);
    }
    object->end(); object->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  }

  void drawHeadingLayer(const nav_msgs::Odometry& odom, bool valid) {
    heading_layer_->clear(); if (!valid) return;
    const auto& p=odom.pose.pose.position;const auto& q=odom.pose.pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    Ogre::Vector3 origin(p.x,p.y,p.z+.1),forward(std::cos(yaw),std::sin(yaw),0),side(-std::sin(yaw),std::cos(yaw),0);
    heading_layer_->begin(heading_layer_->getName(),Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const auto vertex=[&](double x,double y) {heading_layer_->position(origin+forward*x+side*y);heading_layer_->colour(Ogre::ColourValue(1,.05,.05,1));};
    vertex(-.12,.05);vertex(-.12,-.05);vertex(.32,-.05);
    vertex(-.12,.05);vertex(.32,-.05);vertex(.32,.05);
    vertex(.24,.14);vertex(.24,-.14);vertex(.62,0);
    heading_layer_->end();heading_layer_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  }

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
    telemetry_ = new QLabel(QStringLiteral("等待位姿与飞行状态"), root);
    readiness_ = new QLabel(QStringLiteral("等待系统就绪"), root);
    readiness_->setWordWrap(true);
    main->addWidget(telemetry_);
    main->addWidget(readiness_);
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
    pick_button_ = pickGoal;
    auto* resetView = new QPushButton(QStringLiteral("复位视角"), left);
    viewTools->addWidget(resetView);
    viewTools->addStretch();
    leftLayout->addLayout(viewTools);
    render_ = new rviz::RenderPanel(left);
    leftLayout->addWidget(render_, 1);
    manager_ = new rviz::VisualizationManager(render_);
    render_->initialize(manager_->getSceneManager(), manager_);
    render_->setAutoRender(true);
    render_->installEventFilter(this);
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
    // Explicit render queues implement the requested ordering independently
    // of camera angle and physical altitude. Geometry comes from real routes.
    yellow_layer_ = createRouteLayer("global_yellow", 95);
    purple_layer_ = createRouteLayer("executed_purple", 96);
    heading_layer_ = createRouteLayer("heading_red", 97);
    for (const auto& title : {QStringLiteral("MID360 点云"), QStringLiteral("EGO 体素"), QStringLiteral("EGO 轨迹")}) {
      auto* toggle = new QCheckBox(title, left);
      toggle->setChecked(true);
      viewTools->addWidget(toggle);
      connect(toggle, &QCheckBox::toggled, this, [this, title](bool enabled) {
        if (displays_.count(title)) displays_[title]->setEnabled(enabled);
        if (title == QStringLiteral("EGO 轨迹")) purple_layer_->setVisible(enabled);
      });
    }
    connect(resetView, &QPushButton::clicked, this, [this] {
      two_d_view_ = false; picking_goal_ = false;
      manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
      auto* view = manager_->getViewManager()->getCurrent();
      view->subProp("Distance")->setValue(14.0);
      view->subProp("Pitch")->setValue(0.75);
      view->subProp("Yaw")->setValue(0.8);
      static_cast<rviz::VectorProperty*>(view->subProp("Focal Point"))->setVector(Ogre::Vector3(1.5, 0, 1));
    });
    addDisplay("rviz/MarkerArray", "目标与路径", "/drone/gui/navigation_markers", "");

    navigation_hint_ = new QLabel(QStringLiteral("红点：目标  |  黄实线：已观测自由  |  黄虚线：待观测参考  |  紫线：执行中的EGO曲线  |  绿线：实际轨迹"), left);
    navigation_hint_->setWordWrap(true);
    leftLayout->addWidget(navigation_hint_);
    connect(view3d, &QPushButton::clicked, this, [this] {
      two_d_view_ = false; picking_goal_ = false;
      manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
    });
    connect(view2d, &QPushButton::clicked, this, [this] { picking_goal_ = false; showTopDown(); });
    connect(pickGoal, &QPushButton::clicked, this, [this] {
      showTopDown(); picking_goal_ = true;
      log(QStringLiteral("滚轮缩放二维图；左键点选一个目标，Z使用右侧高度。到达或取消前不能发送第二个目标。"));
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
    ops->setFocusPolicy(Qt::StrongFocus);
    auto* auth = new QPushButton(QStringLiteral("操作授权"), ops);
    auto* arm = new QPushButton(QStringLiteral("PX4 解锁 ARM"), ops);
    arm->setFocusPolicy(Qt::NoFocus);
    auto* takeoff = new QPushButton(QStringLiteral("起飞"), ops);
    auto* hold = new QPushButton(QStringLiteral("悬停"), ops);
    auto* cancel = new QPushButton(QStringLiteral("取消目标"), ops);
    auto* land = new QPushButton(QStringLiteral("一键降落"), root);
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
    navigation_speed_ = spin(0.5, ops);
    navigation_speed_->setRange(0.1, 1.0);
    navigation_speed_->setSingleStep(0.05);
    navigation_speed_->setSuffix(QStringLiteral(" m/s"));
    navigation_speed_->setToolTip(QStringLiteral("下一次导航任务采用此规划速度；当前任务及其重规划保持原速度。"));
    opsLayout->addWidget(new QLabel(QStringLiteral("移动速度 m/s")), 2, 0);
    opsLayout->addWidget(navigation_speed_, 2, 1);
    opsLayout->addWidget(takeoff, 3, 0, 1, 2);
    opsLayout->addWidget(hold, 4, 0);
    opsLayout->addWidget(cancel, 4, 1);
    opsLayout->addWidget(new QLabel(QStringLiteral("降落按钮位于底部固定操作栏")), 5, 0, 1, 2);
    opsLayout->addWidget(disarm, 6, 0, 1, 2);
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
    auto* emergency = new QHBoxLayout();
    auto* emergencyHint = new QLabel(QStringLiteral("一键降落：优先管理器，管理器不可用时请求 PX4 AUTO.LAND"), root);
    emergency->addWidget(emergencyHint, 1);
    land->setMinimumWidth(230);
    emergency->addWidget(land);
    main->addLayout(emergency);
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
    connect(arm, &QPushButton::clicked, this, [this, ops] {
      ops->setFocus(Qt::MouseFocusReason);
      trigger(arm_client_, QStringLiteral("解锁"));
    });
    connect(navigation_speed_, &QDoubleSpinBox::editingFinished, this, [this] { configureSpeed(); });
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
    displays_[title] = display;
    // rviz/Marker names its topic property differently from PointCloud2 and
    // Odometry.  Accessing "Topic" here creates an RViz undefined-property
    // error and leaves the trajectory unsubscribed.
    const char* topic_property = (cls == "rviz/Marker" || cls == "rviz/MarkerArray") ? "Marker Topic" : "Topic";
    if (auto* property = display->subProp(topic_property)) property->setValue(topic);
    if (!style.isEmpty()) if (auto* property = display->subProp("Style")) property->setValue(style);
    if (cls == "rviz/Odometry") display->subProp("Keep")->setValue(1);
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

  bool submit(const geometry_msgs::PoseStamped& goal) {
    if (goal_busy_) { log(QStringLiteral("已有目标正在执行，请先到达或取消")); return false; }
    if (!configureSpeed()) return false;
    goal_busy_ = true; navigation_seen_ = false; goal_start_ = ros::Time::now();
    selected_goal_ = goal.pose.position; have_target_marker_ = true;
    planned_path_.clear(); actual_path_.clear(); last_trace_stamp_ = ros::Time(0);
    { std::lock_guard<std::mutex> guard(mutex_); actual_path_.push_back(odom_.pose.pose.position); goal_navigation_epoch_ = navigation_epoch_; }
    pick_button_->setEnabled(false); goal_button_->setEnabled(false);
    publishNavigation();
    drone_stack::LocalGoal srv; srv.request.goal = goal;
    if (!goal_client_.call(srv)) {
      goal_busy_ = false; log(QStringLiteral("目标服务不可用")); return false;
    }
    log(QString::fromStdString(srv.response.message));
    if (!srv.response.success) goal_busy_ = false;
    else active_goal_pub_.publish(goal);
    navigation_hint_->setText(srv.response.success ? QStringLiteral("目标已发送，等待规划黄线；执行期间禁止第二个目标。绿线为实际轨迹。") : QStringLiteral("目标被拒绝，可重新点选。红点为本次选择，未开始执行。"));
    return srv.response.success;
  }

  bool configureSpeed() {
    drone_stack::SetNavigationSpeed srv;
    srv.request.speed_mps = navigation_speed_->value();
    if (!speed_client_.call(srv)) { log(QStringLiteral("移动速度设置服务不可用")); return false; }
    log(QString::fromStdString(srv.response.message));
    return srv.response.success;
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
    // Target clicks are generated by the XY render-panel event filter.
    // Ignore external /clicked_point publishers to avoid unintended commands.
    (void)point;
  }

  void onState(const mavros_msgs::State::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); state_ = *msg; state_receipt_ = ros::WallTime::now(); state_ros_receipt_ = ros::Time::now(); }
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); odom_ = *msg; have_odom_ = true; }
  void onBattery(const sensor_msgs::BatteryState::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); battery_ = *msg; battery_receipt_ = ros::WallTime::now(); battery_ros_receipt_ = ros::Time::now(); }
  void onLio(const std_msgs::Bool::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); lio_valid_ = msg->data; lio_receipt_ = ros::WallTime::now(); lio_ros_receipt_ = ros::Time::now(); }
  void onPhase(const std_msgs::String::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); phase_ = msg->data; if (phase_ == "NAVIGATING") ++navigation_epoch_; }
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
    ros::Time frontStamp, downStamp, stateRos, lioRos, managerRos, cloudStamp, mapStamp, batteryRos;
    ros::WallTime batteryReceipt;
    ros::WallTime cloudReceipt, mapReceipt;
    ros::WallTime stateReceipt, lioReceipt, managerReceipt, frontReceipt, downReceipt;
    geometry_msgs::PointStamped clicked;
    bool click = false, authorized, lio;
    mavros_msgs::State state;
    sensor_msgs::BatteryState battery;
    std::string phase, lioQuality;
    QString error;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      state = state_; battery = battery_; phase = phase_; odom = odom_;
      front = front_; down = down_; frontStamp = front_stamp_; downStamp = down_stamp_;
      authorized = authorized_; lio = lio_valid_;
      lioQuality = lio_quality_;
      batteryRos = battery_ros_receipt_; batteryReceipt = battery_receipt_;
      stateRos = state_ros_receipt_; lioRos = lio_ros_receipt_; managerRos = manager_ros_receipt_;
      cloudStamp = cloud_stamp_; mapStamp = map_stamp_; cloudReceipt = cloud_receipt_; mapReceipt = map_receipt_;
      stateReceipt = state_receipt_; lioReceipt = lio_receipt_; managerReceipt = manager_receipt_;
      frontReceipt = front_receipt_; downReceipt = down_receipt_;
      click = have_click_; clicked = pending_click_; have_click_ = false;
      error = pending_error_; pending_error_.clear();
    }
    if (!error.isEmpty()) log(error);
    if (!error.isEmpty()) last_error_ = error;
    const auto now = ros::WallTime::now();
    const auto rosNow = ros::Time::now();
    if (use_sim_time_) {
      if (!last_clock_.isZero() && rosNow < last_clock_) clock_fault_ = true;
      if (rosNow > last_clock_) clock_progress_ = now;
      last_clock_ = rosNow;
    }
    const bool clockHealthy = !use_sim_time_ || (!clock_fault_ && !rosNow.isZero() &&
        !clock_progress_.isZero() && (now-clock_progress_).toSec() < 10.0);
    // Simulation freshness uses ROS time; a separate wall watchdog catches
    // a paused/dead clock. Real hardware keeps the original wall deadlines.
    const auto fresh = [&](const ros::Time& stamp, const ros::WallTime& receipt, double limit) {
      if (receipt.isZero() || !clockHealthy) return false;
      const double wallAge = (now-receipt).toSec();
      if (stamp.isZero()) return false;
      const double age = (rosNow-stamp).toSec();
      return wallAge >= 0 && wallAge < (use_sim_time_ ? 10.0 : limit) && age >= -0.02 && age < limit;
    };
    const bool stateFresh = fresh(stateRos, stateReceipt, 2.0);
    const bool managerFresh = fresh(managerRos, managerReceipt, 0.75);
    lio = lio && fresh(lioRos, lioReceipt, 0.75);
    const bool cloudFresh = fresh(cloudStamp, cloudReceipt, 1.0);
    const bool mapFresh = fresh(mapStamp, mapReceipt, 2.0);
    const double poseAge = (rosNow-odom.header.stamp).toSec();
    const bool poseFresh = !odom.header.stamp.isZero() && clockHealthy && poseAge >= -0.02 && poseAge < 0.35;
    state.connected = state.connected && stateFresh;
    const bool flightReady = state.connected && lio && managerFresh && poseFresh;
    auth_button_->setEnabled(state.connected && managerFresh);
    if (goal_busy_) {
      { std::lock_guard<std::mutex> guard(mutex_); if (navigation_epoch_ > goal_navigation_epoch_) navigation_seen_ = true; }
      if ((navigation_seen_ && phase != "NAVIGATING") || !authorized || !state.armed ||
          !managerFresh || !lio || phase == "LANDING" || phase == "DESCENDING" || phase == "FAILSAFE") {
        goal_busy_ = false;
        have_target_marker_ = false;
        planned_path_.clear(); actual_path_.clear();
        { std::lock_guard<std::mutex> guard(mutex_); have_global_path_ = false; }
        publishNavigation();
        navigation_hint_->setText(QStringLiteral("任务已结束；目标点与路径已清除。可在HOLD重新点选。"));
      }
    }
    goal_button_->setEnabled(flightReady && authorized && state.armed && phase == "HOLD" && !goal_busy_);
    arm_button_->setEnabled(flightReady && authorized && !state.armed && phase == "READY");
    takeoff_button_->setEnabled(flightReady && authorized && state.armed && phase == "ARMED");
    const bool canHold = flightReady && state.armed &&
        (phase == "TAKEOFF" || phase == "HOLD" || phase == "NAVIGATING");
    hold_button_->setEnabled(canHold); cancel_button_->setEnabled(canHold);
    land_button_->setEnabled(state.connected && state.armed);
    disarm_button_->setEnabled(state.connected && managerFresh && state.armed && phase == "ARMED");
    auth_button_->setText(authorized ? QStringLiteral("撤销授权") : QStringLiteral("操作授权"));
    pick_button_->setEnabled(goal_button_->isEnabled() && !click);
    if (click && picking_goal_) {
      picking_goal_ = false;
      manager_->getToolManager()->setCurrentTool(manager_->getToolManager()->getDefaultTool());
      if (!goal_button_->isEnabled()) {
        log(QStringLiteral("系统状态不允许导航，点选目标未发送"));
      } else {
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = "odom";
      goal.header.stamp = ros::Time::now();
      goal.pose.position = clicked.point;
      goal.pose.position.z = clicked_height_->value();
      goal.pose.orientation = odom.pose.pose.orientation;
      submit(goal);
      }
    }
    bool overlayChanged = false;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (have_global_path_) {
        const auto& m = global_path_message_;
        if (m.poses.empty()) { planned_path_.clear(); overlayChanged = true; }
        else if (have_target_marker_ && m.header.stamp >= goal_start_ && m.header.frame_id == "odom" && m.poses.size()>=2) {
          const auto& end = m.poses.back().pose.position;
          if (std::hypot(end.x-selected_goal_.x,end.y-selected_goal_.y)<.01 && std::abs(end.z-selected_goal_.z)<.01) {
            planned_path_.clear();
            for (const auto& pose : m.poses) planned_path_.push_back(pose.pose.position);
            overlayChanged = true;
            global_path_receipt_ = now; global_path_stamp_ = m.header.stamp;
          }
        }
        have_global_path_ = false;
      }
    }
    if (!planned_path_.empty() && !fresh(global_path_stamp_, global_path_receipt_, 2.0)) {
      planned_path_.clear(); overlayChanged = true;
    }
    QString globalStatus, navigationWait;
    { std::lock_guard<std::mutex> guard(mutex_); globalStatus = global_status_; navigationWait = navigation_wait_; }
    if (goal_busy_) navigation_hint_->setText(QStringLiteral("红点：目标 | 黄实线：已观测自由 | 黄虚线：待观测参考 | 紫线：执行中的EGO曲线 | 绿线：实际轨迹\n") + navigationWait + QStringLiteral("\n") + globalStatus);
    if (goal_busy_ && poseFresh && odom.header.stamp > last_trace_stamp_) {
      const auto& p = odom.pose.pose.position;
      if (actual_path_.empty() || std::hypot(p.x-actual_path_.back().x,p.y-actual_path_.back().y)>.02 || std::abs(p.z-actual_path_.back().z)>.02) {
        actual_path_.push_back(p); overlayChanged = true;
        if (actual_path_.size()>20000) {
          std::vector<geometry_msgs::Point> reduced;
          for (size_t i=0;i<actual_path_.size();i+=2) reduced.push_back(actual_path_[i]);
          reduced.push_back(actual_path_.back()); actual_path_.swap(reduced);
        }
      }
      last_trace_stamp_ = odom.header.stamp;
    }
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (have_yellow_markers_) {
        std::vector<geometry_msgs::Point> points;
        for(const auto& marker:yellow_markers_.markers)
          if(marker.action==visualization_msgs::Marker::ADD && marker.header.frame_id=="odom")
            points.insert(points.end(),marker.points.begin(),marker.points.end());
        drawRouteLayer(yellow_layer_,points,true,Ogre::ColourValue(1,1,0,1),.055);
        have_yellow_markers_=false;
      }
      if (have_local_path_) {
        std::vector<geometry_msgs::Point> points;
        if(local_path_message_.header.frame_id=="odom" && phase=="NAVIGATING")
          for(const auto& pose:local_path_message_.poses) points.push_back(pose.pose.position);
        drawRouteLayer(purple_layer_,points,false,Ogre::ColourValue(.8,.12,1,1),.065);
        have_local_path_=false;
      }
      if(phase!="NAVIGATING" || !managerFresh ||
         (!local_path_message_.poses.empty() && !fresh(local_path_message_.header.stamp,local_path_receipt_,.75)))
        purple_layer_->clear();
      if(phase!="NAVIGATING" || !managerFresh || !fresh(yellow_stamp_,yellow_receipt_,2.0)) yellow_layer_->clear();
    }
    drawHeadingLayer(odom,poseFresh);
    manager_->queueRender();
    if (overlayChanged) publishNavigation();
    const bool rosOnline = ros::master::check();
    cards_["ROS"]->setText(QStringLiteral("ROS\n") + (rosOnline ? QStringLiteral("在线") : QStringLiteral("离线")));
    cards_["PX4"]->setText(QStringLiteral("PX4\n") + (state.connected ? QStringLiteral("已连接") : QStringLiteral("断连")));
    cards_["授权/ARM"]->setText(QStringLiteral("授权/ARM\n%1 / %2").arg(authorized ? QStringLiteral("是") : QStringLiteral("否"), state.armed ? QStringLiteral("已解锁") : QStringLiteral("锁定")));
    cards_["模式"]->setText(QStringLiteral("模式\n") + QString::fromStdString(state.mode));
    const bool batteryValid = fresh(batteryRos, batteryReceipt, 5.0) &&
        std::isfinite(battery.percentage) && battery.percentage >= 0 && battery.percentage <= 1;
    const double batteryPercent = 100.0*static_cast<double>(battery.percentage);
    // The small tolerance accounts for FLOAT32 representations of 0.2/0.6.
    const CardColor batteryColor = !batteryValid || batteryPercent < 20.0-0.0001 ? CardColor::Red :
        (batteryPercent > 60.0+0.0001 ? CardColor::Green : CardColor::Yellow);
    cards_["电池"]->setText(QStringLiteral("电池\n") + (batteryValid ? QString::number(batteryPercent, 'f', 0) + "%" : QStringLiteral("无有效数据")));
    cards_["电池"]->setToolTip(QStringLiteral("大于60%：绿色；20%至60%（含边界）：黄色；低于20%或数据失效：红色"));
    const bool lioWarning = lioQuality == "WARNING" || lioQuality == "HOLD";
    const bool lioSevere = lioQuality == "SEVERE";
    cards_["LIO"]->setText(QStringLiteral("LIO\n") +
        (lioSevere ? QStringLiteral("严重异常") : !lio ? QStringLiteral("失效") :
         lioQuality == "HOLD" ? QStringLiteral("健康异常 / HOLD") :
         lioWarning ? QStringLiteral("健康异常警告") : QStringLiteral("有效")));
    cards_["雷达/EGO"]->setText(QStringLiteral("雷达/EGO\n%1 / %2").arg(cloudFresh ? QStringLiteral("点云在线") : QStringLiteral("点云断流"), mapFresh ? QStringLiteral("体素在线") : QStringLiteral("体素断流")));
    const auto& q = odom.pose.pose.orientation;
    const double yaw = std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))*180.0/3.141592653589793;
    telemetry_->setText(QStringLiteral("飞行阶段：%1  |  ENU X %2  Y %3  Z %4 m  |  航向 %5°  |  %6")
        .arg(managerFresh ? QString::fromStdString(phase) : QStringLiteral("管理器离线"))
        .arg(poseFresh ? QString::number(odom.pose.pose.position.x, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(odom.pose.pose.position.y, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(odom.pose.pose.position.z, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(yaw, 'f', 1) : "--")
        .arg(use_sim_time_ ? QStringLiteral("仿真") : QStringLiteral("实机连接")));
    QString reason;
    if (!clockHealthy) reason = clock_fault_ ? QStringLiteral("仿真时钟倒退，请重启界面并检查仿真") : QStringLiteral("仿真时钟未开始或已冻结");
    else if (!state.connected) reason = QStringLiteral("等待 PX4 连接");
    else if (!managerFresh) reason = QStringLiteral("飞行管理器离线；普通操作禁用");
    else if (!lio) reason = QStringLiteral("等待有效 LIO 定位");
    else if (!poseFresh) reason = QStringLiteral("等待飞控有效位姿");
    else if (!authorized) reason = QStringLiteral("请先操作授权，再解锁与起飞");
    else if (!state.armed) reason = QStringLiteral("已授权；READY 时可解锁，解锁后显式起飞");
    else if (phase == "ARMED") reason = QStringLiteral("已解锁；设置相对高度后起飞，或地面上锁");
    else if (phase == "HOLD" || phase == "NAVIGATING") reason = QStringLiteral("可发送局部目标、悬停或一键降落");
    else reason = QStringLiteral("当前阶段：") + QString::fromStdString(phase);
    readiness_->setText(reason);
    for (auto* button : {auth_button_, arm_button_, takeoff_button_, hold_button_, cancel_button_, goal_button_, pick_button_, disarm_button_})
      button->setToolTip(button->isEnabled() ? QString() : reason);
    land_button_->setToolTip(QStringLiteral("已连接且已解锁时可请求降落；管理器不可用时直接请求 PX4 AUTO.LAND"));
    const bool frontFresh = fresh(frontStamp, frontReceipt, 1.0);
    const bool downFresh = fresh(downStamp, downReceipt, 1.0);
    cards_["双相机"]->setText(QStringLiteral("双相机\n%1 / %2").arg(frontFresh ? QStringLiteral("前在线") : QStringLiteral("前断流"), downFresh ? QStringLiteral("下在线") : QStringLiteral("下断流")));
    QStringList alarms;
    if (!rosOnline) alarms << QStringLiteral("ROS离线");
    if (!clockHealthy) alarms << QStringLiteral("仿真时钟异常");
    if (!state.connected) alarms << QStringLiteral("PX4断连");
    if (!managerFresh) alarms << QStringLiteral("飞行管理器离线");
    if (!lio) alarms << QStringLiteral("LIO无效");
    if (!poseFresh) alarms << QStringLiteral("飞控位姿过期");
    if (!cloudFresh || !mapFresh) alarms << QStringLiteral("点云或体素断流");
    if (!frontFresh || !downFresh) alarms << QStringLiteral("相机断流");
    if (!batteryValid) alarms << QStringLiteral("电池数据失效");
    else if (batteryColor == CardColor::Red) alarms << QStringLiteral("电量低于20%");
    if (phase == "FAILSAFE") alarms << QStringLiteral("飞行保护状态");
    const bool normalLanding = last_error_ == "Operator requested landing" || last_error_ == "Operator revoked authorization";
    if (!last_error_.isEmpty() && !normalLanding && phase != "READY") alarms << last_error_;
    const bool alarmActive = !alarms.isEmpty();
    cards_["告警"]->setText(QStringLiteral("告警\n") + (alarmActive ? QStringLiteral("异常，见提示") : QStringLiteral("无")));
    QString alarmTip = alarms.join(QStringLiteral("\n"));
    if (!last_error_.isEmpty()) alarmTip += QStringLiteral("\n最近管理器消息（历史）：") + last_error_;
    cards_["告警"]->setToolTip(alarmTip);
    setCardColor("ROS", rosOnline ? CardColor::Green : CardColor::Red);
    setCardColor("PX4", state.connected ? CardColor::Green : CardColor::Red);
    // Ground locked/unauthorized is a normal operating state.
    const bool authHealthy = state.connected && managerFresh &&
        (!state.armed || authorized || phase == "LANDING" || phase == "DESCENDING");
    setCardColor("授权/ARM", authHealthy ? CardColor::Green : CardColor::Red);
    const bool modeHealthy = state.connected && managerFresh && !state.mode.empty() &&
        phase != "FAILSAFE" && (!state.armed || state.mode == "OFFBOARD" || state.mode == "AUTO.LAND");
    setCardColor("模式", modeHealthy ? CardColor::Green : CardColor::Red);
    setCardColor("电池", batteryColor);
    setCardColor("LIO", !lio || lioSevere ? CardColor::Red :
        lioWarning ? CardColor::Yellow : CardColor::Green);
    setCardColor("雷达/EGO", cloudFresh && mapFresh ? CardColor::Green : CardColor::Red);
    setCardColor("双相机", frontFresh && downFresh ? CardColor::Green : CardColor::Red);
    setCardColor("告警", alarmActive ? CardColor::Red : CardColor::Green);
    if (frontFresh && !front.isNull()) front_image_->setPixmap(QPixmap::fromImage(front).scaled(front_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    else front_image_->setText(QStringLiteral("前视画面等待中"));
    if (downFresh && !down.isNull()) down_image_->setPixmap(QPixmap::fromImage(down).scaled(down_image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    else down_image_->setText(QStringLiteral("下视画面等待中"));
  }

  void log(const QString& text) { if (events_) events_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ") + text); }

  ros::NodeHandle nh_;
  ros::Publisher navigation_pub_, active_goal_pub_;
  ros::Subscriber global_marker_sub_, local_path_sub_;
  ros::Subscriber global_path_sub_, global_status_sub_, lio_quality_sub_, navigation_wait_sub_;
  std::string lio_quality_ = "UNKNOWN";
  ros::Subscriber cloud_sub_, map_sub_, heartbeat_sub_, state_sub_, odom_sub_, battery_sub_, lio_sub_, phase_sub_, auth_sub_, error_sub_, front_sub_, down_sub_, clicked_sub_;
  ros::ServiceClient auth_client_, arm_client_, takeoff_client_, hold_client_, cancel_client_, land_client_, goal_client_;
  ros::ServiceClient direct_land_client_;
  ros::ServiceClient speed_client_;
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
  bool two_d_view_ = false, goal_busy_ = false, navigation_seen_ = false, have_target_marker_ = false, have_global_path_ = false;
  ros::Time goal_start_, last_trace_stamp_;
  size_t navigation_epoch_ = 0, goal_navigation_epoch_ = 0;
  geometry_msgs::Point selected_goal_;
  std::vector<geometry_msgs::Point> planned_path_, actual_path_;
  nav_msgs::Path global_path_message_, local_path_message_;
  visualization_msgs::MarkerArray yellow_markers_;
  bool have_yellow_markers_ = false, have_local_path_ = false;
  ros::WallTime local_path_receipt_, yellow_receipt_;
  ros::Time yellow_stamp_;
  Ogre::ManualObject *yellow_layer_ = nullptr, *purple_layer_ = nullptr, *heading_layer_ = nullptr;
  QString global_status_, navigation_wait_;
  ros::Time global_path_stamp_;
  ros::WallTime global_path_receipt_;
  QLabel* navigation_hint_ = nullptr;
  bool use_sim_time_ = false, clock_fault_ = false, picking_goal_ = false;
  ros::Time battery_ros_receipt_;
  ros::WallTime battery_receipt_;
  ros::Time last_clock_, state_ros_receipt_, lio_ros_receipt_, manager_ros_receipt_, cloud_stamp_, map_stamp_;
  ros::WallTime clock_progress_, cloud_receipt_, map_receipt_;
  ros::Time front_stamp_, down_stamp_;
  ros::WallTime state_receipt_, lio_receipt_, manager_receipt_, front_receipt_, down_receipt_;
  rviz::RenderPanel* render_ = nullptr;
  rviz::VisualizationManager* manager_ = nullptr;
  std::map<QString, QLabel*> cards_;
  std::map<QString, rviz::Display*> displays_;
  QLabel *telemetry_ = nullptr, *readiness_ = nullptr;
  QPushButton* pick_button_ = nullptr;
  QLabel *front_image_ = nullptr, *down_image_ = nullptr;
  QPushButton *arm_button_ = nullptr, *takeoff_button_ = nullptr, *hold_button_ = nullptr,
      *cancel_button_ = nullptr, *land_button_ = nullptr, *disarm_button_ = nullptr,
      *auth_button_ = nullptr, *goal_button_ = nullptr;
  QDoubleSpinBox *takeoff_height_ = nullptr, *navigation_speed_ = nullptr, *relative_x_ = nullptr, *relative_y_ = nullptr, *relative_z_ = nullptr, *clicked_height_ = nullptr;
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
