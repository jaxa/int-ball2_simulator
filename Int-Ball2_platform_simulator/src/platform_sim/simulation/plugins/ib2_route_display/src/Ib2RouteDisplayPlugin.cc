#include "Ib2RouteDisplayPlugin.hh"

using namespace gazebo;

namespace
{
  /** visualization_msgs::MarkerArrayの配列インデックス：移動経路の線. */
  unsigned int ROUTE_LINE     = 0;
  /** visualization_msgs::MarkerArrayの配列インデックス：移動経路の開始点. */
  unsigned int ROUTE_START    = 1;
  /** visualization_msgs::MarkerArrayの配列インデックス：移動経路の終了点. */
  unsigned int ROUTE_END      = 2;

  /** visualization_msgs::MarkerのID：移動経路の線. */
  unsigned int ROUTE_LINE_ID  = 1;
  /** visualization_msgs::MarkerのID：移動経路の開始点. */
  unsigned int ROUTE_START_ID = 2;
  /** visualization_msgs::MarkerのID：移動経路の終了点. */
  unsigned int ROUTE_END_ID   = 3;
}

Ib2RouteDisplayPlugin::Ib2RouteDisplayPlugin() : 
  force_publish_history_(false),
  is_ib2_moving_(false),
  is_route_marker_deleted_(false),
  history_min_distance_(DEFAULT_HISTORY_MIN_DISTANCE),
  history_max_(DEFAULT_HISTORY_MAX),
  last_published_history_id_(HISTORY_INDEX_MIN - 1),
  msg_history_(visualization_msgs::Marker()),
  msg_route_(visualization_msgs::MarkerArray())
{
}

void Ib2RouteDisplayPlugin::Load(physics::WorldPtr /*_parent*/, sdf::ElementPtr /*_sdf*/)
{
  if(!ros::isInitialized())
  {
    int argc    = 0;
    char **argv = NULL;
    ros::init(argc, argv, "ib2_route_display", ros::init_options::NoSigintHandler);
  }

  nh_         = ros::NodeHandle("ib2_route_display");
  tf_listener_ptr_.reset(new tf::TransformListener);

  double timer_rate = 0.1;
  nh_.getParam("/ib2_route_display/rate", timer_rate);
  timer_ = nh_.createTimer(ros::Duration(1.0 / timer_rate), &Ib2RouteDisplayPlugin::timerCallback,  this);

  sub_ctl_status_  = nh_.subscribe("/ctl/status",  1, &Ib2RouteDisplayPlugin::ctlStatusCallback,  this);
  sub_ctl_profile_ = nh_.subscribe("/ctl/profile", 1, &Ib2RouteDisplayPlugin::ctlProfileCallback, this);

  pub_history_     = nh_.advertise<visualization_msgs::Marker>("/ib2_route_display/history", 1);
  pub_route_       = nh_.advertise<visualization_msgs::MarkerArray>("/ib2_route_display/target", 1);

  /*
   * 移動履歴（History）用マーカーの設定
   */
  nh_.getParam("/ib2_route_display/history/count_max", history_max_);
  nh_.getParam("/ib2_route_display/history/min_distance", history_min_distance_);
  if (!nh_.getParam("/ib2_route_display/history/frame_id", history_frame_id_))
  {
		gzerr << "Cannot Get /ib2_route_display/history/frame_id in ib2_route_display plugin \n";
    return;
  }
  msg_history_.header.frame_id = history_frame_id_;
  msg_history_.ns = "/ib2_route_display/history";
  msg_history_.type = visualization_msgs::Marker::SPHERE;
  msg_history_.pose.orientation.x = 0;
  msg_history_.pose.orientation.y = 0;
  msg_history_.pose.orientation.z = 0;
  msg_history_.pose.orientation.w = 1.0;
  msg_history_.frame_locked = true;
  // デフォルト値設定後にrosparamを読み込む
  msg_history_.color.a = 0.5;
  msg_history_.color.r = 0;
  msg_history_.color.g = 1.0;
  msg_history_.color.b = 0;
  double history_scale = 0.2;
  nh_.getParam("/ib2_route_display/history/scale", history_scale);
  nh_.getParam("/ib2_route_display/history/color/a", msg_history_.color.a);
  nh_.getParam("/ib2_route_display/history/color/r", msg_history_.color.r);
  nh_.getParam("/ib2_route_display/history/color/g", msg_history_.color.g);
  nh_.getParam("/ib2_route_display/history/color/b", msg_history_.color.b);
  msg_history_.scale.x = history_scale;
  msg_history_.scale.y = history_scale;
  msg_history_.scale.z = history_scale;

  /*
   * 移動経路（Route）用マーカーの設定
   */
  msg_route_.markers = {
                          visualization_msgs::Marker(),
                          visualization_msgs::Marker(),
                          visualization_msgs::Marker()
                        };
  std::string route_frame_id;
  if (!nh_.getParam("/ib2_route_display/route/frame_id", route_frame_id))
  {
		gzerr << "Cannot Get /ib2_route_display/route/frame_id in ib2_route_display plugin \n";
    return;
  }

  /*
   * 移動の線（LINE）
   */
  msg_route_.markers[ROUTE_LINE].header.frame_id = route_frame_id;
  msg_route_.markers[ROUTE_LINE].ns = "/ib2_route_display/route";
  msg_route_.markers[ROUTE_LINE].id = ROUTE_LINE_ID;
  msg_route_.markers[ROUTE_LINE].type = visualization_msgs::Marker::LINE_STRIP;
  msg_route_.markers[ROUTE_LINE].pose.orientation.x = 0;
  msg_route_.markers[ROUTE_LINE].pose.orientation.y = 0;
  msg_route_.markers[ROUTE_LINE].pose.orientation.z = 0;
  msg_route_.markers[ROUTE_LINE].pose.orientation.w = 1.0;
  // デフォルト値設定後にrosparamを読み込む
  msg_route_.markers[ROUTE_LINE].color.a = 1.0;
  msg_route_.markers[ROUTE_LINE].color.r = 1.0;
  msg_route_.markers[ROUTE_LINE].color.g = 0;
  msg_route_.markers[ROUTE_LINE].color.b = 1.0;
  msg_route_.markers[ROUTE_LINE].scale.x = 0.02;
  nh_.getParam("/ib2_route_display/route/line/color/a", msg_route_.markers[ROUTE_LINE].color.a);
  nh_.getParam("/ib2_route_display/route/line/color/r", msg_route_.markers[ROUTE_LINE].color.r);
  nh_.getParam("/ib2_route_display/route/line/color/g", msg_route_.markers[ROUTE_LINE].color.g);
  nh_.getParam("/ib2_route_display/route/line/color/b", msg_route_.markers[ROUTE_LINE].color.b);
  nh_.getParam("/ib2_route_display/route/line/scale",   msg_route_.markers[ROUTE_LINE].scale.x);

  /*
   * 移動の開始点（START）
   */
  msg_route_.markers[ROUTE_START].header.frame_id = route_frame_id;
  msg_route_.markers[ROUTE_START].ns = "/ib2_route_display/route";
  msg_route_.markers[ROUTE_START].id = ROUTE_START_ID;
  msg_route_.markers[ROUTE_START].type = visualization_msgs::Marker::SPHERE;
  // デフォルト値設定後にrosparamを読み込む
  msg_route_.markers[ROUTE_START].color.a = 1.0;
  msg_route_.markers[ROUTE_START].color.r = 0;
  msg_route_.markers[ROUTE_START].color.g = 0;
  msg_route_.markers[ROUTE_START].color.b = 1.0;
  double route_start_scale = 0.1;
  nh_.getParam("/ib2_route_display/route/start/color/a", msg_route_.markers[ROUTE_START].color.a);
  nh_.getParam("/ib2_route_display/route/start/color/r", msg_route_.markers[ROUTE_START].color.r);
  nh_.getParam("/ib2_route_display/route/start/color/g", msg_route_.markers[ROUTE_START].color.g);
  nh_.getParam("/ib2_route_display/route/start/color/b", msg_route_.markers[ROUTE_START].color.b);
  nh_.getParam("/ib2_route_display/route/start/scale",   route_start_scale);
  msg_route_.markers[ROUTE_START].scale.x = route_start_scale;
  msg_route_.markers[ROUTE_START].scale.y = route_start_scale;
  msg_route_.markers[ROUTE_START].scale.z = route_start_scale;

  /*
   * 移動の終了点（END）
   */
  msg_route_.markers[ROUTE_END].header.frame_id = route_frame_id;
  msg_route_.markers[ROUTE_END].ns = "/ib2_route_display/route";
  msg_route_.markers[ROUTE_END].id = ROUTE_END_ID;
  msg_route_.markers[ROUTE_END].type = visualization_msgs::Marker::SPHERE;
  // デフォルト値設定後にrosparamを読み込む
  msg_route_.markers[ROUTE_END].color.a = 1.0;
  msg_route_.markers[ROUTE_END].color.r = 1.0;
  msg_route_.markers[ROUTE_END].color.g = 0;
  msg_route_.markers[ROUTE_END].color.b = 0;
  double route_end_scale = 0.1;
  nh_.getParam("/ib2_route_display/route/end/color/a", msg_route_.markers[ROUTE_END].color.a);
  nh_.getParam("/ib2_route_display/route/end/color/r", msg_route_.markers[ROUTE_END].color.r);
  nh_.getParam("/ib2_route_display/route/end/color/g", msg_route_.markers[ROUTE_END].color.g);
  nh_.getParam("/ib2_route_display/route/end/color/b", msg_route_.markers[ROUTE_END].color.b);
  nh_.getParam("/ib2_route_display/route/end/scale",   route_end_scale);
  msg_route_.markers[ROUTE_END].scale.x = route_end_scale;
  msg_route_.markers[ROUTE_END].scale.y = route_end_scale;
  msg_route_.markers[ROUTE_END].scale.z = route_end_scale;
}

void Ib2RouteDisplayPlugin::timerCallback(const ros::TimerEvent& /*_event*/)
{
  /* 移動履歴マーカー用に、Int-Ball2位置姿勢をTFから参照する */
  tf::StampedTransform transform;
  tf_listener_ptr_->lookupTransform(history_frame_id_, "body", ros::Time(0), transform);
  geometry_msgs::TransformStamped geometry;
  tf::transformStampedTFToMsg(transform, geometry);
  nav_msgs::Odometry odm;
  odm.pose.pose.position.x = geometry.transform.translation.x;
  odm.pose.pose.position.y = geometry.transform.translation.y;
  odm.pose.pose.position.z = geometry.transform.translation.z;
  odm.pose.pose.orientation.x = geometry.transform.rotation.x;
  odm.pose.pose.orientation.y = geometry.transform.rotation.y;
  odm.pose.pose.orientation.z = geometry.transform.rotation.z;
  odm.pose.pose.orientation.w = geometry.transform.rotation.w;

  controlHistoryMarkers(odm.pose.pose);
  controlRouteMarkers();
}

void Ib2RouteDisplayPlugin::controlHistoryMarkers(const geometry_msgs::Pose& pose)
{
  auto history_distance = sqrt(pow(pose.position.x - msg_history_.pose.position.x, 2) + 
                                pow(pose.position.y - msg_history_.pose.position.y, 2) + 
                                pow(pose.position.z - msg_history_.pose.position.z, 2));
  if (force_publish_history_ || (history_distance >= history_min_distance_))
  {
    auto next_history_id = last_published_history_id_ + 1;
    if (last_published_history_id_ >  history_max_)
    {
      next_history_id = HISTORY_INDEX_MIN;
    }
    msg_history_.header.stamp = ros::Time::now();
    msg_history_.id = next_history_id;
    msg_history_.action = visualization_msgs::Marker::ADD;
    msg_history_.pose = pose;

    pub_history_.publish(msg_history_);
    last_published_history_id_ = next_history_id;

    force_publish_history_ = false;
  }
}

void Ib2RouteDisplayPlugin::controlRouteMarkers()
{
  if (is_ib2_moving_ && ctl_profile_.header.stamp >= last_ib2_stop_timestamp_)
  {
    geometry_msgs::PoseStamped goal_pose_world;
    auto timestamp = ros::Time::now();

    msg_route_.markers[ROUTE_LINE].header.stamp = timestamp;
    msg_route_.markers[ROUTE_LINE].action = visualization_msgs::Marker::ADD;

    msg_route_.markers[ROUTE_LINE].points.clear();
    for (geometry_msgs::PoseStamped pose: ctl_profile_.poses)
    {
      msg_route_.markers[ROUTE_LINE].points.push_back(pose.pose.position);
    }
    
    msg_route_.markers[ROUTE_START].header.stamp = timestamp;
    msg_route_.markers[ROUTE_START].action = visualization_msgs::Marker::ADD;
    msg_route_.markers[ROUTE_START].pose = ctl_profile_.poses.front().pose;

    msg_route_.markers[ROUTE_END].header.stamp = timestamp;
    msg_route_.markers[ROUTE_END].action = visualization_msgs::Marker::ADD;
    msg_route_.markers[ROUTE_END].pose = ctl_profile_.poses.back().pose;

    pub_route_.publish(msg_route_);
    is_route_marker_deleted_ = false;

  } else {
    if (!is_route_marker_deleted_) {
      msg_route_.markers[ROUTE_LINE].action = visualization_msgs::Marker::DELETE;
      msg_route_.markers[ROUTE_START].action = visualization_msgs::Marker::DELETE;
      msg_route_.markers[ROUTE_END].action = visualization_msgs::Marker::DELETE;

      pub_route_.publish(msg_route_);
      is_route_marker_deleted_ = true;
    }
  }
}

void Ib2RouteDisplayPlugin::ctlStatusCallback(const ib2_msgs::CtlStatus& msg)
{
  if (msg.type.type == ib2_msgs::CtlStatusType::STAND_BY ||
      msg.type.type == ib2_msgs::CtlStatusType::KEEP_POSE ||
      msg.type.type == ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION)
  {
    if (is_ib2_moving_)
    {
      is_ib2_moving_ = false;
      last_ib2_stop_timestamp_ = msg.pose.header.stamp;
    }
  }
  else
  {
    if(!is_ib2_moving_)
    {
      is_ib2_moving_ = true;
      force_publish_history_ = true;
    }
    ctl_status_ = msg;
  }
}

void Ib2RouteDisplayPlugin::ctlProfileCallback(const ib2_msgs::CtlProfile& msg)
{
  ctl_profile_ = msg;
}