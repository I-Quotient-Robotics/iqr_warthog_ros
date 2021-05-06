#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>
#include <std_srvs/Empty.h>
#include <cmath>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <iqr_warthog_app/WaypointNavAction.h>
#include <path_msgs/FollowPathAction.h>

#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <path_msgs/DirectionalPath.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <kdl/kdl.hpp>
#include "tf2_kdl/tf2_kdl.h"
#include <tf/transform_broadcaster.h>
// #include <kdl/chain.hpp>
// #include <kdl/tree.hpp>
// #include <kdl/segment.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define pi 3.1415926

typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;
typedef actionlib::ServerGoalHandle<iqr_warthog_app::WaypointNavAction> GoalHandle;
typedef actionlib::ActionServer<iqr_warthog_app::WaypointNavAction> Server;
bool move_goal_flag, cabine_flag = false;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}

class PathFollowerAction
{
public:

  ros::NodeHandle nh_;
  Server as_;
  FollowPathClient fpc_;
  // GoalHandle goal_handle_;
  // MoveBaseClient move_base_client_;
  std::string action_name_;
  std::vector<std::vector<geometry_msgs::Pose>> paths_, paths_all_;
  std::vector<geometry_msgs::Pose> path_;
  std::vector<std::vector<double>> points_;
  geometry_msgs::TransformStamped map_transform_down_;

  double body_height, height_goal_, gripper_position_, object_distance_;
  int conlum_;
  bool find_table_ = true, find_object_, task_active_ = false, get_goal, away_cabine_, marker_nav, get_object_distanse_ = false, get_table_ = true;
  std::map<std::string, std::vector<double>> mm;

  ros::ServiceClient stop_client, start_client, clear_costmaps_client_;
  ros::Publisher body_pub, head_pub, planning_scene_diff_publisher, base_cmd_pub, path_pub_;
  ros::Subscriber joint_states_sub, marker_pose_sub, table_pose_sub;
  
  geometry_msgs::Pose standby_position_, pre_pick_position_, pick_position_, place_position_, table_pose_, object_pose_;

  PathFollowerAction(std::string name) : 
    as_(nh_, name, boost::bind(&PathFollowerAction::ExecuteCb, this, _1), boost::bind(&PathFollowerAction::preemptCB, this, _1), false),
    fpc_("/follow_path", true)
  {
    base_cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);

    GetParam();
    as_.start();
  }

  ~PathFollowerAction(void)
  {
  }

  void ExcuteThread(GoalHandle gh) {
    // NavTask(gh);
    // FollowPathGoal(gh);
    // PathGenerat();
  }

  void ExecuteCb(GoalHandle gh)
  {
    ROS_INFO("````````````````````````````````````````");
    if (task_active_)
    {
      gh.setRejected();
      return;
    }
    gh.setAccepted();
    task_active_ = true;
    ROS_INFO("call pick task server");
    ROS_INFO("%s, %s", gh.getGoal()->items.c_str(), gh.getGoal()->place.c_str());
    std::thread thread(&PathFollowerAction::ExcuteThread, this, gh);
    thread.detach();
    task_active_ = false;
  }

  void preemptCB(GoalHandle gh)
  {
    // cancel move_base goal
    get_goal = true;    
    // move_base_client_.cancelAllGoals();
    // set the action state to preempted
    gh.setCanceled();
    task_active_ = false;
  }

  void GetParam() {
    // std::vector<std::vector<geometry_msgs::Pose>> paths;
    std::vector<geometry_msgs::Pose> path;
    std::vector<double> point;
    point.resize(3);
    for (int point_num = 1; point_num <= 4; ++point_num)
      {
        // geometry_msgs::Pose pose = GetWaypointsPose(point_num);
        // path_.push_back(pose);
        // ROS_INFO("%lf %lf %lf %lf %lf %lf %lf", path[point_num].position.x, path[point_num].position.y, path[point_num].position.z);
        nh_.getParam("waypoints/point" + std::to_string(point_num), point);
        printf("%lf\n", point[0]);
        points_.push_back(point);
      }
    nh_.getParam("waypoints/conlum", conlum_);
  }

  geometry_msgs::Pose GetWaypointsPose(int point_num) {

    std::vector<float> pose_vector(3), orientation_vector(4);
    nh_.getParam("/point" + std::to_string(point_num) + "/pose", pose_vector);
    nh_.getParam("/point" + std::to_string(point_num) + "/orientation", orientation_vector);

    geometry_msgs::Pose pose;
    pose.position.x = pose_vector[0];
    pose.position.y = pose_vector[1];
    pose.position.z = pose_vector[2];
    pose.orientation.x = orientation_vector[0];
    pose.orientation.y = orientation_vector[1];
    pose.orientation.z = orientation_vector[2];
    pose.orientation.w = orientation_vector[3];

    return pose;
  }

  void NavTask(GoalHandle gh){
    ROS_INFO("path follow task action server");

    // step = 1;
    while(true) {
      ros::spinOnce();

      // if (step == 1) {
      //   /* code */
      //   // feedback_.step_description = "Set arm to ready pose";
      //   // feedback_.step_index = 1;
      //   // gh.publishFeedback(feedback_);
      //   ROS_INFO("setp1: Move to column end 1");
      //   FollowPathGoal(gh);
      //   // feedback_.step_index = 2;
      //   // feedback_.step_description = "Set body height to zero";
      // }
    
      // else if(step == 2) {
      //   // gh.publishFeedback(feedback_);
      //   ros::WallDuration(1.0).sleep();
      //   ROS_INFO("step30: Move to standby position");
      //   ros::Rate loop_rate(10);

      //   for (int i = 0; i < 80; ++i)
      //   {
      //     geometry_msgs::Twist cmd_msg_back;
      //     cmd_msg_back.angular.z = -0.4;
      //     base_cmd_pub.publish(cmd_msg_back);
      //     loop_rate.sleep();
      //   }
      //   // if (!MoveToGoal(standby_position_, gh))
      //   // {
      //   //   return;
      //   // }
      //   // feedback_.step_index = 31;
      //   // feedback_.step_description = "All finished";
      // } 

      // // if cancel
      // if (gh.getGoalStatus().status == 2) {
      //   step = 1;
      //   return;
      // }
      // // if finished
      // else if(step == 30) {
      //   ROS_INFO("%i", step);
      //   // result_.finished_step = 7;
      //   // gh.setSucceeded(result_, "all");
      //   gh.setSucceeded();
      //   ROS_INFO("set succeeded");
      //   return;
      // }
      // step += 1;
    }
  }

  // std::vector<geometry_msgs::Pose> WaypointsGenerat(geometry_msgs::PoseWithCovarianceStamped odom_msg, geometry_msgs::Point gps_odom, int waypoint) {
  //   double point1_odom_x, point1_odom_y, point2_odom_x, point2_odom_y;
  //   double x, y;

  //   point1_odom_x = points_[waypoint][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   point1_odom_y = points_[waypoint][1] - gps_odom.y + odom_msg.pose.pose.position.y;

  //   point2_odom_x = points_[waypoint+1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   point2_odom_y = points_[waypoint+1][1] - gps_odom.y + odom_msg.pose.pose.position.y;
  //   // int point_sum = path_.size()*2;
  //   // std::vector<std::vector<geometry_msgs::Pose>> paths_all_;
  //   // x = path_[1].position.x - path_[0].position.x;
  //   // y = path_[1].position.y - path_[0].position.y;

  //   x = point2_odom_x - point1_odom_x;
  //   y = point2_odom_y - point1_odom_y;
  //   // ROS_INFO("1");
    
  //   float rad = atan2(y, x);
  //   std::vector<double> orientation_path = Rpy2Orientation(rad);
  //   // float rad_utm_base = atan2(points_[0][0] - gps_odom.x, points_[0][0] - gps_odom.y);
  //   // ROS_INFO("2");

  //   // Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
  //   // Eigen::Quaterniond quaternion1(rotation_vector1);
  //   // double orientation_x = quaternion1.x();
  //   // double orientation_y = quaternion1.y();
  //   // double orientation_z = quaternion1.z();
  //   // double orientation_w = quaternion1.w();

  //   // tf2::Quaternion q2;
  //   // q2.setRPY(rad, 0.0, 0.0);
  //   // ROS_INFO("tf: orientation %lf, %lf, %lf, %lf", q2.x(), q2.y(), q2.z(), q2.w());

  //   // ROS_INFO("%lf, %lf, %lf, %lf", orientation_x, orientation_y, orientation_z, orientation_w);
    
  //   // orientation_path = {orientation_x, orientation_y, orientation_z, orientation_w}; 
  //   // orientation_paths.push_back(orientation_path);   
  //   geometry_msgs::Pose pose1, pose2;
  //   std::vector<geometry_msgs::Pose> pose_vector;
  //   pose1.position.x = points_[waypoint][0] - points_[0][0];
  //   pose1.position.y = points_[waypoint][1] - points_[0][1];
  //   pose1.position.z = 0.0;
  //   pose1.orientation.x = orientation_path[0];
  //   pose1.orientation.y = orientation_path[1];
  //   pose1.orientation.z = orientation_path[2];
  //   pose1.orientation.w = orientation_path[3];
  //   pose2.position.x = points_[waypoint+1][0] - points_[0][0];
  //   pose2.position.y = points_[waypoint+1][1] - points_[0][1];
  //   pose2.position.z = 0.0;
  //   pose_vector = {pose1, pose2};

  //   if (waypoint == 0)
  //   {
  //     ROS_INFO("pub map tf");
  //     std::thread thread(&PathFollowerAction::MapTfBroadcast, this, point1_odom_x, point1_odom_y, orientation_path);
  //     thread.detach();
  //   }
  //   return  pose_vector;
  // }

  std::vector<double> Rpy2Orientation(double rad) {
    std::vector<double> orientation_path;
    Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond quaternion1(rotation_vector1);
    double orientation_x = quaternion1.x();
    double orientation_y = quaternion1.y();
    double orientation_z = quaternion1.z();
    double orientation_w = quaternion1.w();
    orientation_path = {orientation_x, orientation_y, orientation_z, orientation_w}; 
    return orientation_path;
  }

  std::vector<std::vector<double>> WaypointsGenerat() {
    
    double spacing_x1 = (points_[3][0] - points_[0][0]) / double(conlum_);
    double spacing_y1 = (points_[3][1] - points_[0][1]) / double(conlum_);
    double spacing_x2 = (points_[2][0] - points_[1][0]) / double(conlum_);
    double spacing_y2 = (points_[2][1] - points_[1][1]) / double(conlum_);
    // ROS_INFO("%lf", spacing_y);
    std::vector<std::vector<double>> waypoints1, waypoints2;
    for (int i = 0; i < conlum_; ++i)
    {
      std::vector<double> v1, v2;
      double waypoint_x1 = points_[0][0] + spacing_x1 * i;
      double waypoint_y1 = points_[0][1] + spacing_y1 * i;
      double waypoint_x2 = points_[1][0] + spacing_x2 * i;
      double waypoint_y2 = points_[1][1] + spacing_y2 * i;
      v1.push_back(waypoint_x1);
      v1.push_back(waypoint_y1);
      v2.push_back(waypoint_x2);
      v2.push_back(waypoint_y2);
      if (i%2 == 0)
      {
        waypoints1.push_back(v1);
        waypoints1.push_back(v2);
      } else {
        waypoints1.push_back(v2);
        waypoints1.push_back(v1);        
      }

      ROS_INFO("%lf", waypoints1[i*2][1]);
      ROS_INFO("%lf", waypoints1[i*2+1][1]);
    }
    return waypoints1;
  }

  void PathAllGenerat() {
    // boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> odom = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", ros::Duration(10));
    // geometry_msgs::PoseWithCovarianceStamped odom_msg = *odom;
    // boost::shared_ptr<geometry_msgs::Point const> gps_point = ros::topic::waitForMessage<geometry_msgs::Point>("/odom/fixed", ros::Duration(10));
    // geometry_msgs::Point gps_odom = *gps_point;
    geometry_msgs::PoseWithCovarianceStamped odom_msg;
    geometry_msgs::Point gps_odom;
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = 1.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    gps_odom.x = 9999.0;
    gps_odom.y = 10001.0;
    std::vector<std::vector<double>> waypoints = WaypointsGenerat();
    std::vector<std::vector<geometry_msgs::Pose>> pose_vectors;

    nav_msgs::Path path;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";

    for (int i = 0; i < waypoints.size() - 1; ++i)
    // for (int i = 0; i < waypoints.size(); i += 1)
      {
        std::vector<geometry_msgs::Pose> pose_vector = PathGenerat(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][1], odom_msg, gps_odom);
        // pose_vectors.push_back(pose_vector);
        pose_stamped.pose = pose_vector[0];
        path.poses.push_back(pose_stamped);
        pose_stamped.pose = pose_vector[1];
        path.poses.push_back(pose_stamped);
      }   

    // std::vector<std::vector<double>> orientation_paths;
    ros::Rate loop_rate(5);
    while(ros::ok()) {
      path_pub_.publish(path);
      loop_rate.sleep();
    } 
  }

  std::vector<geometry_msgs::Pose> PathGenerat(double x1, double y1, double x2, double y2, geometry_msgs::PoseWithCovarianceStamped odom_msg, geometry_msgs::Point gps_odom) {


    // tf2::Transform tx_odom_tf2, tf_two_one, pose_new;
    // tf2::convert(odom_msg.pose.pose, tx_odom_tf2);
    ROS_INFO("generat");
    // double x, y;

    double point1_odom_x, point1_odom_y, point2_odom_x, point2_odom_y;
    double x, y;

    // point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    // point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    point1_odom_x = x1 - gps_odom.x + odom_msg.pose.pose.position.x;
    point1_odom_y = y1 - gps_odom.y + odom_msg.pose.pose.position.y;

    point2_odom_x = x2 - gps_odom.x + odom_msg.pose.pose.position.x;
    point2_odom_y = y2 - gps_odom.y + odom_msg.pose.pose.position.y;

    // int point_sum = path_.size()*2;
    // std::vector<std::vector<geometry_msgs::Pose>> paths_all_;
    // x = path_[1].position.x - path_[0].position.x;
    // y = path_[1].position.y - path_[0].position.y;

    x = point2_odom_x - point1_odom_x;
    y = point2_odom_y - point1_odom_y;
    // ROS_INFO("1");
    
    float rad = atan2(y, x);
    std::vector<double> orientation_path = Rpy2Orientation(rad);
    // float rad_utm_base = atan2(points_[0][0] - gps_odom.x, points_[0][0] - gps_odom.y);
    // ROS_INFO("2");

    // Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
    // Eigen::Quaterniond quaternion1(rotation_vector1);
    // double orientation_x = quaternion1.x();
    // double orientation_y = quaternion1.y();
    // double orientation_z = quaternion1.z();
    // double orientation_w = quaternion1.w();

    // tf2::Quaternion q2;
    // q2.setRPY(rad, 0.0, 0.0);
    // ROS_INFO("tf: orientation %lf, %lf, %lf, %lf", q2.x(), q2.y(), q2.z(), q2.w());

    // ROS_INFO("%lf, %lf, %lf, %lf", orientation_x, orientation_y, orientation_z, orientation_w);
    
    // orientation_path = {orientation_x, orientation_y, orientation_z, orientation_w}; 
    // orientation_paths.push_back(orientation_path);   

    geometry_msgs::Pose pose1, pose2;
    std::vector<geometry_msgs::Pose> pose_vector;
    pose1.position.x = point1_odom_x;
    pose1.position.y = point1_odom_y;
    pose1.position.z = 0.0;
    pose1.orientation.x = orientation_path[0];
    pose1.orientation.y = orientation_path[1];
    pose1.orientation.z = orientation_path[2];
    pose1.orientation.w = orientation_path[3];
    pose2.position.x = point2_odom_x;
    pose2.position.y = point2_odom_y;
    pose2.position.z = 0.0;
    pose2.orientation = pose1.orientation;
    pose_vector = {pose1, pose2};

    // geometry_msgs::Pose pose1, pose2;
    // std::vector<geometry_msgs::Pose> pose_vector;
    // pose1.position.x = points_[0][0] - points_[0][0];
    // pose1.position.y = points_[0][1] - points_[0][1];
    // pose1.position.z = 0.0;
    // pose1.orientation.x = orientation_path[0];
    // pose1.orientation.y = orientation_path[1];
    // pose1.orientation.z = orientation_path[2];
    // pose1.orientation.w = orientation_path[3];
    // pose2.position.x = points_[1][0] - points_[0][0];
    // pose2.position.y = points_[1][1] - points_[0][1];
    // pose2.position.z = 0.0;
    // pose_vector = {pose1, pose2};

    // if (waypoint == 0)
    // {
    // ROS_INFO("pub map tf");
    // std::thread thread(&PathFollowerAction::MapTfBroadcast, this, point1_odom_x, point1_odom_y, orientation_path);
    // thread.detach();

    return pose_vector;

    // }
    // return  pose_vector;

    // printf("%lf\n", points_[0][0]);
    // double point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // double point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    // double point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // double point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    // ROS_INFO("3");
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(point1_odom_x, point1_odom_y, 0.0) );
    // tf::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transform.setRotation(q);

  }

  // void PathGenerat(double x1, double y1, double x2, double y2) {


  //   // tf2::Transform tx_odom_tf2, tf_two_one, pose_new;
  //   // tf2::convert(odom_msg.pose.pose, tx_odom_tf2);
  //   ROS_INFO("generat");
  //   // double x, y;

  //   double point1_odom_x, point1_odom_y, point2_odom_x, point2_odom_y;
  //   double x, y;

  //   point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

  //   point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;
  //   // int point_sum = path_.size()*2;
  //   // std::vector<std::vector<geometry_msgs::Pose>> paths_all_;
  //   // x = path_[1].position.x - path_[0].position.x;
  //   // y = path_[1].position.y - path_[0].position.y;

  //   x = point2_odom_x - point1_odom_x;
  //   y = point2_odom_y - point1_odom_y;
  //   // ROS_INFO("1");
    
  //   float rad = atan2(y, x);
  //   std::vector<double> orientation_path = Rpy2Orientation(rad);
  //   // float rad_utm_base = atan2(points_[0][0] - gps_odom.x, points_[0][0] - gps_odom.y);
  //   // ROS_INFO("2");

  //   // Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
  //   // Eigen::Quaterniond quaternion1(rotation_vector1);
  //   // double orientation_x = quaternion1.x();
  //   // double orientation_y = quaternion1.y();
  //   // double orientation_z = quaternion1.z();
  //   // double orientation_w = quaternion1.w();

  //   // tf2::Quaternion q2;
  //   // q2.setRPY(rad, 0.0, 0.0);
  //   // ROS_INFO("tf: orientation %lf, %lf, %lf, %lf", q2.x(), q2.y(), q2.z(), q2.w());

  //   // ROS_INFO("%lf, %lf, %lf, %lf", orientation_x, orientation_y, orientation_z, orientation_w);
    
  //   // orientation_path = {orientation_x, orientation_y, orientation_z, orientation_w}; 
  //   // orientation_paths.push_back(orientation_path);   
  //   geometry_msgs::Pose pose1, pose2;
  //   std::vector<geometry_msgs::Pose> pose_vector;
  //   pose1.position.x = points_[0][0] - points_[0][0];
  //   pose1.position.y = points_[0][1] - points_[0][1];
  //   pose1.position.z = 0.0;
  //   pose1.orientation.x = orientation_path[0];
  //   pose1.orientation.y = orientation_path[1];
  //   pose1.orientation.z = orientation_path[2];
  //   pose1.orientation.w = orientation_path[3];
  //   pose2.position.x = points_[1][0] - points_[0][0];
  //   pose2.position.y = points_[1][1] - points_[0][1];
  //   pose2.position.z = 0.0;
  //   pose_vector = {pose1, pose2};

  //   // if (waypoint == 0)
  //   // {
  //   ROS_INFO("pub map tf");
  //   std::thread thread(&PathFollowerAction::MapTfBroadcast, this, point1_odom_x, point1_odom_y, orientation_path);
  //   thread.detach();
  //   // }
  //   // return  pose_vector;

  //   // printf("%lf\n", points_[0][0]);
  //   // double point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   // double point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

  //   // double point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
  //   // double point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;

  //   // ROS_INFO("3");
  //   // tf::Transform transform;
  //   // transform.setOrigin( tf::Vector3(point1_odom_x, point1_odom_y, 0.0) );
  //   // tf::Quaternion q;
  //   // q.setRPY(0, 0, 0);
  //   // transform.setRotation(q);
  //   nav_msgs::Path path;
  //   path.header.frame_id = "map";
  //   geometry_msgs::PoseStamped pose_stamped;
  //   pose_stamped.header.frame_id = "map";

  //   std::vector<std::vector<double>> orientation_paths;
  //   // std::vector<double> orientation_path;

  //   // double point1_odom_x, point1_odom_y, point2_odom_x, point2_odom_y;

  //   // for (int waypoint = 0; waypoint < 3; ++waypoint)
  //   // {
  //   // std::vector<geometry_msgs::Pose> poses = WaypointsGenerat(odom_msg, gps_odom, waypoint);
    
  //   pose_stamped.pose = pose_vector[0];
  //   path.poses.push_back(pose_stamped);
  //   pose_stamped.pose = pose_vector[1];
  //   path.poses.push_back(pose_stamped);
  //   // }
  //   // std::vector<double> orientation_path = Rpy2Orientation();

  //   // geometry_msgs::Pose pose1, pose2;
  //   // pose1.position.x = 0.0;
  //   // pose1.position.y = 0.0;
  //   // pose1.position.z = 0.0;
  //   // pose1.orientation.x = orientation_path[0];
  //   // pose1.orientation.y = orientation_path[1];
  //   // pose1.orientation.z = orientation_path[2];
  //   // pose1.orientation.w = orientation_path[3];
  //   // pose2.position.x = x;
  //   // pose2.position.y = y;
  //   // pose2.position.z = 0.0;
  //   // pose2.orientation.x = orientation_path[0];
  //   // pose2.orientation.y = orientation_path[1];
  //   // pose2.orientation.z = orientation_path[2];
  //   // pose2.orientation.w = orientation_path[3];


  //   // pose_stamped.pose = pose1;
  //   // path.poses.push_back(pose_stamped);

  //   // pose_stamped.pose = pose2;
  //   // path.poses.push_back(pose_stamped);
  //   // for (int base_x = 0; base_x < 2; base_x += 1)
  //   // {
  //   //   pose2.position.x += cos_x;
  //   //   pose2.position.y += sin_y;
  //   //   // pose_new.position.x += 0.1;
  //   //   pose_stamped.pose = pose2;
  //   //   path.poses.push_back(pose_stamped);
  //   // }
  //   ros::Rate loop_rate(5);
  //   while(ros::ok()) {
  //     path_pub_.publish(path);
  //     loop_rate.sleep();
  //   }
  // }

  void MapTfBroadcast(double point1_odom_x,double point1_odom_y, std::vector<double> orientation_path) {

    map_transform_down_.header.stamp = ros::Time::now();
    map_transform_down_.header.frame_id = "map";
    map_transform_down_.child_frame_id = "odom";
    // ROS_INFO("%lf %lf", point1_odom_x, point1_odom_y);
    map_transform_down_.transform.translation.x = -point1_odom_x;
    map_transform_down_.transform.translation.y = -point1_odom_y;
    map_transform_down_.transform.translation.z = 0.0;
    // tf2::Quaternion q2;
    // q2.setRPY(0.0, 0.0, 0.0);
    map_transform_down_.transform.rotation.x = orientation_path[0];
    map_transform_down_.transform.rotation.y = orientation_path[1];
    map_transform_down_.transform.rotation.z = orientation_path[2];
    map_transform_down_.transform.rotation.w = orientation_path[3];
    // table_updated = true;
    // table_receive_data = true;
    static tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate1(100);
    while (ros::ok()) {
      br.sendTransform(map_transform_down_);
      loop_rate1.sleep();
    }
  }

  void PathSequentGenerat() {

    // boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> odom = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", ros::Duration(10));
    // geometry_msgs::PoseWithCovarianceStamped odom_msg = *odom;
    // boost::shared_ptr<geometry_msgs::Point const> gps_point = ros::topic::waitForMessage<geometry_msgs::Point>("/odom/fixed", ros::Duration(10));
    // geometry_msgs::Point gps_odom = *gps_point;
    geometry_msgs::PoseWithCovarianceStamped odom_msg;
    geometry_msgs::Point gps_odom;
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = -1.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    gps_odom.x = 1.0;
    gps_odom.y = -1.0;

    tf2::Transform tx_odom_tf2, tf_two_one, pose_new;
    tf2::convert(odom_msg.pose.pose, tx_odom_tf2);

    ROS_INFO("generat");
    double x, y;

    double point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    double point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    double point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    double point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;
    // int point_sum = path_.size()*2;
    // std::vector<std::vector<geometry_msgs::Pose>> paths_all_;
    // x = path_[1].position.x - path_[0].position.x;
    // y = path_[1].position.y - path_[0].position.y;

    x = point2_odom_x - point1_odom_x;
    y = point2_odom_y - point1_odom_y;
    
    float rad = atan2(y, x);

    Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond quaternion1(rotation_vector1);
    double orientation_x = quaternion1.x();
    double orientation_y = quaternion1.y();
    double orientation_z = quaternion1.z();
    double orientation_w = quaternion1.w();
    ROS_INFO("%lf, %lf, %lf, %lf", orientation_x, orientation_y, orientation_z, orientation_w);

    geometry_msgs::Pose pose;
    pose.position.x = path_[0].position.x;
    pose.position.y = path_[0].position.y;
    pose.position.x = 0.0;
    pose.orientation.x = orientation_x;
    pose.orientation.y = orientation_y;
    pose.orientation.z = orientation_z;
    pose.orientation.w = orientation_w;
    tf2::convert(pose, tf_two_one);
    pose_new = tx_odom_tf2 * tf_two_one;

    geometry_msgs::Pose pose2;
    tf2::toMsg(pose_new, pose2);

    double c = sqrt(abs(x) * abs(x) + abs(y) * abs(y));

    // pose2_x = pose2.position.x + x;
    // pose2_y = pose2.position.y + y;
    double cos_x = cos(rad) * 0.1;
    double sin_y = sin(rad) * 0.1;
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "odom";
    for (int base_x = 0; base_x < c; base_x += 0.1)
    {
      pose2.position.x += cos_x;
      pose2.position.y += sin_y;
      // pose_new.position.x += 0.1;
      pose_stamped.pose = pose2;
      path.poses.push_back(pose_stamped);
    }
    ros::Rate loop_rate(5);
    while(ros::ok()) {
      path_pub_.publish(path);
      loop_rate.sleep();
    }
  
    // boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> odom = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", ros::Duration(10));
    // geometry_msgs::PoseWithCovarianceStamped odom_msg = *odom;
    // boost::shared_ptr<geometry_msgs::Point const> gps_point = ros::topic::waitForMessage<geometry_msgs::Point>("/odom/fixed", ros::Duration(10));
    // geometry_msgs::Point gps_odom = *gps_point;


    // geometry_msgs::PoseWithCovarianceStamped odom_msg;
    // geometry_msgs::Point gps_odom;
    // odom_msg.pose.pose.position.x = 1.0;
    // odom_msg.pose.pose.position.y = -1.0;
    // odom_msg.pose.pose.position.z = 0.0;
    // odom_msg.pose.pose.orientation.x = 0.0;
    // odom_msg.pose.pose.orientation.y = 0.0;
    // odom_msg.pose.pose.orientation.z = 0.0;
    // odom_msg.pose.pose.orientation.w = 1.0;
    // gps_odom.x = 1.0;
    // gps_odom.y = -1.0;

    // // tf2::Transform tx_odom_tf2, tf_two_one, pose_new;
    // // tf2::convert(odom_msg.pose.pose, tx_odom_tf2);

    // ROS_INFO("generat");
    // double x, y;
    // // printf("%lf\n", points_[0][0]);
    // double point1_odom_x = points_[0][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // double point1_odom_y = points_[0][1] - gps_odom.y + odom_msg.pose.pose.position.y;

    // double point2_odom_x = points_[1][0] - gps_odom.x + odom_msg.pose.pose.position.x;
    // double point2_odom_y = points_[1][1] - gps_odom.y + odom_msg.pose.pose.position.y;
    // // int point_sum = path_.size()*2;
    // // std::vector<std::vector<geometry_msgs::Pose>> paths_all_;
    // // x = path_[1].position.x - path_[0].position.x;
    // // y = path_[1].position.y - path_[0].position.y;

    // x = point2_odom_x - point1_odom_x;
    // y = point2_odom_y - point1_odom_y;
    // // ROS_INFO("1");
    
    // float rad = atan2(y, x);
    // // ROS_INFO("2");

    // Eigen::AngleAxisd rotation_vector1 (rad, Eigen::Vector3d(0, 0, 1));
    // Eigen::Quaterniond quaternion1(rotation_vector1);
    // double orientation_x = quaternion1.x();
    // double orientation_y = quaternion1.y();
    // double orientation_z = quaternion1.z();
    // double orientation_w = quaternion1.w();
    // ROS_INFO("%lf, %lf, %lf, %lf", orientation_x, orientation_y, orientation_z, orientation_w);
    // // ROS_INFO("3");

    // geometry_msgs::Pose pose1, pose2;
    // pose1.position.x = point1_odom_x;
    // pose1.position.y = point1_odom_y;
    // pose1.position.z = 0.0;
    // pose1.orientation.x = orientation_x;
    // pose1.orientation.y = orientation_y;
    // pose1.orientation.z = orientation_z;
    // pose1.orientation.w = orientation_w;
    // pose2.position.x = point2_odom_x;
    // pose2.position.y = point2_odom_y;
    // pose2.position.z = 0.0;
    // pose2.orientation.x = orientation_x;
    // pose2.orientation.y = orientation_y;
    // pose2.orientation.z = orientation_z;
    // pose2.orientation.w = orientation_w;

    // nav_msgs::Path path;
    // path.header.frame_id = "map";
    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.frame_id = "map";
    // pose_stamped.pose = pose1;
    // path.poses.push_back(pose_stamped);

    // pose_stamped.pose = pose2;
    // path.poses.push_back(pose_stamped);
    // // for (int base_x = 0; base_x < 2; base_x += 1)
    // // {
    // //   pose2.position.x += cos_x;
    // //   pose2.position.y += sin_y;
    // //   // pose_new.position.x += 0.1;
    // //   pose_stamped.pose = pose2;
    // //   path.poses.push_back(pose_stamped);
    // // }
    // ros::Rate loop_rate(5);
    // while(ros::ok()) {
    //   path_pub_.publish(path);
    // }
  }

  bool FollowPathGoal(GoalHandle gh) {
    path_msgs::FollowPathGoal fp_goal;
    // move_base_msgs::MoveBaseGoal mb_goal;
    // fp_goal.path.header.stamp = ros::Time::now();
    fp_goal.path.header.frame_id = "map";
    fp_goal.follower_options.robot_controller.data = "kinematic_hbz";
    fp_goal.follower_options.velocity = 0.6;
    // fp_goal.follower_options.collision_avoider.data = "CollisionAvoiderDifferential";
    path_msgs::DirectionalPath paths;
    paths.forward = true;
    paths.header.frame_id = "map";
    for (int i = 1; i < 40; ++i)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = 64.7528722977; 
      pose.pose.position.y = 0.1 * i + 86.8865691918;
      pose.pose.orientation.z = 0.579524095486;
      pose.pose.orientation.w = 0.814955104746;
      paths.poses.push_back(pose);
    }
    fp_goal.path.paths.push_back(paths);
    // move_goal_flag = false;
    ros::Rate loop_rate(10);
    bool follow_finished = false;
    while(ros::ok () and !follow_finished) {
      fpc_.sendGoal(fp_goal);
      loop_rate.sleep();
      fpc_.waitForResult(ros::Duration(5.0));
      if (fpc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        follow_finished = true;
        return true;
      } else if(fpc_.getState() == actionlib::SimpleClientGoalState::PREEMPTED or fpc_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        return false;
        break;
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follow_server");

  PathFollowerAction averaging("path_follow_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  // ROS_INFO("%s, %s", gh.getGoal()->items.c_str(), gh.getGoal()->place.c_str());
  // std::thread thread(&PathFollowerAction::PathGenerat);
  // thread.start();
  averaging.PathAllGenerat();
  // averaging.WaypointsGenerat();
  ros::waitForShutdown();
  // return 0;
}