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
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <kdl/kdl.hpp>
#include "tf2_kdl/tf2_kdl.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nmea_ros_driver/GPSPoint.h>
#include <nmea_ros_driver/GPSRPY.h>

#define pi 3.1415926

typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;
typedef actionlib::ServerGoalHandle<iqr_warthog_app::WaypointNavAction> GoalHandle;
typedef actionlib::ActionServer<iqr_warthog_app::WaypointNavAction> Server;
bool move_goal_flag, cabine_flag = false;

class PathFollowerAction
{
public:

  ros::NodeHandle nh_;
  Server as_;
  FollowPathClient fpc_;
  std::string action_name_;
  std::vector<std::vector<geometry_msgs::Pose>> paths_, paths_all_;
  std::vector<geometry_msgs::Pose> path_;
  std::vector<std::vector<double>> points_;
  geometry_msgs::TransformStamped map_transform_down_;

  geometry_msgs::Pose odom_pose_;
  int column_;
  bool path_publish_ = false, task_active_ = false;
  std::map<std::string, std::vector<double>> mm;

  ros::Publisher base_cmd_pub_, path_pub_;
  ros::Subscriber odom_sub_;
  
  PathFollowerAction(std::string name) : 
    as_(nh_, name, boost::bind(&PathFollowerAction::ExecuteCb, this, _1), boost::bind(&PathFollowerAction::preemptCB, this, _1), false),
    fpc_("/follow_path", true)
  {
    base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
    // odom_sub_ = nh_.subscribe("robot_pose_ekf/odom_combined", 1000, &PathFollowerAction::odomCallback, this);
    odom_sub_ = nh_.subscribe("gps/odom", 1000, &PathFollowerAction::odomCallback, this);
    GetParam();
    as_.start();
  }

  ~PathFollowerAction(void)
  {
  }

  void ExcuteThread(GoalHandle gh) {
    NavTask(gh);
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
    get_goal = true;    
    gh.setCanceled();
    task_active_ = false;
  }

  void GetParam() {
    std::vector<geometry_msgs::Pose> path;
    std::vector<double> point;
    point.resize(3);
    for (int point_num = 1; point_num <= 4; ++point_num)
      {
        nh_.getParam("waypoints/point" + std::to_string(point_num), point);
        // printf("%lf\n", point[0]);
        points_.push_back(point);
      }
    printf("get params finished");
    nh_.getParam("waypoints/conlum", column_);
    printf("get params finished");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ROS_INFO("odom callback");
    // ROS_INFO("%lf", msg->pose.pose.orientation.w);
    
    odom_pose_ = msg->pose.pose;
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
    std::vector<nav_msgs::Path> paths = PathAllGenerat();
    int step = 1;
    for (int i = 0; i < paths.size(); ++i)
    {
      ros::spinOnce();
      HeadAlign(paths[i]);
      ros::WallDuration(1.0).sleep();
      FollowPathGoal(paths[i]);
    }
  }

  Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
  {
      Eigen::Quaterniond q;
      q.x() = x;
      q.y() = y;
      q.z() = z;
      q.w() = w;

      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
      std::cout << "Quaterniond2Euler result is:" <<std::endl;
      std::cout << "x = "<< euler[2] << std::endl ;
      std::cout << "y = "<< euler[1] << std::endl ;
      std::cout << "z = "<< euler[0] << std::endl << std::endl;
      return euler;
  }

  double toEulerAngle(const double x,const double y,const double z,const double w)
  {
  // roll (x-axis rotation)
      double sinr_cosp = +2.0 * (w * x + y * z);
      double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
      double roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
      double sinp = +2.0 * (w * y - z * x);
      if (fabs(sinp) >= 1)
          double pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
         double pitch = asin(sinp);

  // yaw (z-axis rotation)
      double siny_cosp = +2.0 * (w * z + x * y);
      double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
      double yaw = atan2(siny_cosp, cosy_cosp);
     return yaw;
  }

  void HeadAlign(nav_msgs::Path path) {
    tf2::Quaternion quat;
    geometry_msgs::Quaternion quat_msg, quat2_msg;
    quat_msg = path.poses[0].pose.orientation;
    tf2::convert(quat_msg, quat);
    double yaw_goal = toEulerAngle(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    double angle_now;
    bool goal_reached = false;

    ros::Rate loop_rate(50);

    while (!goal_reached) {

      quat2_msg = odom_pose_.orientation;
      tf2::Quaternion quat2;
      tf2::convert(quat2_msg, quat2);
      double yaw_now = toEulerAngle(quat2_msg.x, quat2_msg.y, quat2_msg.z, quat2_msg.w);

      if (yaw_now < 0)
      {
        yaw_now = pi * 2 - abs(yaw_now);
      }

      if (yaw_goal < 0)
      {
        yaw_goal = pi * 2 - abs(yaw_goal);
      }

      double angle_diff = yaw_now - yaw_goal;

      if (angle_diff > pi) {
        angle_diff = angle_diff - pi * 2;
      }

      if (angle_diff < -pi) {
        angle_diff = angle_diff + pi * 2;
      }

      // ROS_INFO("angle_diff: %lf", angle_diff);
      float vel = 0.0;
      if (angle_diff > -0.1 and angle_diff < 0.1) {
        goal_reached = true;
        geometry_msgs::Twist cmd_msg_back;
        cmd_msg_back.angular.z = 0.0;
        base_cmd_pub_.publish(cmd_msg_back);
      } else {
        geometry_msgs::Twist cmd_msg_back;
        if (angle_diff > 0)
        {
          vel = -1.0;
        } else {
          vel = 1.0;
        }
        cmd_msg_back.angular.z = vel;
        ROS_INFO("angle_goal: %lf angle_now: %lf vel: %f", angle_diff, yaw_now, cmd_msg_back.angular.z);
        base_cmd_pub_.publish(cmd_msg_back);
        loop_rate.sleep();
      }
    }
    ROS_INFO("Head align finished");
  }

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
    
    double spacing_x1 = (points_[3][0] - points_[0][0]) / double(column_-1);
    double spacing_y1 = (points_[3][1] - points_[0][1]) / double(column_-1);
    double spacing_x2 = (points_[2][0] - points_[1][0]) / double(column_-1);
    double spacing_y2 = (points_[2][1] - points_[1][1]) / double(column_-1);
    // ROS_INFO("%lf", spacing_y);
    std::vector<std::vector<double>> waypoints1, waypoints2;
    for (int i = 0; i < column_; ++i)
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

  std::vector<nav_msgs::Path> PathAllGenerat() {
    ROS_INFO("path all generate");
    boost::shared_ptr<nav_msgs::Odometry const> odom = ros::topic::waitForMessage<nav_msgs::Odometry>("gps/odom", ros::Duration(10));
    nav_msgs::Odometry odom_msg = *odom;
    boost::shared_ptr<nmea_ros_driver::GPSPoint const> gps_point = ros::topic::waitForMessage<nmea_ros_driver::GPSPoint>("/gps/point", ros::Duration(10));
    nmea_ros_driver::GPSPoint gps_odom = *gps_point;
    ROS_INFO("recevied topics");
    std::vector<std::vector<double>> waypoints = WaypointsGenerat();
    std::vector<std::vector<geometry_msgs::Pose>> pose_vectors;

    std::vector<nav_msgs::Path> paths;
    nav_msgs::Path path, path_publish;
    path.header.frame_id = "map";
    path_publish.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    std::vector<geometry_msgs::Pose> pose_vector;

    for (int i = 0; i < waypoints.size() - 1; ++i)
      {
        pose_vector = PathGenerat(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][1], odom_msg, gps_odom);

        for (int i = 0; i < pose_vector.size(); ++i)
        {
          pose_stamped.pose = pose_vector[i];
          path.poses.push_back(pose_stamped);
          path_publish.poses.push_back(pose_stamped);
          // ROS_INFO("%i", i);
        }
        paths.push_back(path);
        path.poses.clear();
      }   
    if (column_%2 == 0)
    {
      pose_vector = PathGenerat(waypoints[waypoints.size() -1 ][0], waypoints[waypoints.size() - 1][1], waypoints[0][0], waypoints[0][1], odom_msg, gps_odom);
      // pose_vectors.push_back(pose_vector);

      for (int i = 0; i < pose_vector.size(); ++i)
      {
        pose_stamped.pose = pose_vector[i];
        path.poses.push_back(pose_stamped);
        path_publish.poses.push_back(pose_stamped);
        // ROS_INFO("%i", i);
      }
    } else {
      pose_vector = PathGenerat(waypoints[waypoints.size() -1 ][0], waypoints[waypoints.size() - 1][1], waypoints[waypoints.size() -2 ][0], waypoints[waypoints.size() - 2][1], odom_msg, gps_odom);

      for (int i = 0; i < pose_vector.size(); ++i)
      {
        pose_stamped.pose = pose_vector[i];
        path.poses.push_back(pose_stamped);
        path_publish.poses.push_back(pose_stamped);
        // ROS_INFO("%i", i);
      }
      paths.push_back(path);
      path.poses.clear();
      pose_vector = PathGenerat(waypoints[waypoints.size() -2 ][0], waypoints[waypoints.size() - 2][1], waypoints[0][0], waypoints[0][1], odom_msg, gps_odom);

      for (int i = 0; i < pose_vector.size(); ++i)
      {
        pose_stamped.pose = pose_vector[i];
        path.poses.push_back(pose_stamped);
        path_publish.poses.push_back(pose_stamped);
        // ROS_INFO("%i", i);
      }
    }

    if(!path_publish_) {
      std::thread thread(&PathFollowerAction::PathPublisher, this, path_publish);
      thread.detach();
      path_publish_ = true;
    }
    paths.push_back(path);
    path.poses.clear();
    return paths;

    ROS_INFO("paths size %i", paths.size());
    ros::Rate loop_rate(10);
    while(ros::ok()) {
      path_pub_.publish(path);
      // }
      loop_rate.sleep();
    } 
  }

  void PathPublisher(nav_msgs::Path path) {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
      // for (int i =0; i< paths.size(); ++i) {
      path_pub_.publish(path);
      // }
      loop_rate.sleep();
    } 
  }

  std::vector<geometry_msgs::Pose> PathGenerat(double x1, double y1, double x2, double y2, nav_msgs::Odometry odom_msg, nmea_ros_driver::GPSPoint gps_odom) {

    ROS_INFO("generat");
    double point1_odom_x, point1_odom_y, point2_odom_x, point2_odom_y;
    double x, y;

    point1_odom_x = x1 - gps_odom.x + odom_msg.pose.pose.position.x;
    point1_odom_y = -(y1 - gps_odom.y + odom_msg.pose.pose.position.y);

    point2_odom_x = x2 - gps_odom.x + odom_msg.pose.pose.position.x;
    point2_odom_y = -(y2 - gps_odom.y + odom_msg.pose.pose.position.y);

    x = point2_odom_x - point1_odom_x;
    y = point2_odom_y - point1_odom_y;
    
    float rad = atan2(y, x);
    ROS_INFO("%lf", rad);
    std::vector<double> orientation_path = Rpy2Orientation(rad);
    ROS_INFO("%lf %lf %lf %lf ", orientation_path[0], orientation_path[1], orientation_path[2], orientation_path[3]);

    double c = sqrt(abs(x) * abs(x) + abs(y) * abs(y));

    double cos_x = cos(rad) * 1.0;
    double sin_y = sin(rad) * 1.0;
    geometry_msgs::Pose pose1, pose2;
    std::vector<geometry_msgs::Pose> pose_vector;
    pose1.position.x = point1_odom_x;
    pose1.position.y = point1_odom_y;
    pose1.position.z = 0.0;
    pose1.orientation.x = orientation_path[0];
    pose1.orientation.y = orientation_path[1];
    pose1.orientation.z = orientation_path[2];
    pose1.orientation.w = orientation_path[3];
    pose_vector.push_back(pose1);

    for (int base_x = 0; base_x < c * 10.0 - 10.0; base_x +=10)
    {
      pose1.position.x += cos_x;
      pose1.position.y += sin_y;
      pose_vector.push_back(pose1);
    }

    pose2.position.x = point2_odom_x;
    pose2.position.y = point2_odom_y;
    pose2.position.z = 0.0;
    pose2.orientation = pose1.orientation;
    pose_vector.push_back(pose2);
    return pose_vector;
  }



  void MapTfBroadcast(double point1_odom_x,double point1_odom_y, std::vector<double> orientation_path) {

    map_transform_down_.header.stamp = ros::Time::now();
    map_transform_down_.header.frame_id = "map";
    map_transform_down_.child_frame_id = "odom";
    map_transform_down_.transform.translation.x = -point1_odom_x;
    map_transform_down_.transform.translation.y = -point1_odom_y;
    map_transform_down_.transform.translation.z = 0.0;

    map_transform_down_.transform.rotation.x = orientation_path[0];
    map_transform_down_.transform.rotation.y = orientation_path[1];
    map_transform_down_.transform.rotation.z = orientation_path[2];
    map_transform_down_.transform.rotation.w = orientation_path[3];

    static tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate1(100);
    while (ros::ok()) {
      br.sendTransform(map_transform_down_);
      loop_rate1.sleep();
    }
  }


  bool FollowPathGoal(nav_msgs::Path path) {
    path_msgs::FollowPathGoal fp_goal;

    fp_goal.path.header.frame_id = "map";

    fp_goal.follower_options.robot_controller.data = "pbr";
    fp_goal.follower_options.velocity = 1.0;
    path_msgs::DirectionalPath paths;
    paths.forward = true;
    paths.header.frame_id = "map";

    paths.poses = path.poses;
    ROS_INFO("path size %i", paths.poses.size());
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
  ros::waitForShutdown();
}