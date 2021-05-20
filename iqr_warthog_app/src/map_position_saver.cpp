#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>
#include <std_srvs/Empty.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class MapPositionSaver
{
public:

  ros::NodeHandle nh_;

  std::map<std::string, std::vector<double>> mm;

  geometry_msgs::TransformStamped map_transform_down_;
  ros::Subscriber angle_sub_, gps_odom_sub_, init_pose_sub_;
  
  geometry_msgs::Pose init_position_;

  MapPositionSaver(std::string name)
  {
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1000, MapPositionSaver::OdomCallback, this);
    // gps_odom_sub_ = nh_.subscribe("gps/odom", 1000, &MapPositionSaver::GpsOdomCallback, this);
    // init_pose_sub_ = nh_.subscribe("/initialpose", 1000, &MapPositionSaver::GpsPoseCallback, this);

    GetParam();
  }

  ~MapPositionSaver(void)
  {
  }

  void GetParam() {

    init_position_ = GetGoalPose("init_pose");

  }

  geometry_msgs::Pose GetGoalPose(std::string goal_name) {

    std::vector<float> pose_vector(3), orientation_vector(4);
    nh_.getParam(goal_name+"/pose", pose_vector);
    nh_.getParam(goal_name+"/orientation", orientation_vector);

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

  void GpsPoseCallback(geometry_msgs::PoseWithCovarianceStamped msg) {
    ros::topic::waitForMessage<nav_msgs::Odometry>("gps/odom", ros::Duration(10));

    tf2::Transform tx_odom_tf2;
    // tf2::convert(odom_msg, tx_odom_tf2);
    tf2::Transform pose_old, pose_new, tx_odom_tf2_inverse;
    tf2::convert(msg.pose.pose, pose_old);
    tx_odom_tf2_inverse = tx_odom_tf2.inverse();
    pose_new = pose_old * tx_odom_tf2_inverse;

    geometry_msgs::Pose pose;
    tf2::toMsg(pose_new, pose);

    // map_transform_down_.header.stamp = ros::Time::now();
    // map_transform_down_.header.frame_id = "map";
    // map_transform_down_.child_frame_id = "odom";
    map_transform_down_.transform.translation.x = pose.position.x;
    map_transform_down_.transform.translation.y = pose.position.y;
    map_transform_down_.transform.translation.z = pose.position.z;

    // tf2::Quaternion q2;
    // q2.setRPY(0.0, M_PI/2.0, 0.0);
    map_transform_down_.transform.rotation.x = pose.orientation.x;
    map_transform_down_.transform.rotation.y = pose.orientation.y;
    map_transform_down_.transform.rotation.z = pose.orientation.z;
    map_transform_down_.transform.rotation.w = pose.orientation.w;
  }

  void GpsOdomCallback() {

    // geometry_msgs::TransformStamped tx_odom;
    // try
    // {
    //   ros::Time now = ros::Time::now();
    //   // wait a little for the latest tf to become available
    //   tx_odom = tf_->lookupTransform("base_link", msg.header.stamp,
    //                                  "base_link", ros::Time::now(),
    //                                  "map", ros::Duration(0.5));
    // }
    // catch(tf2::TransformException e)
    // {
    //   // If we've never sent a transform, then this is normal, because the
    //   // global_frame_id_ frame doesn't exist.  We only care about in-time
    //   // transformation for on-the-move pose-setting, so ignoring this
    //   // startup condition doesn't really cost us anything.
    //   // if(sent_first_transform_)
    //   //   ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    //   ROS_ERROR("%s",e.what());
    //   // ros::Duration(1.0).sleep();
    //   continue;
    // }
    // geometry_msgs::Pose pose;
    // tf2::toMsg(tx_odom, pose);


  }

  void Thread() {
    std::thread thread(&MapPositionSaver::GpsOdomCallback, this);
    thread.detach();    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_task_server");
  MapPositionSaver averaging("pick_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  averaging.Thread();

  ros::waitForShutdown();
}