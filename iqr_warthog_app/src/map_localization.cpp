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

class MapLocalization
{
public:

  ros::NodeHandle nh_;

  std::map<std::string, std::vector<double>> mm;

  geometry_msgs::TransformStamped map_transform_down_;
  ros::Subscriber angle_sub_, gps_odom_sub_, init_pose_sub_;
  
  geometry_msgs::Pose init_position_;

  MapLocalization(std::string name)
  {
    init_pose_sub_ = nh_.subscribe("/initialpose", 1000, &MapLocalization::GpsPoseCallback, this);

    GetParam();
  }

  ~MapLocalization(void)
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

    boost::shared_ptr<nav_msgs::Odometry const> odom = ros::topic::waitForMessage<nav_msgs::Odometry>("gps/odom", ros::Duration(10));
    nav_msgs::Odometry odom_msg = *odom;
    tf2::Transform tx_odom_tf2;
    tf2::convert(odom_msg.pose.pose, tx_odom_tf2);
    tf2::Transform pose_old, pose_new, tx_odom_tf2_inverse;
    tf2::convert(msg.pose.pose, pose_old);
    tx_odom_tf2_inverse = tx_odom_tf2.inverse();
    pose_new = pose_old * tx_odom_tf2_inverse;

    map_transform_down_.transform.translation.x = pose_new.getOrigin().x();
    map_transform_down_.transform.translation.y = pose_new.getOrigin().y();
    map_transform_down_.transform.translation.z = pose_new.getOrigin().z();

    map_transform_down_.transform.rotation.x = pose_new.getRotation().x();
    map_transform_down_.transform.rotation.y = pose_new.getRotation().y();
    map_transform_down_.transform.rotation.z = pose_new.getRotation().z();
    map_transform_down_.transform.rotation.w = pose_new.getRotation().w();
  }

  void Map2OdomTfPub() {
  
    boost::shared_ptr<nav_msgs::Odometry const> odom = ros::topic::waitForMessage<nav_msgs::Odometry>("gps/odom", ros::Duration(10));
    nav_msgs::Odometry odom_msg = *odom;

    init_position_.position.x = 0.0; 
    init_position_.position.y = 0.0;
    init_position_.position.z = 0.0;
    init_position_.orientation.x = 0.0;
    init_position_.orientation.y = 0.0;
    init_position_.orientation.z = 0.0;
    init_position_.orientation.w = 1.0;
    tf2::Transform tx_odom_tf2;
    tf2::convert(odom_msg.pose.pose, tx_odom_tf2);
    tf2::Transform pose_old, pose_new, tx_odom_tf2_inverse;
    tf2::convert(init_position_, pose_old);
    tx_odom_tf2_inverse = tx_odom_tf2.inverse();
    pose_new = pose_old * tx_odom_tf2_inverse;
    pose_new = pose_new.inverse();

    map_transform_down_.header.frame_id = "map";
    map_transform_down_.child_frame_id = "odom";

    map_transform_down_.transform.translation.x = pose_new.getOrigin().x();
    map_transform_down_.transform.translation.y = pose_new.getOrigin().y();
    map_transform_down_.transform.translation.z = pose_new.getOrigin().z();

    map_transform_down_.transform.rotation.x = pose_new.getRotation().x();
    map_transform_down_.transform.rotation.y = pose_new.getRotation().y();
    map_transform_down_.transform.rotation.z = pose_new.getRotation().z();
    map_transform_down_.transform.rotation.w = pose_new.getRotation().w();

    tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate(100);
    while (ros::ok()) {
      map_transform_down_.header.stamp = ros::Time::now();

      br.sendTransform(map_transform_down_);
      loop_rate.sleep();
    }
  }

  void Thread() {
    std::thread thread(&MapLocalization::Map2OdomTfPub, this);
    thread.detach();    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_task_server");
  MapLocalization ml("pick_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ml.Thread();

  ros::waitForShutdown();
}