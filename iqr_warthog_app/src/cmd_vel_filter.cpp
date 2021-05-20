#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>
#include <std_srvs/Empty.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class CmdVelFilter
{
public:

  ros::NodeHandle nh_;

  std::map<std::string, std::vector<double>> mm;

  geometry_msgs::TransformStamped map_transform_down_;
  ros::Subscriber angle_sub_, gps_odom_sub_, init_pose_sub_;
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Pose init_position_;

  CmdVelFilter(std::string name)
  {
    init_pose_sub_ = nh_.subscribe("/gerona/cmd_vel", 1000, &CmdVelFilter::CmdVelCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  ~CmdVelFilter(void)
  {
  }

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    geometry_msgs::Twist cmd_msg_back;
    cmd_msg_back.linear.x = msg->linear.x;
    if (msg->angular.z > -0.3 and msg->angular.z < -0.08)
    {
      // cmd_msg_back.angular.z = -0.3;
    } else if (msg->angular.z < 0.3 and msg->angular.z > 0.08){
      // cmd_msg_back.angular.z = 0.3;
    } else if (msg->angular.z < 0.08 and msg->angular.z > -0.08) {
      cmd_msg_back.angular.z = 0.0;
    } else if (msg->angular.z > -0.6 and msg->angular.z < 0.6) {
      cmd_msg_back.linear.x = msg->linear.x * 1.5;
    }
    else {
      cmd_msg_back.angular.z = msg->angular.z;
    }
    cmd_vel_pub_.publish(cmd_msg_back);
    ROS_INFO("publish filtered cmd_vel");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_filter");
  CmdVelFilter cvf("cmd_vel_filter");
  ros::spin();

}