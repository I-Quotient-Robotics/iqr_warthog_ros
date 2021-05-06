#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
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
#include <geometry_msgs/TransformStamped.h>

#include <iqr_warthog_app/WaypointNavAction.h>
#include <path_msgs/FollowPathAction.h>

#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <path_msgs/DirectionalPath.h>
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;
typedef actionlib::ServerGoalHandle<worthog_app::WaypointNavAction> GoalHandle;
typedef actionlib::ActionServer<worthog_app::WaypointNavAction> Server;
bool move_goal_flag, cabine_flag = false;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}

class PickTaskAction
{
public:

  ros::NodeHandle nh_;
  Server as_;
  FollowPathClient fpc_;
  // GoalHandle goal_handle_;
  // MoveBaseClient move_base_client_;
  std::string action_name_;

  double body_height, height_goal_, gripper_position_, object_distance_;
  int get_new_target, obstacle_id_, object_id_, step, box_count_;
  bool find_table_ = true, find_object_, task_active_ = false, get_goal, away_cabine_, marker_nav, get_object_distanse_ = false, get_table_ = true;
  std::map<std::string, std::vector<double>> mm;
  geometry_msgs::TransformStamped map_transform_down_;

  ros::ServiceClient stop_client, start_client;
  ros::Publisher body_pub, head_pub, planning_scene_diff_publisher, base_cmd_pub;
  ros::Subscriber joint_states_sub, marker_pose_sub, table_pose_sub, odom_sub_, angle_sub_;
  
  geometry_msgs::Pose standby_position_, pre_pick_position_, pick_position_, place_position_, table_pose_, object_pose_;

  PickTaskAction(std::string name) : 
    as_(nh_, name, boost::bind(&PickTaskAction::ExecuteCb, this, _1), boost::bind(&PickTaskAction::preemptCB, this, _1), false),
    fpc_("/follow_path", true)
  {
    base_cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odom_sub_ = nh_.subscriber<nav_msgs::Odometry>("odom", 1000, PickTaskAction::OdomCallback, this);
    angle_sub_ = nh_.subscriber<nav_msgs::Odometry>("gps/angle", 1000, PickTaskAction::AngleCallback, this);

    GetParam();
    as_.start();
  }

  ~PickTaskAction(void)
  {
  }

  void ExcuteThread(GoalHandle gh) {
    FollowPathGoal(gh);
    // PickAndPlace(gh);
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
    std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
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

    // get arm standby pose
    // arm_standby_pose_.resize(6);
    // nh_.getParam("arm_pose/standby", arm_standby_pose_);

    // get navigation position 
    standby_position_ = GetGoalPose("standby");
    pick_position_ = GetGoalPose("pick");
    pre_pick_position_ = GetGoalPose("pre_pick");
  }

  geometry_msgs::Pose GetGoalPose(std::string goal_name) {

    std::vector<float> pose_vector(3), orientation_vector(4);
    nh_.getParam("target_goal/"+goal_name+"/pose", pose_vector);
    nh_.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

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

  void PickAndPlace(GoalHandle gh){
    ROS_INFO("pick and place");
    // get Move Group client
 
    geometry_msgs::PoseStamped target;
    std::vector<std::string> object_ids;

    // ros::WallDuration(1.0).sleep();
    // std::vector<double> arm_box_pose(box_count_);
    std::vector<double> arm_box_pose(6);
    double box_position, box_size; 
    int id;
    std::string box_name = "box_information/" + gh.getGoal()->items;

    nh_.getParam(box_name + "/arm_pose", arm_box_pose);
    nh_.getParam(box_name + "/height", box_position);
    nh_.getParam(box_name + "/size", box_size);

    nh_.getParam(box_name+ "/id", id);

    std::string place = gh.getGoal()->place;
    std::string place_test = "/target_goal/" + place;
    geometry_msgs::Pose place_pose;
    if (nh_.hasParam(place_test))
    {
      place_pose = GetGoalPose(place);
    }
    else {
      ROS_INFO("Parameter not found");
      return;
    }

    step = 1;
    while(true) {
      ros::spinOnce();

      if (step == 1) {
        /* code */
        // feedback_.step_description = "Set arm to ready pose";
        // feedback_.step_index = 1;
        // gh.publishFeedback(feedback_);
        ROS_INFO("setp1: Set arm to ready pose");
        // feedback_.step_index = 2;
        // feedback_.step_description = "Set body height to zero";
      }
    
      else if(step == 30) {
        // gh.publishFeedback(feedback_);
        ros::WallDuration(1.0).sleep();
        ROS_INFO("step30: Move to standby position");
        ros::Rate loop_rate(10);

        for (int i = 0; i < 80; ++i)
        {
          geometry_msgs::Twist cmd_msg_back;
          cmd_msg_back.angular.z = -0.4;
          base_cmd_pub.publish(cmd_msg_back);
          loop_rate.sleep();
        }
        // if (!MoveToGoal(standby_position_, gh))
        // {
        //   return;
        // }
        // feedback_.step_index = 31;
        // feedback_.step_description = "All finished";
      } 
      // if cancel
      if (gh.getGoalStatus().status == 2) {
        step = 1;
        return;
      }
      // if finished
      else if(step == 30) {
        ROS_INFO("%i", step);
        // result_.finished_step = 7;
        // gh.setSucceeded(result_, "all");
        gh.setSucceeded();
        ROS_INFO("set succeeded");
        return;
      }
      step += 1;
    }
  }

  void MapTfBroadcast() {
    ros::Duration timeout = 5.0;
    msgtype angle_msg = ros::topic::waitForMessage("gps/angle",  timeout);
    msgtype base_odom = ros::topic::waitForMessage("odom",  timeout);
    msgtype angle_msg = ros::topic::waitForMessage("gps/fix",  timeout);

    angle_diff = angle_map - angle_msg.angle;
    rad = angle_diff/360 * 2 * 3.1415926;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, rad);
    // geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, base_odom.Quaternion);

    caculate Quaternion;
    map_transform_down_.header.stamp = ros::Time::now();
    map_transform_down_.header.frame_id = "odom";
    map_transform_down_.child_frame_id = "map";
    tf2::Quaternion q2;
    q2.setRPY(0.0, M_PI/2.0, 0.0);
    map_transform_down_.transform.rotation.x = q2.x();
    map_transform_down_.transform.rotation.y = q2.y();
    map_transform_down_.transform.rotation.z = q2.z();
    map_transform_down_.transform.rotation.w = q2.w();
    // table_updated = true;
    // table_receive_data = true;
    tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate(100);
    while (ros::OK()) {
      br.sendTransform(marker_transform);
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_task_server");
  PickTaskAction averaging("pick_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  averaging.MapTfBroadcast();
  ros::waitForShutdown();
  // return 0;
}