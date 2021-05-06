#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib
import select
import tf
import PyKDL
from tf_conversions import posemath

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt32MultiArray, Empty, Float64
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

from std_msgs.msg import UInt32MultiArray
from sensor_msgs.msg import JointState, NavSatFix 

class BrainControlInterface:
  _moving = False
  _vel_stop = False
  _drink_pose_saved = False
  _empty_msg = Empty()
  # _pose_drink_marker = Pose()
  _pose_stamped_drink_marker = PoseStamped()
  _pose_stamped_drink_marker.pose.position.x = -0.0236711222678
  _pose_stamped_drink_marker.pose.position.y = 0.586763203144
  _pose_stamped_drink_marker.pose.position.z = 0.256260246038
  _pose_stamped_drink_marker.pose.orientation.x = 90.1911087036
  _pose_stamped_drink_marker.pose.orientation.y = -1.5087274313
  _pose_stamped_drink_marker.pose.orientation.z = 176.417007446
  _pose_stamped_drink_marker.pose.orientation.w = 1.0
  # _joints_person = [-1.7930190102665966, -0.813059885007883, -0.34033973677111184, -1.9760117329847748, 1.618402461808935, -1.2835999695763158, -1.1947441324380401]
  
  # _joints_person = [-0.6559743933739952, 0.9842364948645166, 2.8178790315040456, -1.9159012577044878, 2.192828332601669, 1.3148982376552536, 1.1212547347895212] #  a little to the right
  _joints_person = [-1.0321245920589668, 1.1234470489988744, 2.753361557251895, -1.4093784046996678, 2.114625406139819, 1.3174574023047654, 0.7564660473680183]
  _joints_person1 = [-0.796869463476856, 1.1657176739269937, 2.843103960717058, -1.1297922966120977, 2.275459696482157, 1.6978092702415553, 0.6397798439432781]  #  a little to the forward
  # _joints_table = [0.08517289341015634, 0.7100715174919112, -3.0923816977028733, -2.311939144461844, 0.45694075233048337, 1.4553051503303023, 1.560596419819893]
  _joints_table = [0.04076120347891113, 0.48361224406859976, -3.0863427136150094, -2.34279239832263, 0.45706708603469404, 1.0405597553376298, 1.4736889513351192] # wrist downward
  # _pose_drink = [0.0619054846466, -0.441089421511, 0.323041915894, -64.6799697876, 176.703430176, 121.39691925]
  _joints_drink = [-1.4591108075280257, 0.8634645362744043, 2.823540911981554, -2.323301254936558, -0.15237382703331903, 1.6257926006758503, 1.808233649574219]
  _pose_ready = [-0.0317343100905, -0.523988306522, 0.306685239077, -91.2066879272, 179.187179565, 170.936584473]
  _scale_vel = 0.01           # vel = 1 or -1 * scale
  _scale_step = 0.05          # vel = 1 or -1 * scale
  _translation_speed = 0.1   # speed limit, m/s
  _orientation_speed = 30.0  # speed limit, degree/s
  # _joint_angle_speed = 100.0   # speed limit, degree/s
  _gps_yaw = 0.0
  _angle_odom = 50.0
  def __init__(self):

    self._kinova_pose_init = Pose()
    self._sub_gps = rospy.Subscriber("gps/fixed", NavSatFix, self.GpsPositionCallback)
    self._sub_gps_yaw = rospy.Subscriber("gps/yaw", Float64, self.GpsYawPositionCallback)
    # self._pub_joint_vel = rospy.Publisher('/arm/in/joint_velocity', Base_JointSpeeds, queue_size=1)
    # self._pub = rospy.Publisher('brain_control_msg', brain_control_msg, queue_size=1)
    # self.client = actionlib.SimpleActionClient("/arm/robotiq_2f_85_gripper_controller/gripper_cmd", GripperCommandAction)

  def GpsPositionCallback(self, msg):
    # double x = msg.position.x - odom.x
    # double y = msg.position.y - odom.y
    x = 1.0
    y = 1.0

    # angle_diff = _gps_yaw - self._angle_odom
    angle_diff = 50.0
    rad = angle_diff / 360.0 * 2.0 * math.pi
    (orientation_x, orientation_y, orientation_z, orientation_w) = self.Theta2Quaternion(rad)
    print(rad, orientation_x, orientation_y, orientation_z, orientation_w)
    # pose_drink_marker = markers.markers[0].pose.pose
    # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, rad), PyKDL.Vector(x, y, 0))
    # pose = posemath.toMsg(posemath.fromMsg(odom_ekf.pose.pose)*f)

    br = tf.TransformBroadcaster()

    # while not rospy.is_shutdown():
    br.sendTransform((x, y, 0.0),
                     (orientation_x, orientation_y, orientation_z, orientation_w),
                     rospy.Time.now(),
                     "base_link",
                     "odom") 
    br.sendTransform((0.0, 0.0, 0.0),
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "odom",
                     "map")       
      # rate.sleep()
  def GpsYawPositionCallback(self, msg):
    self._gps_yaw = msg.data
    print(msg.data)
  

  def UDPLinstenLoop(self):
    pass

  def dateCombin(self, data):
    data_combin = "" 
    for index in range(len(data)):
      data_combin = data_combin + data[index]
    return data_combin

  def get_tf_pose(self, pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "/camera_color_frame"
    # pose_stamped.header.stamp = rospy.Time.now()
    # pose_stamped.header.stamp = rospy.get_rostime()
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    get_pose = False
    while not rospy.is_shutdown() and not get_pose:
      try:
        # now = rospy.Time.now() - rospy.Duration(5.0)
        now = rospy.Time.now()
        listener.waitForTransform("/base_link", "camera_color_frame", now, rospy.Duration(0.3))
        pose_stamped_return = listener.transformPose("/base_link", pose_stamped)
        # print(pose_stamped_return.pose.position.x, pose_stamped_return.pose.position.y, pose_stamped_return.pose.position.z, pose_stamped_return.pose.orientation.x, pose_stamped_return.pose.orientation.y, pose_stamped_return.pose.orientation.z, pose_stamped_return.pose.orientation.w)      
        # print(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        get_pose = True
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print("tf echo error")
        continue
        # return
      rate.sleep()
    return pose_stamped_return

  def getTfTransform(self, base_link, target_link):
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    # listener.waitForTransform("/object_link0", "/base_link", rospy.Time(), rospy.Duration(4.0))
    get_pose = False
    while not rospy.is_shutdown() and not get_pose:
      try:
        # now = rospy.Time.now() - rospy.Duration(5.0)
        now = rospy.Time.now()
        listener.waitForTransform(base_link, target_link, now, rospy.Duration(0.3))
        (trans, rot) = listener.lookupTransform(base_link, target_link, now)
        # print(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        get_pose = True
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print("tf echo error")
        continue
        # return
      rate.sleep()
    return (trans, rot)

  def Theta2Quaternion(self, rad):
    rotation = PyKDL.Rotation.RPY(0.0,
                                  0.0,
                                  rad)
    orientation_x = rotation.GetQuaternion()[0]
    orientation_y = rotation.GetQuaternion()[1]
    orientation_z = rotation.GetQuaternion()[2]
    orientation_w = rotation.GetQuaternion()[3]
    return (orientation_x, orientation_y, orientation_z, orientation_w)    

  def Quaternion2Theta(self, pose_stamped, f):
    pose = posemath.toMsg(posemath.fromMsg(pose_stamped.pose)*f)
    rotation = PyKDL.Rotation.Quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)
    theta_x = math.degrees(rotation.GetRPY()[0])
    theta_y = math.degrees(rotation.GetRPY()[1])
    theta_z = math.degrees(rotation.GetRPY()[2])
    return (pose, theta_x, theta_y, theta_z)
                
  def Quaternion2EulerXYZ(self, Q_raw):
    qx_ = Q_raw.orientation.x
    qy_ = Q_raw.orientation.y
    qz_ = Q_raw.orientation.z
    qw_ = Q_raw.orientation.w

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_

  def MapTfBroadcast(self):
    odom_ekf = rospy.wait_for_message("odom", msgtype)
    odom_gps = rospy.wait_for_message("gps/fix", msgtype)
    angle = rospy.wait_for_message("gps/angle", msgtype)

    angle_diff = angle.angle - angle_map
    rad = angle_diff / 360 * 2 * math.pi
    
    x = odom_ekf.pose.pose.position.x
    y = odom_ekf.pose.pose.position.x

    # pose_drink_marker = markers.markers[0].pose.pose
    f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, rad), PyKDL.Vector(x, y, 0))
    pose = posemath.toMsg(posemath.fromMsg(odom_ekf.pose.pose)*f)

    rate = rospy.Rate(30.0)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
      br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                       (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                       rospy.Time.now(),
                       "map",
                       "odom")      
      rate.sleep()

def main():
  rospy.init_node('brain_control_interface')

  BCI = BrainControlInterface()
  time.sleep(0.5)
  # BCI.__move_by_pose()
  # BCI.UDPLinstenLoop()
  # t0 = threading.Thread(target=BCI.OdomTfBroadcast,args=())
  # t0.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass