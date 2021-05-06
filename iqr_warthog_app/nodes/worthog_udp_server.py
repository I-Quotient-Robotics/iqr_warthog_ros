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
from std_msgs.msg import UInt32MultiArray, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class BrainControlInterface:
  _vel_stop = False

  def __init__(self):

    try:
      self._udpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      self._udpSocket_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

      # computer_liuhang
      self._udpSocket.bind(("127.0.0.1", 8848)) #receive
      self.server_address = ("127.0.0.1", 8848) #send
      print 'Bind UDP on 8848...'
    except:
      print 'Build UDP socket error!! exit~'
      exit(-1)

    self._pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    self.vel_x_base = 0
    self.vel_z_base = 0
    self.vel_scale_base = 0.2
    self.allstart = 0
    self.driver_start = 0
    self.base_stop = True
    self.vel_cmd_msg = Twist()

  def UDPLinstenLoop(self):
    rospy.loginfo("start udp linster..")

    while not rospy.is_shutdown():
      rospy.loginfo("linste..")
      timeout=60*60*5
      self._udpSocket.settimeout(timeout)
      data, addr = self._udpSocket.recvfrom(1024)

      data_combin = self.dateCombin(data)
      rospy.loginfo("%s", data_combin)
KeyboardDisable
KeyboardEnable
StartRecord
StopRecord
AutoNavigation
BaseXForward
BaseXBack
BaseZForward
BaseZBack
BaseXVelUp
BaseXVelDown
BaseXStop
BaseZStop

    if state.code == 13:
      if self.enable_keyboard:
        self.enable_keyboard = False
        self.client_socket.sendto("KeyboardDisable", self.server_address)
        rospy.loginfo("Keyboard Disable")
      else:
        self.enable_keyboard = True
        self.client_socket.sendto("KeyboardEnable", self.server_address)
        rospy.loginfo("Keyboard Enable")

    if self.enable_keyboard:
      if state.code == 119:
        self.client_socket.sendto("StartRecord", self.server_address)
        rospy.loginfo("Start Record")
      elif state.code == 113:
        self.client_socket.sendto("StopRecord", self.server_address)
        rospy.loginfo("Stop Record")
      else state.code == 97:
        rospy.loginfo("auto navigation")
        self.client_socket.sendto("AutoNavigation", self.server_address)
      elif state.code == 273:
        rospy.loginfo("base x forward")
        self.client_socket.sendto("BaseXForward", self.server_address)
      elif state.code == 274:
        rospy.loginfo("base x back")
        self.client_socket.sendto("BaseXBack", self.server_address)
      elif state.code == 275:
        rospy.loginfo("base z base")
        self.client_socket.sendto("BaseZForward", self.server_address)
      elif state.code == 276:
        rospy.loginfo("base z forward")
        self.client_socket.sendto("BaseZBack", self.server_address)
      elif state.code == 280:   
        if self.vel_scale_base <= 1.0:
          rospy.loginfo("base vel up")
          self.vel_scale_base += 0.1
          self.client_socket.sendto("BaseXVelUp", self.server_address)
          rospy.loginfo("%f", self.vel_scale_base)
      elif state.code == 281:
        if self.vel_scale_base >= 0:
          rospy.loginfo("base vel down")
          self.vel_scale_base -= 0.1
          self.client_socket.sendto("BaseXVelDown", self.server_address)
          rospy.loginfo("%f", self.vel_scale_base)

      # stop
      if keys.code == 273 or keys.code == 274:
        self.vel_x_base = 0

      if keys.code == 275 or keys.code == 276:
        self.vel_z_base = 0

      if self.vel_x_base == 0 and self.vel_z_base == 0:
        self.base_stop = True


  def dateCombin(self, data):
    data_combin = "" 
    for index in range(len(data)):
      data_combin = data_combin + data[index]
    return data_combin

  def pubBaseMsg(self, vel_x, vel_z):
    vel_cmd_msg = Twist()
    vel_cmd_msg.linear.x = vel_x * self.vel_scale_base
    vel_cmd_msg.angular.z = vel_z * self.vel_scale_base
    rate = rospy.Rate(10)
    while self.vel_x_base != 0 or self.vel_z_base != 0:
      self._pub_cmd_vel.publish(vel_cmd_msg)
      rate.sleep()
      
  def pubBaseThread(self, vel_x, vel_z):
    base = threading.Thread(target=self.pubBaseMsg,args=(vel_x, vel_z))
    base.start()

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

def main():
  rospy.init_node('worthog_udp_server')

  BCI = BrainControlInterface()
  time.sleep(0.5)
  t0 = threading.Thread(target=BCI.UDPLinstenLoop,args=())
  t0.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass