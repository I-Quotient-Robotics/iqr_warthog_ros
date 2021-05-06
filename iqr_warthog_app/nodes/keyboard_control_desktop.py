#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, String
from sensor_msgs.msg import Joy
from keyboard.msg import Key
from bag_record.msg import Rosbag


class BrainControlInterface:
  def __init__(self):
    self._kinova_pose_init = Pose()
    self._sub_keydown = rospy.Subscriber("keyboard/keydown", Key, self.KeyboradDownCallBack)
    self._sub_keyup = rospy.Subscriber("keyboard/keyup", Key, self.KeyboradUpCallBack)
    self._pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    self._pub_start_record = rospy.Publisher("/record/start", Rosbag, queue_size=1)
    self._pub_stop_record = rospy.Publisher("/record/start", String, queue_size=1)
    self.vel_x_base = 0
    self.vel_z_base = 0
    self.vel_scale_base = 0.2

    self.allstart = 0
    self.driver_start = 0
    self.base_stop = True

    self.vel_cmd_msg = Twist()

  def KeyboradDownCallBack(self, state):

    if state.code == 13:
      if self.enable_keyboard:
        self.enable_keyboard = False
        # self.client_socket.sendto("KeyboardDisable", self.server_address)
        rospy.loginfo("Keyboard Disable")
      else:
        self.enable_keyboard = True
        # self.client_socket.sendto("KeyboardEnable", self.server_address)
        rospy.loginfo("Keyboard Enable")

    if enable_keyboard:        
      if state.code == 119:
        rospy.loginfo("Start Record")
        rosbag = Rosbag()
        rosbag.config = standard
        rosbag.bag_name = "bagdatetime"
        self._pub_start_record.publish(rosbag)
      elif state.code == 113:
        rospy.loginfo("Stop Record")
        stop_bag = String()
        stop_bag.data = "standard"
        self._pub_stop_record.publish(stop_bag)
      else state.code == 97:
        rospy.loginfo("auto navigation")

      elif state.code == 273:
        rospy.loginfo("base x forward")
        self.vel_x_base = 1
      elif state.code == 274:
        rospy.loginfo("base x back")
        self.vel_x_base = -1
      elif state.code == 275:
        rospy.loginfo("base z base")
        self.vel_z_base = -1
      elif state.code == 276:
        rospy.loginfo("base z forward")
        self.vel_z_base = 1
      elif state.code == 280:
        if self.vel_scale_base <= 1.0:
          rospy.loginfo("base vel up")
          rospy.loginfo("%f", self.vel_scale_base)
          self.vel_scale_base += 0.1
      elif state.code == 281:
        if self.vel_scale_base >= 0:
          rospy.loginfo("base vel down")
          rospy.loginfo("%f", self.vel_scale_base)
          self.vel_scale_base -= 0.1

    if self.vel_x_base != 0 or self.vel_z_base != 0:
      self.base_stop = False
      self.pubBaseThread(self.vel_x_base, self.vel_z_base)
      
  def KeyboradUpCallBack(self, keys):

    if keys.code == 273 or keys.code == 274:
      self.vel_x_base = 0

    if keys.code == 275 or keys.code == 276:
      self.vel_z_base = 0

    if self.vel_x_base == 0 and self.vel_z_base == 0:
      self.base_stop = True

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

def main():
  rospy.init_node('brain_control_interface')
  BCI = BrainControlInterface()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass