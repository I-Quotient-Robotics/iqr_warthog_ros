#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray
from sensor_msgs.msg import Joy
from keyboard.msg import Key

class BrainControlInterface:
  def __init__(self):

    self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.server_address = ("127.0.0.1", 8848)

    self._kinova_pose_init = Pose()
    self._sub = rospy.Subscriber("keyboard/keydown", Key, self.KeyboradDownCallBack)
    self._sub = rospy.Subscriber("keyboard/keyup", Key, self.KeyboradUpCallBack)
    self._pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    self.vel_x_base = 0
    self.vel_z_base = 0
    self.vel_scale_base = 0.2

    self.enable_keyboard = True
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

    if self.enable_keyboard:
      if state.code == 119:
        # self.client_socket.sendto("StartRecord", self.server_address)
        self.client_socket.sendto("W", self.server_address)
        rospy.loginfo("Start Record")
      elif state.code == 113:
        self.client_socket.sendto("Q", self.server_address)
        rospy.loginfo("Stop Record")
      else state.code == 97:
        rospy.loginfo("auto navigation")
        self.client_socket.sendto("A", self.server_address)
      elif state.code == 273:
        rospy.loginfo("base x forward")
        self.client_socket.sendto("UP", self.server_address)
      elif state.code == 274:
        rospy.loginfo("base x back")
        self.client_socket.sendto("DOWN", self.server_address)
      elif state.code == 275:
        rospy.loginfo("base z base")
        self.client_socket.sendto("LEFT", self.server_address)
      elif state.code == 276:
        rospy.loginfo("base z forward")
        self.client_socket.sendto("RIGHT", self.server_address)
      elif state.code == 280:   
        if self.vel_scale_base <= 1.0:
          rospy.loginfo("base vel up")
          self.vel_scale_base += 0.1
          self.client_socket.sendto("PAGEUP", self.server_address)
          rospy.loginfo("%f", self.vel_scale_base)
      elif state.code == 281:
        if self.vel_scale_base >= 0:
          rospy.loginfo("base vel down")
          self.vel_scale_base -= 0.1
          self.client_socket.sendto("PAGEDOWN", self.server_address)
          rospy.loginfo("%f", self.vel_scale_base)

      if self.vel_x_base != 0 or self.vel_z_base != 0:
        self.base_stop = False
      
  def KeyboradUpCallBack(self, keys):
  
    if keys.code == 273 or keys.code == 274:
      self.client_socket.sendto("BaseXStop", self.server_address)

    elif keys.code == 275 or keys.code == 276:
      self.client_socket.sendto("BaseZStop", self.server_address)

    elif self.vel_x_base == 0 and self.vel_z_base == 0:
      self.base_stop = True

  def pubMsg(self, pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right):

    rospy.loginfo("arm..")
    rate = rospy.Rate(100)
    
    while self.left_stop == False or self.right_stop == False or self.left_joint_stop == False or self.right_joint_stop == False:
      if self.allstart == 1:
        if not self.left_stop:
          self._pub_left_pose_velocity.publish(pose_vel_left)
        if not self.right_stop:
          self._pub_right_pose_velocity.publish(pose_vel_right)
        if not self.left_joint_stop:
          self._pub_left_joint_velocity.publish(joint_vel_left)
        if not self.right_joint_stop:
          self._pub_right_joint_velocity.publish(joint_vel_right)
      rate.sleep()

  def fingerControl(self, position, client):
    finger_goal = SetFingersPositionGoal()
    finger_goal.fingers.finger1 = position
    finger_goal.fingers.finger2 = position
    finger_goal.fingers.finger3 = position
    client.send_goal(finger_goal)

  def pubBaseMsg(self, vel_x, vel_z):
    vel_cmd_msg = Twist()
    vel_cmd_msg.linear.x = vel_x * self.vel_scale_base
    vel_cmd_msg.angular.z = vel_z * self.vel_scale_base
    rate = rospy.Rate(10)
    while self.vel_x_base != 0 or self.vel_z_base != 0:
      self._pub_cmd_vel.publish(vel_cmd_msg)
      rate.sleep()
      
  def pubThread(self, pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right):
    t1 = threading.Thread(target=self.pubMsg,args=(pose_vel_left, pose_vel_right, joint_vel_left, joint_vel_right))
    t1.start()

  def pubFingerThread(self, finger_left, finger_right):
    fingers = threading.Thread(target=self.pubFingerMsg,args=(finger_left, finger_right))
    fingers.start()

def main():
  rospy.init_node('keyboard_control_client')
  BCI = BrainControlInterface()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
