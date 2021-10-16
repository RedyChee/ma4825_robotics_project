#!/usr/bin/env python
import sys
import math
import numpy as np
from vectormath import Vector3 as vector
from Ax12 import Ax12

#ROS Imports
import rospy
from geometry_msgs.msg import Pose

SHOULDER_MOTOR_ID = 2
ELBOW_MOTOR_ID = 1
WRIST_MOTOR_ID = 3
KNUCKLE_MOTOR_ID = 4
FINGER_MOTOR_ID = 5

SHOULDER_MOTOR = Ax12(SHOULDER_MOTOR_ID)
ELBOW_MOTOR = Ax12(ELBOW_MOTOR_ID)
WRIST_MOTOR = Ax12(WRIST_MOTOR_ID)
KNUCKLE_MOTOR = Ax12(KNUCKLE_MOTOR_ID)
FINGER_MOTOR = Ax12(FINGER_MOTOR_ID)

#TODO: complete these fix variables

# Fixed variables in mm
class robot_arm_controller:
  def __init__(self):
    self.link_length = { 
                        'shoulder'  : 20,
                        'elbow'     : 50,
                        'wrist'     : 50,
                        'knuckle'   : 20,
                        'finger'    : 10,
                        'gripper'   : 10
                        }
                          
    self.angle_limit = { 
                        'shoulder'  : [-30, 30],
                        'elbow'     : [-30, 30],
                        'wrist'     : [-30, 30],
                        'knuckle'   : [-30, 30],
                        'finger'    : [-30, 30]            
                        }

    self.joint_positions = {     
                            'origin'    : vector(0, 0, 0),
                            'shoulder'  : vector(),
                            'elbow'     : vector(),  
                            'wrist'     : vector(),
                            'kunckle'   : vector(),
                            'finger'    : vector()
                            }

    self.des_pose_sub = rospy.Subscriber("/arm/des_pose", Pose, self.des_pose_callback)

    self.des_x = 0
    self.des_y = 0
    self.des_z = 0
    self.des_r = 0
    self.des_z_elbow = 0

  def set_position(motor_object, angle_in_deg):
    """Set motor angle"""
    input_analog = angle_in_deg / 150 * 512 + 511 
    motor_object.set_goal_position(input_analog)

  def homing(self):
    """ sets goal position to home position"""
    self.set_position(SHOULDER_MOTOR, 0)   #TODO: redefine home position
    self.set_position(ELBOW_MOTOR, 0)
    self.set_position(WRIST_MOTOR, 0)
    self.set_position(KNUCKLE_MOTOR, 0)
    self.set_position(FINGER_MOTOR, 0)

  def get_shoulder_angle(self):
    """Inverse kinematics for shoulder angle"""
    shoulder_angle = np.arctan2(self.des_y, self.des_x)
    return math.degrees(shoulder_angle)

  def get_elbow_angle(self):
    """Inverse kinematics for elbow angle"""
    wrist_angle = self.get_wrist_angle()
    wrist_length, knuckle_length = self.link_length["wrist"], self.link_length["knuckle"]
    wrist_angle = np.arctan2(self.des_r, self.des_z_elbow) - np.arctan2(knuckle_length * np.sin(wrist_angle), wrist_length + knuckle_length * np.cos(wrist_angle))
    return math.degrees(wrist_angle)

  def get_wrist_angle(self):
    """Inverse kinematics for wrist angle"""
    wrist_length, knuckle_length = self.link_length["wrist"], self.link_length["knuckle"]
    elbow_angle = np.arccos((self.des_r ** 2 + self.des_z_elbow ** 2 - wrist_length ** 2 - knuckle_length ** 2) / (2 * wrist_length * knuckle_length))
    # if elbow_angle > 0 and elbow_angle < np.pi:
    return math.degrees(elbow_angle)

  # def get_knuckle_angle(self):

  # def open_gripper(self): -50 deg

  # def close_gripper(self): >1500 load stop

  def des_pose_callback(self, data):
    self.des_x, self.des_y, self.des_z = data.position.x, data.position.y, data.position.z
    self.des_r = math.sqrt(self.des_x ** 2 + self.des_y ** 2)
    self.des_z_elbow = self.des_z - self.link_length["shoulder"] - self.link_length["elbow"] - self.link_length["finger"] - self.link_length["gripper"]
    print("shoulder angle:", self.get_shoulder_angle())
    print("elbow angle:", self.get_elbow_angle())
    print("wrist angle:", self.get_wrist_angle())

    self.set_position(SHOULDER_MOTOR, self.get_shoulder_angle())
    self.set_position(ELBOW_MOTOR, self.get_shoulder_angle())
    self.set_position(WRIST_MOTOR, self.get_wrist_angle())

    while(True):
      if SHOULDER_MOTOR.is_moving() == 0 and ELBOW_MOTOR.is_moving() == 0 and WRIST_MOTOR.is_moving() == 0:
        self.set_position()

def main(args):
  rospy.init_node("robot_arm_controller_node", anonymous=True)
  rac = robot_arm_controller()
  rospy.sleep(0.05)
  rospy.spin()

if __name__ == "__main__":
  main(sys.argv)