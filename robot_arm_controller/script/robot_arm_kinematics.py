#!/usr/bin/env python
import sys
import math
from xmlrpc.client import TRANSPORT_ERROR
import numpy as np
from vectormath import Vector3 as vector
from Ax12 import Ax12

#ROS Imports
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16, Bool


# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = '/dev/ttyUSB0'

Ax12.BAUDRATE = 1_000_000

# sets baudrate and opens com port
Ax12.connect()

SHOULDER_MOTOR_ID = 0
ELBOW_MOTOR_ID = 1
WRIST_MOTOR_ID = 3
KNUCKLE_MOTOR_ID = 4
FINGER_MOTOR_ID = 2

SHOULDER_MOTOR = Ax12(SHOULDER_MOTOR_ID)
ELBOW_MOTOR = Ax12(ELBOW_MOTOR_ID)
WRIST_MOTOR = Ax12(WRIST_MOTOR_ID)
KNUCKLE_MOTOR = Ax12(KNUCKLE_MOTOR_ID)
FINGER_MOTOR = Ax12(FINGER_MOTOR_ID)

SHOULDER_MOTOR.set_moving_speed(50)
ELBOW_MOTOR.set_moving_speed(80)
WRIST_MOTOR.set_moving_speed(120)
KNUCKLE_MOTOR.set_moving_speed(150)
FINGER_MOTOR.set_moving_speed(50)

rospy.sleep(0.1)

# Fixed variables in mm
class robot_arm_controller:
  def __init__(self):
    self.link_length = { 
                        'shoulder'  : 50,
                        'elbow'     : 193.0,   #211.5
                        'wrist'     : 203.2,  #203.2
                        'knuckle'   : 95.2,
                        'finger'    : 46.34,
                        'gripper'   : 68.0
                        }
                          
    self.angle_limit = { 
                        'shoulder'  : [-120, 120],
                        'elbow'     : [0, 130],
                        'wrist'     : [-90, 90],
                        'knuckle'   : [-130, 130],
                        'finger'    : [-60, 60]            
                        }

    self.dest_position = np.array([[130, 240, 130],
                                  [70, 270, 140],
                                  [130, -240, 130],
                                  [70, -270, 140]])
    
    self.object_id_sub = rospy.Subscriber("/camera/object_id", Int16, self.object_id_callback)
    self.pick_flag_sub = rospy.Subscriber("/arduino/pick_flag" , Bool, self.pick_flag_callback)
    self.des_pose_sub = rospy.Subscriber("/camera/des_pose", Pose, self.des_pose_callback)
    self.detect_flag_sub = rospy.Subscriber("/camera/detect_flag", Bool, self.detect_flag_callback)
    self.robot_in_operation_pub = rospy.Publisher("/arm/in_operation", Bool, queue_size= 1)

    self.des_x = 0
    self.des_y = 0
    self.des_z = 0
    self.des_r = 0
    self.des_z_elbow = 0
    self.object_id = 0
    self.detect_flag = False
    self.pick_flag = False
    self.robot_in_op_msg = Bool()


    self.set_angle_limit(SHOULDER_MOTOR, self.angle_limit['shoulder'][0], self.angle_limit['shoulder'][1])
    self.set_angle_limit(ELBOW_MOTOR, self.angle_limit['elbow'][0], self.angle_limit['elbow'][1])
    self.set_angle_limit(WRIST_MOTOR, self.angle_limit['wrist'][0], self.angle_limit['wrist'][1])
    self.set_angle_limit(KNUCKLE_MOTOR, self.angle_limit['knuckle'][0], self.angle_limit['knuckle'][1])
    self.set_angle_limit(FINGER_MOTOR, self.angle_limit['finger'][0], self.angle_limit['finger'][1])

    self.homing()

  def set_angle_limit(self, motor_object, min_angle, max_angle):
    min_angle_analog = min_angle / 150 * 512 + 511
    max_angle_analog = max_angle / 150 * 512 + 511
    motor_object.set_cw_angle_limit(int(min_angle_analog))
    motor_object.set_ccw_angle_limit(int(max_angle_analog))

  def set_position(self, motor_object, angle_in_deg):
    """Set motor angle"""
    input_analog = angle_in_deg / 150 * 512 + 511 
    motor_object.set_goal_position(int(input_analog))

  def set_torque_limit(self):
    SHOULDER_MOTOR.set_torque_limit(1023)
    ELBOW_MOTOR.set_torque_limit(1023)
    WRIST_MOTOR.set_torque_limit(1023)
    KNUCKLE_MOTOR.set_torque_limit(1023)
    FINGER_MOTOR.set_torque_limit(1023)

  def homing(self):
    """ sets goal position to home position"""
    self.set_torque_limit()
    self.set_position(SHOULDER_MOTOR, 0)   #TODO: redefine home position
    self.set_position(ELBOW_MOTOR, 0)
    self.set_position(WRIST_MOTOR, 0)
    self.set_position(KNUCKLE_MOTOR, 0)
    self.set_position(FINGER_MOTOR, -50)

  def get_shoulder_angle(self):
    """Inverse kinematics for shoulder angle"""
    shoulder_angle = np.arctan2(self.des_y, self.des_x)
    return math.degrees(shoulder_angle)

  def get_elbow_angle(self):
    """Inverse kinematics for elbow angle"""
    # wrist_angle = self.get_wrist_angle()
    # wrist_length, knuckle_length = self.link_length["wrist"], self.link_length["knuckle"]
    # elbow_angle = np.arctan2(self.des_r, self.des_z_elbow) - np.arctan2(knuckle_length * np.sin(wrist_angle), wrist_length + knuckle_length * np.cos(wrist_angle))
    # return math.degrees(elbow_angle)
    wrist_length, knuckle_length = self.link_length["wrist"], self.link_length["knuckle"]
    print("r: ", self.des_r)
    print("z:", self.des_z_elbow)
    print("wrist length: ", wrist_length)
    print("knuckle length: ", knuckle_length)
    wrist_angle = np.arcsin((self.des_r ** 2 + self.des_z_elbow ** 2 - wrist_length ** 2 - knuckle_length ** 2) / (2 * wrist_length * knuckle_length))
    return math.degrees(wrist_angle)

  def get_wrist_angle(self):
    """Inverse kinematics for wrist angle"""
    # wrist_length, knuckle_length = self.link_length["wrist"], self.link_length["knuckle"]
    # print("r: ", self.des_r)
    # print("z:", self.des_z_elbow)
    # print("wrist length: ", wrist_length)
    # print("knuckle length: ", knuckle_length)
    # wrist_angle = np.arccos((self.des_r ** 2 + self.des_z_elbow ** 2 - wrist_length ** 2 - knuckle_length ** 2) / (2 * wrist_length * knuckle_length))
    # return math.degrees(wrist_angle)

    return 90 - self.get_elbow_angle()

  def get_knuckle_angle(self):
    """Inverse kinematics for knuckle angle"""
    return self.des_angle

  def open_gripper(self):
    self.set_position(FINGER_MOTOR, -60)
    rospy.sleep(0.1)

  def close_gripper(self):
    finger_des_angle = -50
    while (FINGER_MOTOR.get_load() < 1900):
      rospy.sleep(0.1)
      self.set_position(FINGER_MOTOR, finger_des_angle)
      finger_des_angle += 5
      # print(FINGER_MOTOR.get_load())
      if finger_des_angle >= 60:
        break
    rospy.sleep(0.1)

  def move_robot(self):
    #Eliminate overload error 
    self.set_torque_limit()
    self.set_position(SHOULDER_MOTOR, self.get_shoulder_angle())

    while(True):
      rospy.sleep(0.1)
      if SHOULDER_MOTOR.is_moving() == 0:
        self.set_position(KNUCKLE_MOTOR, self.get_knuckle_angle())
        break

    while(True):
      rospy.sleep(0.1)
      if KNUCKLE_MOTOR.is_moving() == 0:
        self.set_position(ELBOW_MOTOR, self.get_elbow_angle())
        break

    while(True):
      rospy.sleep(0.1)
      if ELBOW_MOTOR.is_moving() == 0:
        self.set_position(WRIST_MOTOR, self.get_wrist_angle())
        break

    while(True):
      rospy.sleep(0.1)
      if WRIST_MOTOR.is_moving() == 0:
        break

  def object_id_callback(self, data):
    self.object_id = data.data

  def detect_flag_callback(self, data):
    self.detect_flag = data.data

  def pick_flag_callback(self, data):
    print("Pick flag function callback")
    print(data)
    if data.data == True and self.detect_flag == True:
      print("Enter if statement")
      #Publish robot in operation bool msg
      self.robot_in_op_msg.data = True
      self.robot_in_operation_pub.publish(self.robot_in_op_msg)

      #PICK
      self.move_robot()
      self.close_gripper()

      #LIFT UP
      self.des_z_elbow += 55
      self.des_x -= 15
      while(True):
        rospy.sleep(0.1)
        if FINGER_MOTOR.is_moving() == 0:
          self.move_robot()
          break

      
      #set placing position parameters
      self.des_x, self.des_y, self.des_z = self.dest_position[self.object_id][:]
      self.des_r = math.sqrt(self.des_x ** 2 + self.des_y ** 2)
      self.des_z_elbow = self.des_z - self.link_length["shoulder"] - self.link_length["elbow"] + 20
      self.des_angle = 0
      print(self.des_x, self.des_y, self.des_z)

      #PLACE
      rospy.sleep(1.)
      while(True):
        rospy.sleep(0.1)
        if FINGER_MOTOR.is_moving() == 0:
          self.move_robot()
          break
      self.open_gripper()   #TODO: Need to put some delay

      rospy.sleep(1)
      #HOME
      while(True):
        rospy.sleep(0.1)
        if FINGER_MOTOR.is_moving() == 0:
          self.homing()
          break

      #Publish robot in operation bool msg
      self.robot_in_op_msg.data = False
      self.robot_in_operation_pub.publish(self.robot_in_op_msg)

      #Reset detect flag
      self.detect_flag = False
      
  def des_pose_callback(self, data):

    #set desired position parameters
    self.des_x, self.des_y, self.des_z = data.position.x, data.position.y, data.position.z
    self.des_r = math.sqrt(self.des_x ** 2 + self.des_y ** 2)
    self.des_z_elbow = self.des_z - self.link_length["shoulder"] - self.link_length["elbow"] + 20
    # self.des_z_elbow = 0
    self.des_angle = data.orientation.x
    print("shoulder angle:", self.get_shoulder_angle())
    print("elbow angle:", self.get_elbow_angle())
    print("wrist angle:", self.get_wrist_angle())
    print("knuckle angle:", self.get_knuckle_angle())

    

def main(args):
  rospy.init_node("robot_arm_controller_node", anonymous=True)
  rac = robot_arm_controller()
  rospy.sleep(0.05)
  rospy.spin()

if __name__ == "__main__":
  main(sys.argv)
