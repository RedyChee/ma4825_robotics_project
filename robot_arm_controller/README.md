# Robot Arm Controller
This package contains program to control robot arm and convenyor belt.

## Dependencies
1. Go to dynamixel sdk python directory.
 ```
cd ~/catkin_ws/src/ma4825_robotics_project/DynamixelSDK/python/
 ```

2. Install Dynamixel SDK library (python version)
 ```
 sudo python setup.py install
 ```

3. Install Arduino IDE
Follow this guide: https://www.arduino.cc/en/Guide/Linux

4. Install rosserial_python
```
sudo apt-get install ros-noetic-rosserial-python
```
 

## Quick Start
 1. Check all USB ports is define correctly. (3 USB port i.robot arm, ii.arduino, iii.camera )
 2. Launch the program.
 ```
 roslaunch robot_arm_controller whole_setup.launch 
```
3. If cannot run, please call / message me (JC).
