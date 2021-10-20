/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Bool.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

// Function declarations
void cameraDetectCB(const std_msgs::Bool& msg);
void robotInOperationCB(const std_msgs::Bool& msg);

void run_motor();

//ROS Subscribers/Publishers
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Bool> camera_detect_sub("camera/detect_flag", cameraDetectCB);
ros::Subscriber <std_msgs::Bool> robot_in_op_sub("arm/in_operation", robotInOperationCB);

std_msgs::Bool pick_flag_msg;
ros::Publisher pick_flag_pub("/arduino/pick_flag", &pick_flag_msg);

// Parameters
int convenyor_status = 0;
bool starting = true;

void setup() {

  //Init ROS nodehandle
  nh.initNode();

  //Init ROS subscribers and publishers
  nh.subscribe(camera_detect_sub);
  nh.subscribe(robot_in_op_sub);

  nh.advertise(pick_flag_pub);

  // set up Serial library at 9600 bps
  // Serial.begin(9600);           
  nh.loginfo("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    nh.logwarn("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  nh.loginfo("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  delay(1000);
  run_motor();

}

void loop() {
  //wait until you are actually connected

  while (!nh.connected())
  {
    nh.logwarn("Arduino ROS Node not connected");
    nh.spinOnce();
  }
  nh.spinOnce();
  if (starting){
    pick_flag_msg.data = true;
    pick_flag_pub.publish(&pick_flag_msg);
    starting = false;
  }
  // run_motor();
}

void run_motor(){
  nh.logwarn("tick");
  myMotor->run(FORWARD);
  myMotor->setSpeed(45);
  delay(1900);
  myMotor->setSpeed(0); 
}

void cameraDetectCB(const std_msgs::Bool& msg){
  if (msg.data == true && convenyor_status == 1){
    convenyor_status = 0;
  }
}

void robotInOperationCB(const std_msgs::Bool& msg){
  if (msg.data == false){
    run_motor();
    pick_flag_msg.data = true;
    pick_flag_pub.publish(&pick_flag_msg);
  }
}
