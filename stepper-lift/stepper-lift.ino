#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// defines pins numbers
const int stepPinLeft = 5;
const int directionPinLeft = 7;
const int stepPinRight = 6;
const int directionPinRight = 8;
const int enablePin = 4;

// Define a stepper and the pins it will use
// 1 or AccelStepper::DRIVER means a stepper driver (with Step and Direction pins)
AccelStepper stepper(AccelStepper::DRIVER, stepPinLeft, directionPinLeft);


float vx = 1500, vy = 0, vz = 1500, wx = 0, wy = 0, wz = 0;
//ros::NodeHandle nh;
//void messageCb(const geometry_msgs::Twist &tiwst);
//ros::Subscriber<geometry_msgs::Twist> sub("mhr/cmd_vel", &messageCb);

int tgt_spd;

void setup(){
//  nh.getHardware()->setBaud(115200);;
//  nh.initNode();
//  nh.subscribe(sub);
//  nh.loginfo("ros subscriber setup...");
  stepper.setAcceleration(10);
  stepper.setMaxSpeed(1000);
}

void loop(){  

//  nh.spinOnce();
//  tgt_spd = map(vz, 1000, 2000, -1000, 1000);
  
  setVelocity(1000);
//
//  String dataString = "";
//  dataString += String("Vz: ");
//  dataString += String(vz); 
//    dataString += String("tgt: ");
//  dataString += String(tgt_spd);     
//    // Length (with one extra character for the null terminator)
//  int str_len = dataString.length() + 1;
//
//  // Prepare the character array (the buffer)
//  char char_array[str_len];
//
//  // Copy it over
//  dataString.toCharArray(char_array, str_len);
//  nh.loginfo(char_array);
  
}

void setVelocity(float Speed){
  
   stepper.setSpeed(Speed);  
   stepper.runSpeed();
}

//
//void messageCb(const geometry_msgs::Twist &twist) {
//  vx = (float)twist.linear.x;
//  vy = (float)twist.linear.y;
//  vz = (float)twist.linear.z;
//  wx = (float)twist.angular.x;
//  wy = (float)twist.angular.y;
//  wz = (float)twist.angular.z;
//}
