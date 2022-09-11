/*
   1 - INC - Arduino pin 2
   2 - U/D - Arduino pin 3
   3 - VH  - 5V
   4 - VSS - GND
   5 - VW  - Output: 150 Ohm resistor -> LED -> GND
   6 - VL  - GND
   7 - CS  - Arduino pin 4
   8 - VCC - 5V

   9 - arduino 5 logical direction 0/1
*/

#define INC 2
#define DIRECTION 5
#define VH 3
#define VSS 4


#include <DigiPotX9Cxxx.h>
#include <Ramp.h>                           // include library
#include <ros.h>
#include <geometry_msgs/Twist.h>

float vx = 1500, vy = 0, vz = 0, wx = 0, wy = 0, wz = 0;
ros::NodeHandle nh;
void messageCb(const geometry_msgs::Twist &tiwst);
ros::Subscriber<geometry_msgs::Twist> sub("mhr/cmd_vel", &messageCb);

float i;

rampFloat myRamp;
DigiPot pot(INC, VH, VSS);
int tgt_spd, prev_spd;                        /*b serial read data*/

void setup() {
  pinMode(DIRECTION , OUTPUT);
  nh.getHardware()->setBaud(115200);;
  nh.initNode();
  nh.subscribe(sub);
  nh.loginfo("ros subscriber setup...");

}

void loop()
{
  nh.spinOnce();
  tgt_spd = map(vx, 1000, 2000, -100, 100);
  if (prev_spd != tgt_spd){
     myRamp.go(tgt_spd, abs(tgt_spd * 100), EXPONENTIAL_OUT, ONCEFORWARD);
     prev_spd = tgt_spd;
  }
  
  i = (float)myRamp.update();
  
  apply_speed(i);

  String dataString = "";
  dataString += String("Target: ");
  dataString += String(tgt_spd);
  dataString += String("    Applied: ");  
  dataString += String(i);
  dataString += String("    Vx: ");
  dataString += String(vx);     
    // Length (with one extra character for the null terminator)
  int str_len = dataString.length() + 1;

  // Prepare the character array (the buffer)
  char char_array[str_len];

  // Copy it over
  dataString.toCharArray(char_array, str_len);
  nh.loginfo(char_array);
}



void apply_speed(float spd)
{
  spd = constrain(spd, -100, 100);    /* mapping might be needed*/
  if (spd == 0)
  {
    pot.set(spd);
  }
  else if (spd > 0)
  {
    spd = map(spd, 0, 100, 0, 88);
    digitalWrite(DIRECTION, LOW);
    pot.set(spd);
  }
  else if (spd < 0)
  {
    spd = map(abs(spd), 0, 100, 0, 88);
    digitalWrite(DIRECTION, HIGH);
    pot.set(spd);
  }
}

void messageCb(const geometry_msgs::Twist &twist) {
  vx = (float)twist.linear.x;
  vy = (float)twist.linear.y;
  vz = (float)twist.linear.z;
  wx = (float)twist.angular.x;
  wy = (float)twist.angular.y;
  wz = (float)twist.angular.z;
}
