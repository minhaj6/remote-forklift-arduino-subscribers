#include <AccelStepper.h>
#include <DigiPotX9Cxxx.h>
#include <Ramp.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// defines pins numbers
#define INC 3
#define DIRECTION 5
#define VH 2
#define VSS 4

const int stepPin = 6;
const int directionPin = 7;

ros::NodeHandle nh;
void messageCb(const geometry_msgs::Twist &twist);
ros::Subscriber<geometry_msgs::Twist> sub("mhr/cmd_vel", &messageCb);

/*
   Stepper Driver
   PUL+ -> 5
   PUL- -> GND (pin 6)
   DIR+ -> 7
   DIR- -> GND (pin 8)
   ENA+ -> GND
   ENA- -> GND
*/

/* removing arduino mega ,the pins
13dpin-- 
*/

AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);

rampFloat myRamp;
DigiPot pot(INC, VH, VSS);

float vx = 1500;
int tgt_spd, prev_spd, i;

void setup()
{
  pinMode(DIRECTION , OUTPUT);

  // Steering
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  stepper.setAcceleration(1);
  stepper.setMaxSpeed(6000);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.loginfo("ros subscriber setup done...");
}

void loop()
{
  nh.spinOnce();
  stepper.runSpeed();

  tgt_spd = map(vx, 1000, 2000, -100, 100);
  if (prev_spd != tgt_spd){
     myRamp.go(tgt_spd, abs(tgt_spd * 100), EXPONENTIAL_OUT, ONCEFORWARD);
     prev_spd = tgt_spd;
  }
  
  i = (float)myRamp.update();
  apply_speed(i);

//  String dataString = "";
//  dataString += String("Target: ");
//  dataString += String(tgt_spd);
//  dataString += String("    Applied: ");  
//  dataString += String(i);
//  dataString += String("    Vx: ");
//  dataString += String(vx);     
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

void messageCb(const geometry_msgs::Twist &twist)
{
  // Speed +ve is going down and vice-versa
  char buffer[64];
  sprintf(buffer,
    "Getting: %s %s %s",
    String(twist.linear.x).c_str(),
    String(twist.linear.y).c_str(),
    String(twist.linear.z).c_str()
  );
  
  nh.loginfo(buffer);

  vx = (float)twist.linear.x;
  
  if (twist.linear.z == 1000) {
//    nh.loginfo("Going down");
    stepper.setSpeed(6000);
  } else if (twist.linear.z == 2000) {
//    nh.loginfo("Going top");
    stepper.setSpeed(-6000);
  } else {
//    nh.loginfo("Oh no");
    stepper.setSpeed(0);
  }

  if (1350 < twist.linear.y && twist.linear.y < 1650) {
//    nh.loginfo("Straight");
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
  } else if (twist.linear.y < 1350) {
//    nh.loginfo("+GAY");
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
  } else {
//    nh.loginfo("-GAY");
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);
  }
}
