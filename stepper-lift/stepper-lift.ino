#include <AccelStepper.h>

// defines pins numbers
const int stepPinLeft = 5;
const int directionPinLeft = 7;
const int stepPinRight = 6;
const int directionPinRight = 8;
const int enablePin = 4;

// Define a stepper and the pins it will use
// 1 or AccelStepper::DRIVER means a stepper driver (with Step and Direction pins)
AccelStepper stepper(AccelStepper::DRIVER, stepPinLeft, directionPinLeft);

void setup(){
  Serial.begin(115200);
  stepper.setAcceleration(10);
  stepper.setMaxSpeed(1000);
}

void loop(){  
//  float x = 1000 *sin(millis()*0.01);
  setVelocity(1000);

//  Serial.println(x);
}

void setVelocity(float Speed){
  
   stepper.setSpeed(Speed);  
   stepper.runSpeed();
}
