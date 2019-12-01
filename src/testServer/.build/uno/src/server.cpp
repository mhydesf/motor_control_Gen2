#include <Arduino.h>
#include <ros.h>
#include <motor_control/motorPose.h>
void setup();
void loop();
#line 1 "src/server.ino"
#define led 13
//#include <ros.h>
//#include <motor_control/motorPose.h>

ros::NodeHandle  nh;
using motor_control::motorPose;

void positionMotors(const motorPose::Request & req, motorPose::Response & res){
    
    res.state = true;

    Serial.print(req.baseAng);
    Serial.print(req.mainAng);
    Serial.print(req.secAng);
    Serial.print(req.toolAng);

    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
}

ros::ServiceServer<motorPose::Request, motorPose::Response> server("motorPose",&positionMotors);

void setup(){
    pinMode(13, OUTPUT);
    nh.initNode();
    nh.advertiseService(server);
}

void loop(){
  nh.spinOnce();
}