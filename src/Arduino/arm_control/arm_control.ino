#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

int base_rpwm = 4;
int base_lpwm = 5;
int bevel_rpwm = 15;
int bevel_lpwm = 16;
int elbow_rpwm = 11;
int elbow_lpwm = 12;
int gripper_rpwm = 1;
int gripper_lpwm = 2;

double target_baser = 0;  // RPM
double baser = 0; //dir

double target_basel = 0;  // RPM
double basel = 0; //dir

double target_bevelr = 0;  // RPM
double bevelr = 0; //dir

double target_bevell = 0;  // RPM
double bevell = 0; //dir

double target_elbowr = 0;
double elbowr = 0;

double target_elbowl = 0;
double elbowl = 0;

double target_gripperr = 0;
double gripperr = 0;

double target_gripperl = 0;
double gripperl = 0;


ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray &msg) {
    baser = msg.data[0];
    target_baser = abs(baser);
    basel = msg.data[1];
    target_basel = abs(basel);
    bevelr = msg.data[2];
    target_bevelr = abs(bevelr);
    bevell = msg.data[3];
    target_bevell = abs(bevell);
    elbowr = msg.data[4];
    target_elbowr = abs(elbowr);
    elbowl = msg.data[5];
    target_elbowl = abs(elbowl);
    gripperr = msg.data[6];
    target_gripperr = abs(gripperr);
    gripperl = msg.data[7];
    target_gripperl = abs(gripperl);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("arms_vels", &uc_velsCallback);

void setup() {
  Serial.begin(57600);
  pinMode(base_rpwm, OUTPUT);
  pinMode(base_lpwm, OUTPUT);

  pinMode(bevel_rpwm, OUTPUT);
  pinMode(bevel_lpwm, OUTPUT);

  pinMode(elbow_rpwm, OUTPUT);
  pinMode(elbow_lpwm, OUTPUT);

  pinMode(gripper_rpwm, OUTPUT);
  pinMode(gripper_lpwm, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  Serial.print("working");

  analogWrite(base_rpwm, target_baser);
  analogWrite(base_lpwm, target_basel);
  analogWrite(bevel_rpwm, target_bevelr);
  analogWrite(bevel_lpwm, target_bevell);
  analogWrite(elbow_rpwm, target_elbowr);
  analogWrite(elbow_lpwm, target_elbowl);
  analogWrite(gripper_rpwm, target_gripperr);
  analogWrite(gripper_lpwm, target_gripperl);

  nh.spinOnce();
}
