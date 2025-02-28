#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

int rpwm_m1 = 5;
int lpwm_m1 = 4;
int rpwm_m2 = 16;
int lpwm_m2 = 15;
int rpwm_m3 = 12;
int lpwm_m3 = 11;
int rpwm_m4 = 1;
int lpwm_m4 = 2;
int rpwm_m5 = 39;
int lpwm_m5 = 40;
int rpwm_m6 = 21;
int lpwm_m6 = 47;
double targetr_mr = 0;  // RPM
double rwheel_rpwm = 0; //dir

double targetl_mr = 0;  // RPM
double rwheel_lpwm = 0; //dir

double targetr_ml = 0;  // RPM
double lwheel_rpwm = 0; //dir

double targetl_ml = 0;  // RPM
double lwheel_lpwm = 0; //dir

ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray &msg) {
    rwheel_rpwm = msg.data[0];
    targetr_mr = abs(rwheel_rpwm);
    rwheel_lpwm = msg.data[1];
    targetl_mr = abs(rwheel_lpwm);
    lwheel_rpwm = msg.data[2];
    targetl_ml = abs(lwheel_rpwm);
    lwheel_lpwm = msg.data[3];
    targetl_ml = abs(lwheel_lpwm);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("wheel_vels", &uc_velsCallback);

void setup() {
  Serial.begin(57600);
  pinMode(rpwm_m1, OUTPUT);
  pinMode(lpwm_m1, OUTPUT);

  pinMode(rpwm_m2, OUTPUT);
  pinMode(lpwm_m2, OUTPUT);

  pinMode(rpwm_m3, OUTPUT);
  pinMode(lpwm_m3, OUTPUT);

  pinMode(rpwm_m4, OUTPUT);
  pinMode(rpwm_m4, OUTPUT);

  pinMode(rpwm_m5, OUTPUT);
  pinMode(lpwm_m5, OUTPUT);

  pinMode(rpwm_m6, OUTPUT);
  pinMode(lpwm_m6, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  Serial.print("working");

  analogWrite(rpwm_m1, targetr_mr);
  analogWrite(lpwm_m1, targetl_mr);
  analogWrite(rpwm_m2, targetr_mr);
  analogWrite(lpwm_m2, targetl_mr);
  analogWrite(rpwm_m3, targetr_mr);
  analogWrite(lpwm_m3, targetl_mr);
  analogWrite(rpwm_m4, targetr_ml);
  analogWrite(lpwm_m4, targetl_ml);
  analogWrite(rpwm_m5, targetr_ml);
  analogWrite(lpwm_m5, targetl_ml);
  analogWrite(rpwm_m6, targetr_ml);
  analogWrite(lpwm_m6, targetl_ml);


  nh.spinOnce();
}