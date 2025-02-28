#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

// Motor pins
int rpwm_m1 = 5, lpwm_m1 = 4;
int rpwm_m2 = 16, lpwm_m2 = 15;
int rpwm_m3 = 12, lpwm_m3 = 11;
int rpwm_m4 = 1,  lpwm_m4 = 2;
int rpwm_m5 = 39, lpwm_m5 = 40;
int rpwm_m6 = 21, lpwm_m6 = 47;

// Target RPM values
double targetr_mr = 0; // Right motors target PWM value
double targetl_mr = 0; // Left motors target PWM value
double targetr_ml = 0; // Right motors target PWM value (left motors group)
double targetl_ml = 0; // Left motors target PWM value (left motors group)

// ROS NodeHandle
ros::NodeHandle nh;

// Callback function for subscriber
void uc_velsCallback(const std_msgs::Int32MultiArray &msg) {
    targetr_mr = abs(msg.data[0]);
    targetl_mr = abs(msg.data[1]);
    targetr_ml = abs(msg.data[2]);
    targetl_ml = abs(msg.data[3]);
}

// ROS Subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("wheel_vels", &uc_velsCallback);

// LEDC configuration
const int freq = 5000;       // PWM frequency
const int resolution = 8;    // PWM resolution (8-bit: 0-255)

// LEDC channels for right and left motors
const int ledcChannel_rpwm_r = 0;  // Right motor rpwm
const int ledcChannel_lpwm_r = 1;  // Right motor lpwm
const int ledcChannel_rpwm_l = 2;  // Left motor rpwm
const int ledcChannel_lpwm_l = 3;  // Left motor lpwm

void setup() {
  Serial.begin(57600);

  ledcSetup(ledcChannel_rpwm_r, freq, resolution);
  ledcSetup(ledcChannel_lpwm_r, freq, resolution);
  ledcSetup(ledcChannel_rpwm_l, freq, resolution);
  ledcSetup(ledcChannel_lpwm_l, freq, resolution);

  ledcAttachPin(rpwm_m1, ledcChannel_rpwm_r);
  ledcAttachPin(rpwm_m2, ledcChannel_rpwm_r);
  ledcAttachPin(rpwm_m3, ledcChannel_rpwm_r);
  ledcAttachPin(lpwm_m1, ledcChannel_lpwm_r);
  ledcAttachPin(lpwm_m2, ledcChannel_lpwm_r);
  ledcAttachPin(lpwm_m3, ledcChannel_lpwm_r);

  ledcAttachPin(rpwm_m4, ledcChannel_rpwm_l);
  ledcAttachPin(rpwm_m5, ledcChannel_rpwm_l);
  ledcAttachPin(rpwm_m6, ledcChannel_rpwm_l);
  ledcAttachPin(lpwm_m4, ledcChannel_lpwm_l);
  ledcAttachPin(lpwm_m5, ledcChannel_lpwm_l);
  ledcAttachPin(lpwm_m6, ledcChannel_lpwm_l);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  
  Serial.println("Working");

  ledcWrite(ledcChannel_rpwm_r, targetr_mr); // Right motors rpwm
  ledcWrite(ledcChannel_lpwm_r, targetl_mr); // Right motors lpwm
  ledcWrite(ledcChannel_rpwm_l, targetr_ml); // Left motors rpwm
  ledcWrite(ledcChannel_lpwm_l, targetl_ml); // Left motors lpwm

  nh.spinOnce();
}
