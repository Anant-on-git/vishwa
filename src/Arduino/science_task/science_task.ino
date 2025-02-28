#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <ESP32Servo.h> 

#define ANALOG_PIN1 8
#define dirPin1 9
#define stepPin1 10
#define dirPin2 36
#define stepPin2 35
int worm_m1_open = 4;
int worm_m1_close = 5;
int worm_m2_open = 1;
int worm_m2_close = 2;
const int servo = 48;
int pump1_rpwm = 11;
int pump1_lpwm = 12;
int pump2_rpwm = 15;
int pump2_lpwm = 16;
int pump3_rpwm = 40;
int pump3_lpwm = 39;
int pump4_rpwm = 47;
int pump4_lpwm = 21;

Servo myServo;

const int minAngle = 0;
const int maxAngle=180;

double target_servo = 0;
double servo_l = 0;

double target_nema_up = 0;  // RPM
double nema_up = 0; //dir

double target_nema_down = 0;  // RPM
double nema_down = 0; //dir

double target_worm1_open = 0;  // RPM
double worm1_open = 0; //dir

double target_worm2_open = 0;  // RPM
double worm2_open = 0; //dir

double target_worm1_close = 0;
double worm1_close = 0;

double target_worm2_close = 0;
double worm2_close = 0;

double target_p1 = 0;
double p1 = 0;

double target_p2 = 0;
double p2 = 0;

double target_p3 = 0;
double p3 = 0;

double target_p4 = 0;
double p4 = 0;

int x = 90;


ros::NodeHandle nh;

void uc_velsCallback(const std_msgs::Int32MultiArray &msg) {
    target_servo = abs(servo_l);
    servo_l = msg.data[6]; 
    target_nema_up = abs(nema_up);
    nema_up = msg.data[0];
    target_nema_down = abs(nema_down);
    nema_down = msg.data[1];
    target_worm1_open = abs(worm1_open);
    worm1_open = msg.data[2];
    target_worm1_close = abs(worm1_close);
    worm1_close = msg.data[3];
    target_worm2_open = abs(worm2_open);
    worm2_open = msg.data[4];
    target_worm2_close = abs(worm2_close);
    worm2_close = msg.data[5];
    target_p1 = abs(p1);
    p1 = msg.data[7];
    target_p2 = abs(p2);
    p2 = msg.data[8];
    target_p3 = abs(p3);
    p3 = msg.data[9];
    target_p4 = abs(p4);
    p4 = msg.data[10];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("science_vels", &uc_velsCallback);

void setup() {
  Serial.begin(57600);

  myServo.attach(servo);
  myServo.write(x);

  pinMode(servo, OUTPUT);

  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT); 
  pinMode(stepPin2, OUTPUT); 

  pinMode(worm_m1_open, OUTPUT);
  pinMode(worm_m1_close, OUTPUT);

  pinMode(worm_m2_open, OUTPUT);
  pinMode(worm_m2_close, OUTPUT);

  pinMode(pump1_rpwm, OUTPUT);
  pinMode(pump2_rpwm, OUTPUT);
  pinMode(pump3_rpwm, OUTPUT);
  pinMode(pump4_lpwm, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  Serial.print("working");

  while(target_nema_up == 1){
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    int delay = 500;
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(delay);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);

  }
  while(target_nema_down == 1){
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    int delay = 500;
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(delay);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
  }

  int i= x;
  x = x + target_servo;
  for(i; i<x; i++){
   if(x<180 && x<0){
    myServo.write(i);
  }
  }

  analogWrite(worm_m1_open, target_worm1_open);
  analogWrite(worm_m1_close, target_worm1_close);
  analogWrite(worm_m2_open, target_worm2_open);
  analogWrite(worm_m2_close, target_worm2_close);
  analogWrite(pump1_rpwm, target_p1);
  analogWrite(pump2_rpwm, target_p2);
  analogWrite(pump3_rpwm, target_p3);
  analogWrite(pump4_rpwm, target_p4);

  nh.spinOnce();
}
