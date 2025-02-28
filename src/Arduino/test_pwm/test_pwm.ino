
#include <LedControl.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#define LPWM1pin 5 
#define RPWM1pin 4
#define HA1pin 6
#define HB1pin 7

#define LPWM2pin 17
#define RPWM2pin 15
#define HA2pin 17
#define HB2pin 18

#define LPWM3pin 12
#define RPWM3pin 11
#define HA3pin 13
#define HB3pin 14

#define LPWM4pin 2
#define RPWM4pin 1
#define HA4pin 42
#define HB4pin 41

#define LPWM5pin 39
#define RPWM5pin 40
#define HA5pin 38
#define HB5pin 37

#define LPWM6pin 21
#define RPWM6pin 147
#define HA6pin 20
#define HB6pin 19

volatile int pulseCount[6] = {0, 0, 0, 0, 0, 0};  // Count of encoder pulses for each motor
unsigned long lastMillis = 0; // Time tracking for RPM calculation
const int pulsesPerRevolution = 352; // Number of pulses per motor shaft revolution

float targetRPM = 50.0; // Default Target RPM

float currentRPM[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Current measured RPM for each motor

// PID constants
float Kp = 2.7876;   // Proportional gain
float Ki = 0.0007; // Integral gain
float Kd = 0.05;  // Derivative gain

// PID variables
float previousError[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float integral[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long previousMillisPID = 0;

// Output smoothing variables
float previousControlSignal[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float smoothingFactor = 0.8; // Between 0.0 (no smoothing) and 1.0 (max smoothing)

// ESP32 PWM Channels
#define LPWMChannel1 0
#define RPWMChannel1 1
#define LPWMChannel2 2
#define RPWMChannel2 3
#define LPWMChannel3 4
#define RPWMChannel3 5
#define LPWMChannel4 6
#define RPWMChannel4 7
#define LPWMChannel5 8
#define RPWMChannel5 9
#define LPWMChannel6 10
#define RPWMChannel6 11
const int PWMFrequency = 5000; // 5 kHz
const int PWMResolution = 8;   // 8-bit resolution (0-255)

int rpwm_m1 = 5;
int lpwm_m1 = 4;
int rpwm_m2 = 16;
int lpwm_m2 = 15;
int rpwm_m3 = 12;
int lpwm_m3 = 11;
int rpwm_m4 = 2;
int lpwm_m4 = 1;
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

  // Configure PWM channels for all 6 motors
  ledcSetup(LPWMChannel1, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel1, PWMFrequency, PWMResolution);
  ledcSetup(LPWMChannel2, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel2, PWMFrequency, PWMResolution);
  ledcSetup(LPWMChannel3, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel3, PWMFrequency, PWMResolution);
  ledcSetup(LPWMChannel4, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel4, PWMFrequency, PWMResolution);
  ledcSetup(LPWMChannel5, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel5, PWMFrequency, PWMResolution);
  ledcSetup(LPWMChannel6, PWMFrequency, PWMResolution);
  ledcSetup(RPWMChannel6, PWMFrequency, PWMResolution);

  // Attach PWM channels to GPIO pins for all 6 motors
  ledcAttachPin(LPWM1pin, LPWMChannel1);
  ledcAttachPin(RPWM1pin, RPWMChannel1);
  ledcAttachPin(LPWM2pin, LPWMChannel2);
  ledcAttachPin(RPWM2pin, RPWMChannel2);
  ledcAttachPin(LPWM3pin, LPWMChannel3);
  ledcAttachPin(RPWM3pin, RPWMChannel3);
  ledcAttachPin(LPWM4pin, LPWMChannel4);
  ledcAttachPin(RPWM4pin, RPWMChannel4);
  ledcAttachPin(LPWM5pin, LPWMChannel5);
  ledcAttachPin(RPWM5pin, RPWMChannel5);
  ledcAttachPin(LPWM6pin, LPWMChannel6);
  ledcAttachPin(RPWM6pin, RPWMChannel6);

  // Set up interrupts on HApin for each motor
  attachInterrupt(digitalPinToInterrupt(HA1pin), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(HA2pin), countPulse2, RISING);
  attachInterrupt(digitalPinToInterrupt(HA3pin), countPulse3, RISING);
  attachInterrupt(digitalPinToInterrupt(HA4pin), countPulse4, RISING);
  attachInterrupt(digitalPinToInterrupt(HA5pin), countPulse5, RISING);
  attachInterrupt(digitalPinToInterrupt(HA6pin), countPulse6, RISING);

  Serial.println("Enter Target RPM via Serial Monitor:");
  Serial.println("Default Target RPM: 50");
}

void loop() {

  Serial.print("working");

  // Check for Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    input.trim(); // Remove any extra spaces or newline characters

    // Attempt to convert input to float
    float newTargetRPM = input.toFloat();
    if (newTargetRPM > 0) { // Only accept positive values
      targetRPM = newTargetRPM;
      Serial.print("New Target RPM Set: ");
      Serial.println(targetRPM);
    } else {
      Serial.println("Invalid input. Please enter a positive number.");
    }
  }

  // RPM calculation every 200 ms for quicker response
  if (millis() - lastMillis >= 200) {
    lastMillis = millis();

    // Calculate RPM for each motor
    for (int i = 0; i < 6; i++) {
      currentRPM[i] = calculateRPM(i);
      Serial.print("Current RPM Motor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(currentRPM[i]);
    }

    // Call PID controller function for each motor
    for (int i = 0; i < 6; i++) {
      float rawControlSignal = PIDController(targetRPM * 2, currentRPM[i], i);

      // Smooth the control signal using exponential smoothing
      float controlSignal = smoothingFactor * previousControlSignal[i] + (1.0 - smoothingFactor) * rawControlSignal;
      previousControlSignal[i] = controlSignal;

      // Apply control signal to motor
      if (controlSignal > 0) {
        ledcWrite(LPWMChannel1 + i, (int)controlSignal);
        ledcWrite(RPWMChannel1 + i, 0);
      } else if (controlSignal < 0) {
        ledcWrite(RPWMChannel1 + i, (int)(-controlSignal));
        ledcWrite(LPWMChannel1 + i, 0);
      } else {
        ledcWrite(LPWMChannel1 + i, 0);
        ledcWrite(RPWMChannel1 + i, 0);
      }
    }

    // Reset pulse counts for next interval
    for (int i = 0; i < 6; i++) {
      pulseCount[i] = 0;
    }
  }
}

// Interrupt Service Routines (ISRs) to count pulses for each motor
void countPulse1() { pulseCount[0]++; }
void countPulse2() { pulseCount[1]++; }
void countPulse3() { pulseCount[2]++; }
void countPulse4() { pulseCount[3]++; }
void countPulse5() { pulseCount[4]++; }
void countPulse6() { pulseCount[5]++; }

// Function to calculate RPM for each motor
float calculateRPM(int motorIndex) {
  // Calculate revolutions based on pulse count and encoder specs
  float revolutions = pulseCount[motorIndex] / (float)pulsesPerRevolution;
  return revolutions * 60.0 / 0.2; // Adjust for 200 ms interval (0.2 seconds)
}

// PID Controller function
float PIDController(float target, float current, int motorIndex) {
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillisPID) / 1000.0; // Time in seconds
  previousMillisPID = currentMillis;

  // Calculate error
  float error = target - current;

  // Proportional term
  float proportional = Kp * error;

  // Integral term
  integral[motorIndex] += error * elapsedTime;
  float integralTerm = Ki * integral[motorIndex];

  // Derivative term
  float derivative = (error - previousError[motorIndex]) / elapsedTime;
  float derivativeTerm = Kd * derivative;

  // Compute total control signal
  float controlSignal = proportional + integralTerm + derivativeTerm;

  // Save error for the next iteration
  previousError[motorIndex] = error;

  // Constrain control signal to PWM limits (0-255)
  controlSignal = constrain(controlSignal, -255, 255);

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


  return controlSignal;
}
