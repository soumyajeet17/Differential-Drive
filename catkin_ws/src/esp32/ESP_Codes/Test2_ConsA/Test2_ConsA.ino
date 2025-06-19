#define ROSSERIAL_ARDUINO_TCP
#define ROBOT_A
#define ENCODER_PIN_R 21
#define ENCODER_PIN_L 34
#define MOTOR_PWM_R 14  // AIN1
#define MOTOR_DIR_R 27  // AIN2
#define MOTOR_PWM_L 26  // BIN1
#define MOTOR_DIR_L 25  // BIN2
#define ENCODER_N 20

#include <Arduino.h>
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <LDS_YDLIDAR_X2_X2L.h>
#include <HardwareSerial.h>
#include <math.h>

const char* ssid = "dlink";
const char* password = "Ros_lab2025";

IPAddress server(192, 168, 0, 108);
uint16_t serverPort = 11411;

// LiDAR Configuration
const uint8_t LIDAR_EN_PIN = 19;
const uint8_t LIDAR_PWM_PIN = 15;
const uint8_t LIDAR_TX_PIN = 17;
const uint8_t LIDAR_RX_PIN = 16;
HardwareSerial LidarSerial(1);
LDS_YDLIDAR_X2_X2L lidar;

float my_dist = 0;
int pulseR = 0, pulseL = 0;
bool lastR = LOW, lastL = LOW;
unsigned long lastTime = 0;
float rpmR = 0, rpmL = 0;
int pwmR = 0, pwmL = 0;
const int pwmMax = 180;

const float CONSENSUS_ENTER_THRESH = 0.25f;
const float CONSENSUS_EXIT_THRESH = 0.05f;
bool inConsensusZone = false;
float other_robot_dist = 0;
float consensus_dist = 0;

float targetRPM = 60.0;
float kp = 2.0, ki = 0.05, kd = 0.1;
float errorR = 0.0, errorL = 0.0;
float integralR = 0.0, integralL = 0.0;
float prevErrorR = 0.0, prevErrorL = 0.0;

ros::NodeHandle nh;
std_msgs::Float32 dist_msg;
ros::Publisher dist_pub("distA", &dist_msg);
void distCallback(const std_msgs::Float32& msg) {
  other_robot_dist = msg.data;
  consensus_dist = (my_dist + other_robot_dist) / 2.0f;
}
ros::Subscriber<std_msgs::Float32> dist_sub("distC", &distCallback);

bool rosConnected = false;

int lidar_serial_read_callback() { return LidarSerial.read(); }
size_t lidar_serial_write_callback(const uint8_t *buf, size_t len) { return LidarSerial.write(buf, len); }

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (fabs(angle_deg) <= 1.0f) my_dist = distance_mm / 1000.0f;
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t pin) {
  int p = (pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN_PIN : LIDAR_PWM_PIN;
  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      ledcSetup(4, 10000, 11);
      ledcAttachPin(p, 4);
    } else {
      pinMode(p, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }
  digitalWrite(p, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial.println("Robot A Initializing...");
  setupWiFi();

  // LiDAR
  LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.init(); 
  lidar.start();

  pinMode(ENCODER_PIN_R, INPUT_PULLUP);
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);

  // DRV8833: Setup 2 PWM channels per motor
  ledcSetup(0, 1000, 8); ledcAttachPin(MOTOR_PWM_R, 0);  // Right AIN1
  ledcSetup(1, 1000, 8); ledcAttachPin(MOTOR_DIR_R, 1);  // Right AIN2
  ledcSetup(2, 1000, 8); ledcAttachPin(MOTOR_PWM_L, 2);  // Left BIN1
  ledcSetup(3, 1000, 8); ledcAttachPin(MOTOR_DIR_L, 3);  // Left BIN2

  // Initialize all motors off
  ledcWrite(0, 0); ledcWrite(1, 0);
  ledcWrite(2, 0); ledcWrite(3, 0);

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(dist_pub);
  nh.subscribe(dist_sub);

  while (!nh.connected()) {
    nh.spinOnce();
    delay(500);
  }
  rosConnected = true;
  Serial.println("Robot A Operational");
}

void loop() {
  lidar.loop();
  nh.spinOnce();

  if (rosConnected) {
    bool r = digitalRead(ENCODER_PIN_R); 
    if (lastR == LOW && r == HIGH) pulseR++; 
    lastR = r;
    
    bool l = digitalRead(ENCODER_PIN_L); 
    if (lastL == LOW && l == HIGH) pulseL++; 
    lastL = l;

    unsigned long t = millis();
    if (t - lastTime > 1000) {
      rpmR = (pulseR / (float)ENCODER_N) * 60.0;
      rpmL = (pulseL / (float)ENCODER_N) * 60.0;

      float consensus_error = my_dist - consensus_dist;

      if (!inConsensusZone && fabs(consensus_error) > CONSENSUS_ENTER_THRESH) {
        inConsensusZone = true;
        integralR = integralL = 0.0;
      } else if (inConsensusZone && fabs(consensus_error) < CONSENSUS_EXIT_THRESH) {
        inConsensusZone = false;
        pwmR = pwmL = 0;
      }

      if (inConsensusZone) {
        bool moveForward = consensus_error > 0;

        errorR = targetRPM - rpmR;
        errorL = targetRPM - rpmL;
        integralR += errorR;
        integralL += errorL;
        float derivativeR = errorR - prevErrorR;
        float derivativeL = errorL - prevErrorL;

        pwmR = constrain(kp * errorR + ki * integralR + kd * derivativeR, 0, pwmMax);
        pwmL = constrain(kp * errorL + ki * integralL + kd * derivativeL, 0, pwmMax);
        prevErrorR = errorR;
        prevErrorL = errorL;

        if (moveForward) {
          ledcWrite(0, pwmR); ledcWrite(1, 0);  // Right motor forward
          ledcWrite(2, pwmL); ledcWrite(3, 0);  // Left motor forward
        } else {
          ledcWrite(0, 0); ledcWrite(1, pwmR);  // Right motor backward
          ledcWrite(2, 0); ledcWrite(3, pwmL);  // Left motor backward
        }

      } else {
        pwmR = pwmL = 0;
        ledcWrite(0, 0); ledcWrite(1, 0);
        ledcWrite(2, 0); ledcWrite(3, 0);
      }

      dist_msg.data = my_dist;
      dist_pub.publish(&dist_msg);

      Serial.print("My Dist: "); Serial.print(my_dist, 2);
      Serial.print(" | Consensus Dist: "); Serial.print(consensus_dist, 2);
      Serial.print(" | RPM R: "); Serial.print(rpmR);
      Serial.print(" L: "); Serial.print(rpmL);
      Serial.print(" | PWM R: "); Serial.print(pwmR);
      Serial.print(" L: "); Serial.println(pwmL);

      pulseR = pulseL = 0;
      lastTime = t;
    }

  } else {
    ledcWrite(0, 0); ledcWrite(1, 0);
    ledcWrite(2, 0); ledcWrite(3, 0);
    delay(100);
  }
}
