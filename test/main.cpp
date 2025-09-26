#include <esp_system.h>
#include <Arduino.h>
#include <ps5Controller.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "3d.hpp"
#include <math.h>

int Lx = 0;
int Ly = 0;
int Rx = 0;
int Ry = 0;
int Square = 0;
int Circle = 0;
int prev_Circle = 0;
int R1 = 0;
int L1 = 0;
int up = 0;
int down = 0;
int right = 0;
int left = 0;
  
bool calibration_done = false;
bool is_calibration = false;
bool is_motor_running = false;

void get_gamepad_data(){
  Lx = ps5.LStickX();
  Ly = ps5.LStickY();
  Rx = ps5.RStickX();
  Ry = ps5.RStickY();
  Square = ps5.Square();
  Circle = ps5.Circle();
  R1 = ps5.R1();
  L1 = ps5.L1();
  up = ps5.Up();
  down = ps5.Down();
  right = ps5.Right();
  left = ps5.Left();
}

float yaw = 0;

float offset_roll = 0;
float offset_pitch = 0;
float offset_yaw = 0;

float motor_speed;
float offset_motor_speed = 30;

float kq = 8; //5
float kw = 6; //3
float kq_yaw = 6; //3
float kw_yaw = 5; //2

cpp3d::vec3d M(0, 0, 0);

float roll_pid_value, pitch_pid_value, yaw_pid_value;

#define SDA_PIN 21
#define SCL_PIN 22

int pwm_pin0 = 25;
int pwm_pin1 = 12;
int pwm_pin2 = 27;
int pwm_pin3 = 26;

int motor_value0 = 0;
int motor_value1 = 0;
int motor_value2 = 0;
int motor_value3 = 0;

Adafruit_BNO08x bno08x;

#define BNO08X_RESET -1

cpp3d::quaternion q(1, 0, 0, 0);
cpp3d::quaternion q_target(1, 0, 0, 0);
cpp3d::quaternion q_error(1, 0, 0, 0);

float dt;

unsigned long sensor_previous_time = 0;
unsigned long sensor_current_time;

sh2_SensorValue_t sensorValue;

void set_motor(int ch, float on_pulse_rate)
{
  float duty = (1000 + 10 * on_pulse_rate) / 20000;
  uint16_t value = duty * 65535;
  value = round(value);
  ledcWrite(ch, value);
}

void calibrate_sensors()
{
  Serial.println("Calibrating sensors. Keep the drone level...");
  delay(2000);
  float sum_roll = 0;
  float sum_pitch = 0;
  float sum_yaw = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++)
  {
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue))
    {
      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV)
      {
        float r = sensorValue.un.rotationVector.real;
        float i = sensorValue.un.rotationVector.i;
        float j = sensorValue.un.rotationVector.j;
        float k = sensorValue.un.rotationVector.k;

        sum_roll += atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 180 / M_PI;
        sum_pitch += asin(2 * (r * j - k * i)) * 180 / M_PI;
        sum_yaw += atan2(2 * (r * k + i * j), 1 - 2 * (j * j + k * k)) * 180 / M_PI;
      }
    }
    delay(20);
  }

  offset_roll = sum_roll / samples;
  offset_pitch = sum_pitch / samples;
  offset_yaw = sum_yaw / samples;

  Serial.println("Calibration complete!");
  Serial.print("Roll offset: ");
  Serial.print(offset_roll);
  Serial.print(" | Pitch offset: ");
  Serial.print(offset_pitch);
  Serial.print(" | Yaw offset: ");
  Serial.println(offset_yaw);
}

float r, i, j, k;
float wx, wy, wz;

void calc_target(){
  //float roll_euler = up * -40 + down * 40 - offset_roll;
  //float pitch_euler = left * -40 + right * 40 - offset_pitch;
  float roll_euler = -map(Ry, -128, 127, -40, 40) - offset_roll;
  float pitch_euler = map(Rx, -128, 127, -40, 40) - offset_pitch;

  yaw += (R1 * -2 + L1 * 2) % 360;
  float yaw_euluer = yaw + offset_yaw;

  cpp3d::quaternion q_roll(cos(roll_euler * M_PI / 180.0 / 2), sin(roll_euler * M_PI / 180.0 / 2), 0, 0);
  cpp3d::quaternion q_pitch(cos(pitch_euler * M_PI / 180.0 / 2), 0, sin(pitch_euler * M_PI / 180.0 / 2), 0);
  cpp3d::quaternion q_yaw(cos(yaw_euluer * M_PI / 180.0 / 2), 0, 0, sin(yaw_euluer * M_PI / 180.0 / 2));
  //q_target = q_roll * q_pitch * q_yaw;
  q_target = q_yaw * q_pitch * q_roll; // Yaw first, then pitch, then roll
}

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  ps5.begin("24:a6:fa:a4:fe:fe");
  Serial.println("PS5 Controller connectiong...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno08x.begin_I2C(0x4B))
  {
    Serial.println("BNO08x not detected");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x detected!");

  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000))
  {
    Serial.println("Failed to enable stabilized remote vector");
    while (1)
      delay(10);
  }
  Serial.println("Rotation Vector Report Enabled!");

  if(!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)){
    Serial.println("Failed to enable gyroscope report");
    while(1)
      delay(10);
  }
  Serial.println("Gyroscope Report Enabled!");

  ledcSetup(0, 50, 16);
  ledcSetup(1, 50, 16);
  ledcSetup(2, 50, 16);
  ledcSetup(3, 50, 16);

  ledcAttachPin(pwm_pin0, 0);
  ledcAttachPin(pwm_pin1, 1);
  ledcAttachPin(pwm_pin2, 2);
  ledcAttachPin(pwm_pin3, 3);

  set_motor(0, 100);
  set_motor(1, 100);
  set_motor(2, 100);
  set_motor(3, 100);
  delay(1000);
  set_motor(0, 0);
  set_motor(1, 0);
  set_motor(2, 0);
  set_motor(3, 0);
  delay(1000);
}

void loop()
{

  if(!ps5.isConnected()){
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    //Serial.println("PS5 Controller not connected!");
    return;
  }

  get_gamepad_data();

  calc_target();

  if(Circle && !prev_Circle){
    is_motor_running = !is_motor_running;
  }
  prev_Circle = Circle;

  is_calibration = Square;

  if(is_calibration && !calibration_done){
    calibrate_sensors();
    calibration_done = true;
    is_motor_running = false;
    Serial.println("Calibration done.");
  }

  if(!calibration_done){
    return;
  }

  if(!is_motor_running){
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    return;
  }

  sensor_current_time = millis();
  dt = (sensor_current_time - sensor_previous_time) / 1000.0;
  sensor_previous_time = sensor_current_time;

  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV)
    {
      r = sensorValue.un.rotationVector.real;
      i = sensorValue.un.rotationVector.i;
      j = sensorValue.un.rotationVector.j;
      k = sensorValue.un.rotationVector.k;
    }
    if(sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED){
      wx = sensorValue.un.gyroscope.x;
      wy = sensorValue.un.gyroscope.y;
      wz = sensorValue.un.gyroscope.z;
    }
  }

  q = cpp3d::quaternion(r, i, j, k);

  float roll = atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 180 / M_PI;
  float pitch = asin(2 * (r * j - k * i)) * 180 / M_PI;

  if (roll < -45 || roll > 45 || pitch < -45 || pitch > 45)
  {
    Serial.println("Drone is tilted too much, stopping motors.");
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    is_motor_running = false;
    is_calibration = false;
    calibration_done = false;
    Serial.println("Please calibrate the drone.");
    return;
  }

  motor_speed = offset_motor_speed + map(Ly,-128, 128, -offset_motor_speed, 100-offset_motor_speed);

  q_error = q_target * q.conjugate();
  

  float w = constrain(q_error.w, -1.0f, 1.0f);
  float angle = 2 * acos(w);
  float s = sqrt(1 - w * w);
  
  cpp3d::vec3d axis(0, 0, 0);
  if(s > 1e-6){
    axis = cpp3d::vec3d(q_error.x / s, q_error.y / s, q_error.z / s);
  }

  M.x = -kq * axis.x * angle - kw * wx;
  M.y = -kq * axis.y * angle - kw * wy;
  M.z = kq_yaw * axis.z * angle - kw_yaw * wz;

  motor_value0 = (motor_speed + M.x + M.y + M.z);
  motor_value1 = (motor_speed - M.x + M.y - M.z);
  motor_value2 = (motor_speed - M.x - M.y + M.z);
  motor_value3 = (motor_speed + M.x - M.y - M.z);

  motor_value0 = constrain(motor_value0, 0, 100);
  motor_value1 = constrain(motor_value1, 0, 100);
  motor_value2 = constrain(motor_value2, 0, 100);
  motor_value3 = constrain(motor_value3, 0, 100);

  set_motor(0, motor_value0);
  set_motor(1, motor_value1);
  set_motor(2, motor_value2);
  set_motor(3, motor_value3);

  /*Serial.print(r);
  Serial.print(",");
  Serial.print(i);
  Serial.print(",");
  Serial.print(j);
  Serial.print(",");
  Serial.print(k);
  Serial.print(",");
  Serial.print(q_target.w);
  Serial.print(",");
  Serial.print(q_target.x);
  Serial.print(",");
  Serial.print(q_target.y);
  Serial.print(",");
  Serial.print(q_target.z);
  Serial.println();*/
  /*Serial.print("Motor0: "); Serial.print(motor_value0);
  Serial.print(" Motor1: "); Serial.print(motor_value1);
  Serial.print(" Motor2: "); Serial.print(motor_value2);
  Serial.print(" Motor3: "); Serial.println(motor_value3);*/
  /*Serial.print("Left: "); Serial.println(left);
  Serial.print("Right: "); Serial.println(right);
  Serial.print("Up: "); Serial.println(up);
  Serial.print("Down: "); Serial.println(down);*/
  /*Serial.print("Rx: "); Serial.print(Rx);
  Serial.print(" Ry: "); Serial.print(Ry);
  Serial.print(" Lx: "); Serial.print(Lx);
  Serial.print(" Ly: "); Serial.println(Ly);*/
}
