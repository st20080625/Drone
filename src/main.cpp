#include <esp_system.h>
#include <Arduino.h>
#include <ps5Controller.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "3d.hpp"
#include <math.h>
#include "TFLuna_lidar.hpp"
#include <Adafruit_NeoPixel.h>

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
bool was_tilt = false;

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
float offset_motor_speed = 50;

float kq = 14; //13
float kw = 10; //10
float ki = 1;
float kq_yaw = 10; //9
float kw_yaw = 11; //10
float ki_yaw = 1;

cpp3d::vec3d M(0, 0, 0);

float roll_pid_value, pitch_pid_value, yaw_pid_value;
float integral_roll = 0, integral_pitch = 0, integral_yaw = 0;

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


#define LED_PIN 4
#define LED_COUNT 21

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB);

uint32_t Red = strip.Color(255, 0, 0); // State: Motor Stop
uint32_t Blue = strip.Color(0, 0, 255); // State: Calibrated
uint32_t Green = strip.Color(0, 255, 0); // State: Can Move(Waiting Start)
uint32_t Cyan = strip.Color(0, 156, 209); // State: Motor Spinning
uint32_t Orange = strip.Color(255, 92, 0); // State: Tilted Drone
uint32_t Purple = strip.Color(128, 0, 128); // State: Controller is not connected

void set_color(uint32_t color){
  for(int i = 0; i < LED_COUNT; ++i){
    strip.setPixelColor(i, color);
  }
}

void calibrate_sensors()
{
  uint32_t color = strip.Color(255, 0, 0);
  for(int i = 0; i < LED_COUNT; ++i){
    strip.setPixelColor(i, color);
  } 
  strip.show();
  Serial.println("Calibrating sensors. Keep the drone level...");
  delay(2000);
  float sum_roll = 0;
  float sum_pitch = 0;
  float sum_yaw = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++)
  {
    //sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue))
    {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
      {
        float r = sensorValue.un.rotationVector.real;
        float j = -sensorValue.un.rotationVector.i;
        float i = sensorValue.un.rotationVector.j;
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
  was_tilt = false;
}

float r, i, j, k;
float wx, wy, wz;
float ax, ay, az;

void calc_target(){
  //float roll_euler = up * -40 + down * 40 - offset_roll;
  //float pitch_euler = left * -40 + right * 40 - offset_pitch;
  float pitch_euler = map(Ry, -128, 127, -12, 12) + offset_pitch;
  float roll_euler = map(Rx, -128, 127, -12, 12) + offset_roll;

  yaw += (R1 * -0.5 + L1 * 0.5) ;
  if (yaw > 180){
    yaw -= 360;
  }else if (yaw < -180){
    yaw += 360;
  }
  
  float yaw_euluer = yaw + offset_yaw;

  cpp3d::quaternion q_roll(cos(roll_euler * M_PI / 180.0 / 2), sin(roll_euler * M_PI / 180.0 / 2), 0, 0);
  cpp3d::quaternion q_pitch(cos(pitch_euler * M_PI / 180.0 / 2), 0, sin(pitch_euler * M_PI / 180.0 / 2), 0);
  cpp3d::quaternion q_yaw(cos(yaw_euluer * M_PI / 180.0 / 2), 0, 0, sin(yaw_euluer * M_PI / 180.0 / 2));
  //q_target = q_roll * q_pitch * q_yaw;
  q_target = q_yaw * q_pitch * q_roll;
}

int deadzone = 18;

#define RX_PIN 16
#define TX_PIN 17

TFLuna* lidar = nullptr;

uint16_t distance = 0;
uint16_t strength = 0;
float temperature = 0.0;
uint16_t target_distance = 30;

int delay_time = 500;
unsigned long prev_millis = 0;

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(50);
  strip.clear();
  strip.show();

  ps5.begin("A0:FA:9C:2B:D4:DD"); //24:a6:fa:a4:fe:fe, A0:FA:9C:2B:D4:DD
  Serial.println("PS5 Controller connectiong...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno08x.begin_I2C(0x4B))
  {
    Serial.println("BNO08x not detected");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x detected!");

  //if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 5000))
  //{
    //Serial.println("Failed to enable rotation vector");
    //while (1)
      //delay(10);
  //}
  //Serial.println("Rotation Vector Report Enabled!");

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000))
  {
    Serial.println("Failed to enable game rotation vector");
    while(1)
      delay(10);
  }
  Serial.println("Game Rotation Vector Report Enabled!");

  if(!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)){
    Serial.println("Failed to enable gyroscope report");
    while(1)
      delay(10);
  }
  Serial.println("Gyroscope Report Enabled!");

  if(!bno08x.enableReport(SH2_ACCELEROMETER, 20000)){
    Serial.println("Failed to enable linear_acceleration report");
    while(1)
      delay(10);
  }

  //lidar = new TFLuna(Serial2, RX_PIN, TX_PIN);
  //Serial.println("TFLuna Lidar initialized.");
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
  delay(100);
  set_motor(0, 0);
  set_motor(1, 0);
  set_motor(2, 0);
  set_motor(3, 0);
  delay(1000);
}

void loop()
{
  set_color(Purple);
  if(!ps5.isConnected()){
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    //Serial.println("PS5 Controller not connected!");
    strip.show();
    return;
  }

  get_gamepad_data();
  if(Rx < deadzone && Rx > -deadzone){
    Rx = 0;
  }
  if(Ry < deadzone && Ry > -deadzone){
    Ry = 0;
  }

  calc_target();
  strip.setBrightness(50);
  if(!calibration_done){
    set_color(Red);
  }
  if(calibration_done){
    set_color(Blue);
  }
  if(is_motor_running){
    set_color(Cyan);
  }
  unsigned long current_millis = millis();
  if(was_tilt){
  if(current_millis - prev_millis >= delay_time){
    strip.setBrightness(255);
    set_color(Orange);
    prev_millis = current_millis;
  }else{
    strip.clear();
  }}
   
  if(Circle && !prev_Circle){
    is_motor_running = !is_motor_running;
  }
  prev_Circle = Circle;
  
  strip.show();

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
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
      r = sensorValue.un.rotationVector.real;
      j = -sensorValue.un.rotationVector.i;
      i = sensorValue.un.rotationVector.j;
      k = sensorValue.un.rotationVector.k;
    }
    //if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
      //cr = sensorValue.un.rotationVector.real;
      //cj = -sensorValue.un.rotationVector.i;
      //ci = sensorValue.un.rotationVector.j;
      //ck = sensorValue.un.rotationVector.k;

    if(sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED){
      wy = sensorValue.un.gyroscope.x;
      wx = -sensorValue.un.gyroscope.y;
      wz = -sensorValue.un.gyroscope.z;
    }
    if(sensorValue.sensorId == SH2_ACCELEROMETER){
      ax = -sensorValue.un.accelerometer.x;
      ay = -sensorValue.un.accelerometer.y;
      az = sensorValue.un.accelerometer.z;
    }
  }

  q = cpp3d::quaternion(r, i, j, k);

  float roll = atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 180 / M_PI - offset_roll;
  float pitch = asin(2 * (r * j - k * i)) * 180 / M_PI - offset_pitch;
  if (roll < -30 || roll > 30 || pitch < -30 || pitch > 30)
  {
    Serial.println("Drone is tilted too much, stopping motors.");
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    is_motor_running = false;
    is_calibration = false;
    calibration_done = false;
    was_tilt = true;
    roll = 0;
    pitch = 0;
    r = 0;
    i = 0;
    j = 0;
    k = 0;
    Serial.println("Please calibrate the drone.");
    return;
  }

  motor_speed = offset_motor_speed + map(Ly,-128, 128, -offset_motor_speed, 100-offset_motor_speed);

  q_error = q_target * q.conjugate();
  q_error = q_error.normalize();
  if(q_error.w < 0){
    q_error = q_error.scalar(-1);
  }

  integral_roll += q_error.x * dt;
  integral_pitch += q_error.y * dt;
  integral_yaw += q_error.z * dt;
  
  float w = constrain(q_error.w, -1.0f, 1.0f);
  float angle = 2 * acos(w);
  float s = sqrt(1 - w * w);
  
  cpp3d::vec3d axis(0, 0, 0);
  if(s > 1e-6){
    axis = cpp3d::vec3d(q_error.x / s, q_error.y / s, q_error.z / s);
  }

  if(angle < 0.001){
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  M.x = -kq * axis.x * angle + -kw * wx -ki * integral_roll;
  M.y = -kq * axis.y * angle + -kw * wy -ki * integral_pitch;
  M.z = -kq_yaw * axis.z * angle - kw_yaw * wz -ki * integral_yaw; 

  /*if(lidar->readData(distance, strength, temperature)){
    Serial.print("Distance: ");
    Serial.println(distance);
  }*/

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

  /*Serial.print(q_target.w);
  Serial.print(",");
  Serial.print(q_target.x);
  Serial.print(",");
  Serial.print(q_target.y);
  Serial.print(",");
  Serial.print(q_target.z);
  Serial.print(",");*/
  Serial.print(" Motor0: "); Serial.print(motor_value0);
  Serial.print(" Motor1: "); Serial.print(motor_value1);
  Serial.print(" Motor2: "); Serial.print(motor_value2);
  Serial.print(" Motor3: "); Serial.println(motor_value3);
  /*Serial.print("Left: "); Serial.println(left);
  Serial.print("Right: "); Serial.println(right);
  Serial.print("Up: "); Serial.println(up);
  Serial.print("Down: "); Serial.println(down);*/
  /*Serial.print("Rx: "); Serial.print(Rx);
  Serial.print(" Ry: "); Serial.print(Ry);
  Serial.print(" Lx: "); Serial.print(Lx);
  Serial.print(" Ly: "); Serial.println(Ly);*/
  /*Serial.print(r);
  Serial.print(",");
  Serial.print(i);
  Serial.print(",");
  Serial.print(j);
  Serial.print(",");
  Serial.println(k);*/
  /*Serial.print(",");
  Serial.print(wx);
  Serial.print(",");
  Serial.print(wy);
  Serial.print(",");
  Serial.print(wz);
  Serial.print(",");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.println(az);*/
}
