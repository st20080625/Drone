#include "3d.hpp"
#include "TFLuna_lidar.hpp"
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <math.h>
#include <ps5Controller.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Drone";
const char* Raspi_Addr = "10.42.0.1";
const int Imu_Port = 8001;
const int Log_Port = 8002;

IPAddress local_IP(10, 42, 0, 2);
IPAddress gateway(10, 42, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP Imu_Sock;
WiFiUDP Log_Sock;

uint8_t Imu_Buffer[40];
uint8_t Log_Buffer[32];

void init_wifi(){
  if (!WiFi.config(local_IP, gateway, subnet)){
    Serial.println("STA Failed to Configure");
  }
  WiFi.begin(ssid);
  Serial.print("Connecting to  Raspi_AP");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected Raspi_AP");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Imu_Sock.begin(Imu_Port);
  Log_Sock.begin(Log_Port);
}

struct SensorPacket {
  float r;
  float i;
  float j;
  float k;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

struct LogPacket{
  float r;
  float i;
  float j;
  float k;
  float target_r;
  float target_i;
  float target_j;
  float target_k;
};

SensorPacket imu_packet;
bool initialized_send_imu_data = false;
unsigned long prev_send_time = 0;

LogPacket log_packet;

int Lx = 0;
int Ly = 0;
int Rx = 0;
int Ry = 0;
int Square = 0;
int Circle = 0;
int Triangle = 0;
int Cross = 0;
int prev_Circle = 0;
int prev_Triangle = 0;
int prev_Cross = 0;
int R1 = 0;
int L1 = 0;
int up = 0;
int down = 0;
int right = 0;
int left = 0;

bool calibration_done = false;
bool is_calibration = false;
bool is_motor_running = false;
bool prev_is_motor_running = false;
bool was_tilt = false;
bool was_create_socket = false;

void get_gamepad_data() {
  Lx = ps5.LStickX();
  Ly = ps5.LStickY();
  Rx = ps5.RStickX();
  Ry = ps5.RStickY();
  Square = ps5.Square();
  Circle = ps5.Circle();
  Triangle = ps5.Triangle();
  Cross = ps5.Cross();
  R1 = ps5.R1();
  L1 = ps5.L1();
  up = ps5.Up();
  down = ps5.Down();
  right = ps5.Right();
  left = ps5.Left();
}

float motor_speed;
float offset_motor_speed = 65;

// kq = P gain, kw = D gain
// roll, pitch -> PID
// yaw -> PD
float kq = 10;     // 13
float kw = 8;     // 10
float ki = 12;     // 25
float kq_yaw = 15; // 9
float kw_yaw = 13; // 10
float ki_yaw = 0;

float hover_kp = 0.6;
float hover_kd = 0;
float hover_ki = 0;

cpp3d::vec3d M(0, 0, 0);

float integral_roll = 0, integral_pitch = 0, integral_yaw = 0;

#define SDA_PIN 21
#define SCL_PIN 22

int pwm_pin0 = 25;
int pwm_pin1 = 12;
int pwm_pin2 = 27;
int pwm_pin3 = 26;

float motor_value0 = 0;
float motor_value1 = 0;
float motor_value2 = 0;
float motor_value3 = 0;

Adafruit_BNO08x bno08x;

#define BNO08X_RESET -1

cpp3d::quaternion q(1, 0, 0, 0);
cpp3d::quaternion q_target(1, 0, 0, 0);
cpp3d::quaternion q_error(1, 0, 0, 0);

float dt;

unsigned long sensor_previous_time = 0;
unsigned long sensor_current_time;

sh2_SensorValue_t sensorValue;

// frequency = 4Khz (4882.81hz)
// resolution = 14bit (16383)
// min_pulse = 125 us
// max_pulse = 250 us
// period = 204.8 us    1/4882.81

// frequenct = 2.67Khz (2441.41hz)
// resolution = 15bit (32768)
// period = 409.6 us
void set_motor(int ch, float on_pulse_rate) {
  float on_pulse_normalized = on_pulse_rate / 1000.0; //  0.0 ~ 1.0
  float pulse_width = (125.0 + on_pulse_normalized * 125.0);
  // Serial.println(pulse_width);
  float duty = (pulse_width / 409.6) * 32768;
  uint16_t value = round(duty);
  ledcWrite(ch, value);
}

#define LED_PIN 4
#define LED_COUNT 21

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB);

uint32_t Red = strip.Color(255, 0, 0);     // State: Motor Stop
uint32_t Blue = strip.Color(0, 0, 255);    // State: Calibrated
uint32_t Green = strip.Color(0, 255, 0);   // State: Can Move(Waiting Start)
uint32_t Cyan = strip.Color(0, 156, 209);  // State: Motor Spinning
uint32_t Orange = strip.Color(255, 92, 0); // State: Tilted Drone
uint32_t Purple =
    strip.Color(128, 0, 128); // State: Controller is not connected
uint32_t Lime =
    strip.Color(172, 255, 47); // State: Auto Hover Mode
                               // This color is used between after calibration
                               // and before motor starts.

#define RX_PIN 16
#define TX_PIN 17

TFLuna *lidar = nullptr;

uint16_t distance = 0;
uint16_t strength = 0;
float temperature = 0.0;
bool is_auto_hover = false;
uint16_t offset_distance = 0;
float target_distance = 0;
uint16_t prev_distance = 0;
uint16_t integral_distance = 0;

void set_color(uint32_t color) {
  for (int i = 0; i < LED_COUNT; ++i) {
    strip.setPixelColor(i, color);
  }
}

cpp3d::quaternion offset_q(1, 0, 0, 0);
void calibrate_sensors() {
  uint32_t color = strip.Color(255, 0, 0);
  for (int i = 0; i < LED_COUNT; ++i) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  Serial.println("Calibrating sensors. Keep the drone level...");
  // delay(2000);
  float sum_r = 0;
  float sum_i = 0;
  float sum_j = 0;
  float sum_k = 0;
  int samples = 50;

  cpp3d::quaternion first_q(1, 0, 0, 0);
  bool is_first = true;
  int collected = 0;
  int N = 50;

  uint16_t sum_distance = 0;
  int collected_distance = 0;
  while (collected < N) {
    // sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float r = sensorValue.un.rotationVector.real;
        float j = -sensorValue.un.rotationVector.i;
        float i = sensorValue.un.rotationVector.j;
        float k = sensorValue.un.rotationVector.k;

        if (is_first) {
          first_q.w = r;
          first_q.x = i;
          first_q.y = j;
          first_q.z = k;
          is_first = false;
          if (first_q.w < 0.0) {
            first_q = first_q.scalar(-1);
            sum_r += first_q.w;
            sum_i += first_q.x;
            sum_j += first_q.y;
            sum_k += first_q.z;
          }
        } else {
          cpp3d::quaternion current_q(r, i, j, k);
          // float dot = first_q.dot(current_q);
          if (current_q.w < 0.0) {
            current_q = current_q.scalar(-1.0);
          }
          sum_r += current_q.w;
          sum_i += current_q.x;
          sum_j += current_q.y;
          sum_k += current_q.z;
          collected++;
        }
      }
    }
    while (collected_distance < N) {
      if (lidar->readData(distance, strength, temperature)) {
        sum_distance += distance;
        collected_distance++;
      }
    }
    delay(20);
  }

  if (collected > 0) {
    offset_q.w = sum_r / collected;
    offset_q.x = sum_i / collected;
    offset_q.y = sum_j / collected;
    offset_q.z = sum_k / collected;
    offset_q = offset_q.normalize();
  }

  if (collected_distance > 0) {
    offset_distance = sum_distance / collected_distance;
    prev_distance = offset_distance;
    // Serial.println(target_distance);
  }

  Serial.println("Calibration complete!");
  Serial.print("Offset_R: ");
  Serial.print(offset_q.w);
  Serial.print(" | Offset_I: ");
  Serial.print(offset_q.x);
  Serial.print(" | Offset_J: ");
  Serial.print(offset_q.y);
  Serial.print(" | Offset_K: ");
  Serial.print(offset_q.z);
  Serial.print(" | Offset_Distance: ");
  Serial.println(offset_distance);
  was_tilt = false;
}

float r, i, j, k;
float wx, wy, wz;
float ax, ay, az;
float yaw = 0;

void SendImuData(){
  if(!initialized_send_imu_data){
    prev_send_time = millis();
    initialized_send_imu_data = true;
  }
  unsigned long current_time = millis();
  if((current_time - prev_send_time) < 10){ // send / 10ms = 100Hz
    return;
  }
  prev_send_time = current_time;
  imu_packet.acc_x = ax;
  imu_packet.acc_y = ay;
  imu_packet.acc_z = az;
  imu_packet.gyro_x = wx;
  imu_packet.gyro_y = wy;
  imu_packet.gyro_z = wz;
  imu_packet.r = r;
  imu_packet.i = i;
  imu_packet.j = j;
  imu_packet.k = k;
  memcpy(Imu_Buffer, &imu_packet, sizeof(imu_packet));
  
  Imu_Sock.beginPacket(Raspi_Addr, Imu_Port);
  Imu_Sock.write(Imu_Buffer, sizeof(imu_packet));
  Imu_Sock.endPacket();
}

void SendLogData(){
  if(!initialized_send_imu_data){
    prev_send_time = millis();
    initialized_send_imu_data = true;
  }
  unsigned long current_time = millis();
  if((current_time - prev_send_time) < 10){ // send / 10ms = 100Hz
    return;
  }
  prev_send_time = current_time;
  log_packet.r = r;
  log_packet.i = i;
  log_packet.j = j;
  log_packet.k = k;

  log_packet.target_r = q_target.w;
  log_packet.target_i = q_target.x;
  log_packet.target_j = q_target.y;
  log_packet.target_k = q_target.z;

  memcpy(Log_Buffer, &log_packet, sizeof(log_packet));
  
  Imu_Sock.beginPacket(Raspi_Addr, Log_Port);
  Imu_Sock.write(Log_Buffer, sizeof(log_packet));
  Imu_Sock.endPacket();
}

float roll_euler, pitch_euler, yaw_euler;
void calc_target() {
  pitch_euler = map(Ry, -128, 127, -9, 9);
  roll_euler = map(Rx, -128, 127, -9, 9);

  yaw += (R1 * -0.5 + L1 * 0.5);
  if (yaw > 180) {
    yaw -= 360;
  } else if (yaw < -180) {
    yaw += 360;
  }
  yaw_euler = yaw;

  cpp3d::quaternion q_roll(cos(roll_euler * M_PI / 180.0 / 2),
                           sin(roll_euler * M_PI / 180.0 / 2), 0, 0);
  cpp3d::quaternion q_pitch(cos(pitch_euler * M_PI / 180.0 / 2), 0,
                            sin(pitch_euler * M_PI / 180.0 / 2), 0);
  cpp3d::quaternion q_yaw(cos(yaw_euler * M_PI / 180.0 / 2), 0, 0,
                          sin(yaw_euler * M_PI / 180.0 / 2));
  // q_target = q_roll * q_pitch * q_yaw;
  q_target = q_yaw * q_pitch * q_roll;
  q_target = offset_q * q_target;
  q_target = q_target.normalize();
}

int deadzone = 18;

int delay_time = 500;
unsigned long prev_millis = 0;

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(50);
  strip.clear();
  strip.show();

  //init_wifi();

  ps5.begin("A0:FA:9C:2B:D4:DD"); // change here your Mac address
  Serial.println("PS5 Controller connectiong...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("BNO08x not detected");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x detected!");

  // if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 5000))
  //{
  // Serial.println("Failed to enable rotation vector");
  // while (1)
  // delay(10);
  //}
  // Serial.println("Rotation Vector Report Enabled!");

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("Failed to enable game rotation vector");
    while (1)
      delay(10);
  }
  Serial.println("Game Rotation Vector Report Enabled!");

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) {
    Serial.println("Failed to enable gyroscope report");
    while (1)
      delay(10);
  }
  Serial.println("Gyroscope Report Enabled!");

  if (!bno08x.enableReport(SH2_ACCELEROMETER, 20000)) {
    Serial.println("Failed to enable linear_acceleration report");
    while (1)
      delay(10);
  }

  lidar = new TFLuna(Serial2, RX_PIN, TX_PIN);
  Serial.println("TFLuna Lidar initialized.");
  ledcSetup(0, 2441.41, 15);
  ledcSetup(1, 2441.41, 15);
  ledcSetup(2, 2441.41, 15);
  ledcSetup(3, 2441.41, 15);

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
  delay(10);
}

void loop() {
  set_color(Purple);
  if (WiFi.status() != WL_CONNECTED){
    was_create_socket = false;
  }
  if (!ps5.isConnected()) {
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    // Serial.println("PS5 Controller not connected!");
    strip.show();
    return;
  }

  get_gamepad_data();
  if (Rx < deadzone && Rx > -deadzone) {
    Rx = 0;
  }
  if (Ry < deadzone && Ry > -deadzone) {
    Ry = 0;
  }

  calc_target();
  strip.setBrightness(50);
  if (!calibration_done) {
    set_color(Red);
  }
  if (calibration_done) {
    set_color(Blue);
  }
  if (is_auto_hover) {
    set_color(Lime);
  }
  if (is_motor_running) {
    set_color(Cyan);
  }
  unsigned long current_millis = millis();
  if (was_tilt) {
    if (current_millis - prev_millis >= delay_time) {
      strip.setBrightness(255);
      set_color(Orange);
      prev_millis = current_millis;
    } else {
      strip.clear();
    }
  }

  if (Circle && !prev_Circle) {
    prev_is_motor_running = is_motor_running;
    is_motor_running = !is_motor_running;
  }
  if (prev_is_motor_running) {
    target_distance = 0;
  }
  prev_Circle = Circle;

  if (Triangle && !prev_Triangle && !is_motor_running) {
    is_auto_hover = !is_auto_hover;
  }
  prev_Triangle = Triangle;
  if (is_auto_hover) {
    set_color(Lime);
  }

  if (is_motor_running) {
    set_color(Cyan);
  }

  strip.show();

  is_calibration = Square;

  if (is_calibration && !calibration_done) {
    calibrate_sensors();
    calibration_done = true;
    is_motor_running = false;
    Serial.println("Calibration done.");
  }

  if (!calibration_done) {
    return;
  }

  if(calibration_done && Cross && !prev_Cross && !was_create_socket){
    init_wifi();
    was_create_socket = true;
    Serial.println("Create socket");
  }

  if (!is_motor_running) {
    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    return;
  }
  if (!is_auto_hover) {
    sensor_current_time = millis();
    dt = (sensor_current_time - sensor_previous_time) / 1000.0;
    dt = max(dt, 0.0002f);
    sensor_previous_time = sensor_current_time;
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        r = sensorValue.un.rotationVector.real;
        j = -sensorValue.un.rotationVector.i;
        i = sensorValue.un.rotationVector.j;
        k = sensorValue.un.rotationVector.k;
      }
      // if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
      // cr = sensorValue.un.rotationVector.real;
      // cj = -sensorValue.un.rotationVector.i;
      // ci = sensorValue.un.rotationVector.j;
      // ck = sensorValue.un.rotationVector.k;

      if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        wy = sensorValue.un.gyroscope.x;
        wx = -sensorValue.un.gyroscope.y;
        wz = -sensorValue.un.gyroscope.z;
      }
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        ax = -sensorValue.un.accelerometer.x;
        ay = -sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
      }
    }
    if(was_create_socket){SendImuData();};
    if(was_create_socket){SendLogData();};
    q = cpp3d::quaternion(r, i, j, k);
    float roll =
        atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 180 / M_PI;
    float pitch = asin(2 * (r * j - k * i)) * 180 / M_PI;

    if (roll < -30 || roll > 30 || pitch < -30 || pitch > 30) {
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
      yaw = 0;
      r = 1;
      i = 0;
      j = 0;
      k = 0;
      q_target = cpp3d::quaternion(1, 0, 0, 0);
      integral_roll = 0;
      integral_pitch = 0;
      integral_yaw = 0;
      Serial.println("Please calibrate the drone.");
      return;
    }

    if (Ly >= 0) {
      motor_speed =
          offset_motor_speed + map(Ly, 0, 127, 0, 100 - offset_motor_speed);
    }
    if (Ly < 0) {
      motor_speed = map(Ly, -128, 0, 0, offset_motor_speed);
    }

    q_error = q.conjugate() * q_target;
    q_error = q_error.normalize();
    if (q_error.w < 0) {
      q_error = q_error.scalar(-1);
    }

    float w = constrain(q_error.w, -1.0f, 1.0f);
    float angle = 2 * acos(w);
    float s = sqrt(1 - w * w);

    cpp3d::vec3d axis(0, 0, 0);
    if (s > 1e-6) {
      axis = cpp3d::vec3d(q_error.x / s, q_error.y / s, q_error.z / s);
    }

    integral_roll += axis.x * angle * dt;
    integral_pitch += axis.y * angle * dt;
    integral_yaw += axis.z * angle * dt;

    // 1 deg = 0.01745 rad
    if (fabs(axis.x * angle) < 0.01745)
      integral_roll = 0;
    if (fabs(axis.y * angle) < 0.01745)
      integral_pitch = 0;
    // 5 deg = 0.0873 rad
    if (fabs(axis.z * angle) < 0.0873)
      integral_yaw = 0;
    /*Serial.print("integral_roll: ");
    Serial.print(integral_roll);
    Serial.print(", ");
    Serial.print("integral_pitch: ");
    Serial.print(integral_pitch);
    Serial.print(", ");
    Serial.print("integral_yaw: ");
    Serial.println(integral_yaw);*/

    M.x = -kq * axis.x * angle + -kw * wx - ki * integral_roll;
    M.y = -kq * axis.y * angle + -kw * wy - ki * integral_pitch;
    M.z = -kq_yaw * axis.z * angle - kw_yaw * wz - ki_yaw * integral_yaw;

    /*if (lidar->readData(distance, strength, temperature)) {
      Serial.print("Distance: ");
      Serial.println(distance);
    }*/

    motor_value0 = (motor_speed + M.x + M.y + M.z);
    motor_value1 = (motor_speed - M.x + M.y - M.z);
    motor_value2 = (motor_speed - M.x - M.y + M.z);
    motor_value3 = (motor_speed + M.x - M.y - M.z);

    motor_value0 *= 10.0;
    motor_value1 *= 10.0;
    motor_value2 *= 10.0;
    motor_value3 *= 10.0;

    motor_value0 = constrain(motor_value0, 0, 1000.0);
    motor_value1 = constrain(motor_value1, 0, 1000.0);
    motor_value2 = constrain(motor_value2, 0, 1000.0);
    motor_value3 = constrain(motor_value3, 0, 1000.0);
    
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
    Serial.print(" Motor0: ");
    Serial.print(motor_value0);
    Serial.print(" Motor1: ");
    Serial.print(motor_value1);
    Serial.print(" Motor2: ");
    Serial.print(motor_value2);
    Serial.print(" Motor3: ");
    Serial.println(motor_value3);
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
  } else {
    sensor_current_time = millis();
    dt = (sensor_current_time - sensor_previous_time) / 1000.0;
    dt = max(dt, 0.0002f);
    sensor_previous_time = sensor_current_time;
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        r = sensorValue.un.rotationVector.real;
        j = -sensorValue.un.rotationVector.i;
        i = sensorValue.un.rotationVector.j;
        k = sensorValue.un.rotationVector.k;
      }
      // if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
      // cr = sensorValue.un.rotationVector.real;
      // cj = -sensorValue.un.rotationVector.i;
      // ci = sensorValue.un.rotationVector.j;
      // ck = sensorValue.un.rotationVector.k;

      if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        wy = sensorValue.un.gyroscope.x;
        wx = -sensorValue.un.gyroscope.y;
        wz = -sensorValue.un.gyroscope.z;
      }
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        ax = -sensorValue.un.accelerometer.x;
        ay = -sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
      }
    }
    if(was_create_socket){SendImuData();};
    if(was_create_socket){SendLogData();};
    q = cpp3d::quaternion(r, i, j, k);
    float roll =
        atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)) * 180 / M_PI;
    float pitch = asin(2 * (r * j - k * i)) * 180 / M_PI;

    if (roll < -30 || roll > 30 || pitch < -30 || pitch > 30) {
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
      yaw = 0;
      r = 1;
      i = 0;
      j = 0;
      k = 0;
      q_target = cpp3d::quaternion(1, 0, 0, 0);
      integral_roll = 0;
      integral_pitch = 0;
      integral_yaw = 0;
      offset_distance = 0;
      target_distance = 0;
      Serial.println("Please calibrate the drone.");
      return;
    }

    if (Ly >= 0) {
      motor_speed =
          offset_motor_speed + map(Ly, 0, 127, 0, 100 - offset_motor_speed);
    }
    if (Ly < 0) {
      motor_speed = map(Ly, -128, 0, 0, offset_motor_speed);
    }
    // Serial.println(motor_speed);
    Serial.println(target_distance);

    /*if (lidar->readData(distance, strength, temperature)) {
      Serial.print("Distance: ");
      Serial.println(distance);
    }*/
    lidar->readData(distance, strength, temperature);

    if (up) {
      if (target_distance <= 99.5) {
        target_distance += 0.5;
        Serial.println("up");
      }
    }
    if (down) {
      if (target_distance >= 0.5) {
        target_distance -= 0.5;
        Serial.println("down");
      }
    }

    Serial.println(target_distance);

    float target_distance_control_val =
        target_distance + float(offset_distance);
    // Serial.println(target_distance);

    float motor_speed_p = hover_kp * (target_distance_control_val - distance);
    float motor_speed_d = hover_kd * (distance - prev_distance) / dt;
    integral_distance += (target_distance_control_val - distance) * dt;
    if(abs(distance - prev_distance) < 5){
      integral_distance = 0;
    }
    float motor_speed_i = hover_ki * integral_distance;
    motor_speed = offset_motor_speed + motor_speed_p - motor_speed_d - motor_speed_i;
    prev_distance = distance;

    // Serial.println(motor_speed);

    q_error = q.conjugate() * q_target;
    q_error = q_error.normalize();
    if (q_error.w < 0) {
      q_error = q_error.scalar(-1);
    }

    float w = constrain(q_error.w, -1.0f, 1.0f);
    float angle = 2 * acos(w);
    float s = sqrt(1 - w * w);

    cpp3d::vec3d axis(0, 0, 0);
    if (s > 1e-6) {
      axis = cpp3d::vec3d(q_error.x / s, q_error.y / s, q_error.z / s);
    }

    integral_roll += axis.x * angle * dt;
    integral_pitch += axis.y * angle * dt;
    integral_yaw += axis.z * angle * dt;

    // 1 deg = 0.01745 rad
    if (fabs(axis.x * angle) < 0.01745)
      integral_roll = 0;
    if (fabs(axis.y * angle) < 0.01745)
      integral_pitch = 0;
    // 5 deg = 0.0873 rad
    if (fabs(axis.z * angle) < 0.0873)
      integral_yaw = 0;
    /*Serial.print("integral_roll: ");
    Serial.print(integral_roll);
    Serial.print(", ");
    Serial.print("integral_pitch: ");
    Serial.print(integral_pitch);
    Serial.print(", ");
    Serial.print("integral_yaw: ");
    Serial.println(integral_yaw);*/

    M.x = -kq * axis.x * angle + -kw * wx - ki * integral_roll;
    M.y = -kq * axis.y * angle + -kw * wy - ki * integral_pitch;
    M.z = -kq_yaw * axis.z * angle - kw_yaw * wz - ki_yaw * integral_yaw;

    motor_value0 = (motor_speed + M.x + M.y + M.z);
    motor_value1 = (motor_speed - M.x + M.y - M.z);
    motor_value2 = (motor_speed - M.x - M.y + M.z);
    motor_value3 = (motor_speed + M.x - M.y - M.z);

    motor_value0 = motor_value0 * 10.0;
    motor_value1 = motor_value1 * 10.0;
    motor_value2 = motor_value2 * 10.0;
    motor_value3 = motor_value3 * 10.0;

    motor_value0 = constrain(motor_value0, 0, 1000.0);
    motor_value1 = constrain(motor_value1, 0, 1000.0);
    motor_value2 = constrain(motor_value2, 0, 1000.0);
    motor_value3 = constrain(motor_value3, 0, 1000.0);

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
    Serial.print(" Motor0: ");
    Serial.print(motor_value0);
    Serial.print(" Motor1: ");
    Serial.print(motor_value1);
    Serial.print(" Motor2: ");
    Serial.print(motor_value2);
    Serial.print(" Motor3: ");
    Serial.println(motor_value3);
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
}
