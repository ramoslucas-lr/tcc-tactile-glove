#include <Wire.h>
#include <MPU6050.h>
#include <helper_3dmath.h>

MPU6050 mpu;

struct Texture {
  float frequency;
  float duty_cycle;
};

const int pwmPin1 = 3;
const int pwmPin2 = 5;
const int pwmPin3 = 6;
const int pwmPin4 = 9;
const int pwmPin5 = 10;
const int pwmPin6 = 11;

bool connected = false;
bool simulating = false;
unsigned long start_time = 0;

Texture rough_texture = { 7.2, 32.0 };
Texture fine_texture = { 15.6, 34.0 };
Texture smooth_texture = { 333.0, 67.0 };

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setDLPFMode(6);

  TCCR1A = 0;
  TCCR1B = 0;

  // Set non-inverting mode
  TCCR1A |= (1 << COM1A1);

  // Set fast PWM Mode 14
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << WGM13);

  // Set prescaler to 64 and starts PWM
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11); 

  pinMode(pwmPin1, INPUT);
  pinMode(pwmPin2, INPUT);
  pinMode(pwmPin3, INPUT);
  pinMode(pwmPin4, INPUT);
  pinMode(pwmPin5, INPUT);
  pinMode(pwmPin6, INPUT);
}

void loop() {
  while (!connected) {
    if (Serial.available() > 0) {
      String message = Serial.readStringUntil("\n");
      if (message == "ready?\n") {
        Serial.println("yes");
      }

      else if (message == "connected\n") {
        connected = true;
      }
    }
  }

  Quaternion q = calculateQuaternion();  // calculate the quaternion
  sendQuaternion(q);                     // send the quaternion over serial communication

  if (Serial.available() > 0) {
    String message = Serial.readStringUntil("\n");

    if (message == "rough\n") {
      startSimulation(rough_texture);
      start_time = millis(); 
      simulating = true;
    }

    else if (message == "fine\n") {
      startSimulation(fine_texture);
      start_time = millis(); 
      simulating = true;
    }

    else if (message == "smooth\n") {
      startSimulation(smooth_texture);
      start_time = millis(); 
      simulating = true;
    }

    else if (message == "stop\n") {
      stopSimulation();
      start_time = millis(); 
      simulating = false;
    }

    else if (message == "bye\n") {
      stopSimulation();
      start_time = millis(); 
      simulating = false;
      connected = false;
    }
  }

  if (simulating) {
    unsigned long elapsed_time = millis() - start_time;
    if (elapsed_time > 15000) {
      stopSimulation();
    }
  }
}


void startSimulation(Texture texture) {  
  // Set PWM frequency/top value
  ICR1 = round((F_CPU / (64*texture.duty_cycle)) - 1);
  OCR1A = round(ICR1 / (100 / texture.duty_cycle));

  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);
  
}

void stopSimulation() {
  pinMode(pwmPin1, INPUT);
  pinMode(pwmPin2, INPUT);
  pinMode(pwmPin3, INPUT);
  pinMode(pwmPin4, INPUT);
  pinMode(pwmPin5, INPUT);
  pinMode(pwmPin6, INPUT);
}

Quaternion calculateQuaternion() {
  float ax_raw = mpu.getAccelerationX();
  float ay_raw = mpu.getAccelerationY();
  float az_raw = mpu.getAccelerationZ();
  float gx_raw = mpu.getRotationX();
  float gy_raw = mpu.getRotationY();
  float gz_raw = mpu.getRotationZ();

  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gx = gx_raw / 131.0;
  float gy = gy_raw / 131.0;
  float gz = gz_raw / 131.0;

  float roll = atan2(-ay, az);
  float pitch = atan2(ax, sqrt(ay * ay + az * az));
  float yaw = atan2(gy, sqrt(gx * gx + gz * gz));

  float qw = cos(DEG_TO_RAD * roll / 2) * cos(DEG_TO_RAD * pitch / 2) * cos(DEG_TO_RAD * yaw / 2) + sin(DEG_TO_RAD * roll / 2) * sin(DEG_TO_RAD * pitch / 2) * sin(DEG_TO_RAD * yaw / 2);
  float qx = sin(DEG_TO_RAD * roll / 2) * cos(DEG_TO_RAD * pitch / 2) * cos(DEG_TO_RAD * yaw / 2) - cos(DEG_TO_RAD * roll / 2) * sin(DEG_TO_RAD * pitch / 2) * sin(DEG_TO_RAD * yaw / 2);
  float qy = cos(DEG_TO_RAD * roll / 2) * sin(DEG_TO_RAD * pitch / 2) * cos(DEG_TO_RAD * yaw / 2) + sin(DEG_TO_RAD * roll / 2) * cos(DEG_TO_RAD * pitch / 2) * sin(DEG_TO_RAD * yaw / 2);
  float qz = cos(DEG_TO_RAD * roll / 2) * cos(DEG_TO_RAD * pitch / 2) * sin(DEG_TO_RAD * yaw / 2) - sin(DEG_TO_RAD * roll / 2) * sin(DEG_TO_RAD * pitch / 2) * cos(DEG_TO_RAD * yaw / 2);

  Quaternion q = { qw, qx, qy, qz };
  return q;
}

void sendQuaternion(Quaternion q) {
  Serial.print(q.w, 6);
  Serial.print(";");
  Serial.print(q.x, 6);
  Serial.print(";");
  Serial.print(q.y, 6);
  Serial.print(";");
  Serial.println(q.z, 6);
}