//#define interruptPin 19
// non interupt pin set to 14
#define OFFSETS   -624,    -226,     944,      49,      73,     -33

#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board

Simple_MPU6050 mpu;


float yaw_value;


double IMU_Kp = 7.65;
double IMU_Ki = 0.01;
double IMU_Kd = 0.02;



double IMU_setpoint = 90;
double fixed_setpoint = 0;
double IMU_error, IMU_last_error, IMU_error_sum, IMU_error_diff, fixed_error, fixed_integral, fixed_derivative, fixed_last_error;

double IMU_output;

unsigned long IMU_current_time, IMU_last_time, fixed_current_time, fixed_last_time;
double IMU_dt, fixed_dt;

int heading_speed;
int motor_speed;

//3.8
double fixed_kp = 3.8;
double fixed_ki = 0.01;
double fixed_kd = 0.02;


float IMU_raw_pitch;
float IMU_raw_val;


void Print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  yaw_value = fmod((360 + xyz[0]), 360);
  IMU_raw_val = xyz[0];
  IMU_raw_pitch = xyz[1];

  //    Serial.print("Yaw : ");
  //    Serial.print(yaw_value);
  //  Serial.print(F("Yaw "));   Serial.print(xyz[0]);   Serial.print(F(",   "));
  //  Serial.print(F("Pitch ")); Serial.print(xyz[1]);   Serial.print(F(",   "));
  //  Serial.print(F("Roll "));  Serial.print(xyz[2]);   Serial.print(F(",   "));
  //  Serial.print(F("ax "));    Serial.print(accel[0]); Serial.print(F(",   "));
  //  Serial.print(F("ay "));    Serial.print(accel[1]); Serial.print(F(",   "));
  //  Serial.print(F("az "));    Serial.print(accel[2]); Serial.print(F(",   "));
  //  Serial.print(F("gx "));    Serial.print(gyro[0]);  Serial.print(F(",   "));
  //  Serial.print(F("gy "));    Serial.print(gyro[1]);  Serial.print(F(",   "));
  //  Serial.print(F("gz "));    Serial.print(gyro[2]);  Serial.print(F("\n"));
  //  Serial.println();
}

void call_IMU() {
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }

}
