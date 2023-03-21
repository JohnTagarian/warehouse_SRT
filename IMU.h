#define interruptPin 19
#define OFFSETS   -674,      46,     956,      18,      72,      -9

#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board

Simple_MPU6050 mpu;


float yaw_value;


double IMU_Kp = 0.85;
double IMU_Ki = 0.0;
double IMU_Kd = 0.0;



double IMU_setpoint = 30;
double IMU_error, IMU_last_error, IMU_error_sum, IMU_error_diff;

double IMU_output;

unsigned long IMU_current_time, IMU_last_time;
double IMU_dt;

int heading_speed;


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
  //  Serial.print("Yaw : ");
  //  Serial.print(yaw_value);


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
