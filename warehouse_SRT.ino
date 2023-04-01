#include <MCP3008.h>
#include <SPI.h>
#include "IMU.h"
#include "speed_control.h"

#define CS_PIN 12
#define CLOCK_PIN 9
#define MOSI_PIN 11
#define MISO_PIN 10

MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);



int irValue[8];

const float trck_kp = 1.50;
const float trck_kd = 0.05;
const float trck_ki = 0.00;

float trck_error = 0;
float last_trck_err = 0;
float track_errorSum = 0;

const int numSensors = 8;
int sensorThreshold = 60;

int base_speed = 100;
int max_speed = 200;


void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately


  for (int i = 0 ; i < 4 ; i++) {

    pinMode(ENA[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);

    // Initialize the encoder pins as input
    pinMode(encoderPinA[i], INPUT_PULLUP); // Line 25
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), updateEncoder_Q, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), updateEncoder_E, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), updateEncoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[3]), updateEncoder_D, RISING);


  Serial.println(F("Start:"));
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); // set SPI clock to 1MHz
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);



  // Setup the MPU and TwoWire aka Wire library all at once

  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);          // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(Print_Values);               // Set callback function that is triggered when FIFO Data is retrieved
  // Setup is complete!

}

void loop() {

  //  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  //  irValue[0] = adc.readADC(0) / 10;
  //  irValue[1] = adc.readADC(1) / 10;
  //  irValue[2] = adc.readADC(2) / 10 - 10;
  //  irValue[3] = adc.readADC(3) / 10 + 10;
  //  irValue[4] = adc.readADC(4) / 10;
  //  irValue[5] = adc.readADC(5) / 10;
  //  irValue[6] = adc.readADC(6) / 10;
  //  irValue[7] = adc.readADC(7) / 10;
  //  SPI.endTransaction();
  //  for (int i = 0; i < 8; i++) {
  //    Serial.print(irValue[i]);
  //    Serial.print("\t");
  //  }

  compute_pid_heading();
}


// pid line track
void compute_pid_line_track() {

  // Calculate the error
  trck_error = 0;
  for (int i = 0; i < numSensors; i++) {
    if (irValue[i] < sensorThreshold) {
      trck_error += (i - (numSensors / 2)) * (sensorThreshold - irValue[i] );

    }
  }
  Serial.print(" Error :");
  Serial.print(trck_error);

  float track_derivative = trck_error - last_trck_err;
  track_errorSum += trck_error;

  float track_output = (trck_kp * trck_error) + (trck_kd * track_derivative) + (trck_ki * track_errorSum);


  //  analogWrite(ENA, constrain(base_speed - track_output, 0, max_speed));
  //  analogWrite(ENB, constrain(base_speed + track_output, 0, max_speed));


  last_trck_err = trck_error;
}



// pid heading control
void compute_pid_heading() {
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }

  IMU_error = IMU_setpoint - yaw_value;

  if (IMU_error < - 180) {
    IMU_error += 360;
  }
  if (IMU_error > 180) {
    IMU_error -= 360;
  }

  // Calculate the error sum and difference
  IMU_error_sum += IMU_error;
  IMU_error_diff = IMU_error - IMU_last_error;

  // Calculate the PID output
  IMU_output = IMU_Kp * IMU_error + IMU_Ki * IMU_error_sum * IMU_dt + IMU_Kd * IMU_error_diff / IMU_dt;

  IMU_output = constrain(IMU_output, -150, 150);
  heading_speed = abs(IMU_output);
  heading_speed = constrain(heading_speed, 0, 60);



  if (IMU_output < 0) {

    compute_pid_motor(0, heading_speed, -1);
    compute_pid_motor(1, heading_speed, 1);
    compute_pid_motor(2, heading_speed, -1);
    compute_pid_motor(3, heading_speed, 1);
  } else {

    compute_pid_motor(0, heading_speed, 1);
    compute_pid_motor(1, heading_speed, -1);
    compute_pid_motor(2, heading_speed, 1);
    compute_pid_motor(3, heading_speed, -1);
  }


  Serial.print("ERR : ");
  Serial.print(IMU_error);
  Serial.print("\tIMU_output :");
  Serial.print(IMU_output);
  Serial.print("\tYaw : ");
  Serial.println(yaw_value);

  IMU_last_error = IMU_error;

  // Calculate the current time and dt
  IMU_current_time = millis();
  IMU_dt = (IMU_current_time - IMU_last_time) / 1000.0;

  // Store the current time as the last time
  IMU_last_time = IMU_current_time;
}
