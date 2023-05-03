//#include <MCP3008.h>
//#include <SPI.h>
//#include "IMU.h"
#include "speed_control.h"

//#define CS_PIN 12
//#define CLOCK_PIN 9
//#define MOSI_PIN 11
//#define MISO_PIN 10

//MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);


int track_ir[] = {11, 10, 9, 8};
int track_val[] = {0, 0, 0, 0};

const float trck_kp = 1.50;
const float trck_kd = 0.05;
const float trck_ki = 0.00;

float trck_error = 0;
float last_trck_err = 0;
float track_errorSum = 0;

const int numSensors = 8;

int base_speed = 100;
int max_speed = 200;
int last_track_state = 0;

int thredhold = 60;
int speed_track = 45;
void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately


  for (int i = 0 ; i < 4 ; i++) {

    pinMode(ENA[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(track_ir, INPUT_PULLUP);

    // Initialize the encoder pins as input
    pinMode(encoderPinA[i], INPUT_PULLUP); // Line 25
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), updateEncoder_Q, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), updateEncoder_E, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), updateEncoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[3]), updateEncoder_D, RISING);

  Serial.println(F("Start:"));



  // Setup the MPU and TwoWire aka Wire library all at once

  //  mpu.begin();
  //  mpu.Set_DMP_Output_Rate_Hz(100);          // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  //  mpu.CalibrateMPU();                      // Calibrates the MPU.
  //  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  //  mpu.on_FIFO(Print_Values);               // Set callback function that is triggered when FIFO Data is retrieved
  // Setup is complete!

}
unsigned long pretime;
char state = '1';
void loop() {

  //  if (millis() - pretime > 3500) {
  //    pretime = millis();
  //    state = !state;
  //
  //  }
  //  if (state) {
  //    compute_pid_motor(0, 40, 1, true);
  //    compute_pid_motor(1, 40, -1, true);
  //    compute_pid_motor(2, 40, -1, true);
  //    compute_pid_motor(3, 40, 1, true);
  //  }
  //  else {
  //    wait_speed_control();
  //  }

  //    compute_pid_motor(0, 40, 1, true);
  //    compute_pid_motor(1, 10, -1, true);
  //    compute_pid_motor(2, 40, 1, true);
  //    compute_pid_motor(3, 10, -1, true);


  //  compute_pid_heading();


  for (int i = 0 ; i < 4 ; i++ ) {
    if (i == 2)track_val[i] = analogRead(track_ir[i]) / 10 + 8;
    else track_val[i] = analogRead(track_ir[i]) / 10;
  }

  Serial.print(String(track_val[1]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(track_val[3]));
  
//  compute_pid_motor(0, 45, 1, true);
//  compute_pid_motor(1, 45, -1, true);
//  compute_pid_motor(2, 45, 1, true);
//  compute_pid_motor(3, 45, -1, true);
  //
  //  switch (state) {
  //
  //    case '1':
  //      while (!((track_val[0] < thredhold && track_val[1]  < thredhold && track_val[2] < thredhold && track_val[3] - 5 < thredhold))) {
  //        Serial.println("state 1");
  //        compute_line_track();
  //      }
  //      pretime = millis();
  //      while (millis() - pretime < 1200) {
  //        compute_pid_motor(0, 25, 1, true);
  //        compute_pid_motor(1, 25, 1, true);
  //        compute_pid_motor(2, 25, 1, true);
  //        compute_pid_motor(3, 25, 1, true);
  //      }
  //
  //      pretime = millis();
  //      while (millis() - pretime < 3000) {
  //        compute_pid_motor(0, 45, 1, true);
  //        compute_pid_motor(1, 45, -1, true);
  //        compute_pid_motor(2, 45, 1, true);
  //        compute_pid_motor(3, 45, -1, true);
  //      }
  //      state = '2';
  //      break;
  //    case '2':
  //      Serial.println("state 2");
  //      wait_speed_control();
  //
  //      break;
  //
  //
  //  }
}


// pid line track
void compute_line_track() {
  for (int i = 0 ; i < 4 ; i++ ) {
    if (i == 2)track_val[i] = analogRead(track_ir[i]) / 10 + 8;
    else track_val[i] = analogRead(track_ir[i]) / 10;
  }

  Serial.print(String(track_val[1]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(track_val[3]));
  if (track_val[0] < thredhold && track_val[1]  < thredhold && track_val[2] < thredhold && track_val[3] - 5 < thredhold) {
    Serial.println("Find state");
    //    compute_pid_motor(0, 0, 0, false);
    //    compute_pid_motor(1, 0, 0, false);
    //    compute_pid_motor(2, 0, 0, false);
    //    compute_pid_motor(3, 0, 0, false);
  }
  else if (!(track_val[0] < thredhold) && !(track_val[1]  < thredhold) && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("clean");
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, speed_track, 1, true);

  }
  else if (track_val[0] < thredhold && !(track_val[1]  < thredhold) && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("overright error");
    last_track_state = 1;
    compute_pid_motor(0, speed_track - 25, -1, true);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, speed_track - 25, -1, true);
    compute_pid_motor(3, speed_track, 1, true);
  }
  else if (track_val[0] < thredhold && track_val[1]  < thredhold && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("farly right error");
    last_track_state = 2;
    compute_pid_motor(0, speed_track - 25, -1, true);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, speed_track - 25, -1, true);
    compute_pid_motor(3, speed_track, 1, true);
  }
  else if (!(track_val[0] < thredhold) && track_val[1]  < thredhold && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("right error");
    last_track_state = 3;
    compute_pid_motor(0, 0, 0, false);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, 0, 0, false);
    compute_pid_motor(3, speed_track, 1, true);
  }

  else if (!(track_val[0] < thredhold) && !(track_val[1] < thredhold ) && !(track_val[2] < thredhold ) && track_val[3] < thredhold) {
    Serial.println("overleft erorr");
    last_track_state = 4;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, speed_track - 25, -1, true);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, speed_track - 25, -1, true);
  }
  else if (!(track_val[0] < thredhold ) && !(track_val[1]  < thredhold ) && track_val[2] < thredhold && track_val[3] < thredhold) {
    Serial.println("farly left error");
    last_track_state = 5;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, speed_track - 25, -1, true);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, speed_track - 25, -1, true);
  }
  else if (!(track_val[0] < thredhold) && !(track_val[1]  < thredhold ) && track_val[2] < thredhold && !(track_val[3] < thredhold)) {
    Serial.println("left error");
    last_track_state = 6;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, 0, 0, false);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, 0, 0, false);
  }
  else {
    Serial.println("Else");
    if (last_track_state == 1) {
      compute_pid_motor(0, speed_track - 25, -1, true);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, speed_track - 25, -1, true);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 2) {
      compute_pid_motor(0, speed_track - 25, -1, true);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, speed_track - 25, -1, true);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 3) {
      compute_pid_motor(0, 0, 0, false);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, 0, 0, false);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 4) {
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, speed_track - 25, -1, true);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, speed_track - 25, -1, true);
    }
    else if (last_track_state == 5) {
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, speed_track - 25, -1, true);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, speed_track - 25, -1, true);

    }
    else if (last_track_state == 6) {
      last_track_state = 6;
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, 0, 0, false);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, 0, 0, false);
    }
    //    compute_pid_motor(0, 30, 1);
    //    compute_pid_motor(1, 30, 1);
    //    compute_pid_motor(2, 30, 1);
    //    compute_pid_motor(3, 30, 1);
  }
  Serial.println();

}


void wait_speed_control() {
  for (int i = 0; i < 4 ; i++) {
    compute_pid_motor(i , 0, 1, false);
  }
}
// pid heading control
//void compute_pid_heading() {
//  static unsigned long FIFO_DelayTimer;
//  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
//    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
//  }
//
//  IMU_error = IMU_setpoint - yaw_value;
//
//  if (IMU_error < - 180) {
//    IMU_error += 360;
//  }
//  if (IMU_error > 180) {
//    IMU_error -= 360;
//  }
//
//  // Calculate the error sum and difference
//  IMU_error_sum += IMU_error;
//  IMU_error_diff = IMU_error - IMU_last_error;
//
//  // Calculate the PID output
//  IMU_output = IMU_Kp * IMU_error + IMU_Ki * IMU_error_sum * IMU_dt + IMU_Kd * IMU_error_diff / IMU_dt;
//
//  IMU_output = constrain(IMU_output, -150, 150);
//  heading_speed = abs(IMU_output);
//  heading_speed = constrain(heading_speed, 0, 60);
//
//
//
//  if (IMU_output < 0) {
//
//    compute_pid_motor(0, heading_speed, -1);
//    compute_pid_motor(1, heading_speed, 1);
//    compute_pid_motor(2, heading_speed, -1);
//    compute_pid_motor(3, heading_speed, 1);
//  } else {
//
//    compute_pid_motor(0, heading_speed, 1);
//    compute_pid_motor(1, heading_speed, -1);
//    compute_pid_motor(2, heading_speed, 1);
//    compute_pid_motor(3, heading_speed, -1);
//  }
//
//
//  Serial.print("ERR : ");
//  Serial.print(IMU_error);
//  Serial.print("\tIMU_output :");
//  Serial.print(IMU_output);
//  Serial.print("\tYaw : ");
//  Serial.println(yaw_value);
//
//  IMU_last_error = IMU_error;
//
//  // Calculate the current time and dt
//  IMU_current_time = millis();
//  IMU_dt = (IMU_current_time - IMU_last_time) / 1000.0;
//
//  // Store the current time as the last time
//  IMU_last_time = IMU_current_time;
//}
