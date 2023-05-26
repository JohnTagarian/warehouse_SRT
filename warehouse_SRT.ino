#include "IMU.h"
#include "speed_control.h"
#include <Servo.h>


#define GRIPPER_PIN 9
#define JOINT_PIN 8
#define STBTN A15


int track_ir[] = {A0, A8, A7, A3};
int track_val[] = {0, 0, 1023, 0, 0};
int track_filter_val[] = {0, 0, 0, 0, 0};
int filterFactor = 10;


int ir_left = A9;
int ir_right = A8;

const float track_kp = 0.20;
const float track_kd = 0.005;
const float track_ki = 0.00;

float track_error = 0;
float track_last_error = 0;
float track_errorSum = 0;

const int numSensors = 4;

int base_speed = 100;
int max_speed = 200;
int last_track_state = 0;

int thredhold = 79;
int speed_track = 60;
int fixed_speed = 120;
const int wing_thredhold = 550;

unsigned long pretime;

char state = '1';

int cnt = 0;
bool cnt_state = false;

unsigned long pretime_cnt;

unsigned long prev_time;
int sensorThreshold = 720;


Servo gripper;
Servo joint;


void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  pinMode(ir_left, INPUT_PULLUP);
  pinMode(ir_right, INPUT_PULLUP);
  pinMode(STBTN, INPUT_PULLUP);
  gripper.attach(GRIPPER_PIN);
  joint.attach(JOINT_PIN);
  gripper.write(60);
  joint.write(0);


  for (int i = 0 ; i < 4 ; i++) {

    pinMode(ENA[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(track_ir[i], INPUT_PULLUP);

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

  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);          // Set the DMP output rate from 200Hz to 5 Minutes.
  //  mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //  mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(Print_Values);               // Set callback function that is triggered when FIFO Data is retrieved

  //  while (millis() < 30000) {
  //    call_IMU();
  //    //    Serial.println(yaw_value);
  //  }

  while (1) {
    //    Serial.println(digitalRead(STBTN));
    //    wait_speed_control();
    call_IMU();
    if (!digitalRead(STBTN)) {
      delay(10);
      if (!digitalRead(STBTN)) break;
    }

  }
  //    delay(2/000);
  init_time = millis();
  Serial.println("PICK");
  //  prev_time = millis();
  //  pick_obj(prev_time);






}

double distance;
bool s = false;
int cnt_imu = 0;
int fil_wing_val;
bool check_bridge;
int distance_setpoint;

void loop() {

  //  prev_time = millis();
  //  while (millis() - prev_time < 1000) {
  //    fixed_axes(60);
  //  }
  //
  //  prev_time = millis();
  //  pick_obj(prev_time);
  //  while (1);
  //  Serial.println("PICK");
  //  pick_obj(prev_time);
  //
  //  prev_time = millis();
  //  while (millis() - prev_time < 3000) {
  //      compute_pid_motor(0, speed_track , 1, true);
  //      compute_pid_motor(1, speed_track, 1, true);
  //      compute_pid_motor(2, speed_track, 1, true);
  //      compute_pid_motor(3, speed_track , 1, true);
  //  }


  //  IMU_setpoint = 0;
  //  fixed_axes(60);
  //  Serial.println(get_distance());

  //  cross_bridge();
  //  Serial.print(analogRead(ir_left));
  //  Serial.print(" , ");
  //  Serial.println(analogRead(ir_right));

  //  pid_track();
  //  Serial.println(get_distance_avg());

  //  distance = (double)((distance_cnt[1] / 510) * (arc));
  //  Serial.println(distance);



  //  Serial.println(IMU_raw_pitch);
  //  Serial.println();


  //  Serial.println();
  //  compute_line_track();

  //  pid_track();
  //  fixed_axes();


  //  for (int i = 0 ; i < 4 ; i++ ) {
  //    //    if (i == 2)track_val[i] = analogRead(track_ir[i]) / 10 + 8;
  //    //    else track_val[i] = analogRead(track_ir[i]) / 10;
  //    track_val[i] = analogRead(track_ir[i]);
  //  }
  //  Serial.print(String(track_val[0]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(track_val[3]));

  //  Serial.println(String(analogRead(track_ir[1])) + String(" ") + String(analogRead(track_ir[2])));




  switch (state) {
    case '1':
      Serial.println ("state 1");
      IMU_setpoint = 0;
      cross_bridge();
      if (check_bridge)state = '2';
      distance_cnt[2] = 0;
      distance_cnt[3] = 0;
      break;
    case '2':
      Serial.println("state 2 :" + String(get_distance()));
      distance_setpoint = 1600;
      fixed_axes(60);
      if (abs(distance_setpoint - get_distance()) < 20) {
        state = '3';
      }
      break;

    case '3':
      Serial.println("state 3");

      plan_robot(90);
      run_robot(1550, 90);

      state = '4';
      break;

    case '4':
      Serial.println("state 4");
      plan_robot(0);
      run_robot(1850, 0);


      state = '5';
      break;


    case '5':
      Serial.println("state 5----------------------");
      plan_robot(90);
      int val1, val2;
      prev_time = millis();
      while (millis() - prev_time < 1000) {
        wait_speed_control();
        val1 = analogRead(track_ir[1]);
        val2 = analogRead(track_ir[2]);
        //        val1 = 500;
        //        val2 = 800;

      }
      check_center(val1, val2);
      joint.write(180);

      run_robot(1000, 90);
      prev_time = millis();
      pick_obj(prev_time);
      state = '6';
      break;

    case '6':
      Serial.println("state 6");
      run_slide_robot(600, 90, 1);

      prev_time = millis();
      while (millis() - prev_time < 1000) {
        wait_speed_control();
        val1 = analogRead(track_ir[1]);
        val2 = analogRead(track_ir[2]);
        //        val1 = 500;
        //        val2 = 800;

      }

      check_center(val1, val2);
      joint.write(180);
      prev_time = millis();
      while (millis() - prev_time < 200) {
        wait_speed_control();
      }
      run_robot(800, 90);
      prev_time = millis();
      pick_obj(prev_time);

      //      wait_speed_control();

      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //
      //      while (millis() - prev_time < 1000) {
      //        wait_speed_control();
      //        val1 = analogRead(track_ir[1]);
      //        val2 = analogRead(track_ir[2]);
      //
      //      }
      //      check_center(val1, val2);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }


      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }

      //      run_slide_robot(600, 90, 1);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      //
      //      run_slide_robot(600, 90, 1);
      //      prev_time = millis();
      //      while (millis() - prev_time < 2000) {
      //        wait_speed_control();
      //      }
      state = '7';
      break;

    case '7':
      Serial.println("state 7");
      wait_speed_control();

      break;

    case '8':
      Serial.println("state 8");
      distance_setpoint = 700;
      compute_pid_motor(0, speed_track , 1, true);
      compute_pid_motor(1, speed_track , -1, true);
      compute_pid_motor(2, speed_track, -1, true);
      compute_pid_motor(3, speed_track , 1, true);
      if (abs(distance_setpoint - get_distance()) < 20) {
        state = '9';
      }
      break;

    case '9':
      plan_robot(94);
      break;

  }


}


// pid line track


void track_slide() {
  int raw_value1 = analogRead(track_ir[0]) ;
  int raw_value2 = analogRead(track_ir[1]) + 100;
  int raw_value3 = analogRead(track_ir[2]) + 100;
  int raw_value4 = analogRead(track_ir[3]) - 210;

  track_val[0] = (track_val[0] * (filterFactor - 1) + raw_value1) / filterFactor;
  track_val[1] = (track_val[1] * (filterFactor - 1) + raw_value2) / filterFactor;
  track_val[3] = (track_val[3] * (filterFactor - 1) + raw_value3) / filterFactor;
  track_val[4] = (track_val[4] * (filterFactor - 1) + raw_value4) / filterFactor;

  Serial.print(String(track_val[0]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(raw_value4) + " : ");
  if ((raw_value1 < sensorThreshold) && (raw_value2 < sensorThreshold) && (raw_value3 > sensorThreshold) && (raw_value4 > sensorThreshold)) {
    Serial.println("if1");
    fixed_slide(60, 1);
  }
  if ((raw_value1 > sensorThreshold) && (raw_value2 > sensorThreshold) && (raw_value3 < sensorThreshold) && (raw_value4 < sensorThreshold)) {
    Serial.println("if2");
    fixed_slide(60, -1);
  }
  else {
    Serial.println("else");

    fixed_axes(60);
  }
}

void toggle_slide() {
  static bool toggle;
  if (millis() - prev_time > 4000) {

    prev_time = millis();
    toggle = !toggle;
  }
  if (toggle) {
    fixed_slide(60, 1);
  }
  else {
    fixed_slide(60, -1);
  }
}

void wait_speed_control() {
  Serial.println("dic");
  for (int i = 0; i < 4 ; i++) {
    compute_pid_motor(i , 0, 0, false);
  }
}


void fixed_axes(int fixed_speed) {
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }

  fixed_error = fixed_setpoint - IMU_raw_val;
  fixed_integral += fixed_error;
  fixed_derivative = fixed_error - fixed_last_error;


  // Calculate motor speeds using PID control
  int output = fixed_kp * fixed_error + fixed_ki * fixed_integral * fixed_dt + fixed_kd * fixed_derivative / fixed_dt;


  //  Serial.print("Heading: ");
  //  Serial.print(IMU_raw_val);
  //  Serial.print(", Error: ");
  //  Serial.print(fixed_error);
  //  Serial.print(", output: ");
  //  Serial.println(output);

  motor_speed = abs(output);
  motor_speed = constrain(motor_speed, 0, fixed_speed);

  if (output < 0) {
    //    Serial.println("Motor speed 1 : " + String(motor_speed));
    compute_pid_motor(0, fixed_speed - motor_speed, 1, true);
    compute_pid_motor(1, fixed_speed, 1, true);
    compute_pid_motor(2, fixed_speed - motor_speed, 1, true);
    compute_pid_motor(3, fixed_speed, 1, true);
  } else {
    //    Serial.println("Motor speed 2 : " + String(motor_speed));
    compute_pid_motor(0, fixed_speed, 1, true);
    compute_pid_motor(1, fixed_speed - motor_speed, 1, true);
    compute_pid_motor(2, fixed_speed, 1, true);
    compute_pid_motor(3, fixed_speed - motor_speed, 1, true);
  }

  fixed_current_time = millis();
  fixed_dt = (fixed_current_time - fixed_last_time) / 1000.0;
  fixed_last_error = fixed_error;
  // Store the current time as the last time
  fixed_last_time = fixed_current_time;
}

//fixed slide
void fixed_slide(int fixed_speed , int dir) {
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }

  fixed_error = fixed_setpoint - IMU_raw_val;
  fixed_integral += fixed_error;
  fixed_derivative = fixed_error - fixed_last_error;


  // Calculate motor speeds using PID control
  int output = fixed_kp * fixed_error + fixed_ki * fixed_integral * fixed_dt + fixed_kd * fixed_derivative / fixed_dt;


  Serial.print("Heading: ");
  Serial.print(IMU_raw_val);
  Serial.print(", Error: ");
  Serial.print(fixed_error);
  Serial.print(", output: ");
  Serial.println(output);

  motor_speed = abs(output);
  motor_speed = constrain(motor_speed, 0, fixed_speed);

  if (dir == -1) {
    if (output < 0) {
      compute_pid_motor(0, speed_track , -1, true);
      compute_pid_motor(1, speed_track , 1, true);
      compute_pid_motor(2, speed_track - motor_speed, 1, true);
      compute_pid_motor(3, speed_track , -1, true);
    } else {
      compute_pid_motor(0, speed_track - motor_speed, -1, true);
      compute_pid_motor(1, speed_track , 1, true);
      compute_pid_motor(2, speed_track , 1, true);
      compute_pid_motor(3, speed_track , -1, true);
    }
  }

  else if (dir == 1) {
    if (output < 0) {
      compute_pid_motor(0, speed_track , 1, true);
      compute_pid_motor(1, speed_track - motor_speed, -1, true);
      compute_pid_motor(2, speed_track , -1, true);
      compute_pid_motor(3, speed_track , 1, true);
    } else {
      compute_pid_motor(0, speed_track , 1, true);
      compute_pid_motor(1, speed_track , -1, true);
      compute_pid_motor(2, speed_track , -1, true);
      compute_pid_motor(3, speed_track - motor_speed, 1, true);
    }
  }


  fixed_current_time = millis();
  fixed_dt = (fixed_current_time - fixed_last_time) / 1000.0;
  fixed_last_error = fixed_error;
  // Store the current time as the last time
  fixed_last_time = fixed_current_time;
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

    compute_pid_motor(0, heading_speed, -1, true);
    compute_pid_motor(1, heading_speed, 1, true);
    compute_pid_motor(2, heading_speed, -1, true);
    compute_pid_motor(3, heading_speed, 1, true);
  } else {

    compute_pid_motor(0, heading_speed, 1, true);
    compute_pid_motor(1, heading_speed, -1, true);
    compute_pid_motor(2, heading_speed, 1, true);
    compute_pid_motor(3, heading_speed, -1, true);
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



void pid_track() {

  int raw_value1 = analogRead(track_ir[0]) ;
  int raw_value2 = analogRead(track_ir[1]) + 100;
  int raw_value3 = analogRead(track_ir[2]) + 100;
  int raw_value4 = analogRead(track_ir[3]) - 210;

  track_val[0] = (track_val[0] * (filterFactor - 1) + raw_value1) / filterFactor;
  track_val[1] = (track_val[1] * (filterFactor - 1) + raw_value2) / filterFactor;
  track_val[3] = (track_val[3] * (filterFactor - 1) + raw_value3) / filterFactor;
  track_val[4] = (track_val[4] * (filterFactor - 1) + raw_value4) / filterFactor;

  //  for (int i = 0; i < numSensors + 1; i++) {
  //    int raw_value;
  //    if (i < 2) {
  //      if (i == 0 )raw_value = analogRead(track_ir[i]) - 200;
  //      else raw_value = analogRead(track_ir[i]);
  //      track_val[i] = (track_val[i] * (filterFactor - 1) + raw_value) / filterFactor;
  //    }
  //    else if (i > 2) {
  //      if (i == 3) raw_value = analogRead(track_ir[i - 1]) - 100;
  //      if (i == 4) raw_value = analogRead(track_ir[i - 1]) - 100;
  //      else raw_value = analogRead(track_ir[i]);
  //      track_val[i] = (track_val[i] * (filterFactor - 1) + raw_value) / filterFactor;
  //    }
  //
  //  }

  // Calculate the error
  track_error = 0;
  for (int i = 0; i < numSensors + 1; i++) {

    if (i == 3) {
      if (track_val[i] < sensorThreshold - 150 ) {
        track_error += (i - (numSensors / 2)) * (track_val[i] - sensorThreshold) + 10;

        //      Serial.print(i);
        //      Serial.print(",");
      }
    }

    else {
      if (track_val[i] < sensorThreshold ) {
        track_error += (i - (numSensors / 2)) * (track_val[i] - sensorThreshold);

        //      Serial.print(i);
        //      Serial.print(",");
      }
    }

  }
  //  Serial.println();

  // Calculate the derivative and integral terms
  float track_derivative = track_error - track_last_error;
  track_errorSum += track_error;

  // Calculate the PID output
  float output = (track_kp * track_error) + (track_kd * track_derivative) + (track_ki * track_errorSum);

  // Limit the output to the range -255 to 255
  output = constrain(output, -speed_track, speed_track);

  // Set the motor speeds based on the PID output

  // Save the last error
  track_last_error = track_error;
  if (output > 0) {
    compute_pid_motor(0, speed_track - output, 1, true);
    compute_pid_motor(2, speed_track - output, 1, true);

    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(3, speed_track, 1, true);
  }
  else if (output < 0) {
    compute_pid_motor(0, speed_track , 1, true);
    compute_pid_motor(2, speed_track , 1, true);

    compute_pid_motor(1, speed_track + output, 1, true);
    compute_pid_motor(3, speed_track + output, 1, true);
  }
  else {
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(2, speed_track, 1, true);

    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(3, speed_track, 1, true);

  }
  // Print the sensor values and PID output for debugging
  //  Serial.println(track_val[3]);
  //  Serial.print(",");
  //  Serial.print(track_val[1]);
  for (int i = 0; i < 5; i++) {
    if (i != 2) {
      //      Serial.print(",");
      //      Serial.print(track_val[i]);

    }
  }
  //  Serial.print(": ");
  //  Serial.print(track_error);
  //  Serial.print(",");
  //  Serial.println(output);
}

void cross_bridge() {
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (99)) { // 99ms instead of 100ms to start polling the MPU 1ms prior to data arriving.
    if ( mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }
  if (cnt_imu >= 2 && !s) {
    check_bridge = true;
    Serial.println("If1");
    wait_speed_control();

  }
  else if (cnt_imu < 2 && !check_bridge) {
    Serial.println("If2");
    fixed_axes(fixed_speed);
  }
  else {
    Serial.println("Else");
    fixed_axes(fixed_speed);
  }
  if (!s && abs(IMU_raw_pitch) > 7) {
    //    Serial.println("=========================================================================================================");
    s = true;
    cnt_imu ++;
  }
  else if (abs(IMU_raw_pitch) < 7) {
    //    Serial.println(")))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))");
    s = false;
  }
  Serial.println(cnt_imu);

}


int get_distance() {
  return (distance_cnt[2] + distance_cnt[3]) / 2;
}


void plan_robot(int setpoint) {
  IMU_setpoint = setpoint;
  while (1) {
    Serial.println("planState : ");
    Serial.print(state);
    compute_pid_heading();
    if (abs(IMU_error) < 1.0)
      break;
  }
}

void run_robot(int dis , int angle) {
  distance_setpoint = dis;
  fixed_setpoint = angle;
  distance_cnt[2] = 0;
  distance_cnt[3] = 0;
  while (1) {
    Serial.println("runState : " + String(get_distance()));
    Serial.print(String(state));
    //    Serial.println("run robot: " + String(get_distance()));
    fixed_axes(60);
    if (abs(distance_setpoint - get_distance()) < 20) {
      break;
    }
  }
}

void run_slide_robot(int dis , int angle, int dir) {
  distance_setpoint = dis;
  fixed_setpoint = angle;
  distance_cnt[2] = 0;
  distance_cnt[3] = 0;
  while (1) {
    Serial.println("slide run robot: " + String(get_distance()));
    fixed_slide(60, dir);
    if (abs(distance_setpoint - get_distance()) < 20) {
      break;
    }
  }
}

void pick_obj(unsigned long prev) {
  Serial.println("a");

  while (1) {
    wait_speed_control();
    //    Serial.println("a");
    while ((millis() - prev > 1800) && (millis() - prev < 2500)) {
      Serial.println("Back");
      compute_pid_motor(0, speed_track , -1, true);
      compute_pid_motor(1, speed_track, -1, true);
      compute_pid_motor(2, speed_track, -1, true);
      compute_pid_motor(3, speed_track , -1, true);
    }
    if (millis() - prev < 1200) {
      Serial.println("if1");
      joint.write(180);
    }
    else if (millis() - prev < 1800) {
      Serial.println("if2");
      gripper.write(100);
    }
    else if (millis() - prev < 4000) {
      Serial.println("if3");
      joint.write(0);
    }
    else {
      Serial.println("else");
      gripper.write(15);
      break;
    }

  }
}

void check_center(int val1 , int val2) {
  if ((val1 < 600) && (val2 > 600)) {
    prev_time = millis();
    while (millis() - prev_time < 200) {
      Serial.println("----------------------------------------------------------------------------------------------");
      fixed_slide(60, -1);
    }
  }
  else if ((val1 > 600) && (val2 < 600)) {
    prev_time = millis();
    while (millis() - prev_time < 200) {
      Serial.println("----------------------------------------------------------------------------------------------");
      fixed_slide(60, 1);
    }
  }
}
