#include "IMU.h"
#include "speed_control.h"


int track_ir[] = {A3, A2, A1, A0};
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


int sensorThreshold = 670;
void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  pinMode(ir_left, INPUT_PULLUP);
  pinMode(ir_right, INPUT_PULLUP);
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
  // Setup is complete!



  //  noInterrupts();           // disable all interrupts
  //  TCCR1A = 0;
  //  TCCR1B = 0;
  //  timer1_counter = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  //
  //
  //  TCNT1 = timer1_counter;   // preload timer
  //  TCCR1B |= (1 << CS12);    // 256 prescaler
  //  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  //  interrupts();             // enable all interrupts
  //--------------------------timer setup
  //    while (millis() < 4000);
  //  delay(2000);
  init_time = millis();
}
double distance;
bool s = false;
int cnt_imu = 0;
int fil_wing_val;
bool check_bridge;

//unsigned long pretime_wing;

void loop() {
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
  //  if (state == '1') {
  //
  //    pid_track();
  //
  //  }
  //  else if (dmeters >= (double)1.3) {
  //    state = '2';
  //    wait_speed_control();
  //  }
  //  if (state == '2') {
  //    wait_speed_control();
  //  }
  //  if (millis() - pretime_wing > 50 && millis() - init_time > 3000) {
  //    pretime_wing = millis();
  //
  //    int raw_wing = analogRead(ir_right);
  //    fil_wing_val = (fil_wing_val * (filterFactor - 1) + raw_wing) / filterFactor;
  //
  //    if (fil_wing_val > wing_thredhold && state == '1') {
  //        pid_track();
  //
  //      Serial.println("IF : " + String(fil_wing_val));
  //    }
  //    else {
  //      Serial.println("Else : " + String(fil_wing_val));
  //      //      state = '2';
  //      wait_speed_control();
  //    }
  //  }


  //  Serial.println(IMU_raw_pitch);
  //  Serial.println();
  //  compute_pid_motor(0, speed_track, -1, true);
  //  compute_pid_motor(1, speed_track, 1, true);
  //  compute_pid_motor(2, speed_track, 1, true);
  //  compute_pid_motor(3, speed_track, -1, true);
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









  //  switch (state) {
  //
  //    case '1':
  //      Serial.print(String(track_val[1]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(track_val[3]));
  //      while (!((track_val[0] < thredhold && track_val[1]  < thredhold && track_val[2] < thredhold && track_val[3] - 7 < thredhold))) {
  //        Serial.println("state 1");
  //        compute_line_track();
  //      }
  //      pretime = millis();
  //      while (millis() - pretime < 600) {
  //        compute_pid_motor(0, 25, 1, true);
  //        compute_pid_motor(1, 25, 1, true);
  //        compute_pid_motor(2, 25, 1, true);
  //        compute_pid_motor(3, 25, 1, true);
  //      }
  //      pretime = millis();
  //      while (millis() - pretime < 200) {
  //        wait_speed_control();
  //      }
  //      IMU_setpoint = 270;
  //      while (1) {
  //        compute_pid_heading();
  //        if (abs(IMU_error) < 1.0)
  //          break;
  //
  //
  //      }
  //      state = '2';
  //      break;
  //
  //    case '2':
  //      //      Serial.println("state2");
  //      //      wait_speed_control();
  //      while (!((track_val[0] < thredhold && track_val[1]  < thredhold && track_val[2] < thredhold && track_val[3] - 7 < thredhold))) {
  //        Serial.println("state 2");
  //        compute_line_track();
  //      }
  //      pretime = millis();
  //      while (millis() - pretime < 500) {
  //        compute_pid_motor(0, 25, 1, true);
  //        compute_pid_motor(1, 25, 1, true);
  //        compute_pid_motor(2, 25, 1, true);
  //        compute_pid_motor(3, 25, 1, true);
  //      }
  //      IMU_setpoint = 180;
  //      while (1) {
  //        compute_pid_heading();
  //        if (abs(IMU_error) < 1.0)
  //          break;
  //
  //
  //      }
  //      state = '3';
  //      break;
  //
  //    case '3':
  //      while (cnt < 3) {
  //        //        Serial.print((analogRead(ir_right) / 10 ));
  //        //        Serial.print(",");
  //        //        Serial.print((analogRead(ir_left) / 10 ));
  //        //        Serial.print("\t");
  //
  //        Serial.println("cnt  3 = " + String(cnt));
  //        if (((analogRead(ir_right) / 10 < 46) ) && (millis() - pretime_cnt) > 800) {
  //          pretime_cnt = millis();
  //          cnt++;
  //        }
  //        compute_line_track();
  //      }
  //      Serial.println("state3");
  //      state = '4';
  //      break;
  //
  //    case '4':
  //      pretime = millis();
  //      if (millis() - pretime > 400) {
  //        compute_pid_motor(0, 25, 1, true);
  //        compute_pid_motor(1, 25, 1, true);
  //        compute_pid_motor(2, 25, 1, true);
  //        compute_pid_motor(3, 25, 1, true);
  //      }
  //      pretime = millis();
  //      if (millis() - pretime > 200) {
  //        wait_speed_control();
  //      }
  //      IMU_setpoint = 270;
  //      while (1) {
  //
  //        compute_pid_heading();
  //        if (abs(IMU_error) < 1.0)
  //          break;
  //
  //
  //
  //      }
  //      state = '5';
  //      break;
  //    case '5':
  //      Serial.println("state 5");
  //      wait_speed_control();
  //      break;
  //      //  compute_line_track();
  //  }
}


// pid line track
void compute_line_track() {
  for (int i = 0 ; i < 4 ; i++ ) {
    if (i == 2)track_val[i] = analogRead(track_ir[i]) / 10 + 8;
    else track_val[i] = analogRead(track_ir[i]) / 10;
  }

  Serial.print(String(track_val[1]) + "," + String(track_val[1]) + "," + String(track_val[2]) + "," + String(track_val[3]));
  if (track_val[0] < thredhold && track_val[1]  < thredhold && track_val[2] < thredhold && track_val[3] - 7 < thredhold) {
    Serial.println("Find state");
    compute_pid_motor(0, 0, 0, false);
    compute_pid_motor(1, 0, 0, false);
    compute_pid_motor(2, 0, 0, false);
    compute_pid_motor(3, 0, 0, false);
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
    compute_pid_motor(0, speed_track, -1, true);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, speed_track, -1, true);
    compute_pid_motor(3, speed_track, 1, true);
  }
  else if (track_val[0] < thredhold && track_val[1]  < thredhold && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("farly right error");
    last_track_state = 2;
    compute_pid_motor(0, 0, 0, false);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, 0, 0, false);
    compute_pid_motor(3, speed_track, 1, true);
  }
  else if (!(track_val[0] < thredhold) && track_val[1]  < thredhold && !(track_val[2] < thredhold) && !(track_val[3] < thredhold)) {
    Serial.println("right error");
    last_track_state = 3;
    compute_pid_motor(0, speed_track - 10, 1, true);
    compute_pid_motor(1, speed_track, 1, true);
    compute_pid_motor(2, speed_track - 10, 1, true);
    compute_pid_motor(3, speed_track, 1, true);
  }

  else if (!(track_val[0] < thredhold) && !(track_val[1] < thredhold ) && !(track_val[2] < thredhold ) && track_val[3] < thredhold) {
    Serial.println("overleft erorr");
    last_track_state = 4;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, speed_track, -1, true);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, speed_track, -1, true);
  }
  else if (!(track_val[0] < thredhold ) && !(track_val[1]  < thredhold ) && track_val[2] < thredhold && track_val[3] < thredhold) {
    Serial.println("farly left error");
    last_track_state = 5;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, 0, 0, false);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, 0, 0, false);
  }
  else if (!(track_val[0] < thredhold) && !(track_val[1]  < thredhold ) && track_val[2] < thredhold && !(track_val[3] < thredhold)) {
    Serial.println("left error");
    last_track_state = 6;
    compute_pid_motor(0, speed_track, 1, true);
    compute_pid_motor(1, speed_track - 10, 1, true);
    compute_pid_motor(2, speed_track, 1, true);
    compute_pid_motor(3, speed_track - 10, 0, true);
  }
  else {
    Serial.println("Else");
    if (last_track_state == 1) {
      compute_pid_motor(0, speed_track, -1, true);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, speed_track, -1, true);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 2) {
      compute_pid_motor(0, 0, 0, false);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, 0, 0, false);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 3) {
      compute_pid_motor(0, speed_track - 10, 1, true);
      compute_pid_motor(1, speed_track, 1, true);
      compute_pid_motor(2, speed_track - 10, 1, true);
      compute_pid_motor(3, speed_track, 1, true);
    }
    else if (last_track_state == 4) {
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, speed_track, -1, true);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, speed_track, -1, true);
    }
    else if (last_track_state == 5) {
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, 0, 0, false);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, 0, 0, false);

    }
    else if (last_track_state == 6) {
      last_track_state = 6;
      compute_pid_motor(0, speed_track, 1, true);
      compute_pid_motor(1, speed_track - 10, 1, true);
      compute_pid_motor(2, speed_track, 1, true);
      compute_pid_motor(3, speed_track - 10, 0, true);
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


  Serial.print("Heading: ");
  Serial.print(IMU_raw_val);
  Serial.print(", Error: ");
  Serial.print(fixed_error);
  Serial.print(", output: ");
  Serial.println(output);

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

  int raw_value1 = analogRead(track_ir[0]) - 200;
  int raw_value2 = analogRead(track_ir[1]) ;
  int raw_value3 = analogRead(track_ir[2]) ;
  int raw_value4 = analogRead(track_ir[3]) ;

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


double get_distance_avg() {
  int rdistance;
  for (int i = 0 ; i < 4 ; i++) {
    rdistance += distance_cnt[i];
  }
  rdistance = rdistance / 4;
  return rdistance;
}


int get_distance() {
  return (distance_cnt[2] + distance_cnt[3]) / 2;
}
