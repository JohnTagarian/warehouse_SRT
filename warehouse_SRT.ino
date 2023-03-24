#include <MCP3008.h>
#include <SPI.h>
#include "IMU.h"

#define CS_PIN 12
#define CLOCK_PIN 9
#define MOSI_PIN 11
#define MISO_PIN 10

MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

// Motor diver define
#define ENA 7
#define ENB 8
#define IN1 31
#define IN2 35
#define IN3 47
#define IN4 49

int irValue[8];
const float Kp = 1.50;
const float Kd = 0.05;
const float Ki = 0.00;

float error = 0;
float lastError = 0;
float errorSum = 0;

const int numSensors = 8;
int sensorThreshold = 60;

int base_speed = 100;
int max_speed = 200;


boolean motor_state;

void setup() {
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
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

  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
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

  //  compute_pid_line_track();
  //  compute_pid_heading();

  //  if (Serial.available()) {
  //    char inchar = Serial.read();
  //    if (inchar == 'r')motor_state = true;
  //    else if (inchar == 'o') motor_state = false;
  //  }
  //
  //  if (motor_state) {
  //  compute_pid_heading();
  //  }
  //  else {
  //    digitalWrite(IN1, 0);
  //    digitalWrite(IN2, 0);
  //    digitalWrite(IN3, 0);
  //    digitalWrite(IN4, 0);
  //  }
  compute_pid_heading();
}

// pid line track 
void compute_pid_line_track() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  // Calculate the error
  error = 0;
  for (int i = 0; i < numSensors; i++) {
    if (irValue[i] < sensorThreshold) {
      error += (i - (numSensors / 2)) * (sensorThreshold - irValue[i] );

    }
  }
  Serial.print(" Error :");
  Serial.print(error);

  float derivative = error - lastError;
  errorSum += error;

  float output = (Kp * error) + (Kd * derivative) + (Ki * errorSum);


  Serial.print("\tOutput : ");
  Serial.print(output);

  if (constrain(base_speed - output, 0, max_speed) == max_speed) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    analogWrite(ENA, base_speed - 30);
    analogWrite(ENB, base_speed - 30);
  }
  else if (constrain(base_speed + output, 0, max_speed) == max_speed) {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(ENA, base_speed + 30);
    analogWrite(ENB, base_speed + 30);
  }
  else {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(ENA, constrain(base_speed - output, 0, max_speed));
    analogWrite(ENB, constrain(base_speed + output + 10, 0, max_speed));
  }
  //  analogWrite(ENA, constrain(base_speed - output, 0, max_speed));
  //  analogWrite(ENB, constrain(base_speed + output, 0, max_speed));


  Serial.print("\tENA" + String(constrain(base_speed - output, 0, max_speed)));
  Serial.println("\tENB" + String(constrain(base_speed + output, 0, max_speed)));


  lastError = error;
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
  heading_speed = constrain(heading_speed, 50, 150);



  if (IMU_output < 0) {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  } else {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  }
  analogWrite(ENA, heading_speed);
  analogWrite(ENB, heading_speed + 10);


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



void forward_left(int speed) {
  //  Serial.println("Forward left");
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENA, speed);
}
void forward_right(int speed) {
  //  Serial.println("Forward left");
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENB, speed);
}
void backward_left(int speed) {
  Serial.println("backward left");
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENA, speed);
}
void backward_right(int speed) {
  Serial.println("backward left");
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENB, speed);
}
