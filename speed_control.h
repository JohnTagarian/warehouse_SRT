//const int ENA[] = {6, 5, 7, 4};
//const int IN1[] = {22, 28, 24, 26};
//const int IN2[] = {23, 29, 25, 27};

const int ENA[] = {4, 7, 5, 6};
const int IN1[] = {44, 46, 30, 48};
const int IN2[] = {45, 47, 31, 49};
const int encoderPinA[] = {19, 2, 18, 3};
const int encoderPinB[] = {23, 25, 27, 29};

// Define the encoder variables
volatile long encoderCount[] = {0, 0, 0, 0};
volatile long lastEncoderCount[] = {0, 0, 0, 0};
volatile long encoderSpeed[] = {0, 0, 0, 0};
unsigned long lastUpdateTime[] = {0, 0, 0, 0};

// Define the PID variables
const float Kp[] = {2.0, 2.0, 2.0, 2.0};
const float Ki[] = {0.025, 0.025, 0.025, 0.025};
const float Kd[] = {0.001, 0.001, 0.001, 0.001};
const float targetRPM = 35;

float inputRPM[] = {0, 0, 0, 0};
float outputPWM[] = {0, 0, 0, 0};
float prevError[] = {0, 0, 0, 0};
float integralError[] = {0, 0, 0, 0};

volatile int dir[] = {0, 0, 0, 0};


bool check_time_init = false;
unsigned long init_time;

double arc = 0.18857;

volatile long distance_cnt[4];


// compute speed control
void compute_pid_motor(int index, int target , int dir, bool enable) {
  if (!check_time_init) {
    init_time = millis();
    check_time_init = true;
  }
  unsigned long currentTime = millis() - init_time; // Line 36
  unsigned long deltaTime;
  if (!enable) deltaTime = 0;
  else  deltaTime = currentTime - lastUpdateTime[index]; // Line 37
  lastUpdateTime[index] = currentTime; // Line 38

  long countDiff = encoderCount[index] - lastEncoderCount[index]; // Line 40
  encoderSpeed[index] = countDiff * 60000 / (deltaTime * 510); // Line 41, RPM = (countDiff * 60 * 1000) / (deltaTime * PPR)
  lastEncoderCount[index] = encoderCount[index]; // Line 42


  inputRPM[index] = encoderSpeed[index]; // Line 44

  // Compute the PID output value (PWM value for the motor)
  float error = target - inputRPM[index]; // Line 47
  integralError[index] += error * deltaTime; // Line 48
  float derivativeError = (error - prevError[index]) / deltaTime; // Line 49
  outputPWM[index] = Kp[index] * error + Ki[index] * integralError[index] + Kd[index] * derivativeError; // Line 51

  // Update the previous error value
  prevError[index] = error; // Line 53
  outputPWM[index] = constrain(outputPWM[index], 0, 255); // Line 56


  if (dir == 1) {
    digitalWrite(IN1[index], HIGH);
    digitalWrite(IN2[index], LOW);
    analogWrite(ENA[index], outputPWM[index]);
  }
  else if (dir == -1)
  {
    digitalWrite(IN1[index], LOW);
    digitalWrite(IN2[index], HIGH);
    analogWrite(ENA[index], outputPWM[index]);
  }
  else if (dir == 0 ) {
    digitalWrite(IN1[index], LOW);
    digitalWrite(IN2[index], LOW);
    analogWrite(ENA[index], 0);
  }


  //  Serial.print(target);
  //  Serial.print(",");
  //  Serial.println(encoderSpeed[index]);
//    Serial.print(index);
//    Serial.print("\tSpeed : ");
//    Serial.print(encoderSpeed[index]);
//    Serial.print("\tCount :");
//    Serial.print(encoderCount[index]);
//    Serial.print("\tcountdiff : ");
//    Serial.print(countDiff);
//    Serial.print("\tDT : ");
//    Serial.print(deltaTime);
//    Serial.print("\tError : ");
//    Serial.println(error);

}
// ISR section
void updateEncoder_Q() {
  //  dir[0] = digitalRead(encoderPinB[0]);

  //  Serial.println("1--------------------------------");
  encoderCount[0]++;
  distance_cnt[0]++;

}

void updateEncoder_E()
{
  //  Serial.println("2--------------------------------");
  //  dir[1] = digitalRead(encoderPinB[1]);
  encoderCount[1]++;
  distance_cnt[1]++;

}
void updateEncoder_A()
{
  //  Serial.println("3--------------------------------");
  //  dir[2] = digitalRead(encoderPinB[2]);
  encoderCount[2]++;
  distance_cnt[2]++;
}
void updateEncoder_D()
{

  //  Serial.println("4--------------------------------");
  //  dir[3] = digitalRead(encoderPinB[3]);
  encoderCount[3]++;
  distance_cnt[3]++;
}
