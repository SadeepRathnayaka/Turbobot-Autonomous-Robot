#include <PID_v1.h>
#include <EnableInterrupt.h>
#include <math.h>

// Left Motors Pins (Front and Back)
#define LEFT_MOTOR_IN1 16  
#define LEFT_MOTOR_IN2 17 
#define LEFT_MOTOR_EN  3

#define LEFT_ENCODER_A 12
#define LEFT_ENCODER_B 11

#define LEFT_BACK_ENCODER_A 7
#define LEFT_BACK_ENCODER_B 6

// Right Motors Pins (Front and Back)
#define RIGHT_MOTOR_IN1 15   
#define RIGHT_MOTOR_IN2 14   
#define RIGHT_MOTOR_EN  10

#define RIGHT_ENCODER_A 5
#define RIGHT_ENCODER_B 4

#define RIGHT_BACK_ENCODER_A 9
#define RIGHT_BACK_ENCODER_B 8

// Pulses per Revolution
#define PPR_LEFT 2033
#define PPR_RIGHT 2060

#define INTERVAL 100
#define DELTA_LEFT 10.0
#define DELTA_RIGHT 10.0

volatile long leftPulses = 0, rightPulses = 0 ;
String leftDirection = "p", rightDirection = "p";

volatile long leftBackPulses = 0, rightBackPulses = 0 ;
String leftBackDirection = "p", rightBackDirection = "p";

bool is_left_motor   = true, is_right_motor   = true;
bool is_left_cmd     = true, is_right_cmd     = true ;
bool is_left_forward = true, is_right_forward = true;

char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

static unsigned long lastTime = 0;
float leftTheta  = (2.0 * PI) / PPR_LEFT;
float rightTheta = (2.0 * PI) / PPR_RIGHT;

int led_on = 1;

// PID Variables
double left_wheel_cmd_vel  = 0.0, right_wheel_cmd_vel  = 0.0 ;
double left_wheel_meas_vel = 0.0, right_wheel_meas_vel = 0.0;
double left_wheel_cmd      = 0.0, right_wheel_cmd      = 0.0;

double left_meas_vel_prev  = 0.0; 
double right_meas_vel_prev = 0.0 ;

double left_back_wheel_meas_vel = 0.0, right_back_wheel_meas_vel = 0.0;

// PID Parameters for Forward
double Kp_l_forward = 12.8, Ki_l_forward = 9.5, Kd_l_forward = 0.1;
double Kp_r_forward = 12.8, Ki_r_forward = 9.5, Kd_r_forward = 0.1;

double Kp_l_backward = 8.0, Ki_l_backward = 0.1, Kd_l_backward = 1.0;
double Kp_r_backward = 8.0,  Ki_r_backward = 0.1, Kd_r_backward = 1.0;
//
//double Kp_l_backward = 11.5, Ki_l_backward = 0.0, Kd_l_backward = 0.0;
//double Kp_r_backward = 11.5,  Ki_r_backward = 0.0, Kd_r_backward = 0.0;

PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l_forward, Ki_l_forward, Kd_l_forward, DIRECT);
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r_forward, Ki_r_forward, Kd_r_forward, DIRECT);


void setup() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);

  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);

  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);

  pinMode(LEFT_BACK_ENCODER_A, INPUT);
  pinMode(LEFT_BACK_ENCODER_B, INPUT);

  pinMode(RIGHT_BACK_ENCODER_A, INPUT);
  pinMode(RIGHT_BACK_ENCODER_B, INPUT);

  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  leftMotor.SetMode(AUTOMATIC);
  rightMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);

  enableInterrupt(LEFT_ENCODER_A, leftEncoderISR, CHANGE);
  enableInterrupt(RIGHT_ENCODER_A, rightEncoderISR, CHANGE);

  enableInterrupt(LEFT_BACK_ENCODER_A, leftBackEncoderISR, CHANGE);
  enableInterrupt(RIGHT_BACK_ENCODER_A, rightBackEncoderISR, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);

}

// lp08.51,rn08.51,
void loop() {
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'l')
    {
      is_left_motor = true;
      is_right_motor = false;
      is_cmd_complete = false;
      value_idx = 0;
    }
    else if (chr == 'r') 
    {
      is_left_motor = false;
      is_right_motor = true;
      is_cmd_complete = false;
      value_idx = 0;
    } 
    else if (chr == 'p') 
    {
      if (is_left_motor) 
      {
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
        is_left_forward = true;
        updatePIDParams(leftMotor, Kp_l_forward, Ki_l_forward, Kd_l_forward);
//        resetPIDStates(leftMotor);
      } 
      else if (is_right_motor) 
      {
        digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
        is_right_forward = true;
         updatePIDParams(rightMotor, Kp_r_forward, Ki_r_forward, Kd_r_forward);
//         resetPIDStates(rightMotor);
      } 
    } 
    else if (chr == 'n') {
      if (is_left_motor) 
      {
        digitalWrite(LEFT_MOTOR_IN1, LOW);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
        is_left_forward = false;
        updatePIDParams(leftMotor, Kp_l_backward, Ki_l_backward, Kd_l_backward);
//        resetPIDStates(leftMotor);
      } 
      else if (is_right_motor) 
      {
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
        is_right_forward = false;
        updatePIDParams(rightMotor, Kp_r_backward, Ki_r_backward, Kd_r_backward);
//        resetPIDStates(rightMotor);
      } 
    } 
    else if (chr == ',') {
      double parsedValue = atof(value);
      if (is_left_motor) left_wheel_cmd_vel = parsedValue;
      else if (is_right_motor) {
        right_wheel_cmd_vel = parsedValue;
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
      
    }
    else
    {
      if (value_idx < 5)
      {
      value[value_idx] = chr;
      value_idx++;
      }
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= INTERVAL) {
    left_wheel_meas_vel       = (leftPulses * leftTheta) / (currentTime - lastTime) * 1000;
    left_back_wheel_meas_vel  = (leftBackPulses * leftTheta) / (currentTime - lastTime) * 1000;
    right_wheel_meas_vel      = (rightPulses * rightTheta) / (currentTime - lastTime) * 1000;
    right_back_wheel_meas_vel = (rightBackPulses * rightTheta) / (currentTime - lastTime) * 1000;

    double delta_left  = fabs(left_wheel_meas_vel  - left_meas_vel_prev)  ;
    double delta_right = fabs(right_wheel_meas_vel - right_meas_vel_prev) ;

    if (delta_left > DELTA_LEFT)
    {
      left_wheel_meas_vel = left_meas_vel_prev ;
    }
    if (delta_right > DELTA_RIGHT)
    {
      right_wheel_meas_vel = right_meas_vel_prev ;
    }

    leftMotor.Compute();
    rightMotor.Compute();

    if (left_wheel_cmd_vel == 0 )
    {
      left_wheel_cmd = 0 ;
    } 
   
    if (right_wheel_cmd_vel == 0) 
    {
      right_wheel_cmd = 0 ;
    }

//   Serial.print("left front wheel measured vel : ");
//   Serial.println(left_wheel_meas_vel);
//   Serial.print("left back wheel measured vel : ");
//   Serial.println(left_back_wheel_meas_vel);
//   Serial.print("right front wheel measured vel : ");
//   Serial.println(right_wheel_meas_vel);
//   Serial.print("right back wheel measured vel : ");
//   Serial.println(right_back_wheel_meas_vel);

    analogWrite(LEFT_MOTOR_EN, left_wheel_cmd);
    analogWrite(RIGHT_MOTOR_EN, right_wheel_cmd);

    leftPulses = rightPulses = 0;
    leftBackPulses = rightBackPulses = 0 ;
    lastTime = currentTime;
    left_meas_vel_prev  = left_wheel_meas_vel ;
    right_meas_vel_prev = right_wheel_meas_vel ;

    digitalWrite(LED_BUILTIN, !led_on);
    led_on = !led_on;

    String velocity_msg = "l" + leftDirection  + componsateZeros(left_wheel_meas_vel)  + "," +
                          "r" + rightDirection + componsateZeros(right_wheel_meas_vel) + "," ;

    // String velocity_msg = "l" + leftDirection  + String(left_wheel_meas_vel)  + "," +
    //                       "r" + rightDirection + String(right_wheel_meas_vel) + "," ;
                          
    Serial.println(velocity_msg);
  }
}



void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
    leftPulses++;
    leftDirection = "p";
  } else {
    leftPulses--;
    leftDirection = "n";
  }
}


void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    rightPulses++;
    rightDirection = "p";
  } else {
    rightPulses--;
    rightDirection = "n";
  }
}

void leftBackEncoderISR() {
  if (digitalRead(LEFT_BACK_ENCODER_A) != digitalRead(LEFT_BACK_ENCODER_B)) {
    leftBackPulses++;
    leftBackDirection = "p";
  } else {
    leftBackPulses--;
    leftBackDirection = "n";
  }
}


void rightBackEncoderISR() {
  if (digitalRead(RIGHT_BACK_ENCODER_A) == digitalRead(RIGHT_BACK_ENCODER_B)) {
    rightBackPulses++;
    rightBackDirection = "p";
  } else {
    rightBackPulses--;
    rightBackDirection = "n";
  }
}

String componsateZeros(float vel) {
  vel = abs(vel);
  if (vel == 0.0) return "00.00";
  String velStr = String(vel, 2);
  while (velStr.length() < 5) velStr = "0" + velStr;
  return velStr;
}


void updatePIDParams(PID& pid, double Kp, double Ki, double Kd) {
    pid.SetTunings(Kp, Ki, Kd);
}


void resetPIDStates(PID& pid) {
    pid.SetMode(MANUAL);    // Disable the PID controller
    pid.SetMode(AUTOMATIC); // Re-enable it, effectively resetting its states
}