// Self balancing robot PID controller
// 4/3/21
// Uses MPU6050 accel/gyro
// Uses "MPU6050_light" library
// Based off 'get_angle' example 

#include "Wire.h"
#include <MPU6050_light.h>

// Motor A connections
int enA = 5;
int in1 = 29;
int in2 = 28;
// Motor B connections
int enB = 3;
int in3 = 31;
int in4 = 30;

// define variables
unsigned long current_time = 0;
unsigned long prev_time = 0;
int delta_time = 0;
float x;
double acc_y, vel_y, pos_y;
float error, prev_error, delta_error;
float P_e, I_e, D_e;
float PID = 0;
int PID_A, PID_B;
int set_point = 0;

//////////// TUNE //////////////
float k_P = 10;
float k_I = 0;
float k_D = 100;
////////////////////////////////

MPU6050 mpu(Wire);

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin(0, 0);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
//  Serial.println(F("Calculating offsets, do not move MPU6050"));
//  delay(1000);
//  mpu.calcOffsets(); // gyro and accelero
//  Serial.println("Done!\n");
}

void loop() {
  mpu.update();

  // get time step
  prev_time = current_time;
  current_time = millis();
  delta_time = current_time - prev_time;

  // get angle
  x = mpu.getAngleX();
//  acc_y = mpu.getAccY() * 9.81;               // convert from g to ms^-2


//  // calculate velocity, position
//  vel_y += acc_y * (delta_time / 1000);       // integrate 
//  pos_y += vel_y * (delta_time / 1000);
//  set_point = return_to_start();              // shift setpoint depending on position

  // calculate error
  prev_error = error;                         // previous error
  error = x - set_point;                      // error is angle from setpoint
  delta_error = error - prev_error;           // change in error

  // calculate PID parts from error
  P_e = k_P * error;
  I_e = I_e + k_I * error;
  D_e = k_D * (delta_error / delta_time);

  // sum PID paths with gains
  PID = abs(P_e + I_e + D_e);                 // pure magnitude to set motor power

  // set motor speed
  PID_A, PID_B = setMotors();
  
  if (current_time % 20 == 0){                // print data every 20ms
	Serial.print("X : ");
	Serial.print(x);
  Serial.print("\t    PID : ");
  Serial.print(int(PID));
  Serial.print("\t    P : ");
  Serial.print(P_e);
  Serial.print("\t    D : ");
  Serial.print(D_e);
  Serial.print("\t    PID_A : ");
  Serial.print(PID_A);
  Serial.print("\t    PID_B : ");
  Serial.print(PID_B);
  Serial.print("\t    SP : ");
  Serial.print(set_point);
//  Serial.print("\t    acc_y : ");
//  Serial.print(acc_y);
//  Serial.print("\t    vel_y : ");
//  Serial.print(vel_y);
//  Serial.print("\t    pos_y : ");
//  Serial.print(pos_y);
  Serial.print("\t    dt : ");
  Serial.print(delta_time);
  Serial.print("\n");
  }
}

// Sets the motors pased on PID value
int setMotors(){
  if (x >= 0){
    // Set motors to drive forward for +ve angle above 2
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else{
    // Set motors to drive backward for -ve angle below 2
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  // Set bounds
  if (PID > 160){
    PID = 160;
  }
  
  // Map PID value to each motor since each motor behaves slightly differently
  PID_A = map(PID, 0, 160, 29, 255);
  PID_B = map(PID, 0, 160, 25, 255);

  // Cut off motors if angle is small
  if (abs(x) < 1){
    PID_A = 0;
    PID_B = 0;
  }

  // Set motor power equal to PID value
  analogWrite(enA, PID_A);
  analogWrite(enB, PID_B);

  // Return each motor power for reading in serial port
  return PID_A, PID_B;
}

// Shifts setpoint slightly forward to move forward
int moveForward(){
  set_point = 3;
  return set_point;
}

// Shifts setpoint slightly backward to move backward
int moveBackward(){
  set_point = -3;
  return set_point;
}

// Change setpoint to return roughly to start based on position calculations
int return_to_start(){
  if (pos_y > 0.2){
    set_point = -3;
  }
  else if (pos_y < -0.2){
    set_point = 3;
  }
  else {
    set_point = 0;
  }
  return set_point;
}
