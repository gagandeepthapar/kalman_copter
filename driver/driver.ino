// AERO 465 Final Project: Level Altitude Helicopter Control Stick
// Gagandeep Thapar, Jake Pietig, Declan Clause

// Libraries to use
#include <Servo.h>  // servo controller
#include <NewPing.h>  // ultrasonic sensor
#include <SparkFun_TB6612.h>  // motor controller
// NOTE: SimpleKalmanFilter does exist, however, I will be using my own for greater control

// Declarations
// SERVO
#define PITCHSIG 8  // pin for pitch servo signal
#define ROLLSIG 10  // pin for roll servo signal
#define NORM 110 // "0" degree in angle offset
#define DELTA -40 // amount of control allowable in each direction (deg)
Servo pitchRotor; // servo object for pitch control
Servo rollRotor;  // servo object for roll control

// ULTRASONIC SENSOR
#define TRIGGER_PIN 12  // pin for trigger
#define ECHO_PIN 11 // pin for echo
#define MAX_DIST 200  // max distance (cm)
NewPing altSensor(TRIGGER_PIN, ECHO_PIN, MAX_DIST); // ultrasonic sensor object
float distFactor = 1.0; // temp. dist factor to account for errors in ultrasonic sensor

// MOTOR CONTROLLER
#define AIN1 2  // pin for A IN
#define AIN2 4  // pin for A IN
#define PWMA 5  // pin for PWM control
#define STBY 9  // pin for standby
Motor prop = Motor(AIN1, AIN2, PWMA, 1, STBY);  // motor object (propeller)

// KALMAN FILTER (initial values)
float Q = 0.00001;  // universal, process noise
float R = 0.0003;  // universal, measurement noise
float P_PRE[] = {0.0, 0.0, 0.0};  // ultrasonic, joyPITCH, joyROLL; P values
float X[] = {0.0, 0.0, 0.0};  // ultrasonic, joyPITCH, joyROLL; X values

// JOYSTICK
#define joyPITCH A0 // pin for PITCH Control
#define joyROLL A1  // pin for ROLL Control

// OUTPUTS
float altitude;
float pitchAngle, pitchDrive;
float rollAngle, rollDrive;
int propellerSpeed;

void setup() {
  Serial.begin(9600);
  distFactor = calibrateSensor(3);  // [~] Correction Factor for distance readings

  pitchRotor.attach(PITCHSIG);  // correlate the pins to the correct servos
  rollRotor.attach(ROLLSIG);

  pitchRotor.write(0);
  rollRotor.write(0);
  
}

void loop() {
  
  // handle altitude
  altitude = grabFiltDist();  // measure distance AND kalman filter it
  propellerSpeed = (int) map(altitude, 0, 50, 255, 50); // map value to something motor can sense
  prop.drive(propellerSpeed); // drive motor to this speed

  Serial.print(altitude);
  Serial.print(",");
//  Serial.println(propellerSpeed);
//  Serial.print(",");


  // handle pitch
  pitchAngle = grabFiltPitch(); // measure pitch angle AND kalman filter it
  pitchDrive = (int) map(pitchAngle, 0, 1023, -1*DELTA, DELTA) + NORM; // map value to something a servo can do
  pitchRotor.write(pitchDrive); // drive servo

//  Serial.print(pitchAngle);
//  Serial.print(",");
  Serial.print(pitchDrive);
  Serial.print(",");
  
  // handle roll
  rollAngle = grabFiltRoll(); // measure roll angle AND kalman filter it
  rollDrive = (int) map(rollAngle, 0, 1023, -1*DELTA, DELTA) + NORM; // map value to something a servo can do
  rollRotor.write(rollDrive); // drive servo
  
//  Serial.print(rollAngle);
//  Serial.print(",");
  Serial.println(rollDrive);

}

float kalman(float meas, int idx){  // custom Kalman Filter code based on array of X, P values and uniform Q, R values

  // predictor step
  float xPre = X[idx];
  float pPre = P_PRE[idx] + Q;

  // corrector step
  float K = pPre/(pPre + R);
  X[idx] = xPre + K*(meas - xPre);
  P_PRE[idx] = (1-K) * pPre;

  // return the expected value
  return X[idx];
 
}

float grabFiltRoll(){ // kalman filter the roll angle from joystick
  return kalman(analogRead(joyROLL), 2);
}

float grabFiltPitch(){  // kalman filter the pitch angle from joystick
  return kalman(analogRead(joyPITCH), 1);
}

float grabDist() {  // function to grab distance from ultrasonic sensor directly to CM
  return altSensor.ping_cm();
}

float grabFiltDist(){ // function to kalman filter the distance reading from the ultrasonic sensor
  return kalman(grabDist(), 0);
}

float calibrateSensor(int timeSec) {  // function to calibrate sensor over predetermined time
  int rawDist = 0;
  int curDist = 0;
  int sampleSize = 0;

  while (millis() < timeSec*1000) {
    rawDist = rawDist + grabDist();  // find total distance over set period of time
    sampleSize = sampleSize + 1; 
  }

  return ((double)sampleSize * 10.00 / (double)rawDist);  // find average; return correction factor @ 10cm

}
