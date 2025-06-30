#include <SparkFun_TB6612.h>

// Configuration - Easy to modify
const bool IS_BLACK_LINE = true;      // true for black line, false for white line
const int BASE_SPEED = 100;           // Base motor speed (0-255)
const int MAX_SPEED = 200;            // Maximum motor speed
const int NUM_SENSORS = 5;            // Number of sensors (5 or 7)

// PID Constants - Tuned for good performance
const float KP = 0.08;                // Proportional gain
const float KD = 1.8;                 // Derivative gain
const float KI = 0.001;               // Integral gain (small to prevent windup)

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// Control pins
#define CALIBRATE_BUTTON 11
#define START_BUTTON 12
#define LED_PIN 13

// Motor objects
Motor leftMotor = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);

// Sensor arrays
int sensorMin[7] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[7] = {0, 0, 0, 0, 0, 0, 0};
int sensorValues[7];

// PID variables
int lastError = 0;
int integral = 0;
bool onLine = false;

void setup() {
  Serial.begin(9600);
  
  // Setup pins
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Optimize ADC for faster readings
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // Set ADC prescaler to 16 for faster conversion
  
  Serial.println("Line Follower Ready!");
  Serial.println("Press button 11 to calibrate, then button 12 to start");
}

void loop() {
  // Wait for calibration
  if (digitalRead(CALIBRATE_BUTTON) == LOW) {
    calibrateSensors();
    while (digitalRead(CALIBRATE_BUTTON) == LOW) delay(10); // Wait for release
  }
  
  // Wait for start
  if (digitalRead(START_BUTTON) == LOW) {
    delay(1000); // Give time to place robot
    runLineFollower();
  }
}

void calibrateSensors() {
  Serial.println("Calibrating... Move robot over line");
  digitalWrite(LED_PIN, HIGH);
  
  // Reset min/max values
  for (int i = 0; i < 7; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // Calibrate by rotating
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 second calibration
    // Alternate rotation direction
    if ((millis() - startTime) % 1000 < 500) {
      leftMotor.drive(60);
      rightMotor.drive(-60);
    } else {
      leftMotor.drive(-60);
      rightMotor.drive(60);
    }
    
    // Read and update sensor ranges
    updateSensorRanges();
    delay(10);
  }
  
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Calibration complete!");
  printCalibration();
}

void updateSensorRanges() {
  for (int i = 0; i < 7; i++) {
    int reading = analogRead(i);
    if (reading < sensorMin[i]) sensorMin[i] = reading;
    if (reading > sensorMax[i]) sensorMax[i] = reading;
  }
}

void runLineFollower() {
  Serial.println("Starting line following...");
  
  while (true) {
    readSensors();
    
    if (onLine) {
      // Use PID control when on line
      followLineWithPID();
      digitalWrite(LED_PIN, HIGH);
    } else {
      // Search for line when lost
      searchForLine();
      digitalWrite(LED_PIN, LOW);
    }
    
    // Small delay for stability
    delay(5);
  }
}

void readSensors() {
  onLine = false;
  int startSensor = (NUM_SENSORS == 5) ? 1 : 0;
  int endSensor = (NUM_SENSORS == 5) ? 6 : 7;
  
  for (int i = startSensor; i < endSensor; i++) {
    // Read and normalize sensor value (0-1000)
    int raw = analogRead(i);
    sensorValues[i] = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
    sensorValues[i] = constrain(sensorValues[i], 0, 1000);
    
    // Check if we're on the line
    bool sensorOnLine = IS_BLACK_LINE ? (sensorValues[i] > 500) : (sensorValues[i] < 500);
    if (sensorOnLine) onLine = true;
  }
}

void followLineWithPID() {
  // Calculate position error
  int error = calculateError();
  
  // PID calculations
  integral += error;
  integral = constrain(integral, -1000, 1000); // Prevent integral windup
  
  int derivative = error - lastError;
  int pidOutput = (KP * error) + (KI * integral) + (KD * derivative);
  
  lastError = error;
  
  // Calculate motor speeds
  int leftSpeed = BASE_SPEED - pidOutput;
  int rightSpeed = BASE_SPEED + pidOutput;
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Drive motors
  leftMotor.drive(leftSpeed);
  rightMotor.drive(rightSpeed);
}

int calculateError() {
  if (NUM_SENSORS == 5) {
    // Weighted error calculation for 5 sensors
    return (3 * sensorValues[1] + 1 * sensorValues[2] - 1 * sensorValues[4] - 3 * sensorValues[5]);
  } else {
    // Weighted error calculation for 7 sensors  
    return (3 * sensorValues[0] + 2 * sensorValues[1] + 1 * sensorValues[2] 
            - 1 * sensorValues[4] - 2 * sensorValues[5] - 3 * sensorValues[6]);
  }
}

void searchForLine() {
  // Turn in the direction of the last known error
  int searchSpeed = 80;
  
  if (lastError > 0) {
    // Line was to the right, turn right
    leftMotor.drive(searchSpeed);
    rightMotor.drive(-searchSpeed/2);
  } else {
    // Line was to the left, turn left
    leftMotor.drive(-searchSpeed/2);
    rightMotor.drive(searchSpeed);
  }
}

void stopMotors() {
  leftMotor.drive(0);
  rightMotor.drive(0);
}

void printCalibration() {
  Serial.print("Min values: ");
  for (int i = 0; i < 7; i++) {
    Serial.print(sensorMin[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Max values: ");
  for (int i = 0; i < 7; i++) {
    Serial.print(sensorMax[i]);
    Serial.print(" ");
  }
  Serial.println();
}
