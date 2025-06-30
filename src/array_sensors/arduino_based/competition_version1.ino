#include <SparkFun_TB6612.h>

// COMPETITION CONFIGURATION
const bool IS_BLACK_LINE = true;
const int NUM_SENSORS = 7;               // Use all 7 sensors for maximum precision

// AGGRESSIVE SPEED SETTINGS
const int BASE_SPEED = 180;              // High base speed
const int MAX_SPEED = 255;               // Maximum possible speed
const int MIN_SPEED = 80;                // Minimum speed for tight turns
const int STRAIGHT_SPEED = 240;          // Speed for straight sections
const int TURN_SPEED = 120;              // Speed for detected turns

// HIGHLY TUNED PID CONSTANTS for high-speed performance
const float KP = 0.15;                   // Higher P for quick response
const float KD = 3.5;                    // High D for stability at speed
const float KI = 0.0008;                 // Minimal I to prevent oscillation

// ADVANCED CONTROL PARAMETERS
const int STRAIGHT_THRESHOLD = 50;        // Error threshold to detect straight line
const int SHARP_TURN_THRESHOLD = 400;     // Error threshold for sharp turns
const int LINE_LOST_THRESHOLD = 100;      // Cycles before considering line lost
const int SPEED_RAMP_RATE = 8;           // How fast to change speeds

// Motor pins (unchanged)
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

// High-performance sensor arrays
int sensorMin[7] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[7] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorValues[7];
int lastSensorValues[7];

// Advanced PID and control variables
int lastError = 0;
int lastLastError = 0;
float integral = 0;
bool onLine = false;
int currentSpeed = BASE_SPEED;
int targetSpeed = BASE_SPEED;
int lineLostCount = 0;
int straightLineCount = 0;

// Performance tracking
unsigned long lastLoopTime = 0;
unsigned long startTime = 0;
float totalDistance = 0;

void setup() {
  Serial.begin(115200);  // Higher baud rate for faster communication
  
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // MAXIMUM ADC SPEED - Critical for competition
  ADCSRA = (ADCSRA & 0xF8) | 0x02; // Set prescaler to 4 (fastest stable setting)
  
  Serial.println("Press 11: Calibrate | Press 12: RACE!");
}

void loop() {
  if (digitalRead(CALIBRATE_BUTTON) == LOW) {
    competitionCalibration();
    while (digitalRead(CALIBRATE_BUTTON) == LOW) delay(10);
  }
  
  if (digitalRead(START_BUTTON) == LOW) {
    delay(500);
    startRace();
  }
}

void competitionCalibration() {
  Serial.println("CALIBRATION STARTING...");
  digitalWrite(LED_PIN, HIGH);
  
  // Reset calibration
  for (int i = 0; i < 7; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // Fast, comprehensive calibration
  unsigned long calibStart = millis();
  int direction = 1;
  
  while (millis() - calibStart < 4000) { // 4 second calibration
    // Dynamic calibration pattern
    int rotSpeed = 80 + (millis() - calibStart) / 50; // Increasing speed
    
    leftMotor.drive(direction * rotSpeed);
    rightMotor.drive(-direction * rotSpeed);
    
    // Switch direction every 500ms for better coverage
    if ((millis() - calibStart) % 500 < 10) {
      direction *= -1;
    }
    
    // Rapid sensor sampling
    for (int sample = 0; sample < 5; sample++) {
      updateSensorRanges();
      delayMicroseconds(200);
    }
  }
  
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  
  // Validate calibration
  validateCalibration();
  Serial.println("CALIBRATION COMPLETE - READY TO RACE!");
}

void updateSensorRanges() {
  for (int i = 0; i < 7; i++) {
    int reading = analogRead(i);
    if (reading < sensorMin[i]) sensorMin[i] = reading;
    if (reading > sensorMax[i]) sensorMax[i] = reading;
  }
}

void validateCalibration() {
  Serial.println("Calibration Results:");
  for (int i = 0; i < 7; i++) {
    int range = sensorMax[i] - sensorMin[i];
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(range);
    if (range < 100) Serial.print(" [WARNING: Low range]");
    Serial.print(" | ");
  }
  Serial.println();
}

void startRace() {
  Serial.println("🏁 RACE STARTED! 🏁");
  startTime = millis();
  integral = 0;
  lastError = 0;
  currentSpeed = BASE_SPEED;
  lineLostCount = 0;
  
  while (true) {
    lastLoopTime = micros();
    
    readSensorsOptimized();
    
    if (onLine) {
      competitiveLineFollow();
      lineLostCount = 0;
      digitalWrite(LED_PIN, HIGH);
    } else {
      handleLineLoss();
      digitalWrite(LED_PIN, LOW);
    }
    
    // Performance monitoring
    if (millis() % 1000 == 0) {
      printPerformanceStats();
    }
  }
}

void readSensorsOptimized() {
  onLine = false;
  int activesensors = 0;
  
  // Store last values for derivative calculations
  for (int i = 0; i < 7; i++) {
    lastSensorValues[i] = sensorValues[i];
  }
  
  // Ultra-fast sensor reading with minimal processing
  for (int i = 0; i < 7; i++) {
    int raw = analogRead(i);
    sensorValues[i] = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
    sensorValues[i] = constrain(sensorValues[i], 0, 1000);
    
    // Optimized line detection
    if (sensorValues[i] > 400) {
      onLine = true;
      activeServices++;
    }
  }
  
  // Update line lost counter
  if (!onLine) {
    lineLostCount++;
  }
}

void competitiveLineFollow() {
  // Calculate sophisticated error with all sensors
  int error = calculateAdvancedError();
  
  // Advanced PID with predictive elements
  integral += error;
  integral = constrain(integral, -5000, 5000); // Prevent windup
  
  int derivative = error - lastError;
  int secondDerivative = derivative - (lastError - lastLastError);
  
  // Enhanced PID formula with second derivative
  float pidOutput = (KP * error) + (KI * integral) + (KD * derivative) + (0.1 * secondDerivative);
  
  // Adaptive speed control based on track conditions
  adaptiveSpeedControl(error);
  
  // Calculate motor speeds with advanced logic
  int leftSpeed = currentSpeed - pidOutput;
  int rightSpeed = currentSpeed + pidOutput;
  
  // Intelligent speed limiting
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Apply speeds
  leftMotor.drive(leftSpeed);
  rightMotor.drive(rightSpeed);
  
  // Update error history
  lastLastError = lastError;
  lastError = error;
}

int calculateAdvancedError() {
  // Weighted position calculation for maximum precision
  int numerator = 0;
  int denominator = 0;
  
  for (int i = 0; i < 7; i++) {
    int weight = (i - 3) * 1000; // Position weights: -3000, -2000, -1000, 0, 1000, 2000, 3000
    numerator += weight * sensorValues[i];
    denominator += sensorValues[i];
  }
  
  if (denominator == 0) {
    return lastError; // No line detected, use last error
  }
  
  return numerator / denominator;
}

void adaptiveSpeedControl(int error) {
  int absError = abs(error);
  
  // Determine target speed based on track conditions
  if (absError < STRAIGHT_THRESHOLD) {
    // Straight line - maximum speed
    targetSpeed = STRAIGHT_SPEED;
    straightLineCount++;
  } else if (absError > SHARP_TURN_THRESHOLD) {
    // Sharp turn - reduce speed significantly
    targetSpeed = TURN_SPEED;
    straightLineCount = 0;
  } else {
    // Moderate curve - medium speed
    targetSpeed = BASE_SPEED;
    straightLineCount = 0;
  }
  
  // Smooth speed transitions
  if (currentSpeed < targetSpeed) {
    currentSpeed = min(currentSpeed + SPEED_RAMP_RATE, targetSpeed);
  } else if (currentSpeed > targetSpeed) {
    currentSpeed = max(currentSpeed - SPEED_RAMP_RATE, targetSpeed);
  }
  
  // Speed boost for long straight sections
  if (straightLineCount > 50) {
    currentSpeed = min(currentSpeed + 2, MAX_SPEED);
  }
}

void handleLineLoss() {
  if (lineLostCount < LINE_LOST_THRESHOLD) {
    // Recent line loss - continue with last known direction
    continueWithMomentum();
  } else {
    // Extended line loss - aggressive search
    aggressiveLineSearch();
  }
}

void continueWithMomentum() {
  // Continue in the direction of the last error with reduced speed
  int searchSpeed = currentSpeed * 0.6;
  
  if (lastError > 0) {
    leftMotor.drive(searchSpeed);
    rightMotor.drive(searchSpeed * 0.3);
  } else {
    leftMotor.drive(searchSpeed * 0.3);
    rightMotor.drive(searchSpeed);
  }
}

void aggressiveLineSearch() {
  // Rapid oscillation search pattern
  int searchSpeed = 150;
  int oscillation = (millis() / 50) % 4; // Fast oscillation
  
  switch (oscillation) {
    case 0:
    case 2:
      leftMotor.drive(searchSpeed);
      rightMotor.drive(-searchSpeed);
      break;
    case 1:
    case 3:
      leftMotor.drive(-searchSpeed);
      rightMotor.drive(searchSpeed);
      break;
  }
}

void stopMotors() {
  leftMotor.drive(0);
  rightMotor.drive(0);
}

void printPerformanceStats() {
  unsigned long raceTime = millis() - startTime;
  Serial.print("Time: ");
  Serial.print(raceTime / 1000.0, 2);
  Serial.print("s | Speed: ");
  Serial.print(currentSpeed);
  Serial.print(" | Error: ");
  Serial.print(lastError);
  Serial.print(" | Loop: ");
  Serial.print(micros() - lastLoopTime);
  Serial.println("μs");
}
