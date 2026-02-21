#define e1  5
#define i1  6
#define i2  7
#define i3  8
#define i4  9
#define e2  10

// IR SENSORS
#define ir5  A0       //rightmost sensors
#define ir4  A1       // right sensor
#define ir3  A2      // middle sensors
#define ir2  A3      // left sensor
#define ir1  A4      //leftmost sensors

int initial_motor_speed = 100;
int flag = 0;

//PID CONSTANT
float kp = 28;
float ki = 0.0;
float kd = 22;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int s1, s2, s3, s4, s5;

const int lm1 = 80;
const int lm2 = 80;
const int rm1 = 95;
const int rm2 = 95;

// adaptive speed tracking
int consecutive_straight = 0;

void setup() {
  pinMode(e1, OUTPUT);           //pwm pin 1
  pinMode(e2, OUTPUT);           //pwm pin 2
  pinMode(i1, OUTPUT);           //left motor pin 1
  pinMode(i2, OUTPUT);           //left motor pin 2
  pinMode(i3, OUTPUT);           //right motor pin 1
  pinMode(i4, OUTPUT);           //right motor pin 2

  pinMode(ir1 , INPUT);           //leftmost sensor
  pinMode(ir2 , INPUT);           //left sensor
  pinMode(ir3 , INPUT);           //middle sensor
  pinMode(ir4 , INPUT);           //right sensor
  pinMode(ir5 , INPUT);           //rightmost sensor

  Serial.begin(9600);
  delay(1000);
}

void readSensor()
{
  s1 = digitalRead(ir1);   //leftmost sensor
  s2 = digitalRead(ir2);   // left sensor
  s3 = digitalRead(ir3);   // mid sensor
  s4 = digitalRead(ir4);   // right sensor
  s5 = digitalRead(ir5);   // rightmost sensor

  // special case errors first (junction / lost line handling)
  if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1)                   //turn right
  {
    error = 10;
  }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0)              //turn left
  {
    error = 11;
  }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1)              // all sensors on line - go forward slightly
  {
    error = 12;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)             // no sensor on line - go forward slightly
  {
    error = 13;
  }
  else if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1)             // both outer sensors only
  {
    error = 14;
  }

  // pid control sensors
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1)
  {
    error = 4;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1)
  {
    error = 3;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0)
  {
    error = 2;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0)
  {
    error = 1;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0)
  {
    error = 0;
  }
  else if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0)
  {
    error = -1;
  }
  else if (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0)
  {
    error = -2;
  }
  else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0)
  {
    error = -3;
  }
  else if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)
  {
    error = -4;
  }
}

void forward() {
  digitalWrite(i1, HIGH);
  digitalWrite(i2, LOW);
  digitalWrite(i3, HIGH);
  digitalWrite(i4, LOW);
}
void backward() {
  digitalWrite(i1, LOW);
  digitalWrite(i2, HIGH);
  digitalWrite(i3, LOW);
  digitalWrite(i4, HIGH);
}
void Stop() {
  digitalWrite(i1, LOW);
  digitalWrite(i2, LOW);
  digitalWrite(i3, LOW);
  digitalWrite(i4, LOW);
}
void left() {
  digitalWrite(i1, LOW);
  digitalWrite(i2, LOW);
  digitalWrite(i3, HIGH);
  digitalWrite(i4, LOW);
}
void sharpleft() {
  digitalWrite(i1, LOW);
  digitalWrite(i2, HIGH);
  digitalWrite(i3, HIGH);
  digitalWrite(i4, LOW);
}
void right() {
  digitalWrite(i1, HIGH);
  digitalWrite(i2, LOW);
  digitalWrite(i3, LOW);
  digitalWrite(i4, LOW);
}
void sharpright() {
  digitalWrite(i1, HIGH);
  digitalWrite(i2, LOW);
  digitalWrite(i3, LOW);
  digitalWrite(i4, HIGH);
}

void calculate_pid()
{
  P = error;
  I = I + error;
  I = constrain(I, -50, 50);                    // anti-windup: clamp integral so it doesn't build up too much
  D = error - previous_error;

  PID_value = (kp * P) + (ki * I) + (kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // adaptive base speed: go faster on straights, slower on curves
  if (error == 0) {
    consecutive_straight++;
  } else {
    consecutive_straight = 0;
  }

  int adaptive_speed = initial_motor_speed;
  if (consecutive_straight > 5) {
    adaptive_speed = initial_motor_speed + 20;   // boost speed on long straights
  }
  if (abs(error) >= 3) {
    adaptive_speed = initial_motor_speed - 15;   // slow down on sharp curves
  }

  adaptive_speed = constrain(adaptive_speed, 60, 220);

  int left_motor_speed  = adaptive_speed + PID_value;
  int right_motor_speed = adaptive_speed - PID_value;

  left_motor_speed  = constrain(left_motor_speed,  0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(e1, left_motor_speed + 20);   //Left Motor Speed (offset to compensate motor imbalance)
  analogWrite(e2, right_motor_speed);        //Right Motor Speed

  forward();
}

void loop()
{
  readSensor();

  if (error == 10)                                               // Make right turn until it detects straight path
  {
    do {
      analogWrite(e1, rm1);
      analogWrite(e2, rm2);
      right();
      readSensor();
    } while (s2 == 0 && s1 == 0 && error != -3);
    previous_error = 0;                                          // reset PID state after sharp turn
    I = 0;
  }
  else if (error == 11)                                          // Make left turn until it detects straight path
  {
    do {
      analogWrite(e1, lm1);
      analogWrite(e2, lm2);
      left();
      readSensor();
    } while (s4 == 0 && s5 == 0 && error != 3);
    previous_error = 0;                                          // reset PID state after sharp turn
    I = 0;
  }
  else if (error == 12)
  {
    analogWrite(e1, 90);
    analogWrite(e2, 70);
    forward();
    delay(20);
    readSensor();
  }
  else if (error == 13)
  {
    // use last known error direction to recover instead of blindly going forward
    if (previous_error > 0) {
      analogWrite(e1, rm1);
      analogWrite(e2, rm2);
      right();
    } else if (previous_error < 0) {
      analogWrite(e1, lm1);
      analogWrite(e2, lm2);
      left();
    } else {
      analogWrite(e1, 90);
      analogWrite(e2, 70);
      forward();
    }
    delay(40);
    readSensor();
  }
  else if (error == 14)
  {
    analogWrite(e1, 90);
    analogWrite(e2, 70);
    forward();
    delay(60);
    readSensor();
  }
  else {
    calculate_pid();
    motor_control();
  }
}
