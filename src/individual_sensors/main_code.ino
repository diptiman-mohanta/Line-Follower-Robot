//__working 
//10.5 - 10.8 
// MOTOR VARIABLES //analogWrite(e1,90);analogWrite(e2,70);  forward();
//for smooth left_turn e1 == e2 ==75
             //right turn  e1 == e2 == 95

             
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

int initial_motor_speed = 90;
int flag = 0;

//PID CONSTANT
float kp = 25;
float ki = 0;
float kd = 19;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;


int s1, s2, s3, s4, s5;

const int lm1 = 75;
const int lm2 = 75;
const int rm1 = 92;
const int rm2 = 92;
void setup() {
  // put your setup code here, to run once:
  pinMode(e1, OUTPUT);           //pwm pin 1
  pinMode(e2, OUTPUT);           //pwm pin 2
  pinMode(i1, OUTPUT);           //ledt motor pin 1
  pinMode(i2, OUTPUT);           //left motor pin 2
  pinMode(i3, OUTPUT);           //right motor pin 1
  pinMode(i4, OUTPUT);           //rightmotor pin 2

  pinMode(ir1 , INPUT);           //leftmost sensor
  pinMode(ir2 , INPUT);           //left sensor
  pinMode(ir3 , INPUT);           //middle sensor
  pinMode(ir4 , INPUT);           //right sensor
  pinMode(ir5 , INPUT);           //rightmost sensor
  Serial.begin(9600);                      //setting serial monitor at a default baund rate of 9600
  delay(1000);
  //  Serial.println("Started !!");
  //  delay(1000);
}
void readSensor()
{

  s1 = digitalRead(ir1);   //lftmst sensor
  s2 = digitalRead(ir2);   // leftsensor
  s3 = digitalRead(ir3);   // mid sen
  s4 = digitalRead(ir4);   // right sen
  s5 = digitalRead(ir5);    // rightmost sensor

  if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1)                   //turn right
  {
    error = 10;
  }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0)              //turn left
  {
    error = 11;
  }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1)              // Turn left side or stop
  {
    error = 12;
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)             // make u turn
  {
    error = 13;
  }
   else if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1)             // make u turn
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


//function to control the movements;
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
void calculate_pid ()
{
  P = error;
  I = I + error;                                 //I = I+previous_I
  D = error - previous_error;

  PID_value = (kp * P) + (ki * I) + (kd * D);

  previous_I = I;
  previous_error = error;
}


void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);


  analogWrite(e1, left_motor_speed + 20); //Left Motor Speed
  analogWrite(e2, right_motor_speed); //Right Motor Speed

  //following lines of code are to make the bot move forward
  forward();
}
void loop()
{
  readSensor();
  if ( error == 10)                                               // Make right turn untill it detects straight path
  { // calculate it by motor test
//    analogWrite(e1, 90); //Left Motor Speed
//    analogWrite(e2, 70); //Right Motor Speed
//    forward();
//    delay(105);
    do {
      analogWrite(e1, rm1); //Left Motor Speed
      analogWrite(e2, rm2); //Right Motor Speed
      right();
      readSensor();
    } while (s2 == 0 && s1 == 0 && error != -3);
  }
  else if ( error == 11)                                               // Make left turn untill it detects straight path
  {
//    analogWrite(e1, 90); //Left Motor Speed
//    analogWrite(e2, 70); //Right Motor Speed
//    forward();
//    delay(105);
    do {
      //      readSensor();
      analogWrite(e1, lm1); //Left Motor Speed
      analogWrite(e2, lm2); //Right Motor Speed
      left();
      readSensor();
    } while (s4 ==0 && s5 == 0 && error != 3);
  }
  else if (error == 12)
  {
        analogWrite(e1, 90); //Left Motor Speed
    analogWrite(e2, 70); //Right Motor Speed
    forward();
    delay(20);
    readSensor();
  }
 else if (error == 13)
  {
        analogWrite(e1, 90); //Left Motor Speed
    analogWrite(e2, 70); //Right Motor Speed
    forward();
    delay(40);
    readSensor();
  }
   else if (error == 14)
  {
        analogWrite(e1, 90); //Left Motor Speed
    analogWrite(e2, 70); //Right Motor Speed
    forward();
    delay(60);
    readSensor();
  }
  else {
    calculate_pid();
    motor_control();
  }
}
