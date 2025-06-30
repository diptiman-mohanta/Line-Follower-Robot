void setup() {
  // put your setup code here, to run once:
  #define e1  9
  #define i1  7
  #define i2  8
  #define i3  11
  #define i4  12
  #define e2  10

  // IR SENSORS
  #define ir5  6       //rightmost sensors
  #define ir4  5      // right sensor
  #define ir3  4     // middle sensors
  #define ir2  3      // left sensor
  #define ir1  2      //leftmost sensors

  pinMode(e1, OUTPUT);           //pwm pin 1
  pinMode(e2, OUTPUT);           //pwm pin 2
  pinMode(i1, OUTPUT);           //ledt motor pin 1
  pinMode(i2, OUTPUT);           //left motor pin 2
  pinMode(i3, OUTPUT);           //right motor pin 1
  pinMode(i4, OUTPUT);           //rightmotor pin 2

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
 
}
void forward() {
  analogWrite(e1, 70); //Left Motor Speed
  analogWrite(e2,68);
  digitalWrite(i1, HIGH);
  digitalWrite(i2, LOW);
  digitalWrite(i3, HIGH);
  digitalWrite(i4, LOW);
}
