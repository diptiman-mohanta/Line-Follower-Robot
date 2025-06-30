void setup() {
  // put your setup code here, to run once:
#define ir5  A4       //rightmost sensors
#define ir4  A3      // right sensor
#define ir3  A2     // middle sensors
#define ir2  A1      // left sensor
#define ir1  A0  
    //leftmost sensors
Serial.begin(9600);                      //setting serial monitor at a default baund rate of 9600
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
//   int s1 = digitalRead(ir1);   //lftmst sensor
//  int s2 = digitalRead(ir2);   // leftsensor
//  int s3 = digitalRead(ir3);   // mid sen
//  int s4 = digitalRead(ir4);   // right sen
//  int s5 = digitalRead(ir5);    // rightmost sensor

 int s1 = analogRead(ir1);
 int s2 = analogRead(ir2);
 int s3 = analogRead(ir3);
 int s4 = analogRead(ir4);
 int s5 = analogRead(ir5);

Serial.println(s1);
Serial.println(s2);
Serial.println(s3);
Serial.println(s4);
Serial.println(s5);
Serial.println("");
delay(500);
}
