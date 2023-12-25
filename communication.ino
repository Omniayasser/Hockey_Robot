#include <SoftwareSerial.h>
#include <Servo.h>
int data;
#define IR_left 10

SoftwareSerial BTSerial(0, 1); // RX, TX pins for Bluetooth module

#define RX  0
#define TX  1
#define IN3 2
#define IN4 4
#define EN3 5
//motor2
#define IN1 7
#define IN2 3
#define EN1 6
Servo arm;


void setup() {
  arm.attach(9);
  // Start the Bluetooth serial connection
 // BTSerial.begin(9600);
  Serial.begin(9600);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(IR_left, INPUT);
}
void forward (int speed)
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(EN3, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
}
void stop()
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);
}
void right (int speed)
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
}
void left (int speed)
{
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(EN3, speed);
  
}
void backward (int speed)
{
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN3, speed);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  analogWrite(EN1, speed);
}
void rotateCW (int speed)
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(EN3, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speed);
}
void rotateCCW (int speed)
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(EN3, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
}
void loop() {
    int count=0;
   if (BTSerial.available()) 
    char command = BTSerial.read();
  delay(250);                   

  while(Serial.available()) 
    data = Serial.read();

 
  if (data == '0')
{
    if(digitalRead(IR_left)==0)
      {
      stop();
      delay(3000);
      arm.write(120);
      delay(2000);
      arm.write(90);
      delay(5000);
      arm.write(70);
      }
      else if(digitalRead(IR_left)==1)
      {
        arm.write(90);
        forward(80);
      }
}
  else if (data == '2')
{ 
    rotateCCW(80);
}
  else if (data == '1')
  {
   rotateCW(80); 
  }
   else if (data == '3')
  {
    rotateCW(80);    
  }
   }
