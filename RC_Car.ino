#include <SoftwareSerial.h>
#include <Servo.h>
Servo ss;
//motor 1
#define RX  0
#define TX  1
#define IN3 2
#define IN4 4
#define EN3 5
//motor2
#define IN1 7
#define IN2 3
#define EN1 6

SoftwareSerial BTSerial(0, 1); // RX, TX pins for Bluetooth module

void setup()
{
  ss.attach(9);
  //motor1
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN3, OUTPUT);
  //motor2
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(13, OUTPUT);
  
 // Start the Bluetooth serial connection
  BTSerial.begin(9600);
  Serial.begin(115200);
  Serial.println("Bluetooth serial started. Pair your RC controller.");
}



void loop() {
  // Check if data is available from the Bluetooth RC controller
  if (BTSerial.available()) {
    char command = BTSerial.read();

    // Process the received command
    switch (command) {
      case 'X': // servo on
      
       ss.write(130);
        break;
        case 'x': // servo off
        ss.write(90);
        delay(500);
        break;
        case 'W': // servo on clockwise
       ss.write(70);
        break;
        case 'w': // servo off 
        ss.write(90); 
        delay(500);
        break;
      case 'F': // Move forward
        forward(70);
        break;
      case 'B': // Move backward
        backward(70);
        break;
      case 'L': // Turn left
        right(70);
        break;
      case 'R': // Turn right
        left(70);
        break;
      case 'S': // Stop
        stop();
        break;
      default:
        break;
    }
  }
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
  analogWrite(EN1,speed);
}
