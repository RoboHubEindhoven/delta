#include <PIDController.h>

#define encoder0PinA  2
#define encoder0PinB  4
PIDController velocity_pid; // Create an instance of the PID controller class, called "pid"

unsigned int integerValue = 0; // Max value is 65535
int motor_value = 254;

unsigned int encoder0Pos = 0;
int newposition, oldposition, newtime, oldtime, val, incomingByte;
float vel, velocity;
char data ;
String speed_store;
int i  = 0 ;
//Variable for storing received
int dynamixel = 8;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // anable serial print
  pinMode(dynamixel, OUTPUT); // dynamixal

  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2

  Serial.println("start");                // a personal quirk
  pinMode(10, OUTPUT); //Direction
  pinMode(9, OUTPUT); // PWM

  velocity_pid.begin();//initialize the PID instance
  velocity_pid.tune(7, 0.1, 100); // Tune the PID, arguments: kP, kI, kD
  velocity_pid.limit(-254, 254); // Limit the PID output between -255 and 255, this is important to get rid of integral windup!
}

void loop() {
  if (Serial.available() > 0)
  {
    data = Serial.read();
    Serial.println(data);

    switch (data)  {
      case '0':
        val = 0;
        break;

      case '1' :
        val = 10;
        break ;

      case '2' :
        digitalWrite(dynamixel, HIGH);
        break;

      case '3' :
        digitalWrite(dynamixel, LOW);
        break;

      default:
        break ;
    }
  }
  //speed_PID(100);
  analogWrite(9, val);
}
