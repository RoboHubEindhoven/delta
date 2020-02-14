float encoder_speed()
{
  newposition = encoder0Pos;
  newtime = (0.002 * millis());
  vel = (newposition - oldposition) / (newtime - oldtime);
  Serial.print ("\n speed = ");
  Serial.print (velocity * 10);
  oldposition = newposition;
  oldtime = newtime;
  delay(100);
  velocity = vel * 1 / 500;
  return velocity;
}


void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void MotorClockwise(int power) {
  digitalWrite(9, HIGH);
  analogWrite(10, power);
}

void MotorCounterClockwise(int power) {
  digitalWrite(9, LOW);
  analogWrite(10, power);
}

float speed_PID(int speed_in) {
  
  float Speed = encoder_speed();
  motor_value = velocity_pid.compute(speed_in);// Let the PID compute the value, returns the optimal output
  
  if (motor_value > 0) {
    MotorCounterClockwise(motor_value);
  }
  else {
    MotorClockwise(abs(motor_value));
  }
  return motor_value;
  delay(20);
  
}
