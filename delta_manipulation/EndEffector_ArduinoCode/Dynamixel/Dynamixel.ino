bool Dynamixel_320() {
  while (Serial.available() > 0)
  {
    char myservo = Serial.read();


    if (myservo == '0')
    {
      digitalWrite(dynamixel, HIGH);
    }
    else if (myservo == '1')
    {
      digitalWrite(dynamixel, LOW);
    }
  }

}
