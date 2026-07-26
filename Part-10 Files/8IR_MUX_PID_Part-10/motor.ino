void motor(int LPWM, int RPWM) {
  if (LPWM > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  }

  if (RPWM > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  }

  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  //Serial.print(String(LPWM) + ",  " + String(RPWM) + ",  ");
  //Serial.println();

  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}

//motor test function
void motor_test() {
  while (1) {
    u8g.setFont(u8g_font_7x14B);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(5, 35, "Motor Testing...");
    } while (u8g.nextPage());
    motor(250, 250);
    digitalWrite(led, HIGH);
    delay(3000);
    motor(0, 0);
    digitalWrite(led, LOW);
    return;
  }
}
