uint8_t button(uint8_t buttonNumber) {
  uint16_t t = 0;
  if (!digitalRead(buttonNumber)) {
    delay(50);
    while (!digitalRead(buttonNumber)) {
      delay(50);
      t += 50;
      if (t > 500) digitalWrite(led, 1);
    }
    digitalWrite(led, 0);
    if (t > 0) return 1;
  }
  return 0;
}