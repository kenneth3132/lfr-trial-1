void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;

  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    //map analog value x
    //analog to digital
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {    //black line - normal mode.
        sensorDigital[i] = 1;  //black line =1
      } else {
        sensorDigital[i] = 0;  //black line =0 (black line -inverse mode)
      }
    } else {
      if (inverseON == 0) {    //white line - normal mode
        sensorDigital[i] = 0;  //white line =0 - normal mode
      } else {
        sensorDigital[i] = 1;  //white line - inverse mode.
      }
    }

    sumOnSensor += sensorDigital[i];
    sensorWight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void sensor_value_test() {
  while (1) {
    read_black_line();
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(String(sensorADC[i]) + "  ");
    }
    Serial.println();
  }
}

void Bit_Sensor_Show() {
  while (1) {
    read_black_line();  //take sensor reading

    Serial.print("Bit Sensor in Decimal: " + String(bitSensor));  //Decimal Value Show
    Serial.print(" | Bit Sensor in Binary: ");

    for (int i = (sensorNumber - 1); i >= 0; i--) {  //display bitsensor data in binary
      Serial.print(String(bitRead(bitSensor, i)));
    }
    Serial.println();
  }
}

void sensor_value_check() {
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    //map analog value
    Serial.print(String(sensorADC[i]) + "  ");
  }
  for (int i = 0; i < sensorNumber; i++) {
    //analog to digital
    if (sensorADC[i] > theshold) {
      sensorDigital[i] = 1;
    } else {
      sensorDigital[i] = 0;
    }
    Serial.print("  " + String(sensorDigital[i]));
  }
  Serial.println();
}

//function for displaying analog value
void analog_value() {
  while (1) {
    for (int channel = 0; channel < sensorNumber; channel++) {
      selectChannel(channel);
      sensorADC[channel] = analogRead(SIG_PIN);
    }
    u8g.setFont(u8g_font_profont12);  // Set the font
    u8g.firstPage();
    do {
      u8g.setPrintPos(10, 60);
      u8g.print(sensorADC[0]);
      u8g.setPrintPos(15, 45);
      u8g.print(sensorADC[1]);
      u8g.setPrintPos(20, 30);
      u8g.print(sensorADC[2]);
      //
      u8g.setPrintPos(35, 15);
      u8g.print(sensorADC[3]);
      u8g.setPrintPos(65, 15);
      u8g.print(sensorADC[4]);
      //
      u8g.setPrintPos(80, 30);
      u8g.print(sensorADC[5]);
      u8g.setPrintPos(85, 45);
      u8g.print(sensorADC[6]);
      u8g.setPrintPos(90, 60);
      u8g.print(sensorADC[7]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;
  }
}

//function for displaying digital value

void digital_value() {
  while (1) {
    String inverseState;
    read_black_line();
    if (bitSensor == 207 || bitSensor == 231 || bitSensor == 239 || bitSensor == 231 || bitSensor == 247 || bitSensor == 243) {
      inverseON = !inverseON;
      digitalWrite(led, inverseON);
    }
    (inverseON) ? inverseState = "Inv: ON" : inverseState = "Inv: OFF";
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(50, 15);
      u8g.print(inverseState);
      u8g.drawStr(45, 50, "BitSensor");
      u8g.setPrintPos(60, 60);
      u8g.print(bitSensor);
      //
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(10, 60);
      u8g.print(sensorDigital[0]);
      u8g.setPrintPos(20, 45);
      u8g.print(sensorDigital[1]);

      u8g.setPrintPos(35, 35);
      u8g.print(sensorDigital[2]);
      u8g.setPrintPos(55, 35);
      u8g.print(sensorDigital[3]);
      u8g.setPrintPos(75, 35);
      u8g.print(sensorDigital[4]);
      u8g.setPrintPos(95, 35);
      u8g.print(sensorDigital[5]);
      //
      u8g.setPrintPos(110, 45);
      u8g.print(sensorDigital[6]);
      u8g.setPrintPos(120, 60);
      u8g.print(sensorDigital[7]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;
  }
}

