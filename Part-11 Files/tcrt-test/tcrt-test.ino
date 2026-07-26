void setup(){Serial.begin(9600);}
void loop(){int value=analogRead(5);Serial.println(value);}
