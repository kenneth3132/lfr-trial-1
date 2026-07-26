int sensorADC[8];
void setup() {Serial.begin(9600);}
void loop() {
  Read_ADC();
  for(unsigned char x=0;x<8;x++){Serial.print(String(sensorADC[x])+", ");}
  Serial.println();
}
void Read_ADC(){
  sensorADC[0]=analogRead(7);
  sensorADC[1]=analogRead(6);
  sensorADC[2]=analogRead(5);
  sensorADC[3]=analogRead(4);
  sensorADC[4]=analogRead(3);
  sensorADC[5]=analogRead(2);
  sensorADC[6]=analogRead(1);
  sensorADC[7]=analogRead(0);
}
