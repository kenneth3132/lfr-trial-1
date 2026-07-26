/*
---------------------------------------------
𝑬-𝒎𝒂𝒊𝒍: aslamhshakil20@gmail.com
𝑭𝒂𝒄𝒆𝒃𝒐𝒐𝒌: https://www.facebook.com/RoboTechInnovator/
𝑾𝒉𝒂𝒕'𝒔 𝑨𝒑𝒑: https://wa.me/c/8801647467658 (Get the LFR Kit and Components)
Instagram: https://www.instagram.com/robotech_innovator?igsh=aXk4Z2JrZHc4NjJu
*/

#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);  // Fast I2C / TWI
// U8GLIB_SSD1306_128X64 u8g(13, 11, 8, 9, 10); // SPI connection
// for I2C connection, use this wiring:
// GND > GND
// VCC > 3.3V
// SCL > A5
// SDA > A4
#include <EEPROM.h>

//motorpins
#define right_motor_forward A2  //IN1(L298) || TB6612 -- AIN2
#define right_motor_backward 4  //IN2(L298) || TB6612 -- AIN1
#define left_motor_forward 5    //IN3(L298)  || TB6612 --- BIN1
#define left_motor_backward 6   //IN4(L298)  || TB6612 --- BIN2
//speed control pins (PWM)
#define right_motor_speed 3  //EnA (L298) //TB6612 --- PWMA
#define left_motor_speed 9   //EnB (L298) //TB6612 --- PWMB

//MUX PIN
#define S0 11
#define S1 10
#define S2 8
#define S3 7
#define SIG_PIN A7

//Sensor Variables
#define sensorNumber 8
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128 };
int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80 };
int theshold = 500;
int sumOnSensor;
int sensorWight;
int bitSensor;
int Max_ADC[sensorNumber];
int Min_ADC[sensorNumber];
int Reference_ADC[sensorNumber];

//PID Variables
float line_position;
float error;
float center_position = 45;
float derivative, previous_error;
int base_speed = 150;
int kp = 8;
int kd = 100;
//turns variable
String direction = "straight";  //default value straight
#define delay_before_turn 100
#define turnSpeed 200
//timers
#define stop_time 20
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line

//distance calibration parameters
uint16_t test_time = 500;  //need to be adjusted for measured distance
int measured_distance = 30;
//led
#define led 13

const unsigned char upir_logo[] PROGMEM = {  // this is another way how to define images, using binary notation
  B00010101, B11010111,
  B00010101, B01000101,
  B00010101, B10010110,
  B00011001, B00010101
};

// 'scrollbar_background', 8x64px
const unsigned char bitmap_scrollbar_background[] PROGMEM = {
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00
};
// 'item_sel_outline', 128x21px
const unsigned char bitmap_item_sel_outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0
};
// ------------------ end generated bitmaps from image2cpp ---------------------------------
const int NUM_ITEMS = 5;         // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20;  // maximum characters for the item name

char menu_items[NUM_ITEMS][MAX_ITEM_LENGTH] = {
  // array with item names
  { "Line Follow" },
  { "Calibration" },
  { "Analog Value" },
  { "Digital Value" },
  { "Motor Test" },
};
// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes

#define BUTTON_UP_PIN A0      // pin for UP button
#define BUTTON_SELECT_PIN 12  // pin for SELECT button
#define BUTTON_DOWN_PIN A1    // pin for DOWN button

int button_up_clicked = 0;      // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0;  // same as above
int button_down_clicked = 0;    // same as above
int item_selected = 0;          // which item in the menu is selected
int item_sel_previous;          // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next;              // next item - used in the menu screen to draw next item after the selected one
int current_screen = 0;         // 0 = menu, 1 = screenshot, 2 = qr


//************************************SETUP***************************************
void setup() {
  Serial.begin(9600);
  u8g.setColorIndex(1);  // set the color to white
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);

  pinMode(left_motor_speed, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);

  LoadCalibration();  //load calibration value from eeprom whenever the arduino is start or reset
}


//************************************LOOP***************************************
void loop() {
  //front page
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(40, 15, "Team Name");  //Set Your Team Name
      //
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(35, 60, "Sensor View");  //Set Your Team Name
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
    while (button(BUTTON_DOWN_PIN) == 1) menu();
  }
}
