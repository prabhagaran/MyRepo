#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

RF24 radio(7, 8); // CE, CSN
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 20 ID's are available @ the moment
const uint64_t pipeOut[] PROGMEM = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL,
  0xE9E8F0F0C1LL, 0xE9E8F0F0C2LL, 0xE9E8F0F0C3LL, 0xE9E8F0F0C4LL, 0xE9E8F0F0C5LL,
  0xE9E8F0F0C6LL, 0xE9E8F0F0C7LL, 0xE9E8F0F0C8LL, 0xE9E8F0F0C9LL, 0xE9E8F0F0C0LL
};
const int maxVehicleNumber = (sizeof(pipeOut) / (sizeof(uint64_t)));

#define BUTTON_LEFT 2 // - or channel select
#define BUTTON_RIGHT 9 // + or transmission mode select
#define BUTTON_SEL 3 // select button for menu
#define BUTTON_BACK 10 // back button for menu
#define BUTTON_LJY1 5 // LeftJoyStick button for menu
#define BUTTON_RJY1 4 // RightJoyStick button for menu

byte leftButtonState = 7; // init states with 7 (see macro below)!
byte rightButtonState = 7;
byte selButtonState = 7;
//byte backButtonState = 7;
byte LJYButtonState = 7;
byte RJYButtonState = 7;

// Macro for debounced rising edge detection
#define DRE(signal, state) (state=(state<<1)|(signal&1)&15)==7


struct Data_Package {

  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte pot1;
  byte pot2;
  byte D_Right_Indicator;
  byte D_Left_Indicator;
  byte D_Sound_Horn;
  byte D_Head_Light;
};

Data_Package data;
void Serial_Print_Data(void);
void drawTarget(int x, int y, int w, int h, int posX, int posY) {
  u8g2.drawFrame(x, y, w, h);
  u8g2.drawDisc((x + w / 2) - (w / 2) + (posX / 2), (y + h / 2) + (h / 2) - (posY / 2), 5, 5);
}
void draw(void) {


  u8g2.firstPage();
  do {
    u8g2.drawLine(0, 12, 128, 12);
    u8g2.setCursor(0, 3);
    u8g2.print("CH: ");
    u8g2.print(1);
    u8g2.setCursor(50, 3);
    u8g2.print("Bat: ");
    u8g2.print(12);
    u8g2.print("V");

    drawTarget(0, 14, 50, 50, data.j1PotY , data.j1PotX); // left joystick
    drawTarget(74, 14, 50, 50, data.j2PotY, data.j2PotX); // right joystick
    drawTarget(55, 14, 14, 50, 14, data.j1PotY); // potentiometer

  } while ( u8g2.nextPage() );

}

void setup() {
  Serial.begin(115200);
  radio.begin();
  u8g2.begin();
  radio.openWritingPipe(pipeOut[1]);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  //u8g2.setFontRefHeightExtendedText();
  // u8g2.setDefaultForegroundColor();
  u8g2.setFontPosTop();
  u8g2.setFont(u8g2_font_6x10_tr);

  data.j1PotY = 127;
  data.j2PotX = 127; // Values from 0 to 255. When Joystick is in resting position, the value is in the middle, or 127. We actually map the pot value from 0 to 1023 to 0 to 255 because that's one BYTE value
  data.D_Right_Indicator = 0;
  data.D_Left_Indicator = 0;
  data.D_Sound_Horn = 0;
  data.D_Head_Light = 0;
  
// Set the button pin as an input
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_SEL, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT_PULLUP);
  pinMode(BUTTON_LJY1, INPUT_PULLUP);
  pinMode(BUTTON_RJY1, INPUT_PULLUP);
}

void loop() {

  data.j2PotX = map(analogRead(A2), 0, 1023, 0, 100); // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
  data.j2PotY = map(analogRead(A3), 0, 1023, 0, 100);
  data.j1PotX = map(analogRead(A1), 0, 1023, 0, 100); // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
  data.j1PotY = map(analogRead(A0), 0, 1023, 0, 100);


  // Send the whole data from the structure to the receiver
  radio.write(&data, sizeof(Data_Package));
  draw();
  // Check for a debounced rising edge using the DRE macro
  if (DRE(digitalRead(BUTTON_LEFT), leftButtonState)) {
    Serial.println("LEFT_Button pressed!");
    // Add your code to handle the button press here
  }
  if (DRE(digitalRead(BUTTON_RIGHT), rightButtonState)) {
    Serial.println("RIGHT_Button pressed!");
    // Add your code to handle the button press here
  }
   if (DRE(digitalRead(BUTTON_SEL), selButtonState)) {
    Serial.println("SEL_Button pressed!");
    // Add your code to handle the button press here
  }
  //if (DRE(digitalRead(BUTTON_BACK), backButtonState)) {
   // Serial.println("BACK_Button pressed!");
    // Add your code to handle the button press here
 // }
  if (DRE(digitalRead(BUTTON_LJY1), LJYButtonState)) {
    Serial.println("LeftJoyStick_Button pressed!");
    // Add your code to handle the button press here
  }
  if (DRE(digitalRead(BUTTON_RJY1), RJYButtonState)) {
    Serial.println("RightJoyStick_Button pressed!");
    // Add your code to handle the button press here
  }
  
}
void Serial_Print_Data(void)
{

    Serial.print("J1-X : ");
    Serial.print(analogRead(A0));
    Serial.print("   ");
    Serial.print("J1-Y : ");
    Serial.print(analogRead(A1));
    Serial.print("   ");
    Serial.print("J2-X : ");
    Serial.print(analogRead(A2));
    Serial.print("   ");
    Serial.print("J2-Y : ");
    Serial.print(analogRead(A3));
    Serial.print("   ");
    Serial.println();
}
