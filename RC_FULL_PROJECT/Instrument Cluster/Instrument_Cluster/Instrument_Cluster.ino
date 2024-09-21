#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include "steeringCurves.h"

#include <SPI.h>


#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8

#define BLACK   0x0000
#define RED     0x001F
#define BLUE    0xF800
#define GREEN   0x07E0
#define YELLOW  0x07FF
#define PURPLE  0xF81F
#define CYAN    0xFFE0
#define WHITE   0xFFFF


int prev = 0;
unsigned long previousMillis = 0;  // Variable to store the previous time
const long interval = 500;  // Interval in milliseconds (1 second)
bool ledState = LOW;

boolean left;
boolean right;
boolean hazard;


struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  byte pot1;
  byte pot2;
};

RcData data;


// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

const unsigned char rightArrow [] PROGMEM = {
  0b00000001, 0b10000000, //        ##
  0b00000011, 0b11000000, //       ####
  0b00000011, 0b11100000, //       #####
  0b01111111, 0b11110000, //  ###########
  0b11111111, 0b11111000, // #############
  0b11111111, 0b10111100, // ######### ####
  0b11100000, 0b00011110, // ###        ####
  0b11100000, 0b00001111, // ###         ####
  0b11100000, 0b00001111, // ###         ####
  0b11100000, 0b00011110, // ###        ####
  0b11111111, 0b10111100, // ######### ####
  0b11111111, 0b11111000, // #############
  0b01111111, 0b11110000, //  ###########
  0b00000011, 0b11100000, //       #####
  0b00000011, 0b11000000, //       ####
  0b00000001, 0b10000000, //        ##
};
const unsigned char leftArrow [] PROGMEM = {
  0b00000001, 0b10000000, //        ##
  0b00000011, 0b11000000, //       ####
  0b00000111, 0b11000000, //      #####
  0b00001111, 0b11111110, //     ###########
  0b00011111, 0b11111111, //    #############
  0b00111101, 0b11111111, //   #### #########
  0b01111000, 0b00000111, //  ####        ###
  0b11110000, 0b00000111, // ####         ###
  0b11110000, 0b00000111, // ####         ###
  0b01111000, 0b00000111, //  ####        ###
  0b00111101, 0b11111111, //   #### #########
  0b00011111, 0b11111111, //    #############
  0b00001111, 0b11111110, //     ###########
  0b00000111, 0b11000000, //      #####
  0b00000011, 0b11000000, //       ####
  0b00000001, 0b10000000, //        ##
};

void led()
{


  if (data.axis4 < 5) left = true;
  if (data.axis4 >  55)left = false;
  if (data.axis4 > 95)right =  true;
  if (data.axis4 < 45) right =  false;

  static int steeringOld;

  if (data.axis1 > steeringOld + 10)
  {
    left =  false;
    steeringOld = data.axis1;
  }
  if (data.axis1 < steeringOld - 10)
  {
    right = false;
    steeringOld =  data.axis1;
  }

  if (left)
  {
    right =  false;
    unsigned long currentMillis = millis();
    //Serial.println("left");
    if (currentMillis - previousMillis >= interval) { // Check if it's time to change the LED state
      previousMillis = currentMillis;  // Save the current time as previous time

      // Toggle the LED state
      if (ledState == LOW) {
        tft.drawBitmap(10, 50, leftArrow, 16, 16, GREEN);
        //tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        //tft.drawBitmap(110, 50, rightArrow, 16, 16, GREEN);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
        ledState = LOW;
      }

    }
  }
  if (right)
  {
    left =  false;
    unsigned long currentMillis = millis();
    //Serial.println("left");
    if (currentMillis - previousMillis >= interval) { // Check if it's time to change the LED state
      previousMillis = currentMillis;  // Save the current time as previous time

      // Toggle the LED state
      if (ledState == LOW) {
        //tft.drawBitmap(10, 50, leftArrow, 16, 16, GREEN);
        tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, GREEN);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
        ledState = LOW;
      }

    }
  }

  if (data.axis3 > 55) hazard =  true;
  if (data.axis3 < 45) hazard =  false;

  if (hazard) { // Hazard lights
    if (left) {
      left = false;
      tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
    }
    if (right) {
      right = false;
      tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
    }
    unsigned long currentMillis = millis();
    //Serial.println("left");
    if (currentMillis - previousMillis >= interval) { // Check if it's time to change the LED state
      previousMillis = currentMillis;  // Save the current time as previous time

      // Toggle the LED state
      if (ledState == LOW) {
        tft.drawBitmap(10, 50, leftArrow, 16, 16, GREEN);
        //tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, GREEN);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
        ledState = LOW;
      }

    }
  }

  if (!hazard && !left && !right) {
    tft.drawBitmap(10, 50, leftArrow, 16, 16, BLACK);
    tft.drawBitmap(110, 50, rightArrow, 16, 16, BLACK);
  }


}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Use this initializer if you're using TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);
  delay(1000);

}

void loop() {
  if (Serial.available() >= sizeof(data)) {

    Serial.readBytes((char*)&data, sizeof(data));

    tft.invertDisplay(false);
    int rpm;

    uint16_t servo3Microseconds = 1500;
    static uint16_t servo3Microseconds2 = 1500;
    static long previousThrottleRampMillis;


    if (millis() - previousThrottleRampMillis >= 3) {
      previousThrottleRampMillis = millis();
      servo3Microseconds = map(data.axis1, 100, 0, 2000, 1000);
      servo3Microseconds = reMap(curveExponentialThrottle, servo3Microseconds);
      if (servo3Microseconds2 < servo3Microseconds) servo3Microseconds2 ++;
      if (servo3Microseconds2 > servo3Microseconds) servo3Microseconds2 --;
      rpm  = servo3Microseconds2 ;

    }
    rpm =  map(rpm, 1500, 2000, 1, 160);
    if (rpm < 0)
    {
      rpm = 0;
    }
    led();

    tft.setTextColor(RED);
    tft.setTextSize(3);
    tft.setCursor(40, 85);
    tft.println(rpm);


    if (rpm != prev)
    {
      tft.fillRect(30, 75, 70, 50, BLACK);
      prev =  rpm;
    }



  }
}
