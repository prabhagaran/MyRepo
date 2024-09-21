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
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0; // Variable to store the previous time
const long interval = 500;  // Interval in milliseconds (1 second)
bool ledState = LOW;
bool ledState1 = LOW;
int fuelLevel = 0;

boolean left;
boolean right;
boolean hazard;
float p = 3.1415926;

int rpm;
const int GAUGE_RADIUS = 50; // Radius of the gauge
const int GAUGE_CENTER_X = 64; // X coordinate of the center of the gauge
const int GAUGE_CENTER_Y = 64; // Y coordinate of the center of the gauge


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
const unsigned char fillstate1 [] PROGMEM = {
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b01111111, 0b11111110, //  ##############
  0b01111111, 0b11111110, //  ##############
  0b01111111, 0b11111110, //  ##############
};
const unsigned char warning[]PROGMEM =
{
  0b00000000, 0b10000000, //         #
  0b00000001, 0b11000000, //        ###
  0b00000001, 0b11000000, //        ###
  0b00000011, 0b11100000, //       #####
  0b00000011, 0b01100000, //       ## ##
  0b00000111, 0b01110000, //      ### ###
  0b00000110, 0b00110000, //      ##   ##
  0b00001110, 0b10111000, //     ### # ###
  0b00001100, 0b10011000, //     ##  #  ##
  0b00011100, 0b10011100, //    ###  #  ###
  0b00011000, 0b10001100, //    ##   #   ##
  0b00111000, 0b00001110, //   ###       ###
  0b00110000, 0b10000110, //   ##    #    ##
  0b01111111, 0b11111111, //  ###############
  0b01111111, 0b11111111, //  ###############
  0b00000000, 0b00000000, //
};

const unsigned char humidity_icon16x16[] PROGMEM =
{
  0b00000000, 0b00000000, //
  0b00000001, 0b10000000, //        ##
  0b00000011, 0b11000000, //       ####
  0b00000111, 0b11100000, //      ######
  0b00001110, 0b01110000, //     ###  ###
  0b00001100, 0b00110000, //     ##    ##
  0b00011100, 0b00111000, //    ###    ###
  0b00011000, 0b00011000, //    ##      ##
  0b00111000, 0b00011100, //   ###      ###
  0b00111000, 0b00011100, //   ###      ###
  0b00111000, 0b00011100, //   ###      ###
  0b00011100, 0b00111000, //    ###    ###
  0b00011111, 0b11111000, //    ##########
  0b00001111, 0b11110000, //     ########
  0b00000011, 0b11000000, //       ####
  0b00000000, 0b00000000, //
};

const unsigned char tool_icon16x16[]PROGMEM =
{
  0b00000000, 0b00000000, //                 
  0b00000000, 0b01100000, //          ##     
  0b00000000, 0b11100000, //         ###     
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b11100110, //        ####  ## 
  0b00000011, 0b11111110, //       ######### 
  0b00000111, 0b11111100, //      #########  
  0b00001111, 0b11111000, //     #########   
  0b00011111, 0b11000000, //    #######      
  0b00111111, 0b10000000, //   #######       
  0b01111111, 0b00000000, //  #######        
  0b11111110, 0b00000000, // #######         
  0b11111100, 0b00000000, // ######          
  0b11111000, 0b00000000, // #####           
  0b01110000, 0b00000000, //  ###            
};

const unsigned char water_tap_icon16x16[]PROGMEM =
{
  0b00000001, 0b10000000, //        ##       
  0b00000111, 0b11100000, //      ######     
  0b00000001, 0b10000000, //        ##       
  0b00001111, 0b11110000, //     ########    
  0b11111111, 0b11111110, // ############### 
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11111111, // ################
  0b11111111, 0b11111111, // ################
  0b00000000, 0b00001111, //             ####
  0b00000000, 0b00001111, //             ####
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00001100, //             ##  
  0b00000000, 0b00001100, //             ##  
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00001100, //             ##  
  0b00000000, 0b00001100, //             ##  
};


 const unsigned char bulb_icon16x16[]PROGMEM  =
{
  0b00000000, 0b00000000, //                 
  0b00000011, 0b11100000, //       #####     
  0b00000100, 0b00010000, //      #     #    
  0b00001000, 0b00001000, //     #       #   
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00001000, 0b00001000, //     #       #   
  0b00000100, 0b00010000, //      #     #    
  0b00000011, 0b11100000, //       #####     
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
};

 const unsigned char bulb_on_icon16x16[]PROGMEM  =
{
  0b00000000, 0b00000000, //                 
  0b00100011, 0b11100010, //   #   #####   # 
  0b00010100, 0b00010100, //    # #     # #  
  0b00001000, 0b00001000, //     #       #   
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00010000, 0b00000100, //    #         #  
  0b00001000, 0b00001000, //     #       #   
  0b00010100, 0b00010100, //    # #     # #  
  0b00100011, 0b11100010, //   #   #####   # 
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
};

 const unsigned char bulb_off_icon16x16[]PROGMEM  =
{
  0b00000000, 0b00000000, //                 
  0b00000011, 0b11100000, //       #####     
  0b00000111, 0b11110000, //      #######    
  0b00001111, 0b11111000, //     #########   
  0b00011111, 0b11111100, //    ###########  
  0b00011111, 0b11111100, //    ###########  
  0b00011111, 0b11111100, //    ###########  
  0b00011111, 0b11111100, //    ###########  
  0b00011111, 0b11111100, //    ###########  
  0b00001111, 0b11111000, //     #########   
  0b00000100, 0b00010000, //      #     #    
  0b00000011, 0b11100000, //       #####     
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
  0b00000010, 0b00100000, //       #   #     
  0b00000011, 0b11100000, //       #####     
};

 const unsigned char temperature_icon16x16[]PROGMEM =
{
  0b00000001, 0b11000000, //        ###      
  0b00000011, 0b11100000, //       #####     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00001111, 0b11110000, //     ########    
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00001111, 0b11110000, //     ########    
  0b00000111, 0b11100000, //      ######     
};


void led()
{
  if (data.pot1 < 50)
  {
   tft.drawBitmap(60, 7, bulb_on_icon16x16, 16, 16, YELLOW);
   tft.drawBitmap(40, 7, bulb_off_icon16x16, 16, 16, YELLOW);
   tft.drawBitmap(20, 7, bulb_icon16x16, 16, 16, GREEN);
  }else if (data.pot1 >= 50 && data.pot1 <= 75)
  {
   tft.drawBitmap(60, 7, bulb_on_icon16x16, 16, 16, YELLOW);
   tft.drawBitmap(40, 7, bulb_off_icon16x16, 16, 16, GREEN);
   tft.drawBitmap(20, 7, bulb_icon16x16, 16, 16, YELLOW);
    
  }
  else if (data.pot1 >= 75)
  {
    tft.drawBitmap(60, 7, bulb_on_icon16x16, 16, 16, GREEN);
   tft.drawBitmap(40, 7, bulb_off_icon16x16, 16, 16, YELLOW);
   tft.drawBitmap(20, 7, bulb_icon16x16, 16, 16, YELLOW);

  }

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
        tft.drawBitmap(10, 30, leftArrow, 16, 16, GREEN);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, BLACK);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, BLACK);
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
        tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, GREEN);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, BLACK);
        ledState = LOW;
      }

    }
  }

  if (data.axis3 > 55) hazard =  true;
  if (data.axis3 < 45) hazard =  false;

  if (hazard) { // Hazard lights
    if (left) {
      left = false;
      tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
    }
    if (right) {
      right = false;
      tft.drawBitmap(110, 30, rightArrow, 16, 16, BLACK);
    }
    unsigned long currentMillis = millis();
    //Serial.println("left");
    if (currentMillis - previousMillis >= interval) { // Check if it's time to change the LED state
      previousMillis = currentMillis;  // Save the current time as previous time

      // Toggle the LED state
      if (ledState == LOW) {
        tft.drawBitmap(10, 30, leftArrow, 16, 16, GREEN);
        //tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, GREEN);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 30, leftArrow, 16, 16, BLACK);
        tft.drawBitmap(110, 30, rightArrow, 16, 16, BLACK);
        ledState = LOW;
      }

    }
  }

  if (!hazard && !left && !right) {
    tft.drawBitmap(10, 30, leftArrow, 16, 16, YELLOW);
    tft.drawBitmap(110, 30, rightArrow, 16, 16, YELLOW);
  }


  if (fuelLevel < 10)
  {
    unsigned long currentMillis = millis();
    //Serial.println("left");
    if (currentMillis - previousMillis1 >= interval) { // Check if it's time to change the LED state
      previousMillis1 = currentMillis;  // Save the current time as previous time

      // Toggle the LED state
      if (ledState1 == LOW) {
        tft.setTextColor(BLACK);
        tft.setTextSize(1);
        tft.setCursor(43, 35);

        tft.println("LOW FUEL");
        tft.drawBitmap(100, 100, warning, 16, 16, BLACK);
        ledState1 = HIGH;
      } else {
        tft.setTextColor(YELLOW);
        tft.setTextSize(1);
        tft.setCursor(43, 35);

        tft.println("LOW FUEL");
        tft.drawBitmap(100, 100, warning, 16, 16, RED);

        ledState1 = LOW;
      }

    }
  }
  else
  {
    tft.setTextColor(BLACK);
    tft.setTextSize(1);
    tft.setCursor(43, 35);
    tft.println("LOW FUEL");
    tft.drawBitmap(100, 100, warning, 16, 16, YELLOW);

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

  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(43, 73);
  tft.println("KM/h");

  tft.drawLine(40, 32, 40, 52, WHITE );
  tft.drawLine(40, 52, 100, 52, WHITE );
  tft.drawLine(100, 52, 100, 32, WHITE );
  tft.drawLine(100, 32, 40, 32, WHITE );

  tft.drawLine(35, 90, 35, 120, WHITE );
  tft.drawLine(35, 120, 95, 120, WHITE );
  tft.drawLine(95, 120, 95, 90, WHITE );
  tft.drawLine(95, 90, 35, 90, WHITE );

  tft.drawBitmap(117, 7, humidity_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(100, 7, tool_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(80, 7, water_tap_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(60, 7, bulb_on_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(40, 7, bulb_off_icon16x16, 16, 16, YELLOW);
   tft.drawBitmap(20, 7, bulb_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(117, 60, temperature_icon16x16, 16, 16, YELLOW);
  tft.drawBitmap(100, 100, warning, 16, 16, YELLOW);
  tft.drawBitmap(10, 30, leftArrow, 16, 16, YELLOW);
  tft.drawBitmap(110, 30, rightArrow, 16, 16, YELLOW);

}

void loop() {
  if (Serial.available() >= sizeof(data)) {

    Serial.readBytes((char*)&data, sizeof(data));

    tft.invertDisplay(false);


    uint16_t servo3Microseconds = 1500;
    static uint16_t servo3Microseconds2 = 1500;
    static long previousThrottleRampMillis;


    if (millis() - previousThrottleRampMillis >= 1) {
      previousThrottleRampMillis = millis();
      servo3Microseconds = map(data.axis1, 100, 0, 2000, 1000);
      servo3Microseconds = reMap(curveExponentialThrottle, servo3Microseconds);
      if (servo3Microseconds2 < servo3Microseconds) servo3Microseconds2 ++;
      if (servo3Microseconds2 > servo3Microseconds) servo3Microseconds2 --;
    }
    led();
    //displaySpeed(rpm);
    int rpm = map(servo3Microseconds2, 1500, 2000, 0, 160);
    rpm = max(rpm, 0);

    // Update display only if rpm changed
    static int prevRpm = -1;
    if (rpm != prevRpm) {
      tft.fillRect(41, 92, 48, 25, BLACK);
      prevRpm = rpm;
      tft.setTextColor(RED);
      tft.setTextSize(2);
      tft.setCursor(50, 98);
      tft.println(rpm);
      // Update fuel gauge here
    }

    fuelLevel = map(analogRead(A0), 0, 1023, 0, 100); // Assuming fuel sensor range is 0-100%
    drawFuelGauge(fuelLevel);
  }
}

void drawFuelGauge(int fuelLevel) {
  // Clear previous gauge
  tft.fillRect(10, 110, 16, 70, BLACK);

  // Draw fuel gauge bars based on fuel level
  int barHeight = map(fuelLevel, 0, 100, 0, 70);
  for (int i = 0; i < 10; i++) {
    int color = (i * 10 < fuelLevel) ? GREEN : BLACK;
    tft.drawBitmap(10, 110 - (i * 7), fillstate1, 16, 16, color);
  }
}
