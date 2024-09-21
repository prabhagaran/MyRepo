#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
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

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

unsigned long previousMillis = 0;
const long interval = 500;
bool ledState = LOW;
bool left = false;
bool right = false;
bool hazard = false;

struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  byte pot1;
  byte pot2;
};

RcData data;

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


void led() {
  if (data.axis4 < 5) left = true;
  if (data.axis4 > 55) left = false;
  if (data.axis4 > 95) right = true;
  if (data.axis4 < 45) right = false;

  if (left || right || hazard) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        tft.drawBitmap(10, 50, left ? leftArrow : fillstate1, 16, 16, left ? GREEN : BLACK);
        tft.drawBitmap(110, 50, right ? rightArrow : fillstate1, 16, 16, right ? GREEN : BLACK);
        ledState = HIGH;
      } else {
        tft.drawBitmap(10, 50, fillstate1, 16, 16, BLACK);
        tft.drawBitmap(110, 50, fillstate1, 16, 16, BLACK);
        ledState = LOW;
      }
    }
  } else {
    tft.drawBitmap(10, 50, fillstate1, 16, 16, BLACK);
    tft.drawBitmap(110, 50, fillstate1, 16, 16, BLACK);
  }
}

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(BLUE);
  tft.setTextSize(1);
  tft.setCursor(100, 105);
  tft.println("km/H");
  tft.drawLine(40, 32, 40, 52, RED);
  tft.drawLine(40, 52, 88, 52, RED);
  tft.drawLine(88, 52, 88, 32, RED);
  tft.drawLine(88, 32, 40, 32, RED);
}

void loop() {
  if (Serial.available() >= sizeof(data)) {
    Serial.readBytes((char*)&data, sizeof(data));
    tft.invertDisplay(false);
    led();
    uint16_t servo3Microseconds = map(data.axis1, 100, 0, 2000, 1000);
    servo3Microseconds = reMap(curveExponentialThrottle, servo3Microseconds);
    int rpm = map(servo3Microseconds, 1500, 2000, 0, 160);
    rpm = max(rpm, 0);

    // Update display only if rpm changed
    static int prevRpm = -1;
    if (rpm != prevRpm) {
      tft.fillRect(45, 90, 50, 30, BLACK);
      prevRpm = rpm;
      tft.setTextColor(RED);
      tft.setTextSize(2);
      tft.setCursor(50, 95);
      tft.println(rpm);
    }

    // Read analog input for fuel sensor
    int fuelLevel = map(analogRead(A0), 0, 1023, 0, 100); // Assuming fuel sensor range is 0-100%
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
