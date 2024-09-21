// Include application, user and local libraries
#include "SPI.h"
#include "TFT_22_ILI9225.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include <ESP_Google_Sheet_Client.h>
// For SD/SD_MMC mounting helper
#include <GS_SDHelper.h>
Adafruit_INA219 ina219;

#ifdef ARDUINO_ARCH_STM32F1
#define TFT_RST PA1
#define TFT_RS PA2
#define TFT_CS PA0   // SS
#define TFT_SDI PA7  // MOSI
#define TFT_CLK PA5  // SCK
#define TFT_LED 0    // 0 if wired to +5V directly
#elif defined(ESP8266)
#define TFT_RST 4   // D2
#define TFT_RS 5    // D1
#define TFT_CLK 14  // D5 SCK
//#define TFT_SDO 12  // D6 MISO
#define TFT_SDI 13  // D7 MOSI
#define TFT_CS 15   // D8 SS
#define TFT_LED 2   // D4     set 0 if wired to +5V directly -> D3=0 is not possible !!
#elif defined(ESP32)
#define TFT_RST 26  // IO 26
#define TFT_RS 25   // IO 25
#define TFT_CLK 14  // HSPI-SCK
//#define TFT_SDO 12  // HSPI-MISO
#define TFT_SDI 13  // HSPI-MOSI
#define TFT_CS 15   // HSPI-SS0
#define TFT_LED 0   // 0 if wired to +5V directly
SPIClass hspi(HSPI);
#else
#define TFT_RST 8
#define TFT_RS 9
#define TFT_CS 10   // SS
#define TFT_SDI 11  // MOSI
#define TFT_CLK 13  // SCK
#define TFT_LED 3   // 0 if wired to +5V directly
#endif

#define TFT_BRIGHTNESS 200  // Initial brightness of TFT backlight (optional)

#define WIFI_SSID "PRABHAGARAN"
#define WIFI_PASSWORD "8754615471"

// Google Project ID
#define PROJECT_ID "iot-datalogging-429706"

// Service Account's client email
#define CLIENT_EMAIL "iot-datalogging@iot-datalogging-429706.iam.gserviceaccount.com"

// Service Account's private key
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\nMIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCw/sBl4eLX3piJ\nBHLhHOn5tBeuE0uqfIK6JA2qKZLNtx7tjpP1XPvHh0vzzkg5wyMqB/6IM4Pk7uBi\nHmkgfQjXpto7HRegBawHyZM3SQGjKGCwWUACvsGXeR/pVoCXpachVoRuWTTQudhW\nB5SWXsrxhK+P9J4DP8Fc02J9md4UGzfcvXyRE+6Dfs46GjWwhd9EuSpcrAsJrFkb\nZ+dR4USzs2kJmcJM9qKQBxoN41TdajuLxtvrQTQa2C3I1FFZdZiH4Vr5/x+A54bw\nZ9rMMIAUJao7MEnel8gJAgHfrEUjpQgSmak6Ek3iKDpvD5sz7fvKIwXaP4PifX3l\nX15vpmzjAgMBAAECggEABluR3Ko30RCnwp0Cy8kTZhezSNqPlSQPON5GEWH+tt37\njri+4glN6ZWXqfFm3cN/j8LYWg5QdnI8/blHuTxUi3tTJNylwxoN2y/BI0y/i5u0\nZJ4Of93xq6rudWq/VSZecc+fE5NBVus+Dp6lSZ6sRCJ25433xd60ThnK7jKo0mic\nHbnLtnr0WNul3s6IA330HdaNv3PBov60F+yDwWNUtL7Ml0wTKjR4O0tL7TEcjQZB\n+iegtd5QCx3fqxDQye75XV8dDzxetmo8FnsudUbhM5m8PDX+n+RHoErcClD/Y1F1\nM0MQ+eeQ9XRew3Kkqqdpu6h39MNhACoajAoLO2hlMQKBgQDqxgUsmiZuB5U4WoWx\nnZV/YK6P/lW/iDoUq9H3fVoKRcPuLAzpNoODUqY7MQB5rLRC7NcogUOygki436f2\nkBdMVl68ETr9uZduA7GAaXnvoKSqdObqlGU2nu87eyIJyspil0ySlOMUHoQeTqQo\nr+qglMjFjlHRtUq2E6Slcg68sQKBgQDA/2oQqnX2/pq9vqbLQcID2USTxx/sRVOz\nMO+chO1la2PmYijInWOv2s68Hp6Gs4VfSXjSsJ5aEDvlxkR1Mh3xEd4NJqlSGKOi\n1c2DBI5tpMMUmC+NTUvcSg9YjPIunHTvEXY75OIuyhjx8SLTK3lWMsZrcZguhf1G\nqtTvAp0X0wKBgQDWD4VqPIcrrbhN8KOT5hYIVTP00LaIc4JZlKxWVzIAycmy4PFI\nts7Er1goAi7nwiN+HgqzwODj40zBXBP+iTTlAp9QJPm/nDfJNI7kgcePSM67KOnf\nGz0Jx3JjrorDmOZdbIyB3kUJ9CUMqTGec3+fuOkyz+gkKIOdbl2iCFQNkQKBgGzo\nNHwPFN46Mivch9au1sLBOfeCKDt1q5O0i6HxSN0wSoFY5ta+KeC0QnDZfm9Yomxw\n02NhExqTiplQ1pSjoU3F3V3icS3IhZ5/s6a5TX4FuafHcR31fldi22IGtysiUsIN\nQoDt2cy5cnYkspgjMic4I8vBoDSwm4njIKF3AhrNAoGAXwHlj2CRyEJ02zX0dWEl\nKkU0eqJEMGp0mVkgibcPgA8LhODrYa8oJvZI6EAF8qpV5nx5Pq26yGjNCsu6DxLl\n4qr/RCfaDMLRv9AhKgUgSHJhZVzYqLJY6xL+diMU5eYoHrMAuiPv2hHTnhcsRsQy\naj7+gKTne7m7TSozp3JDDfE=\n-----END PRIVATE KEY-----\n";

// The ID of the spreadsheet where you'll publish the data
const char spreadsheetId[] = "1PPHmLmTEkduSNwbR7XF5oCAKJmo1ogJPbKO44VjhIqA";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

// Token Callback function
void tokenStatusCallback(TokenInfo info);

#define DRE(signal, state) (state = (state << 1) | (signal & 1) & 15) == 7
#define DEBUG
#define SEL_BUTTON 33   // select buttom similar to ok
#define BACK_BUTTON 32  // back button
#define INC_BUTTON 4    // Increment button
#define DEC_BUTTON 27   // Decrement button


uint8_t buttonState1 = 0;
uint8_t buttonState2 = 0;
uint8_t buttonState3 = 0;
uint8_t buttonState4 = 0;

bool menuActive = false;
uint8_t menuRow = 1;
int previousValue = 1;
uint8_t activeScreen = 0;
int prev_ = -1;
int prevVoltage = -1;

bool md1 = 0;
bool md2 = 0;
bool md3 = 0;
const int numSamples = 50;

// NTP server to request epoch time
const char* ntpServer = "in.pool.ntp.org";

// Variable to save current epoch time
unsigned long epochTime;

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}


// Use hardware SPI (faster - on Uno: 13-SCK, 12-MISO, 11-MOSI)
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);
// Use software SPI (slower)
//TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK, TFT_LED, TFT_BRIGHTNESS);

// Variables and constants
uint16_t x, y;
boolean flag = false;

const char* getLabelByScreen(int screen = 0) {
  switch (screen == 0 ? activeScreen : screen) {
    case 1: return "  MODE 1 90    ";
    case 2: return "  MODE 2 45 L  ";
    case 3: return "  MODE 3 45 R  ";
    default: return "";
  }
}

// Setup
void setup() {
#if defined(ESP32)
  hspi.begin();
  tft.begin(hspi);
#else
  tft.begin();
#endif
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  // Initialize the INA219.
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range, call:
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage, current, and power with INA219 ...");

  //Configure time
  configTime(0, 0, ntpServer);
  // Initialize BME280 sensor

  GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

  // Connect to Wi-Fi
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Set the callback for Google API access token generation status (for debug only)
  GSheet.setTokenCallback(tokenStatusCallback);

  // Set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
  GSheet.setPrerefreshSeconds(10 * 60);

  // Begin the access token generation for Google API authentication
  GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);

  pinMode(SEL_BUTTON, INPUT_PULLUP);
  pinMode(BACK_BUTTON, INPUT_PULLUP);
  pinMode(INC_BUTTON, INPUT_PULLUP);
  pinMode(DEC_BUTTON, INPUT_PULLUP);

  tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_BLUEVIOLET);
}

// Loop
void loop() {

  bool ready = GSheet.ready();
  handleButtonPress();
}
void handleButtonPress() {
  if (DRE(digitalRead(SEL_BUTTON), buttonState1)) {
#ifdef DEBUG
    Serial.println("Menu ButtonPressed");
#endif
    menuActive = 1;
    menuRow = 1;
    menu();
    tft.clear();
    draw(2);
  }
}

void menu() {
  while (menuActive) {
    static int prev;

    if (DRE(digitalRead(SEL_BUTTON), buttonState1)) { //this button will read data
      modes(menuRow);
    }
    if (DRE(digitalRead(INC_BUTTON), buttonState3)) {
      previousValue = menuRow;
      menuRow = constrain(menuRow + 1, 1, 3);
#ifdef DEBUG
      Serial.println(menuRow);
#endif
      //draw();
    } else if (DRE(digitalRead(DEC_BUTTON), buttonState4)) {
      previousValue = menuRow;
      menuRow = constrain(menuRow - 1, 1, 3);
#ifdef DEBUG
      Serial.println(menuRow);
#endif
    }
    if (menuRow != prev) {
      draw(1);
      prev = menuRow;
    }
    if (DRE(digitalRead(BACK_BUTTON), buttonState2)) {
      menuActive = 0;
      menuRow = 0;


#ifdef DEBUG
      Serial.println("BackButtonPressed");
#endif
    }
  }
}
void modes(int mode_) {
  switch (mode_) {
    case 1:
      md1 = 1;
      tft.fillRectangle(5, 5, 215, 170, COLOR_BLACK);
      tft.drawText(7, 10, getLabelByScreen(mode_), COLOR_BLUEVIOLET);
      tft.setFont(Terminal6x8);
      tft.drawText(100, 120, "VOLTAGE: ", COLOR_LIGHTGRAY);
      tft.drawText(100, 120 + 24, "CURRENT: ", COLOR_LIGHTGRAY);
      mode1();
      break;
    case 2:
      md1 = 1;
      tft.fillRectangle(5, 5, 215, 170, COLOR_BLACK);
      tft.drawText(7, 10, getLabelByScreen(mode_), COLOR_BLUEVIOLET);
      tft.setFont(Terminal6x8);
      tft.drawText(100, 120, "VOLTAGE: ", COLOR_LIGHTGRAY);
      tft.drawText(100, 120 + 24, "CURRENT: ", COLOR_LIGHTGRAY);
      mode1();
      break;
    case 3:
      md1 = 1;
      tft.fillRectangle(5, 5, 215, 170, COLOR_BLACK);
      tft.drawText(7, 10, getLabelByScreen(mode_), COLOR_BLUEVIOLET);
      tft.setFont(Terminal6x8);
      tft.drawText(100, 120, "VOLTAGE: ", COLOR_LIGHTGRAY);
      tft.drawText(100, 120 + 24, "CURRENT: ", COLOR_LIGHTGRAY);
      mode1();
      break;
    default:
      break;
  }
}

void mode1() {
  while (md1) {

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;


    shuntvoltage = ina219.getShuntVoltage_mV();

    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    for (int i = 0; i <= numSamples; i++) {

      busvoltage += ina219.getBusVoltage_V();
      current_mA += ina219.getCurrent_mA();
    }

    busvoltage /= numSamples;
    current_mA /= numSamples;

    float sensvalue = current_mA;
    float voltage = busvoltage;
    int scaledsensorvalue = current_mA * 10;

#ifdef DEBUG
    //Serial.println("entered in mode 1");
#endif

    ///draw(3);

    if (scaledsensorvalue != prev_) {
      Serial.print(scaledsensorvalue);
      Serial.println(prev_);

      char buffer[10];
      char voltagebuffer[10];
      int xPosition = 160;
      int yPosition = 113;

      sprintf(buffer, "%.2f", current_mA);
      sprintf(voltagebuffer, "%.2f", busvoltage);
      //tft.fillRectangle(10, 60, 210, 100, COLOR_BLACK);

      if (prev_ >= 0) {
        char prevBuffer[20];
        char prevvoltagebuffer[20];
        float prevFloatValue = prev_ / 100.0;
        float prevfloatvoltagevalue = prevVoltage /100.0;
        sprintf(prevBuffer, "%0.2f", prevFloatValue);
        sprintf(prevvoltagebuffer, "%0.2f", prevfloatvoltagevalue);
        tft.setFont(Terminal12x16);
        tft.drawText(xPosition, yPosition, prevvoltagebuffer, COLOR_BLACK);
        tft.drawText(100 + 60, 120 + 17, prevBuffer, COLOR_BLACK);
      }
      tft.setFont(Terminal12x16);
      tft.drawText(xPosition, yPosition, voltagebuffer, COLOR_YELLOWGREEN);
      tft.drawText(100 + 60, 120 + 17, buffer, COLOR_YELLOWGREEN);
      prev_ = scaledsensorvalue;
      prevVoltage = busvoltage;
    }
    if (DRE(digitalRead(SEL_BUTTON), buttonState1)) {
      lastTime = millis();

      FirebaseJson response;

      Serial.println("\nAppend spreadsheet values...");
      Serial.println("----------------------------");

      FirebaseJson valueRange;

      // New BME280 sensor readings
      //temp = 1;
      //temp = 1.8*bme.readTemperature() + 32;
      //hum = 1;
      //pres = 1;
      // Get timestamp
      epochTime = getTime();

      valueRange.add("majorDimension", "COLUMNS");
      valueRange.set("values/[0]/[0]", sensvalue);
      valueRange.set("values/[1]/[0]",  voltage);

      // For Google Sheet API ref doc, go to https://developers.google.com/sheets/api/reference/rest/v4/spreadsheets.values/append
      // Append values to the spreadsheet
      if (menuRow == 1) {
        bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet1!A1" /* range to append */, &valueRange /* data range to append */);
        if (success) {
          response.toString(Serial, true);
          valueRange.clear();
        } else {
          Serial.println(GSheet.errorReason());
        }
      } else if (menuRow == 2) {
        bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet2!A1" /* range to append */, &valueRange /* data range to append */);
        if (success) {
          response.toString(Serial, true);
          valueRange.clear();
        } else {
          Serial.println(GSheet.errorReason());
        }
      } else if (menuRow == 3) {
        bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet3!A1" /* range to append */, &valueRange /* data range to append */);
        if (success) {
          response.toString(Serial, true);
          valueRange.clear();
        } else {
          Serial.println(GSheet.errorReason());
        }
      }

      Serial.println();
      Serial.println(ESP.getFreeHeap());
    }
    if (DRE(digitalRead(BACK_BUTTON), buttonState2)) {
      md1 = 0;
      prev_ = -1;
      draw(2);
      draw(1);
      menu();

#ifdef DEBUG
      //Serial.println("exited from mode 1");
#endif
    }
  }
}


void draw(int value) {
  static int prevm;
  switch (value) {
    case 1:

      for (int i = 1; i <= 3; ++i) {
        tft.setOrientation(1);
        tft.setFont(Terminal11x16);
        tft.setBackgroundColor(COLOR_BLACK);
        tft.drawText(40, 50 + (i * 20), getLabelByScreen(i), COLOR_WHITE);
        tft.setBackgroundColor(COLOR_LIGHTGRAY);
        tft.drawText(40, 50 + (menuRow * 20), getLabelByScreen(menuRow), COLOR_BLUEVIOLET);
        tft.setBackgroundColor(COLOR_BLACK);
        tft.drawText(10, 50 + (previousValue * 20), "=>", COLOR_BLACK);
        tft.drawText(10, 50 + (menuRow * 20), "=>", COLOR_BLUEVIOLET);
      }
      break;
    case 2:

      tft.setFont(Terminal11x16);
      tft.fillRectangle(5, 5, 215, 170, COLOR_BLACK);
      tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_BLUEVIOLET);
      tft.drawText(40, 30, "ANIMOTRONEX", COLOR_BLUEVIOLET);
      break;
    case 3:
      tft.fillRectangle(10, 60, 210, 100, COLOR_RED);
      break;
    default:
      tft.clear();
      break;
  }
}
void tokenStatusCallback(TokenInfo info) {
  if (info.status == token_status_error) {
    GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
    GSheet.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
  } else {
    GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
  }
}