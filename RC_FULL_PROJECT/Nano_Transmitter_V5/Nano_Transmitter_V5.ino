/*  Two Way Master Version 1.01
    Author: Schindler Electronics
    Data: 5/23/2019

    This code demonstrates how to use the NRF module in both
    "transmit" and "receive" mode using the acknowledge method.
    One device will be set up as the Master, while the other
    will be set up as the slave.

    When the master sends out a message, the slave will receive.
    When this happens, the slave will immediately send back an
    acknowledge packet. The Master will receive this data and send
    another data packet.

    Note: This code requires another Arduino and NRF

    Note: This code requires the use of the RF24 Library.
    You can easily download this library in the Arduino Library Manager.
*/

#include <SPI.h>   //Comes with Arduino IDE
#include "RF24.h"  //Download and Install (See above)
#include <U8g2lib.h>
#include "transmitterConfig.h"

#define  CE_PIN  7 //The pins to be used for CE and CSN
#define  CSN_PIN 8

byte transmissionMode = 1; // Radio mode is active by default
byte operationMode = 0; // Start in transmitter mode (0 = transmitter mode, 1 = tester mode, 2 = game mode)
int vehicleNumber = 1; // Vehicle number one is active by default


const byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'}; //These are the names for all of the "Pipes" on the NRF Module
// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 20 ID's are available @ the moment

const uint64_t pipeOut[] PROGMEM = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL,
  0xE9E8F0F0C1LL, 0xE9E8F0F0C2LL, 0xE9E8F0F0C3LL, 0xE9E8F0F0C4LL, 0xE9E8F0F0C5LL,
  0xE9E8F0F0C6LL, 0xE9E8F0F0C7LL, 0xE9E8F0F0C8LL, 0xE9E8F0F0C9LL, 0xE9E8F0F0C0LL
};

const int maxVehicleNumber = (sizeof(pipeOut) / (sizeof(uint64_t)));

uint64_t pgm_read_64(const void *ptr, uint8_t index) {
  uint64_t result;
  memcpy_P( &result, (uint8_t*)ptr + (index * 8), sizeof(uint64_t) ); // ptr is counting in bytes!
  return result;
}

RF24 radio(CE_PIN, CSN_PIN);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


#define BUTTON_LEFT   2     // - or channel select
#define BUTTON_RIGHT  9   // + or transmission mode select
#define BUTTON_SEL    3    // select button for menu
#define BUTTON_LJY1   5  // LeftJoyStick button for menu
#define BUTTON_RJY1   4 // RightJoyStick button for menu

byte leftButtonState = 7; // init states with 7 (see macro below)!
byte rightButtonState = 7;
byte selButtonState = 7;
byte LJYButtonState = 7;
byte RJYButtonState = 7;

// The size of this struct should not exceed 32 bytes
struct RcData {
  int axis1; // Aileron (Steering for car)
  int axis2; // Elevator
  int axis3; // Throttle
  int axis4; // Rudder
};
RcData data;


// Joysticks
#define JOYSTICK_1 A1
#define JOYSTICK_2 A0
#define JOYSTICK_3 A3
#define JOYSTICK_4 A2

boolean displayLocked = true;              //This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct1 {     //This is the NRF data sent out by the Slave. Max of 32 bytes
  float vcc; // vehicle vcc voltage
  float batteryVoltage; // vehicle battery voltage
  boolean batteryOk = true; // the vehicle battery voltage is OK!
  byte channel = 1; // the channel number
} myTele;

// Smoothing constants
const float alpha = 0.2; // Adjust this value for the desired smoothing level (0.0 to 1.0)

// Variables to store raw, smoothed, and neutral joystick values
int rawValues[4];
int smoothedValues[4];
int offsetValues[4];

void calibrate() {
  // Read and store average joystick values as offsets
  int sum[4] = {0, 0, 0, 0};
  int samples = 200; // Increase the number of samples for calibration

  for (int i = 0; i < samples; i++) {
    sum[0] += analogRead(JOYSTICK_1);
    sum[1] += analogRead(JOYSTICK_2);
    sum[2] += analogRead(JOYSTICK_3);
    sum[3] += analogRead(JOYSTICK_4);
    delay(10);
  }

  offsetValues[0] = sum[0] / samples;
  offsetValues[1] = sum[1] / samples;
  offsetValues[2] = sum[2] / samples;
  offsetValues[3] = sum[3] / samples;

  Serial.println("Calibration complete!");
}

void setupRadio()
{
  radio.begin();                    //Initialize the nRF24L01 Radio
  radio.setChannel(108);            //Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS);  //Fast enough.. Better range
  radio.setPALevel(RF24_PA_MAX);//PA (Power Amplifier) Level can be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setAutoAck(pgm_read_64(&pipeOut, vehicleNumber - 1), true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(7, 5);  //delay, re-try count
  radio.openWritingPipe(pgm_read_64(&pipeOut, vehicleNumber - 1));
  radio.write( &data, sizeof(data));

}

void readJoystick()
{

  // Read raw joystick values
  rawValues[0] = analogRead(JOYSTICK_1);
  rawValues[1] = analogRead(JOYSTICK_2);
  rawValues[2] = analogRead(JOYSTICK_3);
  rawValues[3] = analogRead(JOYSTICK_4);

  // Apply smoothing
  for (int i = 0; i < 4; i++) {
    smoothedValues[i] = alpha * (rawValues[i] - offsetValues[i]) + (1 - alpha) * smoothedValues[i];
  }

  // Map the smoothed values to the range 1 to 100
  int mappedValues[4];
  for (int i = 0; i < 4; i++) {
    mappedValues[i] = map(smoothedValues[i], 0, 1023, 1, 100);
    if (mappedValues[i] == -1 )
    {
      mappedValues[i] = 0;
    }
  }

  //readJoysticks();
  data.axis1  = map(mappedValues[0] - 14, 100, 0, 2000, 1000);  //read values from the Joysticks
  data.axis2  = map(mappedValues[1] - 14, 100, 0, 2000, 1000);

  data.axis3 = map(mappedValues[2] - 14, 0, 100, 1590, 1290); // 45 - 135°
  data.axis4 = map(mappedValues[3] - 15, 100, 0, 2000, 1000);

}

void transmitRadio()
{
  if (radio.write( &data, sizeof(data) )) {  //Transmit data to the Slave
    if ( radio.isAckPayloadAvailable() ) {       //If an Acknowledge is received, read the data
      radio.read(&myTele, sizeof(myTele));
    }
  }
}
void setup() {
  Serial.begin(115200);
  //NRF Setup//
  setupRadio();
  calibrate();
}


void loop() {
  readJoystick();
  transmitRadio();


  Serial.print(myTele.vcc); Serial.print("\t");
  Serial.print(myTele.batteryVoltage); Serial.print("\t");
  Serial.print(myTele.batteryOk); Serial.print("\t");
  Serial.print(myTele.channel); Serial.print("\t");
  Serial.println();


}
