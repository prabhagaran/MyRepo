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
#include <EEPROM.h>
#include "transmitterConfig.h"

#define DEBUG

#define  CE_PIN  7 //The pins to be used for CE and CSN
#define  CSN_PIN 8
// Is the radio or IR transmission mode active?
byte transmissionMode = 1; // Radio mode is active by default

// Select trannsmitter operation mode
byte operationMode = 0; // Start in transmitter mode (0 = transmitter mode, 1 = tester mode, 2 = game mode)

// Vehicle address
int vehicleNumber = 1; // Vehicle number one is active by default

// Radio channels (126 channels are supported)
byte chPointer = 0; // Channel 1 (the first entry of the array) is active by default
const byte NRFchannel[] {
  1, 2
};

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

// Joysticks
#define JOYSTICK_1 A1
#define JOYSTICK_2 A0
#define JOYSTICK_3 A3
#define JOYSTICK_4 A2


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

struct dataStruct {      //This is the NRF data sent out by the Master. Max of 32 bytes
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Mode1 (toggle speed limitation)
  boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  boolean momentary1 = false; // Momentary push button
  byte pot1; // Potentiometer
} myData;                //This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct1 {     //This is the NRF data sent out by the Slave. Max of 32 bytes
  float feedback_1;
  float feedback_2;
  boolean feedback_3;
  byte feedback_4;
} myTele;

boolean displayLocked = true;



boolean joystickReversed[maxVehicleNumber + 1][4] = { // 1 + 10 Vehicle Addresses, 4 Servos
  {false, false, false, false}, // Address 0 used for EEPROM initialisation

  {false, false, false, false}, // Address 1
  {false, false, false, false}, // Address 2
  {false, false, false, false}, // Address 3
  {false, false, false, false}, // Address 4
  {false, false, false, false}, // Address 5
  {false, false, false, false}, // Address 6
  {false, false, false, false}, // Address 7
  {false, false, false, false}, // Address 8
  {false, false, false, false}, // Address 9
  {false, false, false, false}, // Address 10
};

//Joystick percent negative
byte joystickPercentNegative[maxVehicleNumber + 1][4] = { // 1 + 10 Vehicle Addresses, 4 Servos
  {100, 100, 100, 100}, // Address 0 not used

  {100, 100, 100, 100}, // Address 1
  {100, 100, 100, 100}, // Address 2
  {100, 100, 100, 100}, // Address 3
  {100, 100, 100, 100}, // Address 4
  {100, 100, 100, 100}, // Address 5
  {100, 100, 100, 100}, // Address 6
  {100, 100, 100, 100}, // Address 7
  {100, 100, 100, 100}, // Address 8
  {100, 100, 100, 100}, // Address 9
  {100, 100, 100, 100}, // Address 10
};

//Joystick percent positive
byte joystickPercentPositive[maxVehicleNumber + 1][4] = { // 1 + 10 Vehicle Addresses, 4 Channels
  {100, 100, 100, 100}, // Address 0 not used

  {100, 100, 100, 100}, // Address 1
  {100, 100, 100, 100}, // Address 2
  {100, 100, 100, 100}, // Address 3
  {100, 100, 100, 100}, // Address 4
  {100, 100, 100, 100}, // Address 5
  {100, 100, 100, 100}, // Address 6
  {100, 100, 100, 100}, // Address 7
  {100, 100, 100, 100}, // Address 8
  {100, 100, 100, 100}, // Address 9
  {100, 100, 100, 100}, // Address 10
};



int offset[4]; // the auto calibration offset of each joystick

// Auto-zero subfunction (called during setup, if a pot and no 3 position switch is connected) ----
void JoystickOffset() {
#ifndef CH1Switch
  offset[0] = 512 - analogRead(JOYSTICK_1);
#endif

#ifndef CH2Switch
  offset[1] = 512 - analogRead(JOYSTICK_2);
#endif

#ifndef CH3Switch
  offset[2] = 512 - analogRead(JOYSTICK_3);
#endif

#ifndef CH4Switch
  offset[3] = 512 - analogRead(JOYSTICK_4);
#endif
}


// Mapping and reversing subfunction ----
byte mapJoystick(byte input, byte arrayNo) {
  int reading[4];
  reading[arrayNo] = analogRead(input) + offset[arrayNo]; // read joysticks and add the offset
  reading[arrayNo] = constrain(reading[arrayNo], (1023 - range[arrayNo]), range[arrayNo]); // then limit the result before we do more calculations below

#ifndef CONFIG_4_CH // In most "car style" transmitters, less than one half of the throttle potentiometer range is used for the reverse. So we have to enhance this range!
  if (reading[2] < (range[2] / 2) ) {
    reading[2] = constrain(reading[2], reverseEndpoint, (range[2] / 2)); // limit reverse range, which will be mapped later on
    reading[2] = map(reading[2], reverseEndpoint, (range[2] / 2), 0, (range[2] / 2)); // reverse range mapping (adjust reverse endpoint in transmitterConfig.h)
  }
#endif

  if (transmissionMode == 1 && operationMode != 2 ) { // Radio mode and not game mode
    if (joystickReversed[vehicleNumber][arrayNo]) { // reversed
      return map(reading[arrayNo], (1023 - range[arrayNo]), range[arrayNo], (joystickPercentPositive[vehicleNumber][arrayNo] / 2 + 50), (50 - joystickPercentNegative[vehicleNumber][arrayNo] / 2));
    }
    else { // not reversed
      return map(reading[arrayNo], (1023 - range[arrayNo]), range[arrayNo], (50 - joystickPercentNegative[vehicleNumber][arrayNo] / 2), (joystickPercentPositive[vehicleNumber][arrayNo] / 2 + 50));
    }
  }
  else { // IR mode
    return map(reading[arrayNo], (1023 - range[arrayNo]), range[arrayNo], 0, 100);
  }
}

void readJoysticks() {

  // save previous joystick positions
  byte previousAxis1 = myData.axis1;
  byte previousAxis2 = myData.axis2;
  byte previousAxis3 = myData.axis3;
  byte previousAxis4 = myData.axis4;

  // Read current joystick positions, then scale and reverse output signals, if necessary (only for the channels we have)
#ifdef CH1
  myData.axis1 = mapJoystick(JOYSTICK_1, 0); // Aileron (Steering for car)
#endif

#ifdef CH2
  myData.axis2 = mapJoystick(JOYSTICK_2, 1); // Elevator
#endif

#ifdef CH3
  myData.axis3 = mapJoystick(JOYSTICK_3, 2); // Throttle
#endif

#ifdef CH4
  myData.axis4 = mapJoystick(JOYSTICK_4, 3); // Rudder
#endif

  // in case of an overflow, set axis to zero (prevent it from overflowing < 0)
  if (myData.axis1 > 150) myData.axis1 = 0;
  if (myData.axis2 > 150) myData.axis2 = 0;
  if (myData.axis3 > 150) myData.axis3 = 0;
  if (myData.axis4 > 150) myData.axis4 = 0;

  // Only allow display refresh, if no value has changed!
  if (previousAxis1 != myData.axis1 ||
      previousAxis2 != myData.axis2 ||
      previousAxis3 != myData.axis3 ||
      previousAxis4 != myData.axis4) {
    displayLocked = true;
  }
  else {
    displayLocked = false;
  }
}

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif

  //NRF Setup//
  radio.begin();                    //Initialize the nRF24L01 Radio
  radio.setChannel(108);            //Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS);  //Fast enough.. Better range

  //PA (Power Amplifier) Level can be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(pgm_read_64(&pipeOut, vehicleNumber - 1), true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(7, 5);  //delay, re-try count
  radio.openWritingPipe(pgm_read_64(&pipeOut, vehicleNumber - 1));

  JoystickOffset(); // Compute all joystick center points
  readJoysticks(); // Then do the first jocstick read
}


void loop() {

  //readJoysticks();
  myData.axis1  = map(analogRead(A0), 0, 1023, 0, 100);  //read values from the Joysticks
  myData.axis2  = map(analogRead(A1), 0, 1023, 0, 100);
  myData.axis3 = map(analogRead(A2), 0, 1023, 0, 100);
  myData.axis4 = map(analogRead(A3), 0, 1023, 0, 100);
  
  if (radio.write( &myData, sizeof(myData) )) {  //Transmit data to the Slave
    if ( radio.isAckPayloadAvailable() ) {       //If an Acknowledge is received, read the data
      radio.read(&myTele, sizeof(myTele));

      Serial.print(myTele.feedback_1); Serial.print("\t");
      Serial.print(myTele.feedback_2); Serial.print("\t");
      Serial.print(myTele.feedback_3); Serial.print("\t");
      Serial.print(myTele.feedback_4); Serial.print("\t");
      Serial.println();
    }
  }
 Serial.print(myData.axis1);
    Serial.print("\t");
    Serial.print(myData.axis2);
    Serial.print("\t");
    Serial.print(myData.axis3);
    Serial.print("\t");
    Serial.print(myData.axis4);
    Serial.print("\t");
  Serial.println(F_CPU / 1000000, DEC);
}
