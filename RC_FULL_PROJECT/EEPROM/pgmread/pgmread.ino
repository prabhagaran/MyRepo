#include "transmitterConfig.h"

byte transmissionMode = 1; // Radio mode is active by default
byte operationMode = 0; // Start in transmitter mode (0 = transmitter mode, 1 = tester mode, 2 = game mode)
int vehicleNumber = 1; // Vehicle number one is active by default

const uint64_t pipeOut[] PROGMEM = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL,
  0xE9E8F0F0C1LL, 0xE9E8F0F0C2LL, 0xE9E8F0F0C3LL, 0xE9E8F0F0C4LL, 0xE9E8F0F0C5LL,
  0xE9E8F0F0C6LL, 0xE9E8F0F0C7LL, 0xE9E8F0F0C8LL, 0xE9E8F0F0C9LL, 0xE9E8F0F0C0LL
};
const int maxVehicleNumber = (sizeof(pipeOut) / (sizeof(uint64_t)));

// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Mode1 (toggle speed limitation)
  boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  boolean momentary1 = false; // Momentary push button
  byte pot1; // Potentiometer
};
RcData data;


// Joysticks
#define JOYSTICK_1 A1
#define JOYSTICK_2 A0
#define JOYSTICK_3 A3
#define JOYSTICK_4 A2

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
  byte previousAxis1 = data.axis1;
  byte previousAxis2 = data.axis2;
  byte previousAxis3 = data.axis3;
  byte previousAxis4 = data.axis4;

  // Read current joystick positions, then scale and reverse output signals, if necessary (only for the channels we have)
#ifdef CH1
  data.axis1 = mapJoystick(JOYSTICK_1, 0); // Aileron (Steering for car)
#endif

#ifdef CH2
  data.axis2 = mapJoystick(JOYSTICK_2, 1); // Elevator
#endif

#ifdef CH3
  data.axis3 = mapJoystick(JOYSTICK_3, 2); // Throttle
#endif

#ifdef CH4
  data.axis4 = mapJoystick(JOYSTICK_4, 3); // Rudder
#endif

  // in case of an overflow, set axis to zero (prevent it from overflowing < 0)
  if (data.axis1 > 150) data.axis1 = 0;
  if (data.axis2 > 150) data.axis2 = 0;
  if (data.axis3 > 150) data.axis3 = 0;
  if (data.axis4 > 150) data.axis4 = 0;

  // Only allow display refresh, if no value has changed!
  if (previousAxis1 != data.axis1 ||
      previousAxis2 != data.axis2 ||
      previousAxis3 != data.axis3 ||
      previousAxis4 != data.axis4) {
    displayLocked = true;
  }
  else {
    displayLocked = false;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//Joystick setup
  JoystickOffset(); // Compute all joystick center points
  readJoysticks(); // Then do the first jocstick read

}

void loop() {
  // put your main code here, to run repeatedly:
 readJoysticks();
  Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.print("\t");
    Serial.println(F_CPU / 1000000, DEC);
  
  }
