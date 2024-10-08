/*  Two Way Slave Version 1.01
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

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     #include <SPI.h>    //Comes with Arduino IDE
#include "RF24.h"   //Download and Install (See above)
#include "steeringCurves.h"
//#include "vehicleConfig.h"
#include <Servo.h>

#define  CE_PIN  7  //The pins to be used for CE and SN
#define  CSN_PIN 8
int vehicleNumber = 1;
const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'}; //These are the names for all of the "Pipes" on the NRF Module
int Throttle;
int Servo_POS = 90;
Servo ESC;
Servo servo1;




const uint64_t pipeIn[] PROGMEM = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL,
  0xE9E8F0F0C1LL, 0xE9E8F0F0C2LL, 0xE9E8F0F0C3LL, 0xE9E8F0F0C4LL, 0xE9E8F0F0C5LL,
  0xE9E8F0F0C6LL, 0xE9E8F0F0C7LL, 0xE9E8F0F0C8LL, 0xE9E8F0F0C9LL, 0xE9E8F0F0C0LL
};
uint64_t pgm_read_64(const void *ptr, uint8_t index) {
  uint64_t result;
  memcpy_P( &result, (uint8_t*)ptr + (index * 8), sizeof(uint64_t) ); // ptr is counting in bytes!
  return result;
}

RF24 radio(CE_PIN, CSN_PIN);

struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder

};
RcData data;                //This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct1 {     //This is the NRF data sent out by the Slave. Max of 32 bytes
  float feedback_1;
  float feedback_2;
  boolean feedback_3;
  byte feedback_4;
} myTele;



void setup() {
  Serial.begin(115200);
  ESC.attach(5);
  servo1.attach(6);

  //NRF Setup//
  radio.begin();                   //Initialize the nRF24L01 Radio
  radio.setChannel(108);           //Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS); //Fast enough.. Better range

  //PA (Power Amplifier) Level can be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(pgm_read_64(&pipeIn, vehicleNumber - 1), true);
  radio.openReadingPipe(1, pgm_read_64(&pipeIn, vehicleNumber - 1));
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.writeAckPayload(1, &myTele, sizeof(myTele)); //pre-load data for transmission
  radio.startListening();
  
}


void loop() {
  if ( radio.available() ) {
    radio.read( &data, sizeof(data) );              //If data is available, read the data
    myTele.feedback_1 = 1.00;
    myTele.feedback_2 = 1.00;
    myTele.feedback_3 = 1.00;
    myTele.feedback_4 = 1.00;

    radio.writeAckPayload(1, &myTele, sizeof(myTele));  //Send acknowledge data back

  uint16_t servo3Microseconds = 1500;
  static uint16_t servo3Microseconds2 = 1500;
  static long previousThrottleRampMillis;

  if (millis() - previousThrottleRampMillis >= 1) {
    previousThrottleRampMillis = millis();
    servo3Microseconds = map(data.axis1, 100, 0, 2000, 1000);
    servo3Microseconds = reMap(curveExponentialThrottle, servo3Microseconds);
    if (servo3Microseconds2 < servo3Microseconds) servo3Microseconds2 ++;
    if (servo3Microseconds2 > servo3Microseconds) servo3Microseconds2 --;
    Throttle  = servo3Microseconds2 ;
  }
Throttle =  map(data.axis1, 100, 0, 2000, 1000);
ESC.writeMicroseconds(Throttle);
Servo_POS = (map(data.axis3, 0, 100, 70, 100)); // 45 - 135°
servo1.write(Servo_POS);

    Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.print("\t");
    Serial.print(Throttle);
    Serial.print("\t");
    Serial.print(Servo_POS);
    Serial.print("\t");
    Serial.println();
  }
}
