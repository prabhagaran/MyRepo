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
#include <Servo.h>

#define  CE_PIN  7  //The pins to be used for CE and SN
#define  CSN_PIN 8

#define DEBUG

int vehicleNumber = 1;

int Throttle;
int Servo_POS = 90;
Servo ESC;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

// Indicators
boolean left;
boolean right;
boolean hazard;


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
  int axis1; // Aileron (Steering for car)
  int axis2; // Elevator
  int axis3; // Throttle
  int axis4; // Rudder

};
RcData data;                //This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct1 {     //This is the NRF data sent out by the Slave. Max of 32 bytes
  float vcc; // vehicle vcc voltage
  float batteryVoltage; // vehicle battery voltage
  boolean batteryOk = true; // the vehicle battery voltage is OK!
  byte channel = 1; // the channel number
} myTele;


void RadioSetup()
{
  //NRF Setup//
  radio.begin();                   //Initialize the nRF24L01 Radio
  radio.setChannel(108);           //Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS); //Fast enough.. Better range
  radio.setPALevel(RF24_PA_MAX);//PA (Power Amplifier) Level can be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setAutoAck(pgm_read_64(&pipeIn, vehicleNumber - 1), true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 5);
  radio.openReadingPipe(1, pgm_read_64(&pipeIn, vehicleNumber - 1));
  radio.writeAckPayload(1, &myTele, sizeof(myTele)); //pre-load data for transmission
  radio.startListening();

}

void setup() {
  data.axis1 = 50; // Aileron (Steering for car)
  data.axis2 = 50; // Elevator
  data.axis3 = 50; // Throttle
  data.axis4 = 50; // Rudder

  Serial.begin(115200);
  ESC.attach(5);
  servo1.attach(6);
  servo2.attach(4);
  /*servo2.attach(7);
    servo3.attach(8);
    servo4.attach(9);
    servo5.attach(10);*/
  RadioSetup();

  myTele.vcc = 7.4;
  myTele.batteryVoltage = 3.3;
  myTele.batteryOk = true;
  myTele.channel = 1;
  ESC.writeMicroseconds(1000);
  delay(2000);

}

void ReadRadio()
{
  static unsigned long lastRecvTime = 0;
  if ( radio.available() ) {
    radio.writeAckPayload(1, &myTele, sizeof(myTele));  //Send acknowledge data back
    radio.read( &data, sizeof(data) );              //If data is available, read the data
    lastRecvTime = millis();

    if (millis() - lastRecvTime > 1000) { // set all analog values to their middle position, if no RC signal is received during 1s!
      data.axis1 = 50; // Aileron (Steering for car)
      data.axis2 = 50; // Elevator
      data.axis3 = 50; // Throttle
      data.axis4 = 50; // Rudder
      hazard = true; // Enable hazard lights
      myTele.batteryOk = true; // Clear low battery alert (allows to re-enable the vehicle, if you switch off the transmitter)
#ifdef DEBUG
      Serial.println("No Radio Available - Check Transmitter!");
#endif
    }


  }
}


void WriteServos()
{
  
  ESC.writeMicroseconds(data.axis2);
  servo2.writeMicroseconds(data.axis2);
  servo1.writeMicroseconds(data.axis4);


}


void loop()
{
  ReadRadio();
  WriteServos();

#ifdef DEBUG
  Serial.print(data.axis1);
  Serial.print("\t");
  Serial.print(data.axis2);
  Serial.print("\t");
  Serial.print(data.axis3);
  Serial.print("\t");
  Serial.print(data.axis4);
  Serial.print("\t");
  Serial.println();
#endif

}
