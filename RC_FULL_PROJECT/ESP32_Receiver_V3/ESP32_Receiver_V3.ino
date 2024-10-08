/*  Two Way Slave Version 1.01
 *  Author: Schindler Electronics
 *  Data: 5/23/2019
 *  
 *  This code demonstrates how to use the NRF module in both
 *  "transmit" and "receive" mode using the acknowledge method.
 *  One device will be set up as the Master, while the other
 *  will be set up as the slave.
 *  
 *  When the master sends out a message, the slave will receive.
 *  When this happens, the slave will immediately send back an
 *  acknowledge packet. The Master will receive this data and send
 *  another data packet.
 *  
 *  Note: This code requires another Arduino and NRF
 *  
 *  Note: This code requires the use of the RF24 Library. 
 *  You can easily download this library in the Arduino Library Manager.
 */

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     #include <SPI.h>    //Comes with Arduino IDE
#include "RF24.h"   //Download and Install (See above)

#define  CE_PIN  4  //The pins to be used for CE and SN
#define  CSN_PIN 5
int vehicleNumber = 1;
const byte thisSlaveAddress[5] = {'R','x','A','A','A'}; //These are the names for all of the "Pipes" on the NRF Module


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

struct dataStruct {      //This is the NRF data sent out by the Master. Max of 32 bytes
  byte Xposition;         //int     = 2 bytes
  byte Yposition;         //double  = 4 bytes
  bool switchOn;         //boolean = 1 byte

  byte X2position;       
  byte Y2position;
  bool switch2On; 
} myData;                //This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct1 {     //This is the NRF data sent out by the Slave. Max of 32 bytes     
  float feedback_1;
  float feedback_2;
  boolean feedback_3; 
  byte feedback_4;
} myTele;

void setup() { 
  Serial.begin(115200);
  //Joystick Setup//
  pinMode(4, OUTPUT);  
  pinMode(5, OUTPUT);  
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
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
        radio.read( &myData, sizeof(myData) );              //If data is available, read the data
        myTele.feedback_1 = 1.00;
        myTele.feedback_2 = 1.00;
        myTele.feedback_3 = 1.00;
        myTele.feedback_4 = 1.00;
        
        radio.writeAckPayload(1, &myTele, sizeof(myTele));  //Send acknowledge data back

        Serial.print(myData.Xposition);  Serial.print("\t");
        Serial.print(myData.Yposition);  Serial.print("\t");
        Serial.print(myData.switchOn);   Serial.print("\t");
        Serial.print(myData.X2position); Serial.print("\t");
        Serial.print(myData.Y2position); Serial.print("\t");
        Serial.print(myData.switch2On);  Serial.print("\t");
        Serial.println();
    }
}
