                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     #include <SPI.h>    //Comes with Arduino IDE
#include "RF24.h"   //Download and Install (See above)
#include <Servo.h>
#include <statusLED.h>

#define  CE_PIN  7  //The pins to be used for CE and SN
#define  CSN_PIN 8

#define DEBUG

int Throttle = 1000;
int steeringServo =  1500;

int vehicleNumber = 1;

boolean left;
boolean right;
boolean hazard;


const int threshold = 50; // Threshold for center position
const int debounceDelay = 50; // Debounce time in milliseconds

int lastState = LOW; // Variable to store the last state of the switch
int lastDebounceTime = 0;


int Servo_POS = 90;
Servo ESC;
Servo SteeringServo;
Servo CH1;
Servo CH2;
Servo CH3;
Servo CH4;
Servo CH5;
Servo CH6;


statusLED headLight(false);
statusLED indicatorL(false);
statusLED indicatorR(false);
statusLED fogLight(false);
statusLED brakeLight(false);
statusLED backLight(false);



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
  byte pot1;
  byte pot2;

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
  SteeringServo.attach(6);
  
  CH1.attach(11);
  CH2.attach(10);
  CH3.attach(9);
  CH4.attach(12);
  CH5.attach(3);
  CH6.attach(4);
  

  headLight.begin(2);
  indicatorL.begin(23);
  indicatorR.begin(24);
  fogLight.begin(13);
  brakeLight.begin(26);
  backLight.begin(25);

  RadioSetup();

  myTele.vcc = 7.4;
  myTele.batteryVoltage = 3.3;
  myTele.batteryOk = true;
  myTele.channel = 1;
  ESC.writeMicroseconds(1000);
  CH1.writeMicroseconds(1500);
  CH2.writeMicroseconds(1500);
  CH3.writeMicroseconds(1500);
  CH4.writeMicroseconds(1500);
  CH5.writeMicroseconds(1500);
  CH6.writeMicroseconds(1500);
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
      //hazard = true; // Enable hazard lights
      myTele.batteryOk = true; // Clear low battery alert (allows to re-enable the vehicle, if you switch off the transmitter)
#ifdef DEBUG
      Serial.println("No Radio Available - Check Transmitter!");
#endif
    }


  }
}

void led()
{
  if (Throttle >= 1400  && Throttle <= 1500)
  {
    brakeLight.on();
  } else
  {
    brakeLight.off();
  }


  if (data.axis2 < 5) left = true;
  if (data.axis2 >  55)left = false;
  if (data.axis2 > 95)right =  true;
  if (data.axis2 < 45) right =  false;

  static int steeringOld;

  if (data.axis3 > steeringOld + 10)
  {
    left =  false;
    steeringOld = data.axis3;
  }
  if (data.axis3 < steeringOld - 10)
  {
    right = false;
    steeringOld =  data.axis3;
  }

  if (left)
  {
    right =  false;
    indicatorL.flash(375, 375, 0, 0);
    indicatorR.off();
    //Serial.println("left");
  }
  if (right)
  {
    left =  false;
    indicatorR.flash(375, 375, 0, 0);
    indicatorL.off();
    //Serial.println("right");
  }

  if (data.axis3 > 55) hazard =  true;
  if (data.axis3 < 45) hazard =  false;

  if (hazard) { // Hazard lights
    if (left) {
      left = false;
      indicatorL.off();
    }
    if (right) {
      right = false;
      indicatorR.off();
    }
    indicatorL.flash(375, 375, 0, 0);
    indicatorR.flash(375, 375, 0, 0);
  }

  if (!hazard && !left && !right) {
    indicatorL.off();
    indicatorR.off();
  }

  if (data.pot1 < 50)
  {
    headLight.off();
    backLight.off();
    fogLight.off();
  }
  else if (data.pot1 >= 50 && data.pot1 <= 75)
  {
    headLight.pwm(50);
    if (data.axis1 <= 45)
    {
      backLight.on();

    }
    else
    {
      backLight.off();
    }
  }
  else if (data.pot1 >= 75)
  {
    headLight.pwm(255);
    fogLight.on();
    
    if (data.axis1 <= 45)
    {
      backLight.on();

    }
    else
    {
      backLight.off();
    }
   
  }
   else if(data.pot1 >= 90)
   {
    fogLight.on();
   }
}

void WriteServos()
{
  Throttle  = map(data.axis1, 0, 100, 1000, 2000);
  steeringServo = map(data.axis4, 0 , 100, 1000, 2000);
  int steering1 = map(data.axis4,0,100,2000,1000);
  int ch2 = map(data.axis2,0,100,1000,2000);
  int ch3 = map(data.axis3,0,100,1000,2000);
  int ch4 =  map(data.axis4,0,100,1000,2000);
  
  ESC.writeMicroseconds(Throttle);
  //EngineSound.writeMicroseconds(steering1);
  SteeringServo.writeMicroseconds(steeringServo);
  CH1.writeMicroseconds(Throttle);
  CH2.writeMicroseconds(ch2);
  CH3.writeMicroseconds(ch3);
  CH4.writeMicroseconds(ch4);
  CH5.writeMicroseconds(ch3);
  CH6.writeMicroseconds(ch4);


}


void loop()
{
  ReadRadio();
  WriteServos();
  led();

#ifdef DEBUG
  Serial.print(data.axis1);
  Serial.print("\t");
  Serial.print(data.axis2);
  Serial.print("\t");
  Serial.print(data.axis3);
  Serial.print("\t");
  Serial.print(data.axis4);
  //Serial.print("\t");
  //Serial.print(left);
  Serial.print("\t");
  Serial.print(data.pot1);
  Serial.print("\t");
  Serial.print(data.pot2);
  Serial.print("\t");
  Serial.println();
#endif

}
