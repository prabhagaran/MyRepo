
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>


RF24 radio(4, 5);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 20 ID's are available @ the moment
const uint64_t pipeOut[] PROGMEM = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL,
  0xE9E8F0F0C1LL, 0xE9E8F0F0C2LL, 0xE9E8F0F0C3LL, 0xE9E8F0F0C4LL, 0xE9E8F0F0C5LL,
  0xE9E8F0F0C6LL, 0xE9E8F0F0C7LL, 0xE9E8F0F0C8LL, 0xE9E8F0F0C9LL, 0xE9E8F0F0C0LL
};
const int maxVehicleNumber = (sizeof(pipeOut) / (sizeof(uint64_t)));

const byte address[6] = "00001";


unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {

  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte pot1;
  byte pot2;
  byte D_Right_Indicator;
  byte D_Left_Indicator;
  byte D_Sound_Horn;
  byte D_Head_Light;
};
Data_Package data; //Create a variable with the above structure
void Serial_Print_Data(void);

void drawTarget(int x, int y, int w, int h, int posX, int posY) {
  u8g2.drawFrame(x, y, w, h);
  u8g2.drawDisc((x + w / 2) - (w / 2) + (posX / 2), (y + h / 2) + (h / 2) - (posY / 2), 5, 5);
}
void draw(void) {


  u8g2.firstPage();
  do {
    u8g2.drawLine(0, 12, 128, 12);
    u8g2.setCursor(0, 3);
    u8g2.print("CH: ");
    u8g2.print(1);
    u8g2.setCursor(50, 3);
    u8g2.print("Bat: ");
    u8g2.print(12);
    u8g2.print("V");

    drawTarget(0, 14, 50, 50, data.j1PotY , data.j1PotX); // left joystick
    drawTarget(74, 14, 50, 50, data.j2PotY, data.j2PotX); // right joystick
    drawTarget(55, 14, 14, 50, 14, data.j1PotY); // potentiometer

  } while ( u8g2.nextPage() );

}


void setup()
{
  Serial.begin(115200);
  radio.begin();
  u8g2.begin();

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();

  u8g2.setFontPosTop();
  u8g2.setFont(u8g2_font_6x10_tr);

}
  void loop()
  {
    // Check whether there is data to be received
    if (radio.available()) {
      radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
      lastReceiveTime = millis(); // At this moment we have received the data
      //Serial.println("Connected");
    }
    // Check whether we keep receving data, or we have a connection between the two modules
    currentTime = millis();
    if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
      //resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
    }
    // Print the data in the Serial Monitor
    Serial_Print_Data();
    draw();
  }
  void resetData() {
  byte j1PotX = 50;
  byte j1PotY = 50;
  byte j2PotX = 50;
  byte j2PotY = 50;
  byte pot1 = 50 ;
  byte pot2 = 50;
  byte D_Right_Indicator = 0;
  byte D_Left_Indicator = 0 ;
  byte D_Sound_Horn = 0;
  byte D_Head_Light = 0;
  }
  void Serial_Print_Data(void)
  {
    
  Serial.print(" JY1y : ");
  Serial.print(data.j1PotY);

  Serial.print(" JY2x : ");
  Serial.print(data.j2PotX);


  Serial.print(" POT1 : ");
  Serial.print(data.pot1);
  Serial.print(" POT2 : ");
  Serial.print(data.pot2);

  Serial.print(" HO : ");
  Serial.print(data.D_Sound_Horn);

  Serial.print(" HL : ");
  Serial.print(data.D_Head_Light);


  Serial.println();
  }
