
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(4, 5);
const byte address[6] = "00001";

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotY;
  byte j2PotX;
  byte pot1;
  byte pot2;
  byte D_Right_Indicator;
  byte D_Left_Indicator;
  byte D_Sound_Horn;
  byte D_Head_Light;
};

Data_Package data; //Create a variable with the above structure
void Serial_Print_Data(void);



void setup()
{
  Serial.begin(115200);
  radio.begin();

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();

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
    //draw();
  }
  void resetData() {
    // Reset the values when there is no radio connection - Set initial default values
    data.j1PotY = 127;
  data.j2PotX = 127; // Values from 0 to 255. When Joystick is in resting position, the value is in the middle, or 127. We actually map the pot value from 0 to 1023 to 0 to 255 because that's one BYTE value
  data.D_Right_Indicator = 0;
  data.D_Left_Indicator = 0;
  data.D_Sound_Horn = 0;
  data.D_Head_Light = 0;
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
