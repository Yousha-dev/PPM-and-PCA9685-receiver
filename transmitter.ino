/*
        DIY Arduino based RC Transmitter
  by Dejan Nedelkovski, www.HowToMechatronics.com
  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// Define the digital inputs
#define jB1 4  // Joystick button 1
#define jB2 5  // Joystick button 2
//#define t1 7   // Toggle switch 1
//#define t2 4   // Toggle switch 1

RF24 radio(2, 3);   // nRF24L01 (CE, CSN)
const byte address[6] = "81989"; // Address

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
//  byte tSwitch1;
//  byte tSwitch2;
};

Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(9600);
  // Define the radio communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  
  // Activate the Arduino internal pull-up resistors
  pinMode(jB1, INPUT_PULLUP);
  pinMode(jB2, INPUT_PULLUP);
//  pinMode(t1, INPUT_PULLUP);
//  pinMode(t2, INPUT_PULLUP);
  
  // Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
//  data.tSwitch1 = 1;
//  data.tSwitch2 = 1;
}

void loop() {
  // Read all analog inputs and map them to one Byte value
  data.j1PotX = map(analogRead(A0), 0, 1023, 0, 255); // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
  data.j1PotY = map(analogRead(A1), 0, 1023, 0, 255);
  data.j2PotX = map(analogRead(A2), 0, 1023, 0, 255);
  data.j2PotY = map(analogRead(A3), 0, 1023, 0, 255);
//  data.pot1 = map(analogRead(A4), 0, 1023, 0, 255);
//  data.pot2 = map(analogRead(A5), 0, 1023, 0, 255);
  // Read all digital inputs
  data.j1Button = digitalRead(jB1);
  data.j2Button = digitalRead(jB2);
//  data.tSwitch2 = digitalRead(t2);
  
  // Print the values to the Serial Monitor
  Serial.print("j1PotX: "); Serial.print(data.j1PotX);
  Serial.print(" j1PotY: "); Serial.print(data.j1PotY);
  Serial.print(" j2PotX: "); Serial.print(data.j2PotX);
  Serial.print(" j2PotY: "); Serial.print(data.j2PotY);
//  Serial.print(" pot1: "); Serial.print(data.pot1);
//  Serial.print(" pot2: "); Serial.print(data.pot2);
  Serial.print(" j1Button: "); Serial.print(data.j1Button);
  Serial.print(" j2Button: "); Serial.println(data.j2Button);
//  Serial.print(" tSwitch1: "); Serial.print(data.tSwitch1);
//  Serial.print(" tSwitch2: "); Serial.println(data.tSwitch2);

  // Send the whole data from the structure to the receiver
  radio.write(&data, sizeof(Data_Package));
}
