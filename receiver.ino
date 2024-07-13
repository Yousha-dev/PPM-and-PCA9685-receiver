#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
const byte address[6] = "81989";

const int external = 10;
const int internal = 9;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

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
};

Data_Package data; // Create a variable with the above structure

const uint8_t PCA9685_ADDR = 0x40;

// PCA9685 Registers
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_OFF_L 0x08

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

bool pca9685Connected = false;

void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void i2cWrite16(uint8_t addr, uint8_t reg, uint16_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data & 0xFF);
  Wire.write(data >> 8);
  Wire.endTransmission();
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void setupPCA9685() {
  Wire.begin();
  // Check if PCA9685 is connected
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() == 0) {
    pca9685Connected = true;
    Serial.println("PCA9685 is connected");

    i2cWrite8(PCA9685_ADDR, PCA9685_MODE1, 0x10);  // Sleep
    uint32_t osc_clock = 25000000UL;
    uint32_t prescale_val = ((osc_clock / (SERVO_FREQ * 4096UL)) - 1);
    uint8_t prescale = (uint8_t)prescale_val;
    i2cWrite8(PCA9685_ADDR, PCA9685_PRESCALE, prescale);
    i2cWrite8(PCA9685_ADDR, PCA9685_MODE1, 0x20);  // Restart
    delay(10);  // Give some time for the PCA9685 to initialize
  } else {
    Serial.println("PCA9685 is not connected");
  }
}

void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
  i2cWrite16(PCA9685_ADDR, LED0_ON_L + 4 * channel, on);
  i2cWrite16(PCA9685_ADDR, LED0_OFF_L + 4 * channel, off);
}

// PPM Signal Generation Configuration
#define chanel_number 8  //set the number of channels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the Arduino

/*this array holds the servo values for the PPM signal
  change these values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening(); // Set the module as receiver
  resetData();

  setupPCA9685();

  //initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  if (!pca9685Connected) {
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  }
  sei();
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Check whether we keep receiving data, or we have a connection between the two modules
  currentTime = millis();
  if (currentTime - lastReceiveTime > 1000) { // If current time is more than 1 second since we have received the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example, if a drone has throttle up and we lose connection, it can keep flying unless we reset the values
  }
  if (pca9685Connected) {
    controlServos();
  } else {
    generatePPM();
  }
  // Print the data in the Serial Monitor
  Serial.print("j1PotX: ");
  Serial.print(data.j1PotX);
  Serial.print("; j1PotY: ");
  Serial.print(data.j1PotY);
  Serial.print("; j1Button: ");
  Serial.print(data.j1Button);
  Serial.print("; j2PotX: ");
  Serial.print(data.j2PotX);
  Serial.print("; j2PotY: ");
  Serial.print(data.j2PotY);
  Serial.print("; j2Button: ");
  Serial.print(data.j2Button);
  Serial.print("; pot1: ");
  Serial.print(data.pot1);
  Serial.print("; pot2: ");
  Serial.println(data.pot2);
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
}

void controlServos() {
  // Convert data to PWM values (assuming 0-255 input range to 150-600 PWM range)
  setPWM(0, 0, map(data.j1PotX, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(1, 0, map(data.j1PotY, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(2, 0, map(data.j2PotX, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(3, 0, map(data.j2PotY, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(4, 0, map(data.pot1, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(5, 0, map(data.pot2, 0, 255, SERVOMIN, SERVOMAX));
  setPWM(6, 0, data.j1Button ? SERVOMAX : SERVOMIN);
  setPWM(7, 0, data.j2Button ? SERVOMAX : SERVOMIN);
}

void generatePPM() {
  // Convert data to PPM values (assuming 0-255 input range to 1000-2000 microseconds range)
  ppm[0] = map(data.j1PotX, 0, 255, 1000, 2000);
  ppm[1] = map(data.j1PotY, 0, 255, 1000, 2000);
  ppm[2] = map(data.j2PotX, 0, 255, 1000, 2000);
  ppm[3] = map(data.j2PotY, 0, 255, 1000, 2000);
  ppm[4] = map(data.pot1, 0, 255, 1000, 2000);
  ppm[5] = map(data.pot2, 0, 255, 1000, 2000);
  ppm[6] = data.j1Button ? 2000 : 1000;
  ppm[7] = data.j2Button ? 2000 : 1000;
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
