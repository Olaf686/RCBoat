
//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//

#include <nRF24L01.h>  //library for transceiver (nrf24l01+ PA+LNA) https://github.com/nRF24/RF24
#include <RF24.h>
#include <SPI.h>

#include <Mux.h>       //library for multiplexer (74HC4067DB) https://github.com/stechio/arduino-ad-mux-lib
using namespace admux;

#include <Adafruit_SSD1306.h>  //library for display (SSD1306 128x64) https://github.com/adafruit/Adafruit_SSD1306
#include <Wire.h>              //library for I2C

const int sig = A3;  //signal pin
const int s0 = 2;    //address pin 0
const int s1 = 3;    //address pin 1
const int s2 = 4;    //address pin 2
const int s3 = 5;    //address pin 3

const int up1pin = 1;        //foward left motor
const int down1pin = 0;      //backwards left motor
const int up2pin = 3;        //foward right motor
const int down2pin = 2;      //backwards right motor
const int modifierpin = 4;   //pwm speed modifier
const int trimmerpin = 5;    //pwm trimmer
const int navlightpin = 6;   //navigation lights
const int intlightpin = 7;   //interior lights
const int extlightpin = 8;   //exterior lights
const int camerapin = 9;     //fpv camera
const int autoModepin = 10;  //autonomous mode

const int CE = 9;           //radio CE pin
const int CSN = 10;         //radio CSN pin
const byte address[5] = "00001";  //pipe address

float latFromSerial;       //parsed lattitude from serial input
float lonFromSerial;       //parsed longitude from serial input
float batteryVoltage;      //calculated battery voltage
float batteryPercentage;   //estimated battery charge
int timer;                 //millis timer

struct values{  //structure for control values
  bool up1;
  bool down1;
  bool up2;
  bool down2;
  int trimmer;
  int modifier;
  bool navlight;
  bool intlight;
  bool extlight;
  bool camera;
  bool autoMode;
  float targetLat;
  float targetLon;
};

struct info{  //structure for answer from boat
  int autoMode;
  int battery;
  int boatHeading;
  float boatSpeed;
  int targetDistance;
  int targetHeading;
};

struct values control;  //control values
struct info answer;     //answer from boat

Mux mux(Pinset(s0, s1, s2, s3));               //multiplexer object
Adafruit_SSD1306 display(128, 64, &Wire, -1);  //display object
RF24 radio(CE, CSN);                           //wireless object

void setup() {

  radio.begin();                    //start radio service as transmitter
  radio.setPALevel(RF24_PA_MIN);    //transmission power
  radio.setDataRate(RF24_250KBPS);  //data rate
  radio.enableAckPayload();         //enable ack payloads
  radio.setRetries(5,5);            //delay, count
  radio.openWritingPipe(address);   //set pipe address
  radio.stopListening();            //stop listening
  
  Serial.setTimeout(10);  //set timeout for waiting for incoming data
  Serial.begin(115200);   //set serial baudrate
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  delay(2000);                 //allow startup of display
  display.clearDisplay();      //clear the display
  display.setTextSize(1);      //standard 1:1 pixel scale
  display.setTextColor(WHITE); //set white text
  display.cp437(true);         //use full 256 char 'Code Page 437' font

  control.targetLat = 51.966451;  //default target lattitude
  control.targetLon = 6.274279;   //default target longitude
  
}

void loop() {
  if(millis() - timer >= 100){  //activate every 100 milliseconds
    timer = millis();  //new timestamp
    
    mux.signalPin(sig, INPUT_PULLUP, PinType::Digital);  //read all digital switches
    control.up1 = !mux.read(up1pin);
    control.down1 = !mux.read(down1pin);
    control.up2 = !mux.read(up2pin);
    control.down2 = !mux.read(down2pin);
    control.navlight = !mux.read(navlightpin);
    control.intlight = !mux.read(intlightpin);
    control.extlight = !mux.read(extlightpin);
    control.camera = !mux.read(camerapin);
    control.autoMode = !mux.read(autoModepin);

    mux.signalPin(sig, INPUT, PinType::Analog);  //read the analog potentiometers
    control.modifier = mux.read(modifierpin);
    control.trimmer = mux.read(trimmerpin);

    if(Serial.available()){                 //get target location from serial input
      latFromSerial = Serial.parseFloat();  //read lattitude
      lonFromSerial = Serial.parseFloat();  //read longitude
      if(latFromSerial != 0)                //only non zero values
        control.targetLat = latFromSerial;
      if(lonFromSerial != 0)                //only non zero values
        control.targetLon = lonFromSerial;
    }  
  
    if(radio.write(&control, sizeof(control))){  //send control values
      Serial.println(F("Data sent successfully"));
    
      if(radio.isAckPayloadAvailable()){                                     //do we get an answer back?
        radio.read(&answer, sizeof(answer));                                 //read answer
        batteryVoltage = (batteryVoltage+(answer.battery*15.0/1023.0))/2.0;  //calculate battery voltage (measured as 1/3th with voltage divider)
        batteryPercentage = (batteryVoltage-11.5)/(13.0-11.5)*100;           //estimate battery charge remaining
        answerToDisplay();                                                   //put all info on oled display
      }
      else{
        Serial.println(F("Message received but no answer..."));
        display.clearDisplay();  //display message for no answer from boat
        display.setCursor(0, 1);
        display.print("RC Boat: no answer");
        display.display();
      }
    }
    else{
      Serial.println(F("Data sending failed"));  //display message for no connection to boat
      display.clearDisplay();
      display.setCursor(0, 1);
      display.print("RC Boat: no contact");
      display.display();
    }
  }
}

void answerToDisplay(){  //function for putting answer from boat on display
    
    display.clearDisplay();
    
    display.setCursor(0, 1);
    display.print("RC Boat: connected!");
    
    display.setCursor(0, 15);
    display.print("Battery: ");
    display.print(batteryPercentage, 0);
    display.print("% ");
    display.print(batteryVoltage, 1);
    display.print("V");

    display.setCursor(0, 29);
    display.print("Course: ");
    display.print(answer.boatHeading);
    display.print((char)248);
    display.print(" ");
    display.print(answer.boatSpeed, 1);
    display.print("km/h");
    
    display.setCursor(0, 43);
    display.print("AutoMode: ");
    if(control.autoMode == 1){
      if(answer.autoMode == 0)
        display.print("running...");
      if(answer.autoMode == 1)
        display.print("arrived!");
      if(answer.autoMode == 2)
        display.print("no gps!");
    }
    else{
      display.print("off");
    }
    
    display.setCursor(0, 57);
    display.print("Target: ");
    display.print(answer.targetDistance);
    display.print("m ");
    if(answer.targetHeading >= 0)
      display.print("+");
    display.print(answer.targetHeading);
    display.print((char)248);
    
    display.display();
    
}
