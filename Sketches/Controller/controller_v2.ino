      //             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//

#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

const int up1pin = 3;       //foward left motor
const int down1pin = 2;     //backwards left motor
const int up2pin = 5;       //foward right motor
const int down2pin = 4;     //backwards right motor
const int lightpin = 6;     //navigation lights
const int camerapin = 7;    //interior lights
const int autoModepin = 8;  //Autonomous mode
const int CE = 9;           //radio CE pin
const int CSN = 10;         //radio CSN pin
const int modifierpin = A0; //pwm speed modifier
const int trimmerpin = A2;  //pwm trimmer
const int greenLED = A4;    //green LED indicator
const int redLED = A5;      //red LED indicator

RF24 radio(CE, CSN);             //set radio pins
const byte address[5] = "00001"; //pipe address

struct values{  //structure for control values
  bool up1;
  bool down1;
  bool up2;
  bool down2;
  int trimmer;
  int modifier;
  bool light;
  bool camera;
  bool autoMode;
  float targetLat;
  float targetLon;
};

struct values control;  //control values
int timer;              //millis timer
float latFromSerial;
float lonFromSerial;

void setup() {
  pinMode(up1pin, INPUT_PULLUP);   //set switches as pulled up input
  pinMode(down1pin, INPUT_PULLUP);
  pinMode(up2pin, INPUT_PULLUP);
  pinMode(down2pin, INPUT_PULLUP);
  pinMode(lightpin, INPUT_PULLUP);
  pinMode(camerapin, INPUT_PULLUP);
  pinMode(autoModepin, INPUT_PULLUP);
  pinMode(greenLED, OUTPUT);    //set status LEDs as output
  pinMode(redLED, OUTPUT);
  
  radio.begin();  //start radio service as transmitter
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.setTimeout(10);
  Serial.begin(115200);
}

void loop() {
  
  control.up1 = !digitalRead(up1pin);     //obtain control values
  control.down1 = !digitalRead(down1pin);
  control.up2 = !digitalRead(up2pin);
  control.down2 = !digitalRead(down2pin);
  control.trimmer = analogRead(trimmerpin);
  control.modifier = analogRead(modifierpin);
  control.light = !digitalRead(lightpin);
  control.camera = !digitalRead(camerapin);
  control.autoMode = !digitalRead(autoModepin);
  if(Serial.available()){                 //get target location from serial input
    latFromSerial = Serial.parseFloat();
    lonFromSerial = Serial.parseFloat();
    if(latFromSerial != 0)                //only non zero values
      control.targetLat = latFromSerial;
    if(lonFromSerial != 0)                //only non zero values
      control.targetLon = lonFromSerial;
  }  
  
    Serial.print(control.up1);    //print control values
    Serial.print(" ");
    Serial.print(control.down1);
    Serial.print(" ");
    Serial.print(control.up2);
    Serial.print(" ");
    Serial.print(control.down2);
    Serial.print(" ");
    Serial.print(control.trimmer);
    Serial.print(" ");
    Serial.print(control.modifier);
    Serial.print(" ");    
    Serial.print(control.light);
    Serial.print(" ");
    Serial.print(control.camera);
    Serial.print(" ");
    Serial.print(control.autoMode);
    Serial.print(" ");
    Serial.print(control.targetLat, 6);
    Serial.print(" ");
    Serial.print(control.targetLon, 6);
    Serial.print(" ");
  
  if(radio.write(&control, sizeof(control))){  //send control values
    Serial.println("Data sent successfully");
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
  else{
    Serial.println("Data sending failed");
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  delay(100);

}
