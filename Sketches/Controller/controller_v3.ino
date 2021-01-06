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
#include <RF24.h>

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

struct info{
  int autoMode;
  int battery;
  int targetDistance;
  int targetHeading;
};

struct values control;  //control values
struct info answer;     //answer from boat
int timer;              //millis timer
float latFromSerial;    //parsed lattitude from serial input
float lonFromSerial;    //parsed longitude from serial input
float batteryVoltage;   //calculated battery voltage

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
  radio.enableAckPayload();
  radio.setRetries(5,5); // delay, count
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.setTimeout(10);
  Serial.begin(115200);

  control.targetLat = 51.966421;  //home location
  control.targetLon = 6.274239;
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
  /*
  if(Serial.available()){                 //get target location from serial input
    latFromSerial = Serial.parseFloat();
    lonFromSerial = Serial.parseFloat();
    if(latFromSerial != 0)                //only non zero values
      control.targetLat = latFromSerial;
    if(lonFromSerial != 0)                //only non zero values
      control.targetLon = lonFromSerial;
  }  
  */
  
  if(radio.write(&control, sizeof(control))){  //send control values
    Serial.println("Data sent successfully");
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    
    if(radio.isAckPayloadAvailable()){            //do we get an answer back?
      radio.read(&answer, sizeof(answer));
      batteryVoltage = answer.battery*15/1023.0;  //calculate battery voltage (measured as 1/3th with voltage divider)
      Serial.print("AutoMode:");                  //print answer in serial (later on lcd)
      Serial.print(answer.autoMode);
      Serial.print(" Battery: ");
      Serial.print(batteryVoltage, 1);
      Serial.print("V ");
      Serial.print(answer.targetDistance);
      Serial.print("m ");
      Serial.print(answer.targetHeading);
      Serial.println("*");
    }
    else{
      Serial.println("No answer...");
    }
  
  }
  else{
    Serial.println("Data sending failed");
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  delay(100); //send data at ~10Hz
  
}
