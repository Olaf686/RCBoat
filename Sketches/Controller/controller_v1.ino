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

const int up1 = 3;       //foward left motor
const int down1 = 2;     //backwards left motor
const int up2 = 5;       //foward right motor
const int down2 = 4;     //backwards right motor
const int nav = 6;       //navigation lights
const int light = 7;     //interior lights
const int horn = 8;      //horn
const int CE = 9;        //radio CE pin
const int CSN = 10;      //radio CSN pin
const int modifier = A0; //pwm speed modifier
const int trimmer = A2;  //pwm trimmer
const int greenLED = A4; //green LED indicator
const int redLED = A5;   //red LED indicator

RF24 radio(CE, CSN);             //set radio pins
const byte address[5] = "00001"; //pipe address
int control[9];                  //array to be sent

void setup() {
  pinMode(up1, INPUT_PULLUP);   //set switches as pulled up input
  pinMode(down1, INPUT_PULLUP);
  pinMode(up2, INPUT_PULLUP);
  pinMode(down2, INPUT_PULLUP);
  pinMode(nav, INPUT_PULLUP);
  pinMode(light, INPUT_PULLUP);
  pinMode(horn, INPUT_PULLUP);
  pinMode(greenLED, OUTPUT);    //set status LEDs as output
  pinMode(redLED, OUTPUT);
  
  radio.begin();  //start radio service as transmitter
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening();
  
  Serial.begin(250000);
}

void loop() {
  control[0] = !digitalRead(up1);  //obtain control values
  control[1] = !digitalRead(down1);
  control[2] = !digitalRead(up2);
  control[3] = !digitalRead(down2);
  control[4] = analogRead(trimmer);
  control[5] = analogRead(modifier);
  control[6] = !digitalRead(nav);
  control[7] = !digitalRead(light);
  control[8] = !digitalRead(horn);
  
  if(radio.write(control, sizeof(control))){  //send control values
    Serial.println("Data sent successfully");
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
  else{
    Serial.println("Data sending failed");
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  for(int i = 0; i < 9; i++){  //print control values
    Serial.print(control[i]);
    Serial.print(" ");
  }
  Serial.println("");
  
  delay(100);  //send data at rate of 10Hz
}
