//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// This code is for controlling a remote rc boat.
// IN1/IN2 and IN3/IN4 are used with a l298n H-bridge to control the direction of two electric motors.
// Two potentiometers are used to provide a pwm value for both motors: the first one determines the speed for both motors, the second one can slow down one motor to provide trim.
// Values for 'nav', 'light' and 'horn' are used to control navigation/strobe lights, interior lights and the horn.
// Data transmission is handled by two nrf24l01+ modules using TMRh20's RF24 library.

#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

const int IN1 = 4;     //foward left motor
const int IN2 = 5;     //backwards left motor
const int IN3 = 3;     //foward right motor
const int IN4 = 2;     //backwards right motor
const int CE = 7;      //radio CE pin
const int CSN = 8;     //radio CSN pin
const int ENA = 9;     //pwm left motor
const int ENB = 10;    //pwm right motor
const int nav = A0;    //navigation lights
const int strobe = A1; //strobe light
const int light = A4;  //interior lights
const int horn = A5;   //horn

RF24 radio(CE, CSN);             //radio pins
const byte address[5] = "00001"; //pipe address
int control[9];                  //array to be received
//up1 - down1 - up2 - down2 - trim - speed - nav/strobe - light - horn

long pwmA = 0;  //pwm target value for left motor
long pwmB = 0;  //pwm target value for right motor
int pwmAcounter = 0;  //pwm value for left motor
int pwmBcounter = 0;  //pwm value for left motor
int trimA = 0;  //trim value for left motor
int trimB = 0;  //trim value for right motor
int timer = 0;  //timer for strobe

void setup() {
  pinMode(IN1, OUTPUT); //set all pins as output
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(nav, OUTPUT);
  pinMode(strobe, OUTPUT);
  pinMode(light, OUTPUT);
  pinMode(horn, OUTPUT);
  
  TCCR1A = 0;             //Clear timer registers
  TCCR1B = 0;
  TCNT1 = 0;
  
  TCCR1B |= _BV(CS10);    //No prescaler for timer1
  ICR1 = 32000;           //PWM mode counts up then down (16MHz/64000=250Hz)
  
  TCCR1A |= _BV(COM1A1);  //Output A clear rising/set falling
  TCCR1A |= _BV(COM1B1);  //Output B clear rising/set falling
  
  TCCR1B |= _BV(WGM13);   //PWM mode with ICR1 Mode 10
  TCCR1A |= _BV(WGM11);   //WGM13:WGM10 set 1010 (PWM, Phase correct)
  
  radio.begin();  //start radio service as receiver
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();
  
  Serial.begin(9600);
  
  tone(horn, 70); //indicator for successful startup
  delay(100);
  noTone(horn);
  delay(100);
  tone(horn, 70);
  delay(100);
  noTone(horn);
}

void loop() {
  if(radio.available()){  //obtain new control values if available
    radio.read(control, sizeof(control));
    
    if(control[4] < 512){  //determine trim (slow down left motor)
      trimA = control[4];
      trimB = 511;
    }
    if(control[4] > 511){  //determine trim (slow down right motor)
      trimA = 511;
      trimB = 1023 - control[4];
    }
    
    if(control[0] && !control[1])              //forwards state
    pwmA = 32000L*trimA/511*control[5]/1023;   //generate positive pwm target value
    if(!control[0] && control[1])              //backwards state
    pwmA = -32000L*trimA/511*control[5]/1023;  //generate negative pwm target value
    if(!control[0] && !control[1])             //neutral state
    pwmA = 0;                                  //generate idle pwm target value
    
    if(control[2] && !control[3])              //forwards state
    pwmB = 32000L*trimB/511*control[5]/1023;   //generate positive pwm target value
    if(!control[2] && control[3])              //backwards state
    pwmB = -32000L*trimB/511*control[5]/1023;  //generate negative pwm target value
    if(!control[2] && !control[3])             //neutral state
    pwmB = 0;                                  //generate idle pwm target value
    
    digitalWrite(nav, control[6]);  //write values for lights
    digitalWrite(light, control[7]);
    
    if(control[8])  //play horn sound
      tone(horn, 70);
    else
      noTone(horn);
    
    for(int i = 0; i < 9; i++){  //print control values
      Serial.print(control[i]);
      Serial.print(" ");
    }
    Serial.print(pwmAcounter);  //print pwm values
    Serial.print(" ");
    Serial.println(pwmBcounter);
    
  }
  
  if(pwmAcounter < pwmA)  //increase pwm value to reach target value
    pwmAcounter++;
  if(pwmAcounter > pwmA)  //decrease pwm value to reach target value
    pwmAcounter--;
  
  if(pwmBcounter < pwmB)  //increase pwm value to reach target value
    pwmBcounter++;
  if(pwmBcounter > pwmB)  //decrease pwm value to reach target value
    pwmBcounter--;
  
  if(pwmAcounter > 0){        //Write left motor values for forward motion
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if(pwmAcounter < 0){        //Write left motor values for backwards motion
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if(pwmAcounter == 0){       //Write left motor values for no motion
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
  if(pwmBcounter > 0){        //Write right motor values for forward motion
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  if(pwmBcounter < 0){        //Write right motor values for backwards motion
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  if(pwmBcounter == 0){       //Write right motor values for no motion
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  
  OCR1A = abs(pwmBcounter);  //apply pwm value
  OCR1B = abs(pwmAcounter);  //apply pwm value
  
  if(control[6]){                                               //strobe light
    timer = millis() % 2000;                                    //use millis for cycles of 2 seconds
    if(timer > 0 && timer < 100 || timer > 200 && timer < 300)  //blink twice for 100ms
      digitalWrite(strobe, HIGH);
    else
      digitalWrite(strobe, LOW);
  }
  
}
