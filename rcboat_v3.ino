//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//

#include <SPI.h>              //library for Wireless (nrf24l01+)
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <TinyGPS++.h>        //library for GPS (Neo6M)
#include <QMC5883LCompass.h>  //library for Compass (QMC5883L)

const int IN1 = 34;      //foward left motor
const int IN2 = 36;      //backwards left motor
const int IN3 = 22;      //foward right motor
const int IN4 = 24;      //backwards right motor
const int ENA = 46;      //pwm left motor
const int ENB = 45;      //pwm right motor

const int CE = 7;        //radio CE pin
const int CSN = 8;       //radio CSN pin

const int nav = A0;      //navigation lights
const int strobe = A1;   //strobe light
const int extlight = A2; //exterior lights
const int intlight = A3; //interior lights
const int camera = A4;   //fpv camera

const double TARGET_LAT = 51.966421;  //home location
const double TARGET_LON = 6.274239;

long pwmA = 0;        //pwm target value for left motor
long pwmB = 0;        //pwm target value for right motor
int pwmAcounter = 0;  //pwm value for left motor
int pwmBcounter = 0;  //pwm value for left motor
int trimA = 0;        //trim value for left motor
int trimB = 0;        //trim value for right motor
int timer = 0;        //timer for timed events that use millis()

TinyGPSPlus gps;                  //gps object
QMC5883LCompass compass;          //compass object
RF24 radio(CE, CSN);              //radio object
const byte address[5] = "00001";  //pipe address
int control[9];                   //array to be received
//up1 - down1 - up2 - down2 - trim - speed - lights - camera - autoMode

void setup() {
  pinMode(IN1, OUTPUT); //set all pins as output
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(nav, OUTPUT);
  pinMode(strobe, OUTPUT);
  pinMode(intlight, OUTPUT);
  pinMode(extlight, OUTPUT);
  pinMode(camera, OUTPUT);
  
  TCCR5A = 0;             //Clear timer registers of timer5 (motors)
  TCCR5B = 0;
  TCNT5 = 0;
  
  TCCR5B |= _BV(CS50);    //No prescaler for timer5
  ICR5 = 32000;           //PWM mode counts up then down (16MHz/64000=250Hz)
  
  TCCR5A |= _BV(COM1A1);  //Output A clear rising/set falling
  TCCR5A |= _BV(COM1B1);  //Output B clear rising/set falling
  
  TCCR5B |= _BV(WGM53);   //PWM mode with ICR5 Mode 10
  TCCR5A |= _BV(WGM51);   //WGM53:WGM50 set 1010 (PWM, Phase correct)

  OCR5A = 0;  //start channel A at 0
  OCR5B = 0;  //start channel B at 0
  
  //digitalWrite(ENA, HIGH);
  //digitalWrite(ENB, HIGH);
  
  radio.begin();  //start radio service as receiver
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();

  compass.init();        //start compass and calibrate it
  compass.setCalibration(-1793, 847, -1815, 823, -987, 1830);
  
  Serial.begin(115200);  //Arduino <-> PC baudrate
  Serial1.begin(9600);   //Arduino <-> GPS baudrate
  
}

void loop() {
  
  timer = millis() % 2000;  //use millis for cycles of 2 seconds
  /*
  while (Serial1.available()){  //acquire stream of GPS data
    gps.encode(Serial1.read()); 
  }
  
  if(timer == 0 || timer == 1000){                                                                                                                 //acquire GPS/compass data every second
    unsigned long distanceToTarget = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);  //distance to target (m)
    int courseToTarget = int((TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON)+0.5));                         //absolute course to target from current position (deg)
    
    compass.read();                            //read compass device
    int azi = compass.getAzimuth();            //acquire azimuth (= current heading)
    int targetHeading = courseToTarget - azi;  //contains relative heading of the target with respect to current heading
    if(targetHeading < 0){                     //correction 1: compass values are now all positive
      targetHeading += 360;
    }
    if(targetHeading > 180){                   //correction 2: 'left' values now run from 0 to -180, 'right' 0 to 180
      targetHeading -= 360;                    //-180: behind, -90: left, 0: straight ahead, 90: right, 180: behind
    }
    
    for(int i = 0; i < 9; i++){  //print control values
      Serial.print(control[i]);
      Serial.print(" ");
    }
    
    Serial.print("A: ");        //print pwm values
    Serial.print(pwmAcounter);
    Serial.print(" B: ");
    Serial.print(pwmBcounter);
    Serial.print(" ");
    
    Serial.print("Heading: ");  //print heading and position info
    Serial.print(azi);
    Serial.print("* Target: ");
    Serial.print(targetHeading);
    Serial.print("* Distance: ");
    Serial.print(distanceToTarget);
    Serial.println("m ");
    
  }
  */
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
    
    digitalWrite(nav, control[6]);  //write values for lights and fpvcamera to transistor circuits
    digitalWrite(intlight, control[6]);
    digitalWrite(extlight, control[6]);
    digitalWrite(camera, control[7]);
    
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
  
  OCR5A = abs(pwmAcounter);  //apply pwm value
  OCR5B = abs(pwmBcounter);  //apply pwm value
  
  if(control[6]){                                               //strobe light
    if(timer > 0 && timer < 100 || timer > 200 && timer < 300)  //blink twice for 100ms
      digitalWrite(strobe, HIGH);
    else
      digitalWrite(strobe, LOW);
  }
  
}
