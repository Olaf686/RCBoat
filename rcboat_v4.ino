
//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//

#include <SPI.h>              //library for SPI communication
#include <TinyGPS++.h>        //library for gps (GY-NEO6MV2) https://github.com/mikalhart/TinyGPSPlus
#include <QMC5883LCompass.h>  //library for compass (QMC5883L) https://github.com/mprograms/QMC5883LCompass
#include <RF24.h>             //library for transceiver (nRF24l01+ PA+LNA) https://github.com/nRF24/RF24
#include <nRF24L01.h>
#include <RF24_config.h>

const int IN1pin = 34;      //foward left motor
const int IN2pin = 36;      //backwards left motor
const int IN3pin = 22;      //foward right motor
const int IN4pin = 24;      //backwards right motor
const int ENApin = 46;      //pwm left motor
const int ENBpin = 45;      //pwm right motor

const int CE = 7;            //radio CE pin
const int CSN = 8;           //radio CSN pin

const int navpin = A0;       //navigation lights
const int strobepin = A1;    //strobe light
const int extlightpin = A2;  //exterior lights
const int intlightpin = A3;  //interior lights
const int camerapin = A4;    //fpv camera
const int batterypin = A15;  //battery voltage (voltage divider)

long pwmA = 0;        //pwm target value for left motor
long pwmB = 0;        //pwm target value for right motor
int pwmAcounter = 0;  //pwm value for left motor
int pwmBcounter = 0;  //pwm value for left motor
int trimA = 0;        //trim value for left motor
int trimB = 0;        //trim value for right motor
int timer = 0;        //timer for strobe light
long timeOut = 0;     //timer for transmitter connection failsafe 

long distanceToTarget;  //distance in meters to target location (from GPS)
int courseToTarget;     //course in degrees to target location (from GPS)
int azi;                //azimuth heading of the boat (from compass)
int targetHeading;      //course to target with respect to current heading (range: -180* <-> 180*)
int pwmMax;             //max throttle (pwm duty cycle) while in autoMode
int autoStatus;         //status codes (function return values) for autoMode function
int batteryVoltage;     //battery voltage of the lead acid battery (measured with voltage divider as 1/3th)

TinyGPSPlus gps;                      //gps object
QMC5883LCompass compass;              //compass object
RF24 radio(CE, CSN);                  //wireless object
const byte address[5] = "00001";      //pipe address

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

struct info{  //structure for reporting data back to controller
  int autoMode;
  int battery;
  int boatHeading;
  float boatSpeed;
  int targetDistance;
  int targetHeading;
};

struct values control;  //control values from controller to boat
struct info answer;     //answer from boat to controller

void setup() {
  pinMode(IN1pin, OUTPUT); //set all pins as output
  pinMode(IN2pin, OUTPUT);
  pinMode(IN3pin, OUTPUT);
  pinMode(IN4pin, OUTPUT);
  pinMode(ENApin, OUTPUT);
  pinMode(ENBpin, OUTPUT);
  pinMode(navpin, OUTPUT);
  pinMode(strobepin, OUTPUT);
  pinMode(intlightpin, OUTPUT);
  pinMode(extlightpin, OUTPUT);
  pinMode(camerapin, OUTPUT);
  
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
  
  radio.begin();                                       //start radio service as receiver
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.openReadingPipe(1, address);
  radio.startListening();
  radio.writeAckPayload(1, &answer, sizeof(answer));  //when receiving a message from the controller, send this data back
  
  compass.init();                                                //start compass
  compass.setCalibration(-1490, 328, -612, 1190, -1690, 1500);   //calibration
  
  Serial.begin(115200);  //Arduino <-> PC baudrate
  Serial1.begin(9600);   //Arduino <-> GPS baudrate
  
}

void loop() {
  
  while (Serial1.available()){   //acquire stream of GPS data
    gps.encode(Serial1.read());  //process GPS data
  }

  if(radio.available()){  //obtain new control values if available
    radio.read(&control, sizeof(control));

    timeOut = 0;  //reset timeout, as we have a connection with the transmitter

    if(control.autoMode){                                           //determine pwmA and pwmB with GPS location, compass heading and target location as input
      autoStatus = autoMode(control.targetLat, control.targetLon);  //status code of autoMode() for info and debugging
    }
    else{
      manualMode();                //determine pwmA and pwmB with controller values as input
      compass.read();              //read compass device for sending answer to controller
      azi = compass.getAzimuth();  //acquire azimuth (= current heading)
    }
    
    answer.autoMode = autoStatus;                       //gather values for a reply to the controller
    answer.battery = analogRead(batterypin);            //battery voltage
    answer.boatHeading = azi;                           //heading of the boat
    if(gps.speed.isValid())
      answer.boatSpeed = gps.speed.kmph();              //speed of the boat
    else
      answer.boatSpeed = -1.0;                          //default speed value
    answer.targetDistance = distanceToTarget;           //distance to target
    answer.targetHeading = targetHeading;               //course to target
    radio.writeAckPayload(1, &answer, sizeof(answer));  //write answer values for the next reply
    
    digitalWrite(navpin, control.navlight);       //write value for navigation lights
    digitalWrite(intlightpin, control.intlight);  //write value for interior lights
    digitalWrite(extlightpin, control.extlight);  //write value for exterior lights
    digitalWrite(camerapin, control.camera);      //write value for fpv camera
    
    //printValues();  //print control values for debugging
    
  }
  else{                             //nothing received from transmitter
    if(timeOut == 0)                //if last loop iteration received something, start the timeout counter
      timeOut = millis();           //timestamp for last transmission received
    if(millis() - timeOut > 5000){  //after 5 seconds of nothing received -> something is wrong!
      pwmA = 0;                     //stop the motors
      pwmB = 0;
    }
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
    digitalWrite(IN1pin, HIGH);
    digitalWrite(IN2pin, LOW);
  }
  if(pwmAcounter < 0){        //Write left motor values for backwards motion
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, HIGH);
  }
  if(pwmAcounter == 0){       //Write left motor values for no motion
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, LOW);
  }
  
  if(pwmBcounter > 0){        //Write right motor values for forward motion
    digitalWrite(IN3pin, HIGH);
    digitalWrite(IN4pin, LOW);
  }
  if(pwmBcounter < 0){        //Write right motor values for backwards motion
    digitalWrite(IN3pin, LOW);
    digitalWrite(IN4pin, HIGH);
  }
  if(pwmBcounter == 0){       //Write right motor values for no motion
    digitalWrite(IN3pin, LOW);
    digitalWrite(IN4pin, LOW);
  }
  
  OCR5A = abs(pwmAcounter);  //apply pwm value
  OCR5B = abs(pwmBcounter);  //apply pwm value
  
  if(control.navlight){                                         //strobe light
    timer = millis() % 2000;                                    //use millis for cycles of 2 seconds
    if(timer > 0 && timer < 100 || timer > 200 && timer < 300)  //blink twice every 2 seconds
      digitalWrite(strobepin, HIGH);
    else
      digitalWrite(strobepin, LOW);
  }

}

void manualMode(){  //determine pwmA and pwmB with controller values as input

  if(control.trimmer < 512){  //determine trim (slow down left motor)
    trimA = control.trimmer;
    trimB = 511;
  }
  if(control.trimmer > 511){  //determine trim (slow down right motor)
    trimA = 511;
    trimB = 1023 - control.trimmer;
  }
  
  if(control.up1 && !control.down1)                  //forwards state
    pwmA = 32000L*trimA/511*control.modifier/1023;   //generate positive pwm target value
  if(!control.up1 && control.down1)                  //backwards state
    pwmA = -32000L*trimA/511*control.modifier/1023;  //generate negative pwm target value
  if(!control.up1 && !control.down1)                 //neutral state
    pwmA = 0;                                        //generate idle pwm target value
  
  if(control.up2 && !control.down2)                  //forwards state
    pwmB = 32000L*trimB/511*control.modifier/1023;   //generate positive pwm target value
  if(!control.up2 && control.down2)                  //backwards state
    pwmB = -32000L*trimB/511*control.modifier/1023;  //generate negative pwm target value
  if(!control.up2 && !control.down2)                 //neutral state
    pwmB = 0;                                        //generate idle pwm target value
  
}

int autoMode(float targetLat, float targetLon){  //determine pwmA and pwmB with GPS location, compass heading and target location as input
                                                 //returns 0 for not there yet, 1 for destination reached and 2 for no gps data
  if(gps.location.isValid()){                                                                                             //wait for gps fix to start calculating
    distanceToTarget = (long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);  //distance to target (m)
    courseToTarget = int((TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon)+0.5));      //absolute course to target from current position (deg)
  }
  else{        //autoMode will not work if not receiving a valid location from gps
    pwmA = 0;  //stop the motors
    pwmB = 0;
    return 2;  //exit autoMode with error code 2: no gps
  }
  
  if(distanceToTarget < 4){  //very close to target -> destination reached!
    pwmA = 0;                //stop the motors
    pwmB = 0;
    return 1;                //exit autoMode with status code 1: destination reached
  }
  
  compass.read();                        //read compass device
  azi = compass.getAzimuth();            //acquire azimuth (= current heading)
  targetHeading = courseToTarget - azi;  //contains relative heading of the target with respect to current heading
  if(targetHeading < 0){                 //correction 1: compass values are now all positive
    targetHeading += 360;
  }
  if(targetHeading > 180){               //correction 2: 'left' values now run from 0 to -180, 'right' 0 to 180
    targetHeading -= 360;                //-180: behind, -90: left, 0: straight ahead, 90: right, 180: behind
  }
  
  pwmMax = control.modifier*31;  //set the maximum duty cycle of the motors (32000=100%)
  
  if(targetHeading == 0){                         //when current heading is equal to course to target go straight ahead
    pwmA = pwmMax;                                //set left motor pwm value
    pwmB = pwmMax;                                //set right motor pwm value
  }
  if(targetHeading < 0){                          //when course to target is to the left of current heading, boat should turn left 
    pwmA = pwmMax + 1L*targetHeading*pwmMax/120;  //set left motor pwm value
    pwmB = pwmMax;                                //set right motor pwm value
  }
  if(targetHeading > 0){                          //when course to target is to the right of current heading, boat should turn right
    pwmA = pwmMax;                                //set left motor pwm value
    pwmB = pwmMax - 1L*targetHeading*pwmMax/120;  //set right motor pwm value
  }
  
  if(pwmA > pwmMax)  //failsafes for wonky pwm calculations
    pwmA = pwmMax;
  if(pwmB > pwmMax)
    pwmB = pwmMax;
  if(pwmA < -pwmMax)
    pwmA = -pwmMax;
  if(pwmB < -pwmMax)
    pwmB = -pwmMax;
  
  return 0;  //autoMode function status code 0: succesfully executed but destination has not been reached yet
  
}

void printValues(){             //print control values
    Serial.print(control.up1);
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
    Serial.print(control.navlight);
    Serial.print(" ");
    Serial.print(control.intlight);
    Serial.print(" ");
    Serial.print(control.extlight);
    Serial.print(" ");
    Serial.print(control.camera);
    Serial.print(" ");
    Serial.print(control.autoMode);
    Serial.print(" ");
    
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
