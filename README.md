# Project rc boat

![rcboat](https://github.com/Olaf686/RCBoat/blob/main/Pictures%20and%20videos/RC%20Boat.jpg)
This project contains resource material for an Arduino based radio controlled boat. Here you can find:

- Sketches for the boat
- Sketches for the controller
- Circuit schematics
- Pictures and video material

## Building and testing

<a href="http://www.youtube.com/watch?feature=player_embedded&v=36PeCzNrH44" target="_blank"><img src="https://github.com/Olaf686/RCBoat/blob/main/Pictures%20and%20videos/Build%20and%20test.jpg" alt="building and testing"  border="10" /></a>

## FAQ

Q: What components did you use?

A: Here is a list of main electronic components:

For the controller:
  - Arduino Uno r3
  - 74HC4067DB multiplexer
  - nRF24l01+ PA+LNA module
  - SSD1306 display
  - Random buttons from ebay
  - 2000 mAh USB powerbank
  
For the boat:
  - Arudino Mega 2560 r3
  - GY-NEO6MV2 gps
  - QMC5883L compass
  - nRF24l01+ PA+LNA module
  - L298N motor driver
  - 12V 3.4Ah lead acid battery
  <br/>
  
Q: What are the different versions of the sketches?

A: Whenever a significant amount of new functionality was added I started a new version.
Please note that controller and rcboat versions are not synced, they have their own versions.

For the controller:
- v1: Simple transmission using int array. Used with rcboat v2.
- v2: Introduction of structures, used with rcboat v3.
- v3: Added acknowledgement packets, not compatible anymore.
- v4: Added display, multiplexer and extra buttons. Used with rcboat v4.

For the boat:
- v1: Testing with motor driver and wireless connection. Direct value written to motor. Only for testing.
- v2: Introduction of PWM target values. Motors now accelerate more slowly to reduce torque on axis. Used for maiden voyage.
- v3: Transition from Uno to Mega, GPS and compass code added. Only for testing.
- v4: Autononous mode ("autoMode") functionality added. Succesfully tested in the water.
