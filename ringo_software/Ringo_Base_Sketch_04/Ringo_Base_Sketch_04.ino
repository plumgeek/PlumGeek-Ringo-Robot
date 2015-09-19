/*

Ringo Robot
Ringo_Base_Sketch_04
Version 4.0 9/2015

This is a basic sketch that can be used as a starting point
for various functionality of the Ringo robot.

This code was written by Plum Geek LLC with most
significant portions written by Dustin Soodak.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/
Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RestartTimer();
}

int i;

void loop(){ 
  

Motors(250, 0);
delay(4000);
Motors(-250, 0);
delay(4000);

}




