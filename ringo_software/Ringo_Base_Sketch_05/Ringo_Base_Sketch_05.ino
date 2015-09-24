
/*

Ringo Robot
Ringo_PreLoaded_Behaviors_Rev05
Version 5.1 9/2015

IMPORTANT NOTE:  THIS SKETCH REQUIRES ARDUINO IDE 1.6.5 OR LATER TO COMPILE
(Previous versions compile this sketch too large to fit into Ringo's processor)

This sketch showcases several possible Ringo behaviors.
Use these behaviors as starting points for your own work.
When reset or turned on, press one of the number keys on your
IR remote control to activate one of the behaviors. If Ringo
blinks red, then the key was not recognized. If Ringo blinks
green, then the key was recognized and a beavior has
been started.

This code was written by Plum Geek LLC with most
significant portions written by Dustin Soodak and Kevin King.
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

void loop(){ 

  // put your code here inside the loop() function.  Here's a quick example that makes the eyes alternate colors....
  OnEyes(0,50,50);      // you can remove this stuff and put your own code here
  TxIRKey(1);
  delay(1000);          // delay 1 second (1000 milliseconds = 1 second)
  OnEyes(50,0,50);      //
  TxIRKey(1);
  delay(1000);          //
  
}


