
/*

Ringo Robot
Ringo_Examples_DriveSquare_05
Version 5.1 9/2015

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

// ************************************************************************************************************
// Behavior 7 Drive a Square
// ************************************************************************************************************

#include "RingoHardware.h"

                                                                 
void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  delay(2000);            //give you a second to let go of Ringo so he is still when navigation is calibrated
  NavigationBegin();      //initialize navigation hardware
  RandomEyes();           //set eyes to an initial random value
}


void loop(){ //Ver. 1.0, Kevin King  

    SwitchSerialToMotors();     // make sure we're setup to run the motors    
    MoveWithOptions(0, 200, 200, 3000, 1000, 0,0);  //direction, distance, speed
    PauseNavigation();
    RandomEyes();
    delay(150);
    ResumeNavigation();
    RotateAccurate(90,2000);//exact rotation
    RandomEyes();
    delay(150);
    CalibrateNavigationSensors();
    ZeroNavigation();

} // end of loop();



