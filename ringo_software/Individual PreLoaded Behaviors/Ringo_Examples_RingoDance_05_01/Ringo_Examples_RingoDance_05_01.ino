
/*

Ringo Robot
Ringo_Examples_RingoDance_05
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
// Behavior 3 Code - The Ringo Dance
// ************************************************************************************************************

#include "RingoHardware.h"

//Setup for this example
int r,g,b,freq,right,left;        // initialize variables used by this function


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  NavigationBegin();      //initialize navigation functionality. Ringo should be completely still when this happens.
  RestartTimer();
}


void loop(){ //Ver. 1.0, Kevin King  
  
  // NO EDGE DETECTION - DON'T LET RINGO JUMP OFF YOUR TABLE!

  // This is a very simple behavior that produces all kinds of personality.
  // The Arduino "Random" function is used to generate pseudo random numbers
  // which Ringo uses to dance and sing.
  // https://www.arduino.cc/en/Reference/Random
  
    r = random(120);                      // random pick Red eye component
    g = random(120);                      // random pick Green eye component
    b = random(120);                      // random pick Blue eye component
    right = random(-200,200);             // pick number between -200 and 200 for right motor speed
    left = random(-200,200);              // pick number between -200 and 200 for left motor speed
    freq = random (2000,8000);            // pick frequency between 2000 Hz and 8000 Hz for tone

    OnEyes(r,g,b);                        // set eye color
    PlayChirp(freq,100);                  // play note
    Motors(left,right);                   // set motors to the random speeds and direction
    delay(100);                           // play first note for 100 mS
    PlayChirp(random(2000,8000),100);     // pick random and play for second tone (note you can call random() in place of
                                          //                        the variable "freq". Cool!!
    delay(200);                           // play for 200 ms
    PlayChirp(random(2000,8000),100);     // set another note
    delay(300);                           // play for 300 ms
    PlayChirp(random(2000,8000),100);     // play another note, but this note plays very briefly before looping (almost can't hear it)

}



