
/*

Ringo Robot
Ringo_Examples_SpinSqueek_05
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
// Behavior 4 Code - Spin Squeek
// ************************************************************************************************************

#include "RingoHardware.h"

//Setup for this example
int RightLast, RightNow, LeftLast, LeftNow;
int RightDiff, LeftDiff;

int TriggerDifference = 25;   // this sets how sensitive the trigger is. Making this lower will make it more sensitive
                              // (to the point it is triggering on its own just due to noise), and making the value
                              // higher will make the trigger less sensitive
                              
void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchEdgeToAmbient();               //make sure we're reading the top light sensors (not the bottom "edge" sensors)
  LeftLast = ReadLeftLightSensor();    // get some initial values for the light sensors
  RightLast = ReadRightLightSensor();  // get some initial values for the light sensors
}


void loop(){ //Ver. 1.0, Kevin King  
  
   LeftNow = ReadLeftLightSensor();       // go read the sensors (light level). We will use these two readings for the rest of the loop
   RightNow = ReadRightLightSensor();     // go read the sensors (light level). We will use these two readings for the rest of the loop
   LeftDiff = LeftLast-LeftNow;           // determine the difference (drop in light) between this read and the previous read
   RightDiff = RightLast-RightNow;        // determine the difference (drop in light) between this read and the previous read

   SwitchMotorsToSerial();                // enable the serial port
   Serial.print(LeftNow);Serial.print("/");Serial.print(LeftDiff);    //if you open your serial monitor, you can see the values
   Serial.print("   ");                                               //being read by Ringo. This can be useful for debugging your bug. ;p
   Serial.print(RightNow);Serial.print("/");Serial.print(RightDiff);  //
   Serial.println();                                                  //print a blank line feed    
   
   if(((LeftDiff)>TriggerDifference) || ((RightDiff)>TriggerDifference)){  // trigger if either light sensor drops by more than
                                                                           // the TriggerDifference
                                                                              
       if((LeftDiff)>(RightDiff)){    //if the left drop was greater than the right drop, spin to the left
         Motors(-240,240);            //motor speed. Change this number to make motors spin faster or slower. If the numbers are
                                      //different, Ringo can drive an arc when triggered.
         OnEyes(75,0,75);             //set the eyes to 75 red, 0 green, 75 blue (red+blue=purple) Adjust these numbers to adjust eye color.
         PlaySweep(4000, 1000, 330);  //sweep tone from 4000 hertz to 1000 hertz, 330 microseconds dwell on each tone. Adjust these numbers
                                      //to adjust the sweep. Ringo won't do anything else until the sweep is complete, so if you make
                                      //the 330 microseconds too long, or give too wide a range, he may take a long time to complete the sweep.
         Motors(0,0);                 //Ringo will spin in a circle until the sweep completes, then turn off motors
         OffPixels();                 //turn off all pixels
         delay(200);
         //LeftLast = ReadLeftLightSensor();    // get some initial values for the light sensors
         //RightLast = ReadRightLightSensor();  // get some initial values for the light sensors
       }
       else{                          //if the left drop was not greater, then right must have been greater. 
         Motors(240,-240);            //motor speed. Change this number to make motors spin faster or slower. If the numbers are
                                      //different, Ringo can drive an arc when triggered.
         OnEyes(75,75,0);             //set the eyes to 75 red, 75 green, 0 blue (red+green=yellow) Adjust these numbers to adjust eye color.
         PlaySweep(1000, 4000, 330);  //sweep tone from 1000 hertz to 4000 hertz, 330 microseconds dwell on each tone. Adjust these numbers
                                      //to adjust the sweep. Ringo won't do anything else until the sweep is complete, so if you make
                                      //the 330 microseconds too long, or give too wide a range, he may take a long time to complete the sweep.
         Motors(0,0);                 //Ringo will spin in a circle until the sweep completes, then turn off motors
         OffPixels();                 //turn off all pixels
         delay(200);
         //LeftLast = ReadLeftLightSensor();    // get some initial values for the light sensors
         //RightLast = ReadRightLightSensor();  // get some initial values for the light sensors
       }
   } // end "if triggered" function
   LeftLast = LeftNow;
   RightLast = RightNow;
   delay(200);              // delay a short time before taking the next reading

}



