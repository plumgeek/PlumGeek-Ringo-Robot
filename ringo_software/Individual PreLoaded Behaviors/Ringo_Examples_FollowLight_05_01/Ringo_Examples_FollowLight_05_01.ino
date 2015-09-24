
/*

Ringo Robot
Ringo_Examples_FollowLight_05
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
// Behavior 6 Code - Follow The Light
// ************************************************************************************************************

#include "RingoHardware.h"

  //Setup for this example
  int RightMotorSpeed = 120;                            //Base motor speed. Increase these numbers to make Ringo drive faster. A starting value of about 120 is good.
  int LeftMotorSpeed = 120;                             
  int TurnStrength = 30;                                //this determines how "hard" ringo will turn toward the light. Larger numbers make for sharper turns
  int Deadband = 5;                                     //if the two sensors are reading  within this "dead band" then Ringo will just go straight
  int LeftSensor, RightSensor, SensorDiff;
  int LeftSensorOffset = 0;                             //If you observe that one of the light sensors appears to be a bit more attracted to light than
  int RightSensorOffset = 0;                            //the other sensor (which is normal as there is some minor varience in the sensors and the
                                                        //amplification circuit for each side), then you can make one sensor a bit less sensitive by
                                                        //changing "LeftSensorOffset" or "RightSensorOffset" for the applicable side to a negative number. 
                                                        //Normally you don't need much to accomplish this. A value of -1 to -5 is usually enough.

                                                        //for my own Ringo, the following values produce really good results:
                                                        //RightMotorSpeed=140; LeftMotorSpeed=120; TurnStrength=30; Deadband=5; LeftSensorOffset=-3; RightSensorOffset=0;
                                                                 
void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchEdgeToAmbient();       //make sure we're reading the top light sensors (not the bottom "edge" sensors)
  Motors(LeftMotorSpeed,RightMotorSpeed);
  SetPixelRGB(TAIL_TOP,0,0,50);
  delay(2000);                 //drive "straight" for 2 seconds to observe motor drift
                               //if Ringo tends to veer to the Right, increase "RightMotorSpeed" above a bit (maybe 10 at a time)
                               //if Ringo tends to veer to the Left, increase "LeftMotorSpeed" above a bit (maybe 10 at a time
}


void loop(){ //Ver. 1.0, Kevin King  
     LeftSensor = ReadLeftLightSensor()+LeftSensorOffset;     // get some initial values for the light sensors
     RightSensor = ReadRightLightSensor()+RightSensorOffset;  // get some initial values for the light sensors
     SensorDiff = abs(LeftSensor-RightSensor);                // get the difference between the two sensors (absolute value)

     if(SensorDiff >= Deadband){                              // if the difference is greater than the allowed deadband... then begin a turning action

      if(RightSensor>LeftSensor){                                                //if the Right sensor was a larger value, begin a right turn
       Motors((LeftMotorSpeed+TurnStrength),(RightMotorSpeed-TurnStrength));     //Add POSITIVE turn strength to left, and NEGATIVE turn strength to right = right turn
       SetPixelRGB(TAIL_TOP,0,50,0);                                             //set rear tail pixel to Green to show Ringo is trying to track to the Right
       PlayChirp(NOTE_G7, 100);                                                  //play a tone
      }
      else{                                                                      //if the Right sensor was not greater, then the Left must have been. Turn left.
       Motors((LeftMotorSpeed-TurnStrength),(RightMotorSpeed+TurnStrength));     //Add NEGATIVE turn strength to left, and POSITIVE turn strength to right = left turn
       SetPixelRGB(TAIL_TOP,50,0,0);                                             //set rear tail pixel to Red to show Ringo is trying to track to the Left
       PlayChirp(NOTE_E7, 100);                                                  //play a tone
      }
     }
     else{
      Motors(LeftMotorSpeed,RightMotorSpeed);                                    //if light difference is within the dead band (both sensors picking up about the same
                                                                                 //amount of light), then drive both motors the same speed for a "straight" direction
      SetPixelRGB(TAIL_TOP,0,0,50);                                              //set rear tail pixel to Blue to show Ringo is trying to drive straight 
      OffChirp();                                                                //don't play any tone when driving straight
     }

} // end of loop();


