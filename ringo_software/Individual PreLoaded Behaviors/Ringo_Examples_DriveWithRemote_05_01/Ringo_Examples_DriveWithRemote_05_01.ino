
/*

Ringo Robot
Ringo_Examples_DriveWithRemote_05
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
// Behavior 1 Code - DriveWithRemote
// ************************************************************************************************************

#include "RingoHardware.h"

//Setup for this example
#define EYE_COLORS 6
byte EyeColors[EYE_COLORS][3]={
{20,0,80},          //These are the 6 colors (Red, Green, Blue) each eye can index 
{80,0,20},          //through.  Edit them if you like. You can add more or less to
{0,70,30},          //this list, just edit the EYE_COLORS count in the the #define above.
{0,0,100},          //Example: If you want to have only 4 possibe colors, delete
{60,60,60},         //two of these, then change the #define above from 6 to 4.
{35,35,0}
};

byte button;
byte RightEyeIndex=0, LeftEyeIndex=0;  //keeps track of which of the above colors the
                                       //left and right eyes are set to at any give time


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  RxIRRestart(4);         //wait for 4 byte IR remote command
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
}


void loop(){ 
  
   //This behavior allows simple "driving" with the remote control buttons
   //This is also a great example of how to make Ringo react to different
   //button presses.

  if(IsIRDone()){      //wait for an IR remote control command to be received
    button = GetIRButton();  // read which button was pressed, store in "button" variable
    if(button){              // if "button" is not zero...
      switch (button){       // activate a behavior based on which button was pressed

       case 11:              // FWD key, drive forward
       NavigationBegin();
       ZeroNavigation();
       PlayAck();
       MoveWithOptions(0, 100, 200, 2000, 500, 0, 0);
       break;

       case 14:               // BACK key, drive backward
       NavigationBegin();
       ZeroNavigation();
       PlayAck();
       MoveWithOptions(0, -100, -200, 2000, 500, 0, 0);
       break;

       case 12:               // Left key, turn left
       NavigationBegin();
       ZeroNavigation();
       PlayAck();
       RotateAccurate(-90, 1000);
       break;

       case 13:               // Right key, turn right
       NavigationBegin();
       ZeroNavigation();
       PlayAck();
       RotateAccurate(90, 1000);
       break;

       case 18:               // "A" key, Change Left eye color
       PlayChirp(NOTE_C7,100);
       LeftEye(EyeColors[LeftEyeIndex][0],EyeColors[LeftEyeIndex][1],EyeColors[LeftEyeIndex][2]);
       LeftEyeIndex+=1;
       if ((LeftEyeIndex-1)>EYE_COLORS){
        LeftEyeIndex=0;
       }
       delay(10);
       OffChirp();
       break;

       case 19:               // "B" key, Change Right eye color
       PlayChirp(NOTE_C7,100);
       RightEye(EyeColors[RightEyeIndex][0],EyeColors[RightEyeIndex][1],EyeColors[RightEyeIndex][2]);
       RightEyeIndex+=1;
       if ((RightEyeIndex-1)>EYE_COLORS){
        RightEyeIndex=0;
       }
       delay(10);
       OffChirp();
       break;

       case 21:               // "X" key, Change Both Eyes Color
       PlayChirp(NOTE_C7,100);
       LeftEye(EyeColors[RightEyeIndex][0],EyeColors[RightEyeIndex][1],EyeColors[RightEyeIndex][2]);
       RightEye(EyeColors[RightEyeIndex][0],EyeColors[RightEyeIndex][1],EyeColors[RightEyeIndex][2]);
       RightEyeIndex+=1;
       if ((RightEyeIndex-1)>EYE_COLORS){
        RightEyeIndex=0;
       }
       LeftEyeIndex=RightEyeIndex;
       delay(10);
       OffChirp();
       break;
       
       case 1:               // play tone
       PlayChirp(NOTE_C7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 2:               // play tone
       PlayChirp(NOTE_D7, 100);
       delay(75);
       PlayChirp(0,0);         
       break;

       case 3:               // play tone
       PlayChirp(NOTE_E7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 4:               // play tone
       PlayChirp(NOTE_F7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 5:               // play tone
       PlayChirp(NOTE_G7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 6:               // play tone
       PlayChirp(NOTE_A7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 7:               // play tone
       PlayChirp(NOTE_B7, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 8:               // play tone
       PlayChirp(NOTE_C8, 100);
       delay(75);
       PlayChirp(0,0);
       break;

       case 10:               // play string
       PlayChirp(NOTE_C7, 100);
       OnEyes(80,0,20);
       delay(75);
       PlayChirp(NOTE_D7, 100);
       OnEyes(0,70,30);
       delay(75);
       PlayChirp(NOTE_E7, 100);
       OnEyes(0,0,100);
       delay(75);
       PlayChirp(NOTE_F7, 100);
       OnEyes(80,0,20);
       delay(75);
       PlayChirp(NOTE_G7, 100);
       OnEyes(0,70,30);
       delay(75);
       PlayChirp(NOTE_A7, 100);
       OnEyes(0,0,100);
       delay(75);
       PlayChirp(NOTE_B7, 100);
       OnEyes(80,0,20);
       delay(75);
       PlayChirp(NOTE_C8, 100);
       OnEyes(0,70,30);
       delay(75);
       PlayChirp(0,0);
       LeftEye(EyeColors[LeftEyeIndex][0],EyeColors[LeftEyeIndex][1],EyeColors[LeftEyeIndex][2]);
       RightEye(EyeColors[RightEyeIndex][0],EyeColors[RightEyeIndex][1],EyeColors[RightEyeIndex][2]);
       
       break;

       default:     // if no match, break out and wait for a new code
       break;
      }
    Serial.println(button);  // send button number pressed to serial window
    }
  }//end if(IsIRDone())

}


