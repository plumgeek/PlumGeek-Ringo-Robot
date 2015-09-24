
/*

Ringo Robot
Ringo_Examples_MaintainHeading_05
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
// Behavior 8 Code - Maintain Heading
// ************************************************************************************************************

#include "RingoHardware.h"

//For this example to work properly, Ringo must be completely stationary when you press the
//button on the remote to begin the behavior. Even very small movements (at the very start
//of the behavior, where "NavigationBegin()" is called) will result in the gyroscope
//exhibiting excessive drift. These small movements can come from a fan running on your desk,
//music playing from your speakers, bumping your desk, etc.
//If you want to see gyro drift over time in real time, start this behavior and let Ringo
//sit still on a desk for 15~30 minutes. You may notice he slowly tends to rotate one
//direction or the other.

//Press User button one time to re-set the direction Ringo wants to face.

//Setup for this example
int Heading,Skid,MotorSpeed; //variables we'll use for this example
                                                                 
void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  OnEyes(30,30,0);        //eyes yellow
  delay(1000);            //give you a second to let go of Ringo so he is still when navigation is calibrated
  OnEyes(0,50,0);
  OffEyes();
  NavigationBegin();      //initialize navigation hardware
}


void loop(){ //Ver. 1.0, Dustin Soodak  //This example uses Ringo's gyroscope to maintain his initial heading.
  
  SimpleGyroNavigation();      //this function reads the gyro and does math. This needs to be
                               //called at least every 80 milliseconds  

  Heading=PresentHeading();    //returns present heading (in pos or neg degrees relative to start point)         
  Skid=GetDegreesToStop();     //Skid is based on present rate of rotation and guestimates how many
                               //degrees Ringo will "skid" if motors are stopped immediately. This is
                               //used later to gradually slow the motors as Ringo reaches his starting
                               //heading.  
  if(abs(Heading)<2){          //if heading is within +/- 2 degrees of starting point, do nothing, just sit still
    Motors(0,0);
  }
  else{                                //else, try to move back to starting heading
    MotorSpeed=50+abs(Heading+Skid);   //base motor speed is 50. Add to this the absolute value of the present heading
                                       //(which is how far away from the zero point you are), plus the expected skid time
                                       //Remember Ringo will be doign this calculation constantly has he spins back to the
                                       //starting heading. As he approaches the target heading, the offset will become
                                       //less and thus the motors will move progressively slower as he returns to zero.
    if(MotorSpeed>230){                //make sure motor speed is not greater than 230 (this allows fast speed but limits 
      MotorSpeed=230;                  //the amount of eventual overshoot
    } 
    if(Heading+Skid>0){                  //if the heading offset is greater than zero (positive rotation).... then.....
      Motors(-MotorSpeed,MotorSpeed);    //left motor backward, right motor forward => left hand turn
    } 
    else{                                //if the heading offset is less than zero (negative rotation).... then.....
      Motors(MotorSpeed,-MotorSpeed);    //left motor forward, right motor backward => right hand turn
      OnEyes(25,0,50);                   
    }
  }


  //Indicate where robot is with eyes
  if(abs(Heading)<2){                    //if heading is within +/- 2 degrees of starting point, do nothing, just sit still
    OffPixels();  
  }
  else if(Heading+Skid>0){               //if the heading offset is greater than zero (positive rotation).... then.....
    OnEyes(50,0,25);
  } 
  else{
    OnEyes(25,0,50);                   //left motor forward, right motor backward => right hand turn
  }
  //re-calibrate gyro zeroes if button pressed
  if(ButtonPressed()){         //go see if the user button has been pressed.  If so re-set heading.
    Motors(0,0);
    while(ButtonPressed());   //wait for the button to be released
    OffPixels();
    OnEyes(30,30,0);           //eyes yellow
    delay(1000);
    OnEyes(0,50,0);
    NavigationBegin();         //Initialize navigation. Be completely still here!!!
    OffPixels();
  }

} // end of loop();



