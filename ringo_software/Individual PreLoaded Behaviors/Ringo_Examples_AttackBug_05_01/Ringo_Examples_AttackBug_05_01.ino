
/*

Ringo Robot
Ringo_Examples_AttackBug_05
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
// Behavior 2 Code - Attack Bug
// ************************************************************************************************************

#include "RingoHardware.h"

//Setup for this example
int i,n,accelx,accely,bumpdir,heading,currentheading,degr;
int32_t largenum;


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  NavigationBegin();      //initialize navigation functionality. Ringo should be completely still when this happens.
  RestartTimer();
}


void loop(){ 
  
//Ver. 1.0, Dustin Soodak    
//NOTE: Some lines of code are commented out. Uncomment them for debugging.
         
  n=AccelBufferSize();    
  if(n){
    AccelGetAxes(AccelAcceleration);
    AccelAcceleration[0]=AccelAcceleration[0]-AccelZeroes[0];//x-axis raw
    AccelAcceleration[1]=AccelAcceleration[1]-AccelZeroes[1];//y-axis raw
  }
  accelx=-AccelAcceleration[0];//still raw x, but facing correct direction
  accely=-AccelAcceleration[1];//still raw y, but facing correct direction
  
  //if(GetTime()>200){                                                            //Debugging Code
  //  Serial.print(accelx,DEC);Serial.print(" ");Serial.println(accely,DEC);
  //  RestartTimer(); 
  //}
  
  if(abs(accelx)>4000 || abs(accely)>4000){                 //This is how sensitve the "poke" trigger is. 4000 is default.
    bumpdir=180+(90-atan2(accely,accelx)*180/3.14159);
    heading=GetDegrees()+bumpdir;
    RestartTimer();
    OnEyes(50,0,0);                                         //make eyes red when poked. You can change this color if you like.

    while(GetTime()<100){//wait till is is most likely skidding
      SimpleGyroNavigation();//look at gyroscope as it turns
    }
    AccelGetAxes(AccelAcceleration);
    AccelAcceleration[0]=AccelAcceleration[0]-AccelZeroes[0];//x-axis raw
    AccelAcceleration[1]=AccelAcceleration[1]-AccelZeroes[1];//y-axis raw
    largenum=((int32_t)accelx)*(-AccelAcceleration[0])+((int32_t)accely)*(-AccelAcceleration[1]);//dot product of old & new {xaccel,yaccel}
    if(largenum>0){//largenum is now = cosine of angle between new and old multiplied by their amplitudes
      bumpdir-=180;//if skid direction within 180 degrees of bump direction, then "bump" was probably actually part of skid
      //or can put code here to check again in 20ms, just in case this is just an artifact of vibration        
    }
    while(GetTime()<400){
      SimpleGyroNavigation();//look at gyroscope as it turns
    }
    CalibrateNavigationSensors();//assuming that it will be stopped by this point, so safe to use this function    
    //Serial.print(accelx,DEC);Serial.print(" ");Serial.println(accely,DEC);
    currentheading=GetDegrees();
    degr=heading-currentheading;
    //the following makes sure it turns the least distance (in degrees) to point at the source of the disturbance
    if(degr>=0){
      if(degr>=360) degr-=360;
      if(degr>180) degr=-(360-degr);
    }
    else{
      if(degr<=-360) degr+=360;
      if(degr<-180) degr=(360-(-degr));
    }
    SwitchMotorsToSerial();
    
    //Serial.print(" reldir ");Serial.print(bumpdir,DEC);
    //Serial.print(" absdir ");Serial.print(heading,DEC);
    //Serial.print(" now facing ");Serial.print(currentheading,DEC);
    //Serial.print(" turn ");Serial.print(degr,DEC);
    //Serial.print(" (");Serial.print(largenum,DEC);Serial.println(")");
    //behavior: (where currentheading+degr is the heading of the offending poke)
    
    //This is what Ringo does after being poked
    RotateAccurate(currentheading+degr,2000);           //rotate to the direciton he thinks he was poked by ("degr")
    delay(200);
    OnEyes(50,0,0);                                     //make eyes Red. You can change this if you like
    Motors(200,200);                                    //drive motors at speed 240 for "initial strike" 200 to 255 here are good values
    delay(200);                                         //making this value larger will make the "initial strike" go further
    PlayAnger();                                        //play anger sound. You can edit PlayAnger() in the "FunStuff" tab
    for(i=0;i<3;i++){                                   //This is how many "bites" Ringo does (how many times through loop)
                                                        //Default is 3.  You can change this. Each "bite" will do the following
      Motors(-200,-200);                                //drive motors to "recoil". 200 to 255 here are good values      
      OnEyes(50,50,0);                                  //make Ringo's eyes Yellow.  You can change this if you like.
      PlayChirp(NOTE_C6,100);                           //this is one of the tones made during the "bite". Change if you like.  
      delay(100);                                       //making this value larger will make Ringo "recoil" further
      Motors(200,200);                                  //drive motors to "attack". 200 to 255 here are good values
      OnEyes(50,0,0);                                   //make Ringo's eyes Red.  You can change this if you like.
      PlayChirp(NOTE_CS6,100);                          //this is the other tone made during the "bite". Change if you like.
      delay(100);                                       //making this value larger will make Ringo "strike" further
    }                                                   //end of "bite"
    
    Motors(-200,-200);                                  //this is how fast Ringo will "back away" after "biting"
    PlayAnger();                                        //one final "anger" tone
    delay(200);                                         //making this value larger will make Ringo "back away" further after "biting"
    Motors(0,0);                                        //stop motors  
    delay(500);                                         //to let any movement or rocking settle
    OffPixels();                                        //turn off pixels
    SwitchMotorsToSerial();
    //end don't touch me      
  }//end if(abs(accelx)>4000 || abs(accely)>4000)

}


