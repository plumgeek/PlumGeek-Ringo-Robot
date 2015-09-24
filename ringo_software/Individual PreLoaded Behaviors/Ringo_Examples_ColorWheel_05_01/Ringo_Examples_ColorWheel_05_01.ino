
/*

Ringo Robot
Ringo_Examples_ColorWheel_05
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
// Behavior 5 Code - Color Wheel
// ************************************************************************************************************

#include "RingoHardware.h"

//Setup for this example
int MotorTime=4000;       //how long the motors run after pressing the button
int redIntensity = 0;
int greenIntensity = 0;
int blueIntensity = 0;
int red;
int green;
int blue;
int presentDirection = 0;
int hue = 0;
int hueOpposite = 180;
                              
void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  NavigationBegin();      //initialize and start navigation
  RestartTimer(); 
}


void loop(){ //Ver. 1.0, Kevin King  
      SimpleGyroNavigation();              //just does gyroscope (not accelerometer)
      presentDirection = abs(GetDegrees());     //gets the present heading in degrees, result is between -180 ~ +180
      
      presentDirection = presentDirection%360;      //If it's been turned multiple times, this divides the large degree number back into a 360 base 
      hue = presentDirection;            //Eduardo's function conveniently takes the same 360 degree value we have related to the gyro
      hueOpposite = (hue + 180)%360;     //Get the opposite color on the wheel
      
      SwitchButtonToPixels();            //Make sure this line is in Pixels mode before attempting to update the pixels
      
      setLedColorHSV(hue,1,1);           //Calculate the RGB values based on hue
      OnEyes(red,green,blue);            //Put those colors on the eyes
            
      //Set rear pixel to opposite color   //<--- uncomment the next 2 lines to show the complementary color on the rear pixel
      //setLedColorHSV(hueOpposite,1,1);   //Calculate the RGB values based on hueOpposite
      //SetPixelRGB(0,red,green,blue);     //Top rear NeoPixel
      
      RefreshPixels();                   //Update the pixels on the string
      
      SwitchPixelsToButton();
      if(ButtonPressed()){
        delay(10);//debounce button
        while(ButtonPressed())
          SimpleGyroNavigation();
        RestartTimer();
        while(GetTime()<500){//to let user get finger away
          SimpleGyroNavigation();
          } 
        SwitchButtonToPixels();  
        SwitchSerialToMotors();
        Motors(100,-100);
        RestartTimer();    
      }
      
      if(GetTime()>MotorTime){
        Motors(0,0);
        RestartTimer();
      }

} // end of loop();


//Sets the current color for the RGB LED
void setLedColor(int red, int green, int blue) {
  //Note that we are reducing 1/4 the intensity for the green and blue components because 
  //  the red one is too dim on my LED. You may want to adjust that.
  SetPixelRGB(5,red,green,blue);
  SetPixelRGB(6,red,green,blue);
  RefreshPixels();
  //analogWrite(9,red); //Red pin attached to 9
  //analogWrite(10,green/3); //Red pin attached to 9
  //analogWrite(11,blue/3); //Red pin attached to 9
}

//Convert a given HSV (Hue Saturation Value) to RGB(Red Green Blue) and set the led to the color
//  h is hue value, integer between 0 and 360
//  s is saturation value, double between 0 and 1
//  v is value, double between 0 and 1
//http://splinter.com.au/blog/?p=29
void setLedColorHSV(int h, double s, double v) {
  //this is the algorithm to convert from RGB to HSV
  double r=0; 
  double g=0; 
  double b=0;

  double hf=h/60.0;

  int i=(int)floor(h/60.0);
  double f = h/60.0 - i;
  double pv = v * (1 - s);
  double qv = v * (1 - s*f);
  double tv = v * (1 - s * (1 - f));

  switch (i)
  {
  case 0: //rojo dominante
    r = v;
    g = tv;
    b = pv;
    break;
  case 1: //verde
    r = qv;
    g = v;
    b = pv;
    break;
  case 2: 
    r = pv;
    g = v;
    b = tv;
    break;
  case 3: //azul
    r = pv;
    g = qv;
    b = v;
    break;
  case 4:
    r = tv;
    g = pv;
    b = v;
    break;
  case 5: //rojo
    r = v;
    g = pv;
    b = qv;
    break;
  }

  //set each component to a integer value between 0 and 255
  red=constrain((int)255*r,0,255);
  green=constrain((int)255*g,0,255);
  blue=constrain((int)255*b,0,255);

} //end of Behavior_ColorWheel() helper functions



