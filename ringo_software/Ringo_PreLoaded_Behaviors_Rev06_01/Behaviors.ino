

// ************************************************************************************************************
// Behavior 1 Code - DriveWithRemote
// ************************************************************************************************************

void Behavior_DriveWithRemote(void){ //Ver. 3.0, Kevin King    //This behavior allows simple "driving" with the remote control buttons
                                                               //This is also a great example of how to make Ringo react to different
                                                               //button presses.
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

 PlayAck();                   //play the Ack tone
 
 byte button;
 byte RightEyeIndex=0, LeftEyeIndex=0;  //keeps track of which of the above colors the
                                        //left and right eyes are set to at any give time
 while(1){ 

  if(IsIRDone()){      //wait for an IR remote control command to be received
    button = GetIRButton();  // read which button was pressed, store in "button" variable
    if(button){              // if "button" is not zero...
      switch (button){       // activate a behavior based on which button was pressed

       case 11:              // FWD key, drive forward

       Motors(200,200);
       RxIRRestart(4);
       PlayAck();
       delay(150);
       Motors(0,0);
       
       //NavigationBegin();
       //ZeroNavigation();
       //PlayAck();
       //MoveWithOptions(0, 10, 200, 250, 50, 0, 0);
       break;

       case 14:               // BACK key, drive backward

       Motors(-200,-200);
       PlayAck();
       delay(150);
       Motors(0,0);
       
       //NavigationBegin();
       //ZeroNavigation();
       //PlayAck();
       //MoveWithOptions(0, -10, -200, 250, 50, 0, 0);
       break;

       case 12:               // Left key, turn left

       Motors(-200,200);
       PlayAck();
       Motors(0,0);

       //NavigationBegin();
       //ZeroNavigation();
       //PlayAck();
       //RotateAccurate(-5, 200);
       break;

       case 13:               // Right key, turn right

       Motors(200,-200);
       PlayAck();
       Motors(0,0);

       //NavigationBegin();
       //ZeroNavigation();
       //PlayAck();
       //RotateAccurate(5, 200);
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
    RxIRRestart(4);
    }
  }//end if(IsIRDone())

    if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
      OffPixels();
      PlayStartChirp();
      break;
    }
 } // end while(1); loop
}  // end Behavior_DriveWithRemote()



// ************************************************************************************************************
// Behavior 2 Code - Attack Bug
// ************************************************************************************************************

void Behavior_AttackBug(void){ //Ver. 1.0, Dustin Soodak    //NOTE: Some lines of code are commented out. Uncomment them for debugging.
  //Setup for this example
  extern int32_t AverGyroVelocity;
  int i,n,accelx,accely,bumpdir,heading,currentheading,degr;
  int32_t largenum;
  int AccelAcceleration[3];
  int AccelZeroes[3];
  
  while(1){
    
  PlayAck();                //indicate that a command was received 
                            //(you can customize what happens here by editing the PlayAck() function in the "FunStuff" tab 
  NavigationBegin();        //initialize navigation functionality. Ringo should be completely still when this happens.
  RestartTimer();
  
  while(1){
    
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

    if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
      OffPixels();
      PlayStartChirp();
      break;
    }
  }//end while(1)  
  break;
  }  // end pseudo loop while(1)
}    // end Behavior_AttackBug();



// ************************************************************************************************************
// Behavior 3 Code - The Ringo Dance
// ************************************************************************************************************

void Behavior_TheRingoDance(void){ //Ver. 1.0, Kevin King  
  
                                    // Does what it says. NO EDGE DETECTION - DON'T LET RINGO JUMP OFF YOUR TABLE!

                                    // This is a very simple behavior that produces all kinds of personality.
                                    // The Arduino "Random" function is used to generate pseudo random numbers
                                    // which Ringo uses to dance and sing.
                                    // https://www.arduino.cc/en/Reference/Random
                                    
  int r,g,b,freq,right,left;        // initialize variables used by this function

  while(1){
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
  } // end of while(1);

} // end of Behavior_TheRingoDance();


// ************************************************************************************************************
// Behavior 4 Code - Spin Squeek
// ************************************************************************************************************

void Behavior_SpinSqueek(void){ //Ver. 1.0, Kevin King

 PlayAck();   //indicate that a command was received 
              //(you can customize what happens here by editing the PlayAck() function in the "FunStuff" tab 
 
 int RightLast, RightNow, LeftLast, LeftNow;
 int RightDiff, LeftDiff;

 int TriggerDifference = 25;    // this sets how sensitive the trigger is. Making this lower will make it more sensitive
                                // (to the point it is triggering on its own just due to noise), and making the value
                                // higher will make the trigger less sensitive

 SwitchEdgeToAmbient();         //make sure we're reading the top light sensors (not the bottom "edge" sensors)
 
 LeftLast = ReadLeftLightSensor();    // get some initial values for the light sensors
 RightLast = ReadRightLightSensor();  // get some initial values for the light sensors
 
 while(1){

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
   delay(200);
   
    if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
      OffPixels();
      PlayStartChirp();
      break;
    }
 } // end while(1);
}  // end Behavior_SpinSqueek();



// ************************************************************************************************************
// Behavior 5 Code - Color Wheel
// ************************************************************************************************************

void Behavior_ColorWheel(void){ //Ver. 1.0, Kevin King      //This behavior uses the gyroscope to create a virtual "color wheel".  As you manually
                                                            //rotate Ringo around in a circle, his eyes change hue to match the corresponding color
                                                            //on a color wheel pointed that direction. Press the User Button once to turn on the
                                                            //motors for about one revolution to see how the colors change automatically.
                                                            //NOTE:  There is a bug in this code somewhere - if Ringo is rotated more than negative
                                                            //one turn from the start, the eyes will go out, but will turn back on when rotated
                                                            //again in the positive direction. He can spin many rotations in the positive direction
                                                            //with no problems. Extra Credit: The first person to figure out what's causing this
                                                            //and correct it will get a high five and three bonus points in the League of Awesomeness.
    PlayAck();
    NavigationBegin();//initialize and start navigation
    RestartTimer();  

    int MotorTime=4000;                    //how long the motors run after pressing the button
    while(1){
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
  }// end of while(1);
} //end of Behavior_ColorWheel() - there are several helper functions below

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



// ************************************************************************************************************
// Behavior 6 Code - Follow The Light
// ************************************************************************************************************

void Behavior_FollowLight(){ //Ver. 1.0, Kevin King     //This example Ringo follows the light. He does not use his navigation sensors to remain pointed in a straight
                                                        //direction. This is a really simple example of how a robot can track lights.

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
                                                        
  PlayAck();                   //play the Ack tone
  SwitchEdgeToAmbient();       //make sure we're reading the top light sensors (not the bottom "edge" sensors)


  Motors(LeftMotorSpeed,RightMotorSpeed);
  SetPixelRGB(TAIL_TOP,0,0,50);
  delay(2000);                 //drive "straight" for 2 seconds to observe motor drift
                               //if Ringo tends to veer to the Right, increase "RightMotorSpeed" above a bit (maybe 10 at a time)
                               //if Ringo tends to veer to the Left, increase "LeftMotorSpeed" above a bit (maybe 10 at a time)
                               
  while(1){

     LeftSensor = ReadLeftLightSensor()+LeftSensorOffset;     // get some initial values for the light sensors
     RightSensor = ReadRightLightSensor()+RightSensorOffset;  // get some initial values for the light sensors
     SensorDiff = abs(LeftSensor-RightSensor);                // get the difference between the two sensors (absolute value)

     if(SensorDiff >= Deadband){                              // if the difference is greater than the allowed deadband... then begin a turning action

       if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
         Motors(0,0);
         OffPixels();
         PlayStartChirp();
         break;
      }
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

    
     
  }//end of while(1);
}    // end Behavior_FollowLight();



// ************************************************************************************************************
// Behavior 7 Drive a Square
// ************************************************************************************************************
void Behavior_DriveSquare(){ //Ver. 2.0, Kevin King     //This example uses Ringo's front facing IR LED together with his ambient
                                                         //light sensors to sense and react to objects in front of him.
  //PlayAck();                   //play the Ack tone
  //delay(50);
  RandomEyes();
  NavigationBegin();
  while(1){

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

    if(CheckMenuButton()){
      OffPixels();
      PlayNonAck();
      break;
    }
    
  }//end of while(1);
}    // end Behavior_DriveSquare();



// ************************************************************************************************************
// Behavior 8 Code - Maintain Heading
// ************************************************************************************************************

void Behavior_MaintainHeading(){ //Ver. 1.0, Dustin Soodak  //This example uses Ringo's gyroscope to
                                                            //maintain his initial heading.
  
  //For this example to work properly, Ringo must be completely stationary when you press the
  //button on the remote to begin the behavior. Even very small movements (at the very start
  //of the behavior, where "NavigationBegin()" is called) will result in the gyroscope
  //exhibiting excessive drift. These small movements can come from a fan running on your desk,
  //music playing from your speakers, bumping your desk, etc.
  //If you want to see gyro drift over time in real time, start this behavior and let Ringo
  //sit still on a desk for 15~30 minutes. You may notice he slowly tends to rotate one
  //direction or the other.
  
  NavigationBegin();           //Initialize navigation. Be completely still here!!!
  int Heading,Skid,MotorSpeed; //variables we'll use for this example
  
  PlayAck();                   //play the Ack tone
  ResumeNavigation();
  
  while(1){ // loop inside this function
  
  SimpleGyroNavigation();      //this function reads the gyro and does math. This needs to be
                               //called at least every 80 milliseconds  

  Heading=PresentHeading();    //returns present heading (in pos or neg degrees relative to start point)         
  Skid=GetDegreesToStop();     //Skid is based on present rate of rotation and guestimates how many
                               //degrees Ringo will "skid" if motors are stopped immediately. This is
                               //used later to gradually slow the motors as Ringo reaches his starting
                               //heading.  
  if(abs(Heading)<2){             //if heading is within +/- 2 degrees of starting point, do nothing, just sit still
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

  if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
    OffPixels();
    PlayStartChirp();
    break;
  }
}// end of while(1);
}    // end Behavior_MaintainHeading();



// ************************************************************************************************************
// Behavior 9 Code - Follow The Line
// ************************************************************************************************************

void Behavior_FollowLine(){ //Ver. 1.0, Kevin King     //This example Ringo follows a line. He does not use his navigation sensors to remain pointed in a straight
                                                       //direction. This is a really simple example of how a robot can track lines.
                                                       //It is suggested you run this behavior on a large piece of white posterboard paper. Use a heavy black
                                                       //marker to draw a "race track" for Ringo to follow. The black line should be about 1 centemeter (about 1/2 inch)
                                                       //wide. Following a line is actually a bit more tricky than you would expect. This example shows you
                                                       //one possible technique. There are many others. Play with speeds, turn strength, and loop hold times
                                                       //to tune this behavior.
  //Setup for this example
  char edge;                              //holds a binary representation of which edge was triggered, and whether the trigger was bright or dark
  
  int LoopHold=1;                         //See notes below. Set this value low and Ringo will need to have a more abrupt encounter with a line to see it,
                                          //but he will react to the line more quickly. Set this value higher and Ringo is more likely to see a line
                                          //or more suttle differences in the surface, but he'll react more slowly, so he may over-shoot the line.
                                          //60 seems to be a good starting point, with values between 5 and 150 being suggested limits.
  
  int RightMotorSpeed = 100;              //Base motor speed. Increase these numbers to make Ringo drive faster. A starting value of about 75 is good.
  int LeftMotorSpeed = 100;               //If you notice Ringo wants to "veer off" in one direction, then increase the motor speed in that direction a bit
                                          //For example, if he wants to veer off to the right, then increase the RightMotorSpeed a bit (about 5). If
                                          //he is still veering off, go a little larger.
                                                       
  int TurnStrength = 60;                  //This determines how "hard" ringo will turn to correct when a line is seen. Larger numbers make for snappier
                                          //corrective turns, but can also make Ringo over-shoot so far that he can't see the line anymore.
  int RightAverageHold;
  int LeftAverageHold;
  
                                                          
  PlayAck();                   //play the Ack tone
  SwitchAmbientToEdge();       //make sure we're reading the bottom "edge" sensors (not the top "ambient" sensors)


  Motors(LeftMotorSpeed,RightMotorSpeed);   // start driving!
                               
  while(1){                                 // keep looping inside this "while"

     OffPixels();                           // turn off all pixels
     edge=LookForEdge();                    // See if any edge sensors have seen bright or dark transitions on the edge sensors
   
     if(LeftDetected(edge)){                // if the Left edge sensor detected a change (from bright to dark, or also from dark to bright)
      LeftEye(0,50,0);                      // left eye green
      Motors(LeftMotorSpeed-TurnStrength,RightMotorSpeed+TurnStrength);  //offset the motor speeds by TurnStrength to produce a turn the opposite direction
      LeftAverageHold=LeftEdgeSensorAverage;                             //grab a snapshot of the present EdgeSensorAverage. We'll compare against
                                                                         //this in the next bit of code
      while(LeftEdgeSensorValue<LeftAverageHold+10){                        //We'll stay in this "while" until the EdgeSensorValue returns to the previous
                                                                         //average (which happens when his light sensor moves back off the line
                                                                         //he is tracking.
       LookAtEdge();                                                     //LookAtEdge updates "LeftEdgeSensorValue" to what the sensor presently sees.
      }
      ResetLookAtEdge(); // reset the running average
     }
     else if(RightDetected(edge)){          //"else if" gives the above "if" another option. Read up on "else if" at www.arduino.com. It's super useful.
      RightEye(0,50,0);
      Motors(LeftMotorSpeed+TurnStrength,RightMotorSpeed-TurnStrength);  //This part is the same as for the Left side above, only reversed.
      RightAverageHold=RightEdgeSensorAverage;
      while(RightEdgeSensorValue<RightAverageHold+10){
       LookAtEdge();
      }
      ResetLookAtEdge(); // reset the running average
     }
     else{                                 //if none of the above conditions were met (no edge sensor was triggered), then do this instead
      OffPixels();                         //turn off all pixels
      Motors(LeftMotorSpeed,RightMotorSpeed);   //drive both motors at normal speed to drive mostly "straight" while no line is seen
     }

     //delay(LoopHold);       //This is a super important part of the function. The "EdgeSensorAverage" variables contain a running average of
                            //the last 8 times each sensor was read. Another reading is averaged into this running average each time
                            //LookForEdge(); or LookAtEdge(); is called. The delay(LoopHold) causes the loop to delay a bit before
                            //running again. There can be trouble if this value is set too high or too low.
                            //If set too low (so the loop runs much faster), then as Ringo begins to move over a dark area or line,
                            //the average is adjusted much more quickly. So the average basically adjusts to the line being there so
                            //quickly that LookForEdge() never triggers (it never "sees" the line). When running in a fast loop
                            //(LoopHold is set to a lower number), Ringo must have a more sudden encouter with the line to see it.
                            //On the other hand, if LoopHold is set to a higher number, the average will not update as quickly. This
                            //makes Ringo able to see more suttle changes in the line. However, the delay also delays his reaction time,
                            //so he may drive right past the line before reacting to it (or stopping reacting to it). This may cause
                            //him to miss lines, or over-react to lines.
                            //There are many other ways to accomlish looking for lines - each of these issues could be addressed by
                            //writing a more complex line handler. That's your challenge. Use this as a start and see if you can
                            //make the line following more robust (more "reliable") to a variety of conditions. Please share your
                            //results on the forum.  http://forum.plumgeek.com

    if(CheckMenuButton()){    // if "MENU" button on IR remote has been pressed, break out back to main menu
      OffPixels();
      Motors(0,0);
      PlayStartChirp();
      break;
    }
    
  }//end of while(1);
}    // end Behavior_FollowLine();



// ************************************************************************************************************
// Behavior 10 Code - Exoplore Example
// ************************************************************************************************************

//The common "Ringo" inhabits environments with large smooth surfaces punctuated by precipitous drops.
//As a result it has evolved antenna that can sense edges. Modern individuals even have a sensor in the rear
//portion of its abdomen for backing up. Though the Ringo is capable of considerable speeds for its size, it prefers
//to go slow enough (depending on the type of surface) so that it can come to a complete stop before plummeting off
//of a nearby cliff.
//In order to help survive the occasional fall, the antenna are relatively short. Since this makes it difficult
//to see an edge that is close to paralell to its direction of motion, the species uses a characteristic 
//"buggy" gait (measured in units of EXPLORE_WIGGLE) in harsh terrain to extend its sensory reach further to the left & right.
//Ringo "sees" in the infrared with extremely simple infrared optical sensors. In habitats that have lots of background
//noise in this region of the electromagnetic spectrum, their visual system may be tricked into sensing objects that
//are not there. A larger value of EXPLORE_BARRIER_DETECTION_THRESHOLD can prevent this, but will also diminish its sensing range.

//This example code will make it explore an area, avoiding edges and obstacles, and
//pulling itself free if stuck.
//It gets frustrated if trapped in a corner and bored if it keeps turning in the same 
//direction (stuck in a loop).
//In both cases it picks a new direction at random.
//Featured functions:
//SimpleGyroNavigation();
//MaintainHeading(heading,speed,wiggle);
//ReadSideSensors();
//LookForEdge();
void Explore_BackAwayWhileLookingForEdge(int Speed, int TravelTimeMs, int BackEdgeResponseMs);//defined below
#define EXPLORE_BARRIER_DETECTION_THRESHOLD 10
#define EXPLORE_WIGGLE 12
void ExploreExample(int Speed, unsigned int RunTimeSec, int MaxExpectedSkidTimeMs){//Ver. 1.0, Dustin Soodak
  uint32_t timeout;
  int Heading;
  char detect,IsEdge;
  int BackAway=0,BackAwayPrev=0;
  int Time;
  char ShortTimes=0;
  char LeftTimes=0,RightTimes=0,SwitchTimes=0;
  int TimeSinceSmallFeedback=0;//used to help determine when stuck
  char ResetAndRecalibrate=0;
  signed char dir;
  int i,t; 
   
  if(NavigationOn)
    CalibrateNavigationSensors();
  else{
    NavigationBegin();
    PauseNavigation();
  }     
  OffEyes();    
  ResetLookAtEdge();
  ResumeNavigation();      
  Heading=GetDegrees();
  timeout=millis()+((uint32_t)RunTimeSec)*1000;
  RestartTimer();//used to calulate "Time" (since last barrier or edge detected)
  MaintainHeadingReset(); 
    
  while(1){  //was while(millis()<timeout)
    //re-calibrate gyro zero (after turning to new heading & waiting for it to come to a complete stop), and prepare to continue exploring:
    if(ResetAndRecalibrate){ 
      //Turn until it looks like it should skid to a halt pointing to the next heading:
      if(Heading>PresentHeading())
        dir=1;
      else
        dir=-1;      
      Motors(dir*100, -dir*100);        
      RestartTimer();
      while(GetTime()<1000 && (dir>0?GetDegrees()+GetDegreesToStop()<=Heading:GetDegrees()+GetDegreesToStop()>=Heading)){
        SimpleGyroNavigation(); 
      }
      Motors(0,0);
      //Wait till mostly stopped moving:
      OnEyes(50,50,50);
      RestartTimer();
      while(GetDegreesToStop()!=0 && GetTime()<500){
        SimpleNavigation();
      }
      Motors(0,0);     
      ResetAndRecalibrate=0;
      //Wait till still for 50ms before recalibrating sensors
      i=GetDegreesPerSecond();
      SwitchMotorsToSerial();
      RestartTimer();
      t=50;
      while(GetTime()<t && millis()<timeout){
        SimpleGyroNavigation();
        if(abs(GetDegreesPerSecond()-i)>0){
          Serial.print("moved: ");Serial.print(GetDegreesPerSecond());Serial.print(" vs ");Serial.print(i);  
          Serial.println();  
          t=GetTime()+50;
          if(t>3000)
            break;
        }
        i=GetDegreesPerSecond();
      }
      OffEyes();
      delay(10);
      CalibrateNavigationSensors(); 
      ResetLookAtEdge();  
      MaintainHeadingReset();      
      TimeSinceSmallFeedback=0;
      RestartTimer();//used to calulate "Time" (since last barrier or edge detected) 
    }//end if(ResetAndRecalibrate)

    
    //These 2 lines are all that is needed to keep it moving in the desired direction:
    SimpleGyroNavigation();
    MaintainHeading(Heading,Speed,EXPLORE_WIGGLE);  
      
    //Handle being trapped so can't continue going in desired direction:
    if(abs(RightMotor-LeftMotor)>MAINTAIN_HEADING_MAX_INTEGRAL_TERM){//if having to try really hard to maintain heading
      if( GetTime()/100-TimeSinceSmallFeedback>=15){//if been having to try really hard for 1.5 seconds
        PlayExcited();
        //try to back out, while being careful not to back over edge
        Explore_BackAwayWhileLookingForEdge(100, 400, 200);
        //Wait till no longer moving
        RestartTimer();
        while(GetTime()<500){
          SimpleGyroNavigation();
          if(abs(GetDegreesPerSecond())!=0 && millis()<timeout){
            RestartTimer();
            OnEyes(30,0,30);    
          }
          else
           OffEyes();
        }      
        if(PresentHeading()>Heading)
          Heading=PresentHeading()+90;
        else
          Heading=PresentHeading()-90;   
        
        ResetAndRecalibrate=1; 
        continue;//skips rest of enclosing while(millis()<timeout) so can go right to the desired section
      }//end if( GetTime()/100-TimeSinceSmallFeedback>10)
    }//end if(abs(RightMotor-LeftMotor)>150)
    else//not having to try particularly hard to maintain heading
      TimeSinceSmallFeedback=GetTime()/100;
    //Check if edges or barriers:
    ReadSideSensors();
    detect=LookForEdge();      
    if(detect){
      Motors(-255,-255);
      BackAway=abs(LeftEdgeSensorAverage-LeftEdgeSensorValue)>abs(RightEdgeSensorAverage-RightEdgeSensorValue)?1:-1;
      IsEdge=1;
    }
    if((LeftLightLevel-LeftLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD && LeftLightLevelPrev-LeftLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD) 
    || (RightLightLevel-RightLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD && RightLightLevelPrev-RightLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD)){
      Motors(-255,-255);
      BackAway= LeftLightLevel-LeftLightLevelAverage>RightLightLevel-RightLightLevelAverage?1:-1;
      IsEdge=0;
    }
    //What to do next after initial back-away reflex:
    if(BackAway!=0){
      Time=GetTime();           
      DelayWithSimpleNavigation(50);
      LookAtEdge();//update edge sensor data (LookForEdge() just calls LookAtEdge() followed by IsOverEdge())
      //Respond similarly if edge or barrier:
      if(IsEdge){//if edge/tape (as opposed to barrier)
        if(LeftFrontEdgeDetected(detect) && RightFrontEdgeDetected(detect)){
          BackAway=BackAway*2;//if both saw an edge
          SetPixelRGB(EYE_LEFT,50,0,0);SetPixelRGB(EYE_RIGHT,50,0,0);//both
        }
        else if(LeftFrontEdgeDetected(detect)){
          SetPixelRGB(EYE_LEFT,50,0,0);SetPixelRGB(EYE_RIGHT,0,0,0);//left
        }
        else if(RightFrontEdgeDetected(detect)){
          SetPixelRGB(EYE_LEFT,0,0,0);SetPixelRGB(EYE_RIGHT,50,0,0);//right
        }
      }
      else{//if barrier
        if(LeftLightLevel-LeftLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD && RightLightLevel-RightLightLevelAverage>EXPLORE_BARRIER_DETECTION_THRESHOLD){
          BackAway=BackAway*2;//if both saw an edge
          SetPixelRGB(EYE_LEFT,50,20,20);SetPixelRGB(EYE_RIGHT,50,20,20);//both
        }
        else if(LeftLightLevel-LeftLightLevelAverage>RightLightLevel-RightLightLevelAverage){
          SetPixelRGB(EYE_LEFT,50,20,20);SetPixelRGB(EYE_RIGHT,0,0,0);//left
        }
        else{
          SetPixelRGB(EYE_LEFT,0,0,0);SetPixelRGB(EYE_RIGHT,50,20,20);//right    
        } 
      }      
      //Slightly slower now, but still back up (while looking out for edges) 
      if(FrontDarkDetected(detect)){
        Explore_BackAwayWhileLookingForEdge(200, 300,0);//last argument 0 means won't respond to back edge sensor, just in case its from tipping forward over edge
      }
      else
        Explore_BackAwayWhileLookingForEdge(100, 500,200);
      DelayWithSimpleNavigation(300);
      //Reflect on past memories (used to determine if stuck in loop or corner):
      if(Time<500)
       ShortTimes++;
      else
       ShortTimes=0;      
      if(BackAway>0){
        RightTimes++;     
        LeftTimes=0; 
      }
      else{
        LeftTimes++; 
        RightTimes=0; 
      }
      if(BackAwayPrev==-BackAway)
        SwitchTimes++;
      else
        SwitchTimes=0;
      //If stuck in a corner:
      if((SwitchTimes>=4 || (SwitchTimes>=2 && ShortTimes>=2)) && abs(BackAway)<2){        
        OnEyes(0,0,50);
        PlayAnger();
        SwitchTimes=0;
        ShortTimes=0;
        Heading=GetDegrees()+45*BackAway*((abs(GetDegrees())%4)+2);//buggy example behavior 019 uses this but with %4 inside abs()
      }
      //If in a loop:
      else if(LeftTimes>=6 || RightTimes>=6){         
        OnEyes(0,50,0);
        PlayBoredom();       
        LeftTimes=0;
        RightTimes=0;
        Heading=GetDegrees()+45*BackAway*((abs(GetDegrees())%4)+2);//buggy example behavior 019 uses this but with %4 inside abs()     
      }   
      //If not stuck in corner or loop:
      else{
        Heading=GetDegrees()+45*BackAway;
      }          
      BackAwayPrev=BackAway; 
      BackAway=0;
      ResetAndRecalibrate=1;
      continue;//skip rest of enclosing while(millis()<timeout) (not needed yet since this is the last behavior in the loop)
    }//end if(BackAway!=0)    
  }//end while(millis()<timeout)
  //Stop motors and wait for it to come to a halt:
  Motors(0,0);     
  OffEyes();
  PlayAck();
  timeout=millis()+MaxExpectedSkidTimeMs;
  while(millis()<timeout){
    SimpleNavigation();
  }  
}//end ExploreExample() main loop (there is one more helper function below)

void Explore_BackAwayWhileLookingForEdge(int Speed, int TravelTimeMs, int BackEdgeResponseMs){
 char detect=0,Heading;
 SimpleGyroNavigation(); 
 Heading=PresentHeading();
 Motors(-Speed,-Speed);
  RestartTimer();
  while(GetTime()<TravelTimeMs){
    if(BackEdgeResponseMs>0)
      detect=LookForEdge();
    if(RearDarkDetected(detect))
      break;
    SimpleGyroNavigation(); 
    //MaintainHeading(Heading,-Speed,0);
  }
  if(RearDarkDetected(detect)){
    Motors(250,250);     
    SetPixelRGB(TAIL_TOP,50,0,0);
    DelayWithSimpleNavigation(BackEdgeResponseMs);  
    SetPixelRGB(TAIL_TOP,0,0,0);  
  }
  Motors(0,0);
} // end ExploreExample() and helper functions



