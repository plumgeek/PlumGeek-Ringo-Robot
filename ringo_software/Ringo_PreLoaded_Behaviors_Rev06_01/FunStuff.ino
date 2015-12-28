/*

Ringo Robot:  Animation  Rev01.02  12/2015

This code includes various "animation" things Ringo can do. For example, sounds,
lighting sequences, dance moves, etc. Users are encouraged to build their own
"animations" and share their work on the forum.  http://forum.plumgeek.com/

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/

Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "FunStuff.h"


// ***************************************************
// Short Animations
// ***************************************************


void PlayStartChirp(void){//Ver. 1.0, Kevin King   
  // This is the startup sequence that plays when Ringo is reset(put in begin())
  PlayChirp(1000, 50);
  OnEyes(0,0,50);  // blue
  delay(50);
  PlayChirp(2000, 50);
  delay(50);
  PlayChirp(3000, 50);
  delay(50);
  PlayChirp(4000, 50);
  delay(100);
  OffChirp();
  OffEyes();
}


void PlayAck(void){//Ver. 1.0, Kevin King
  PlayChirp(NOTE_C7,100);
  SetPixelRGB(BODY_TOP,0,30,0);//RefreshPixels();
  delay(50);
  OffChirp();
  SetPixelRGB(BODY_TOP,0,0,0);//RefreshPixels();
}


void PlayNonAck(void){//Ver. 1.0, Kevin King
  PlayChirp(NOTE_CS6,100);
  SetPixelRGB(BODY_TOP,30,0,0);//RefreshPixels();
  delay(75);
  OffChirp();
  SetPixelRGB(BODY_TOP,0,0,0);//RefreshPixels();
}


void PlayAnger(void){//Ver. 1.0, Dustin Soodak
  char i;
  for(i=0;i<10;i++){
    PlayChirp(NOTE_C6,100);
    delay(10);
    PlayChirp(NOTE_CS6,100);   
    delay(10);  
  }
  OffChirp();  
}


void PlayBoredom(void){//Ver. 1.0, Dustin Soodak
  char i;
  unsigned int freq,dfreq;
  dfreq=(NOTE_DS6-NOTE_C6)>>4;
  PlayChirp(NOTE_C6,100);
  delay(50);
  for(i=0;i<16;i++){    
    PlayChirp(NOTE_C6+dfreq*i,100);
    delay(10);
  }
  PlayChirp(NOTE_DS6,100);
  delay(100);
    for(i=0;i<16;i++){    
    PlayChirp(NOTE_DS6-dfreq*i,100);
    delay(10);
  }
  PlayChirp(NOTE_C6,100);
  delay(50);
  OffChirp();
}


void PlayExcited(void){//Ver. 1.0, Dustin Soodak, Kevin King (note sequence)
  PlayChirp(NOTE_G6,100);
  delay(100);
  PlayChirp(NOTE_C7,100);
  delay(100);
  PlayChirp(NOTE_A6,100);
  delay(100);
  PlayChirp(NOTE_C7,100);
  delay(100);
  PlayChirp(NOTE_B6,100);
  delay(100);  
  PlayChirp(NOTE_G6,100);
  delay(100);
  OffChirp();
}


// ***************************************************
// end Short Animations
// ***************************************************


// ***************************************************
// Chirp, Sound, Piezo Core Functions
// ***************************************************

void PlaySweep(int StartNote, int EndNote, int DwellTime){//Ver. 1.0, Kevin King
  if(StartNote<EndNote){
     for(StartNote; StartNote<=EndNote; StartNote++){
      PlayChirp(StartNote,100);
      delayMicroseconds(DwellTime);
     }
  }
   else{
     for(StartNote; StartNote>=EndNote; StartNote--){
      PlayChirp(StartNote,100);
      delayMicroseconds(DwellTime);
     }  
  }
 OffChirp(); 
}

void PlayChirp(unsigned int Frequency, unsigned int Amplitude){//Ver. 1.0, Dustin Soodak
  //Frequency is in Hz
  //Amplitude is in units of UsOn
  uint16_t temp;
  uint16_t period,dutycycle;
  uint8_t prescalerbits;
  if(Amplitude>100)
    Amplitude=100;
  if (F_CPU/Frequency/2>=0x10000){
    if(F_CPU/Frequency/2>=0x10000*8){
        prescalerbits=0b011;//prescaler 64
        period=F_CPU/Frequency/(2*16);
        dutycycle=F_CPU/1000000*Amplitude/2/64;               
    }
    else{
      prescalerbits = 0b010;// prescaler 8
      period=F_CPU/Frequency/(2*8);
      dutycycle=F_CPU/1000000*Amplitude/2/8;
    }
  }
  else{
    prescalerbits = 0b001;  //on but no prescaling
    period=F_CPU/Frequency/(2*1);
    dutycycle=F_CPU/1000000*Amplitude/2/1;
  }
  if(Amplitude==0){
    TCCR1A=0;
    TCCR1B=0;
    pinMode(Chirp,INPUT);
    
    //Serial.println("off");    // debugging code
  }
  else{
    TCCR1B&=~0b00000111;//turn off timer
    ICR1=period;
    OCR1A=dutycycle;
    TCCR1A = (0b10<<6) | 0b10;//COM1A1 COM1A0, and WGM11 WGM10
    TCCR1B = (0b10<<3) | prescalerbits;//WGM13 WGM12, and off/(on with prescaler)
    pinMode(Chirp,OUTPUT);//DDRB|=1;//make PortB pin1 an output 
  }
}

void OffChirp(void){//Ver. 1.0, Kevin King              //stops chirp tone
  PlayChirp(0,0);       //turn off chirp
}

void OffPixels(void){//Ver. 1.0, Kevin King              //turns off all pixels
  SetAllPixelsRGB(0,0,0);                                //set all pixels to off
}

void OffPixel(byte Pixel){//Ver. 1.0, Kevin King    //turns off a specific pixel
  SetPixelRGB(Pixel,0,0,0);                              //set this pixel to off
}

void OnEyes(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes eyes the given color, automatically calls RefreshPixels()
  SetPixelRGB(4,Red,Green,Blue);                         //set right eye
  SetPixelRGB(5,Red,Green,Blue);                         //set left eye
}

void LeftEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes left eye the given color, automatically calls RefreshPixels()
  SetPixelRGB(EYE_LEFT,Red,Green,Blue);                  //set left eye
}

void RightEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes right eye the given color, automatically calls RefreshPixels()
  SetPixelRGB(EYE_RIGHT,Red,Green,Blue);                 //set right eye
}

void OffEyes(void){//Ver. 1.0, Kevin King                //turns off eye pixels
  SetPixelRGB(4,0,0,0);                                  //blank right eye
  SetPixelRGB(5,0,0,0);                                  //blank left eye
}

void RandomEyes(void){//Ver. 1.0, Kevin King             //Sets the pair of eyes to a random color
  int red, green, blue;
  red = random(120);
  green = random(120);
  blue = random(120);
  OnEyes(red,green,blue);
}

// ***************************************************
// end Chirp, Sound, Piezo Core Functions
// ***************************************************



// ***************************************************
// Additional Notes:  Chirp / Sound / Piezo
// ***************************************************

  //Terms:
  //TCNT1 is counter
  // goes from BOTTOM = 0x0000 to MAX = 0xFFFF in normal mode.
  //TOP can be either OCR1A or ICR1
  //
  //Table 15-4. Waveform Generation Mode Bit Description(16 different pwm modes, 2 of which are phase & freq correct)
  //Want "phase and frequency correct PWM mode" on p.130
  //Mode is set by WGM 13:0 bits.
  //Mode WGM: 13 12 11 10
  //10         1  0  1  0 PWM, Phase Correct, TOP is ICR1  [so can use OCR1A as duty cycle and OC1A as pin output]
  //
  //TCNT1 goes from BOTTOM to TOP then TOP to BOTTOM
  //TOP can be OCR1A or ICR1
  //Output Compare OC1x cleared when TCNT1=OCR1x while upcounting, and set when down counting.
  //
  // "The Output Compare Registers contain a 16-bit value that is continuously compared with the
  //counter value (TCNT1). A match can be used to generate an Output Compare interrupt, or to
  //generate a waveform output on the OC1x pin."
  //
  //Table 15-3. Compare Output Mode, when in Phase Correct and Phase and Frequency Correct mode
  //(what these bits do depends on what WGM 13:0 are set to)
  //COM1A1 COM1A0
  //     1      0    Clear OC1A on Compare Match when upcounting. Set OC1AB on Compare Match when downcounting.
  //If we want this single pin mode, can't use OCR1A for TOP since will be used for duty cycle of PWM
  //
  //"...note that the Data Direction Register (DDR) bit corresponding
  //to the OC1A or OC1B pin must be set in order to enable the output driver."
  //
  //CS12 CS11 CS10
  //off mode(all 0), set prescvaler, or external clock source
  //
  //COM1A1 COM1A0 are bits 7:6 in                   TCCR1A
  //data direction register: bit 1 of               DDRB 
  //WGM13 WGM12 are bits 4:3 in                     TCCR1B
  //WGM11 WGM10 are bits 1:0 in                     TCCR1A
  //CS12,11,10 are bits 2:0 in                      TCCR1B
  //total count (/2-1 to get period of waveform)is  ICR1
  //duty cycle (/2 to get period it is on) is       OCR1A
  //
  //TCCR1A=(TCCR1A&0b00111111)|0b01000000;
  //
  //
  //If we had the pwm output to the chirper on another pin:
  //"Using the ICR1 Register for defining TOP works well when using fixed TOP values. By using
  //ICR1, the OCR1A Register is free to be used for generating a PWM output on OC1A. However,
  //if the base PWM frequency is actively changed by changing the TOP value, using the OCR1A as
  //TOP is clearly a better choice due to its double buffer feature."
  //
  //Another doc for R/W of 16 bit registers: 
  //http://www.embedds.com/programming-16-bit-timer-on-atmega328/

// ***************************************************
// end Additional Notes:  Chirp / Sound / Piezo
// ***************************************************








