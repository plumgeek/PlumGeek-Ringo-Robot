/*

Ringo Robot:  Animation.h  Rev01.02  12/2015

This code includes various fun and animated things Ringo can do. For example, sounds,
lighting sequences, dance moves, etc. Users are encouraged to build their own
"fun stuff" examples and share their work on the forum.  http://forum.plumgeek.com/

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

#ifndef FUNSTUFF_H
#define FUNSTUFF_H




// ***************************************************
// Short Animations
// ***************************************************

extern void PlayStartChirp(void); //put in begin() to indicate that unit has restarted.
extern void PlayAck(void);        //simple acknowledgement chirp. flashes middle pixel green.
extern void PlayNonAck(void);     //simple non-acknowledgement chirp. flashes middle pixel red.
extern void PlayAnger(void);
extern void PlayBoredom(void);
extern void PlayExcited(void);

// ***************************************************
// end Short Animations
// ***************************************************



// ***************************************************
// Chirp, Sound, Piezo, Lighting Functions
// ***************************************************

// Sound functions
extern void PlayChirp(unsigned int Frequency, unsigned int Amplitude);//Amplitude: 0-50 (effects loudness and tonal quality)
extern void OffChirp(void); // silences chirp tone
extern void PlaySweep(int StartNote, int EndNote, int DwellTime);   //allows to sweep between two tones at a given rate

// Pixel wrapper functions - easy functions for common lighting effects
extern void OffPixels(void);                                 //turns off all pixels
extern void OffPixel(byte Pixel);                            //turns off a specific pixel
extern void OnEyes(byte Red, byte Green, byte Blue);         //makes eyes the given color, automatically calls RefreshPixels()
extern void LeftEye(byte Red, byte Green, byte Blue);        //sets left eye to given color, automatically calls RefreshPixels()
extern void RightEye(byte Red, byte Green, byte Blue);       //sets left eye to given color, automatically calls RefreshPixels()
extern void OffEyes(void);                                   //turns off eye lights
extern void RandomEyes(void);                                //Sets the pair of eyes to a random color


// ***************************************************
// end Chirp, Sound, Piezo, Lighting Functions
// ***************************************************


// ***************************************************
// Note to Tone Mapping
// ***************************************************

//This list maps keyboard notes like "middle C" (NOTE_C4) to
//tones in hertz. These can be used with 
//Copied from http://arduino.cc/en/Tutorial/tone

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// ***************************************************
// end Note to Tone Mapping
// ***************************************************

#endif

