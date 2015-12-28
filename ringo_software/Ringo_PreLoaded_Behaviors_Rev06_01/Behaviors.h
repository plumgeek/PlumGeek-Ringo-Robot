

#ifndef BEHAVIORS_H
#define BEHAVIORS_H


extern void Behavior_DriveWithRemote(void);   //allows user to drive Ringo around using remote control.
                                              //buttons left,right,front,back,eyes via A&B, tones via 1-8, press 0 to cheer

extern void Behavior_AttackBug(void );        //Attack when poked

extern void Behavior_TheRingoDance(void);     // what it says NO EDGE DETECTION - DON'T LET RINGO JUMP OFF YOUR TABLE!

extern void Behavior_SpinSqueek(void);        //When either eye sensor sees significant drop in light
                                              //Ringo will spin and squeek in that direction

extern void Behavior_ColorWheel(void);        //Use Ringo's gyroscope to create a virtual color wheel on Ringo's eyes
                                              //Manually turn Ringo around.  Press User Button to make motors turn Ringo for you.

extern void Behavior_FollowLight(void);       //Follow a light. Both front ambient sensors read light level,
                                              //Ringo steers toward the brightest light.

extern void Behavior_DriveSquare(void);       //Use Ringo's front facing IR LED together with his ambient
                                              //light sensors to sense and react to objects in front of him.

extern void Behavior_MaintainHeading(void);   //Use Ringo's gyroscope to maintain a starting heading.
                                              //Rotate Ringo one direction or the other and he'll return
                                              //to his initial heading.

extern void Behavior_FollowLine(void);        //Follow a line using the left and right edge sensors under
                                              //Ringo's feelers.

extern void ExploreExample(int Speed, unsigned int RunTimeSec, int MaxExpectedSkidTimeMs);
                                              //This will make it explore an area, avoiding edges and obstacles.
                                              //It gets frustrated if stuck in a corner and bored if it keeps turning in the same 
                                              //direction (stuck in a loop).
                                              //In both cases it picks a new direction at random.

#endif
