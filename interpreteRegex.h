#ifndef INTERPRETEREGEX_H
#define INTERPRETEREGEX_H
 
#include "mbed.h"
#define isNumber(x) ((x >= '0') && (x <= '9'))
/*
enum errorCodes
{
    NO_ERROR,
    ERROR_INTERPRETE_CHAR,
    ERROR_INTERPRETE_START_CHAR,
    ERROR_INTERPRETE_ROTATION_CHAR,
    ERROR_INTERPRETE_VELOCITY_CHAR,
    ERROR_INTERPRETE_TUNE_CHAR,
}; 
*/

//returns if motor commands are available
int getMotorCommands(float* revFloatPtr, float* velFloatPtr);

//returns the number of tones
int getTune(float* freqPtr[], int* durationPtr[]);

//R, float(revolutions), V, float(velocity)
//T, float(frequency), int(duration) 
//rotation commands: R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,2})?)?
void interpreteRegex(char regex[], int length);

void interpreteTune(char regex[], int length);

//returns the next state
int interpreteStartCharacter(char c);
 
#endif
