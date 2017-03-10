#include "interpreteRegex.h"
#include "string.h"

int state = 0;

enum states
{
    START,
    REVOLUTION,
    VELOCITY,
    TUNE, 
    END,
}; 
 
float frequencies[] = 
{
    440,    //A
    493.88, //B
    523.25, //C
    587.33, //D
    659.25, //E
    698.46, //F
    783.99, //G
};
float halfToneFactor = 1.059463094359; //2^(1/12)

float frequency[16];
int duration[16];
int toneSum = 0;
float revFloat = 0; //number of revolutions and direction
float velFloat = 0; //velocity

//return if motor commands are available
int getMotorCommands(float* revFloatPtr, float* velFloatPtr)
{
    if (revFloat == 0 && velFloat == 0)
        return 0;
        
    *revFloatPtr = revFloat;
    *velFloatPtr = velFloat;
    return 1;
}

//returns the number of tones
int getTune(float* freqPtr[], int* durationPtr[])
{
    *freqPtr = frequency;
    *durationPtr = duration;
    return toneSum;
}

void interpreteRegex(char regex[], int length)
{
    char* pEnd;
    int i = 0;  //regex index

    toneSum = 0;
    revFloat = 0;
    velFloat = 0;
    
    //get state
    state = interpreteStartCharacter(regex[i]);
    i++;
    
    switch (state) 
    {
    case REVOLUTION:      //Expect minus or digit
        //extract float
        revFloat = strtof(regex+1, &pEnd);
//        printf("state=%d, revFloat=%f, rest=%s\n", state, revFloat, pEnd);
        if (pEnd[0] == '\0') {
            printf("pEnd=\\0\n");
            return;
        }else if(pEnd[0] == 'V') {
            state = VELOCITY; //fallthrough 
        }else{
            printf("ERROR undefined pEnd in REVOLUTION\n");
            return;   
        }    
    case VELOCITY:
        velFloat = strtof(regex+1, &pEnd);
//        printf("state=%d, velFloat=%f, rest=%s\n", state, velFloat, pEnd);
        if (pEnd[0] == '\0') {
            printf("pEnd=\\0\n");
        }else{
            printf("ERROR undefined pEnd in VELOCITY\n");
        }
        return;
    case TUNE:
        printf("Tune!\n");
        interpreteTune(&regex[i], length-i);
        
        printf("Tones: \n");
        for (int i = 0; i < toneSum; i++)
        {
            printf("f=%4.3f, d=%d\n", frequency[i], duration[i]);  
        }
        return;   
    case END:
        printf("End!\n");
        return;  
    default:
        printf("ERROR in RegEx!\n");
        return;
    }
}

void interpreteTune(char regex[], int length)
{
    int pos = 0;
    for (int i = 0; i < 16; i++)
    {
        //read tone
        if (regex[pos] >= 'A' && regex[pos] <= 'G')
        {
             frequency[i] = frequencies[(int)(regex[pos]-'A')];
             pos++;
        }else if (regex[pos] == '\0'){
            toneSum = i;
            return;
        }else{
            printf("ERROR in interpreteTune, undefined tone!\n");
            toneSum = 0;
            return;
        }
        
        //read chromatic sign
        if (regex[pos] == '#') {
            frequency[i] *= halfToneFactor;
            pos++;
        }else if (regex[pos] == '^') {
             frequency[i] /= halfToneFactor;
             pos++;
        }
        
        //read tone duration
        if (regex[pos] >= '1' && regex[pos] <= '8'){
            duration[i] = (int) (regex[pos] - '0');
            pos++;
        }else{
            printf("ERROR in interpreteTune, wrong duration!\n");
            toneSum = 0;
            return;
        }
    }
}
 
//returns the next state
int interpreteStartCharacter(char c)
{
    switch (c) 
    {
        case 'R': //rotate as fast as possible
            return REVOLUTION;
//            break;
        case 'V':
            return VELOCITY;
//            break;
        case 'T':
            return TUNE;
//            break;   
        default:
            printf("ERROR in interpreteStartCharacter!\n");
            return START;
//            break;
    } 
}
