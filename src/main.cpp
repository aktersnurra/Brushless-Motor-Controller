#include "mbed.h"
#include "rtos.h"
#include "BufferedSerial.h"
#include "interpreteRegex.h"

// Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

// Incremental encoder input pins
#define CHApin   D7
#define CHBpin   D8

// Motor Drive output pins  //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

// Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

int8_t orState = 0;    // Rotot offset at motor state 0

// Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
// const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

// motor speed
float motor_speed = 0;

// Status LED
DigitalOut led1(LED1);

// Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

// Incremental Encoder interrupt inputs
InterruptIn CHA(CHApin);
InterruptIn CHB(CHBpin);

// Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

// Timer for measuring the speed
Timer speedTimer;

// last time the timer was read
uint32_t last_time_fine;
uint32_t last_time_coarse;

// number of increments (of the 117) have been passed since last time measurement
uint8_t increments;

// current rotor position
float rotor_position;

// current motor speed in Hz
float measured_speed_fine;
float measured_speed_coarse;

// Initialise the serial port
// Serial pc(SERIAL_TX, SERIAL_RX, 115200);
BufferedSerial pc(USBTX, USBRX);

// bool to determine if the rotor has passed through the zero state yet
bool passedHome = false;

// float that holds how many degrees have been travelled in total
float total_distance = 0;

// bool to check if timer has been reset since last measurement
bool timer_reset_fine;
bool timer_reset_coarse;

// Set a given drive state
void motorOut(int8_t driveState)
{
    // Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    // Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = motor_speed;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = motor_speed;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = motor_speed;

    // Then turn on
    if (driveOut & 0x01) L1L = motor_speed;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = motor_speed;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = motor_speed;
    if (driveOut & 0x20) L3H = 0;
}

// Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState()
{
    return stateMap[I1 + 2*I2 + 4*I3];
}

// Basic synchronisation routine
int8_t motorHome()
{
    // Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);

    // Get the rotor state
    return readRotorState();
}

void photointerrupt_ISR(void)
{
    int8_t intState;
    uint32_t current_time;
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); // +6 to make sure the remainder is positive

    // only do when state is zero because states are not evenly spaced through the rotation
    if (intState == 0)
    {
        // calculate the speed based on the number of full revolutions
        current_time = speedTimer.read_us();
        if (!timer_reset_coarse)
        {
            measured_speed_coarse = 1000000/((current_time - last_time_coarse));
            last_time_coarse = current_time;
        }
        else
        {
            timer_reset_coarse = false;
            last_time_coarse = current_time;
        }

        // update the total distance travelled
        if (!passedHome)
        {
            // if the rotor hasn't passed through the zero state yet, the rotor position is the
            // distance from the initia state to the zero state and the total distance travelled so far
            total_distance += rotor_position;
        }
        else
        {
            // if the rotor has passed through zero already then it has gone through a full revolution
            // and thus travelled 360 degrees
            total_distance += 360;
        }

        // reset rotor position measured by incremental encoder to eliminate errors due to slipping
        rotor_position = 0.0f;
    }
}

void incremental_ISR(void)
{
    // assign arbitrary values to the 4 states of the incremental encoder
    uint8_t state = (CHA*2 + CHB);
    uint32_t current_time;

    // only do when state is zero because states are not evenly spaced
    if (state == 0)
    {
        // update the number of increments since last speed measurement
        increments++;
        // update rotor position (one increment corresponds to 360/117 degrees)
        rotor_position = rotor_position + 3.07692;

        // measure the speed based on the number of increments and time since the last speed measurement
        current_time = speedTimer.read_us();
        if (!timer_reset_fine)
        {
            measured_speed_fine = increments*8547.0f/(current_time-last_time_fine);
            // reset increments
            increments = 0;
            // update last time speed was measured
            last_time_fine = current_time;
        }
        else
        {
            timer_reset_fine = false;
            last_time_fine = current_time;
        }
    }
}

void set_PWM_period_us(int period_us)
{
    L1L.period_us(period_us);
    L1H.period_us(period_us);
    L2L.period_us(period_us);
    L2H.period_us(period_us);
    L3L.period_us(period_us);
    L3H.period_us(period_us);
}

void setup(void)
{
    // Run the motor synchronisation
    orState = motorHome();

    // Set up interrupts for absolute encoder
    I1.rise(&photointerrupt_ISR);
    I1.fall(&photointerrupt_ISR);
    I2.rise(&photointerrupt_ISR);
    I2.fall(&photointerrupt_ISR);
    I3.rise(&photointerrupt_ISR);
    I3.fall(&photointerrupt_ISR);

    // Set up interrupts for incremental encoder
    CHA.rise(incremental_ISR);
    CHA.fall(incremental_ISR);
    CHB.rise(incremental_ISR);
    CHB.fall(incremental_ISR);

    // Set up PWM outputs
    L1L.period_us(100);
    L1H.period_us(100);
    L2L.period_us(100);
    L2H.period_us(100);
    L3L.period_us(100);
    L3H.period_us(100);

    // Set initial motor speed
    motor_speed = 0.5f;

    // Start timer
    speedTimer.start();
    last_time_fine = 0;

    increments = 0;
    rotor_position = 0.0f;
}

//Controller
void controller(float refRev, float refVel) {
    float K = 1;
    float Ki = 1;
    float Kp = 1;
    float Kd = 1;
    float dTarget = 0;
    float sTarget = 0;

    if(refRev != 0) {
        dTarget = refRev - total_distance;
    }

    if(refVel != 0) {
        sTarget = refVel - measured_speed;
    }

    motor_speed = K * (Kp * dTarget + Kd * sTarget);
}

float* freqPtr;
int* durationPtr;

void playTune(int toneSum)
{
    for (int i = 0; i < toneSum; i++)
    {
        printf("play %f Hz for %d us.\n", freqPtr[i], durationPtr[i]*100);
        set_PWM_period_us(1000000/freqPtr[i]);
        wait_ms(durationPtr[i]*150);
        set_PWM_period_us(50);
    }
}

void readRegex()
{
    char regex[64];
    //char regex[64];
    int i = 0;
    char c;

    float revFloat = 0;
    float velFloat = 0;
    int toneSum = 0;

    if (pc.readable())
    {
        while (pc.readable())
        {
            c = pc.getc();
            //               pc.putc(c+1);    //debug
            regex[i] = c;
            i++;
        }
        regex[i] = '\0';
        printf("\nRegex=%s, ", regex);
        interpreteRegex(regex, i);

        if(getMotorCommands(&revFloat, &velFloat))
        {
            printf("TODO: execute motor command, rev=%f, vel=%f\n", revFloat, velFloat);
            controller(revFloat,velFloat);
        }else{
            toneSum = getTune(&freqPtr, &durationPtr);
            if (toneSum){
                playTune(toneSum);
            }
        }



        i = 0;
    }
}

int main(void)
{
    setup();

    pc.baud(115200);

    while(1)
    {
        wait(1);
        printf("Rotor speed: %f\n\r",measured_speed_fine);
        printf("Rotor position: %f\n\r",rotor_position);
        readRegex();

        // reset the timer every 20 minutes to prevent it from overflowing
        if (speedTimer.read_us() > 1200000000)
        {
            speedTimer.reset();
            timer_reset_fine = true;
            timer_reset_coarse = true;
        }
    }
}
