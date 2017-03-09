#include "mbed.h"
#include "rtos.h"

// Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

// Incremental encoder input pins
#define CHApin   D7
#define CHBpin   D8  

// Motor Drive output pins   //Mask in output byte
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
uint32_t last_time;

// number of increments (of the 117) have been passed since last time measurement
uint8_t increments;

// current rotor position
float rotor_position;

// current motor speed in Hz
float measured_speed;

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
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); // +6 to make sure the remainder is positive
    
    // reset rotor position
    if (intState == 0)
    {
        rotor_position = 0.0f;
    }
}

void incremental_ISR(void)
{
    uint8_t state = (CHA*2 + CHB);
    uint32_t current_time;
    if (state == 0)
    {
        increments++;
        rotor_position = rotor_position + 3.07692;
        if (last_time > 0);
        {
            current_time = speedTimer.read_us();
            measured_speed = increments*8547.0f/(current_time-last_time);
            increments = 0;
        }
        last_time = current_time;
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
    last_time = 0;
    
    increments = 0;
    rotor_position = 0.0f;
}

int main(void)
{
    setup();
    // Initialise the serial port
    Serial pc(SERIAL_TX, SERIAL_RX);
    while(1)
    {
        wait(1);
        pc.printf("Rotor speed: %f\n\r",measured_speed);
        pc.printf("Rotor position: %f\n\r",rotor_position);
    }
}
