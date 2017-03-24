#include "mbed.h"
#include "rtos.h"
#include <string>
#include "interpreteRegex.h"
#include "BufferedSerial.h"
//#include <cstring>

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
const int8_t driveTable[] = {0x12, 0x18, 0x09, 0x21, 0x24, 0x06, 0x00, 0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07, 0x05, 0x03, 0x04, 0x01, 0x00, 0x02, 0x07};
// const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//Init number revolutions that the user wants the motor to spin for
extern float revFloat;
//Init the velocity that the user wants the motor to spin at
extern float velFloat;

// motor speed
volatile float pwm_duty_cycle = 0;

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
volatile uint32_t last_time_fine;
volatile uint32_t last_time_coarse;

// number of increments (of the 117) have been passed since last time measurement
volatile uint8_t increments;

// current rotor position
volatile float rotor_position;

// current motor speed in Hz
volatile float measured_speed_fine;
volatile float measured_speed_coarse;

// Initialise the serial port
BufferedSerial pc(USBTX, USBRX);

// bool to determine if the rotor has passed through the zero state yet
volatile bool passedHome = false;

// float that holds how many degrees have been travelled in total, uses incremental encoder until first zero crossing and photointerrupts after
volatile float total_distance_coarse = 0;
// float that holds how many degrees have been travelled in total, uses incremental encoder only
volatile float total_distance_fine = 0;

// bool to check if timer has been reset since last measurement
volatile bool timer_reset_fine;
volatile bool timer_reset_coarse;

// Set a given drive state
void motorOut(int8_t driveState) {
    // Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    // Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = pwm_duty_cycle;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = pwm_duty_cycle;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = pwm_duty_cycle;

    // Then turn on
    if (driveOut & 0x01) L1L = pwm_duty_cycle;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = pwm_duty_cycle;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = pwm_duty_cycle;
    if (driveOut & 0x20) L3H = 0;
}

// Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState() {
    return stateMap[I1 + 2 * I2 + 4 * I3];
}

// Basic synchronisation routine
int8_t motorHome() {
    // Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);

    // Get the rotor state
    return readRotorState();
}

void startMotor()
{
    int8_t intState;
    intState = readRotorState();
    {
        motorOut((intState - orState + lead + 6) % 6); // +6 to make sure the remainder is positive
    }
}

void photointerrupt_ISR(void) {
    int8_t intState;
    uint32_t current_time;
    intState = readRotorState();
    motorOut((intState - orState + lead + 6) % 6); // +6 to make sure the remainder is positive

    // only do when state is zero because states are not evenly spaced through the rotation
    if (intState == 0) {
        // calculate the speed based on the number of full revolutions
        current_time = speedTimer.read_us();
        if (!timer_reset_coarse) {
            measured_speed_coarse = 1000000 / ((current_time - last_time_coarse));
            last_time_coarse = current_time;
        } else {
            timer_reset_coarse = false;
            last_time_coarse = current_time;
        }

        // update the total distance travelled
        if (!passedHome) {
            // if the rotor hasn't passed through the zero state yet, the rotor position is the
            // distance from the initia state to the zero state and the total distance travelled so far
            total_distance_coarse += rotor_position;
            passedHome = true;
        } else {
            // if the rotor has passed through zero already then it has gone through a full revolution
            // and thus travelled 360 degrees
            total_distance_coarse += 360;
            // to prevent errors due to slippage in incremental encoder,
            // set total_distance_fine equal to total_distance_coarse which does not sufer from slippage
            total_distance_fine = total_distance_coarse;
        }

        // reset rotor position measured by incremental encoder to eliminate errors due to slipping
        rotor_position = 0.0f;
    }
}

void incremental_ISR(void) {
    // assign arbitrary values to the 4 states of the incremental encoder
    uint8_t state = (CHA * 2 + CHB);
    uint32_t current_time;

    // only do when state is zero because states are not evenly spaced
    if (state == 0) {
        // update the number of increments since last speed measurement
        increments++;
        // update rotor position (one increment corresponds to 360/117 degrees)
        rotor_position = rotor_position + 3.07692f;
        
        // update total_distance_fine
        total_distance_fine += 3.07692f;

        // measure the speed based on the number of increments and time since the last speed measurement
        current_time = speedTimer.read_us();
        if (!timer_reset_fine) {
            measured_speed_fine = increments * 8547.0f / (current_time - last_time_fine);
            // reset increments
            increments = 0;
            // update last time speed was measured
            last_time_fine = current_time;
        } else {
            timer_reset_fine = false;
            last_time_fine = current_time;
        }
    }
}

void set_PWM_period_us(int period_us) {
    L1L.period_us(period_us);
    L1H.period_us(period_us);
    L2L.period_us(period_us);
    L2H.period_us(period_us);
    L3L.period_us(period_us);
    L3H.period_us(period_us);
}




//integral error for the controllers
volatile float IVelError, IPosError;

float velocityController(float refVel) {
    //Controller gains
    float Kp = 10, Ki = 0.001;
    //error: velocity error
    //pi: controller output
    //PI_out: output of the controller within the constraints
    float error = 0, pi, PI_out;
    //Init the proportional and integral terms
    float P = 0, I = 0;
    //Define lower and upper bounds of the output
    float pwmL = 0, pwmU = 1;

    //proportional
    error = refVel - measured_speed_fine;
    P = Kp * error;

    //integral
    IVelError += error;
    I = Ki * IVelError;

    //pi
    pi = P + I;

    //if pi output is between the boundaries, pi is set as the output
    if (pi > pwmL && pi < pwmU) {
        PI_out = pi;
    }
        //pi less than 0, PI_out = 0, i.e. motor turned off
    else if (pi <= pwmL) {
        PI_out = pwmL;
    }
        //pi above 1, PI_out = 1, i.e. maximum input to motor
    else if (pi >= pwmU) {
        PI_out = pwmU;
    }

    return PI_out;
}

float positionController(float refRev) {
    //Controller gains
    float K = 0.00005, Kp = 1.7, Ki = 0, Kd = 1.5;
    //error: distance error
    //pid: controller output
    //PID_out: output of the controller within the constraints
    float error = 0, pid, PID_out;
    //Init proportional–integral–derivative terms
    float P = 0, I = 0, D = 0;
    //Define lower and upper bounds of the output
    float pwmL = 0, pwmU = 1;

    //proportional
    error = (refRev * 360) - total_distance_fine;
    P = Kp * error;

    //integral
    IPosError += error;
    I = Ki * IPosError;

    //derivative
    D = Kd * measured_speed_fine;

    //PID
    pid = K * (P + I + D);
    

    //if pid output is between the boundaries, pi is set as the output
    if (pid > pwmL && pid < pwmU) {
        PID_out = pid;
    }
        //pi less than 0, PI_out = 0, i.e. motor turned off
    else if (pid <= pwmL) {
        PID_out = pwmL;
    }
        //pi above 1, PI_out = 1, i.e. maximum input to motor
    else if (pid >= pwmU) {
        PID_out = pwmU;
    }
    return PID_out;
}



//Controller
void controller(float refRev = 0, float refVel = 0) {
    //init variables for controller values
    float velocityControl = 0, positionControl = 0;

    //Check if a velocity controller is needed, i.e. user want a desired speed
    if (refVel != 0) {
        velocityControl = velocityController(refVel);
    }

    //Check if a position controller is needed
    if (refRev != 0) {
        positionControl = positionController(refRev);
    }

    //Check if both controllers are active
    if (refRev != 0 & refVel != 0) {
        //sets the output to the motor to the smallest controller output
        if (velocityControl > positionControl) {
            pwm_duty_cycle = positionControl;
        } else {
            pwm_duty_cycle = velocityControl;
        }
    }
        //If not both controllers are active, set motor output to contoller output
    else if (refVel != 0) {
        pwm_duty_cycle = velocityControl;
    } else {
        pwm_duty_cycle = positionControl;
    }

}

float *freqPtr;
int *durationPtr;

void playTune(int toneSum) {
    for (int i = 0; i < toneSum; i++) {
        pc.printf("play %f Hz for %d us.\n", freqPtr[i], durationPtr[i] * 100);
        pwm_duty_cycle = 1;
        set_PWM_period_us(1000000 / freqPtr[i]);
        wait_ms(durationPtr[i] * 150);
        set_PWM_period_us(50);
    }
}

void readRegex() {
    char regex[64];
    int i = 0;
    char c;
    int toneSum = 0;
    if (pc.readable()) {
        while (pc.readable()) {
            c = pc.getc();
            regex[i] = c;
            i++;
        }
        regex[i] = '\0';
        pc.printf("\nRegex=%s, ", regex);
        interpreteRegex(regex, i);

        if (getMotorCommands(&revFloat, &velFloat)) {
            total_distance_fine = 0;
            total_distance_coarse = 0;
            passedHome = false;
            IVelError = 0;
            IPosError = 0;
            pc.printf("TODO: execute motor command, rev=%f, vel=%f\n\r", revFloat, velFloat);
            startMotor();
        } else {
            toneSum = getTune(&freqPtr, &durationPtr);
            if (toneSum) {
                playTune(toneSum);
            }
        }
        i = 0;
    }
}

void printInfoThread() {
    //while (true) {
        //pc.printf("Rotor speed: %f, Rotor position: %f, PWM: %f, total distance: %f\n\r", measured_speed_fine,rotor_position, pwm_duty_cycle, total_distance_fine / 360);
        //Thread::wait(3000);
    //}
}

// reset the timer every 20 minutes to prevent it from overflowing
void resetTimer() {
    if (speedTimer.read_us() > 1200000000) {
        speedTimer.reset();
        timer_reset_fine = true;
        timer_reset_coarse = true;
    }
}


void controlThread() {
    
    while (true) {
        controller(revFloat, velFloat);
        Thread::wait(10);
    }
}

void regexThread() {
    while(true) {
        readRegex();
        resetTimer();
        Thread::wait(2000);
    }
}


void setup(void) {
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
    int pwm_period = 100;
    set_PWM_period_us(pwm_period);

    // Set initial motor speed
    pwm_duty_cycle = 0.9f;

    // Start timer
    speedTimer.start();
    last_time_fine = 0;

    increments = 0;
    rotor_position = 0.0f;
}


int main(void) {
    Thread t1(osPriorityLow, 512);
    Thread t2(osPriorityHigh, 512);
    Thread t3(osPriorityLow, 512);

    setup();
    pc.baud(115200);
    velFloat = 0;
    revFloat = 0;
    //t1.start(printInfoThread);
    //t3.start(regexThread);
    t2.max_stack();
    t2.start(controlThread);

    while(true) {
        //t1.free_stack();
        //t3.free_stack();
        t2.free_stack();
        Thread::wait(1000);
        pc.printf("Rotor speed: %f, Rotor position: %f, PWM: %f, total distance: %f\n\r", measured_speed_fine,rotor_position, pwm_duty_cycle, total_distance_fine / 360);
        pc.printf("velFloat: %f, revFloat: %f\n\r", velFloat, revFloat);
        readRegex();    
    }


}