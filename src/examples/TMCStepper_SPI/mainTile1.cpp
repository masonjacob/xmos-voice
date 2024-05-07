#include <stdio.h>
#include <iomanip> // For std::hex
#include <stdint.h>
#include <xs1.h>
#include <syscall.h>
#include <timer.h>
#include <stdio.h>
#include <platform.h>
#include <xclib.h>
#include <iostream>
#include <xcore/hwtimer.h>
#include <vector>
#include <xcore/parallel.h>
#include <xcore/channel.h>
#include <xcore/channel_streaming.h> // non blocking channel comms
#include <xcore/port.h>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <array>
#include <utility> // for std::pair
#include <algorithm>
#include <atomic>

#include <type_traits>  
#include <chrono>

//xsim main.xe --vcd-tracing "-o trace.vcd -tile tile[0] -ports"

uint32_t INITIATE_ACTION = 0xFEEDF00D;
uint32_t DONE_STATUS = 0xD04ED04E;
uint32_t STALLED = 0xBAAAAAAD;
uint32_t SHUTDOWN = 0xEC11EC11;
uint32_t WAITING = 0x00000000;
uint32_t NOSTALL = 0xBADABEBA; 
uint32_t GENERATINGWAVE = 0xA1A1A1A1;

bool stallDetected = false;
volatile bool DONE0, DONE1, DONE2, DONE3, DONE4; // All initialized to false. DONE flag for each driver

// port_t SHOULDER_SWIVEL_STEP_PIN = XS1_PORT_1K; //XS1_PORT_1A I2S DOUT, X1D00, 3.3v Pin 40 Tile 1 //Spins in theta, that is 360 degrees parrallel plane to the ground

port_t SHOULDER_SWIVEL_STEP_PIN = XS1_PORT_1C; //pin 8
port_t SHOULDER_JOINT_STEP_PIN = XS1_PORT_1D; // PIN 6


// port_t SHOULDER_JOINT_STEP_PIN = XS1_PORT_1N; //Moves in the phi, that is up and down


port_t ELBOW_JOINT_STEP_PIN = XS1_PORT_1F; //I2S_DOUT, XD00, Pin 40 on breakout


// XS1_PORT_1B; //I2S_LRCK, X1D01, pin 35
port_t WRIST_SWIVEL_STEP_PIN = XS1_PORT_1G; //I2S_LRCK, X1D01, pin 35


port_t WRIST_JOINT_STEP_PIN = XS1_PORT_1K; //XS1_PORT_1A, I2S DOUT, X1D00, 3.3v Pin 38 Tile 1
// port_t WRIST_JOINT_STEP_PIN = XS1_PORT_1A; //XS1_PORT_1K I2S DIN, X1D34, 3.3v Pin 40 Tile 1

inline void delay_ticks_longlong_cpp(uint32_t period)
{
    hwtimer_t t = hwtimer_alloc();
    hwtimer_delay(t, period); 
    hwtimer_free(t);
}

inline void delay_microseconds_cpp(unsigned microseconds)
{
    uint32_t microsec = microseconds *100;
    delay_ticks_longlong_cpp(microsec);
}

inline int myGenerateWave(port_t stepPin, int stepCount, int stepDelayMicroseconds) 
{        
    for (int i = 0; i < stepCount; ++i) 
    {
        port_out(stepPin, 1);
        delay_microseconds_cpp(stepDelayMicroseconds);
        port_out(stepPin, 0);
        delay_microseconds_cpp(stepDelayMicroseconds);

        if(stallDetected)
        {
            stepCount = 0;
            port_out(stepPin, 0);
            return -1;
        }

    }

    //These will keep getting run over and over in the PAR, so we want stepCount to be 0 until updated
    stepCount = 0;
    port_out(stepPin, 0); //Not needed but just in case
    return 0;
    
}

DECLARE_JOB(parWavegen, (port_t, int, int, volatile bool&));  
void parWavegen(port_t pin, int stepCount, int stepDelayMicroseconds, volatile bool& DONE_FLAG)
{   
    int return_status = myGenerateWave(pin, stepCount, stepDelayMicroseconds);
    DONE_FLAG = true;
    delay_microseconds_cpp(10);
    // printf("Wave gen done");
    return;
}

DECLARE_JOB(parStepsControlLoop, (chanend_t));
void parStepsControlLoop(chanend_t tileToTile)
{

    DONE0 = false;
    DONE1 = false;
    DONE2 = false;
    DONE3 = false;
    DONE4 = false;

    stallDetected = false;

    while(1)
    {       
       uint32_t msgFromTile0 = chan_in_word(tileToTile);
  
        if(msgFromTile0 == STALLED)
        {
            stallDetected = true;
            return;
        }

        if(DONE0 && DONE1 && DONE2 && DONE3 && DONE4)
        {       
            printf("Tile1 StepGen Done\n");
            chan_out_word(tileToTile, DONE_STATUS);
            return;
        }
        else
        {
            chan_out_word(tileToTile, GENERATINGWAVE);
        }

    }

    return;
}

#define SHOULDER_SWIVEL_BELT_RATIO 10
#define SHOULDER_SWIVEL_ANGLE_TO_STEP_COEFFICIENT (1.8 / SHOULDER_SWIVEL_BELT_RATIO)
#define SHOULDER_SWIVEL_CURRENT 2000

#define SHOULDER_JOINT_BELT_RATIO 5.5
#define SHOULDER_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / SHOULDER_JOINT_BELT_RATIO)
#define SHOULDER_JOINT_CURRENT 3000

#define ELBOW_JOINT_BELT_RATIO 22.2 //60:14 for belt, 5.18:1 for the gearbox stepper itself
#define ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / ELBOW_JOINT_BELT_RATIO)
#define ELBOW_JOINT_CURRENT 1680

#define WRIST_SWIVEL_BELT_RATIO 1 
#define WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_SWIVEL_BELT_RATIO)
#define WRIST_SWIVEL_CURRENT 400

#define WRIST_JOINT_BELT_RATIO 4.5
#define WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_JOINT_BELT_RATIO)
#define WRIST_JOINT_CURRENT 800


extern "C" //Step Pin Generation
void main_tile1(chanend_t c)
{           

    while(1)
    {
        port_enable(SHOULDER_SWIVEL_STEP_PIN);
        port_enable(SHOULDER_JOINT_STEP_PIN);
        port_enable(ELBOW_JOINT_STEP_PIN); //I2S_DOUT pin 38
        port_enable(WRIST_SWIVEL_STEP_PIN); //I2S_LCRK pin 
        port_enable(WRIST_JOINT_STEP_PIN); //I2S_DIN pin 40

        int stepDelay = 5000; //In microseconds

        // int shoulderswivelStepDelay = 1;
        int shoulderswivelStepDelay = stepDelay / SHOULDER_SWIVEL_BELT_RATIO;
        int shoulderjointStepDelay = stepDelay / SHOULDER_JOINT_BELT_RATIO;
        int elbowStepDelay = stepDelay / ELBOW_JOINT_BELT_RATIO;
        int wristSwivelStepDelay = stepDelay / WRIST_SWIVEL_BELT_RATIO;
        int wristJointStepDelay = stepDelay / WRIST_JOINT_BELT_RATIO;
        
        int SHOULDER_SWIVEL_FULL_STEPS = chan_in_word(c);
        int SHOULDER_JOINT_FULL_STEPS = chan_in_word(c);
        int ELBOW_JOINT_FULL_STEPS = chan_in_word(c);
        int WRIST_SWIVEL_FULL_STEPS = chan_in_word(c);
        int WRIST_JOINT_FULL_STEPS = chan_in_word(c);

        std::cout << SHOULDER_SWIVEL_FULL_STEPS << std::endl;

        PAR_JOBS( //Step pin wave generatorion with control
            PJOB(parStepsControlLoop, (c)),
            
            PJOB(parWavegen, (SHOULDER_SWIVEL_STEP_PIN, SHOULDER_SWIVEL_FULL_STEPS, shoulderswivelStepDelay, DONE0)), //make sure to seperate PJOB by a ","
            PJOB(parWavegen, (SHOULDER_JOINT_STEP_PIN, SHOULDER_JOINT_FULL_STEPS, shoulderjointStepDelay, DONE1)),
            PJOB(parWavegen, (ELBOW_JOINT_STEP_PIN, ELBOW_JOINT_FULL_STEPS, elbowStepDelay, DONE2 )), //make sure to seperate PJOB by a ","
            PJOB(parWavegen, (WRIST_SWIVEL_STEP_PIN, WRIST_SWIVEL_FULL_STEPS, wristSwivelStepDelay, DONE3)), //make sure to seperate PJOB by a ","
            PJOB(parWavegen, (WRIST_JOINT_STEP_PIN,  WRIST_JOINT_FULL_STEPS, wristJointStepDelay, DONE4))
        
        );
    }

}