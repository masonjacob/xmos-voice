#include <stdio.h>
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

//xsim main.xe --vcd-tracing "-o trace.vcd -tile tile[0] -ports"

float Rsense = 0.075;

#include "TMCStepper.h"


#define WRIST_ROTATION_CURRENT 2000

uint32_t INITIATE_ACTION = 0xFEEDF00D;
uint32_t DONE_STATUS = 0xD04ED04E;
uint32_t STALLED = 0xBAAAAAAD;
uint32_t SHUTDOWN = 0xEC11EC11;


// port_t p_miso  = XS1_PORT_1P;
// port_t p_ss = XS1_PORT_1A; //Active Low CHIP SELECT
// port_t p_sclk = XS1_PORT_1C; //SPI CLK
// port_t p_mosi = XS1_PORT_1D;
// xclock_t cb = XS1_CLKBLK_1;
// port_t led = XS1_PORT_4F;
// port_t step_pin = XS1_PORT_1O; //X0D38, I2C_SDA, pin 3 on expansion header
bool stallDetected = false;

//"XS1_PORT_1D"  Name="PORT_SPI_MOSI"/>
//<Port Location="XS1_PORT_1P"  Name="PORT_SPI_MISO"/>
//
//
void setup_driver(TMC5160Stepper* driver, int current) {
    driver->begin();
    std::cout << "Setup check 1" << std::endl;
    driver->toff(4); //off time
    driver->microsteps(16); //16 microsteps
    driver->rms_current(current); //400mA RMS
    driver->en_pwm_mode(true);
    driver->pwm_autoscale(true);
	driver->AMAX(500);
    driver->VSTART(10);
	driver->VMAX(2000);
	driver->DMAX(700);
	driver->VSTOP(10);
	driver->RAMPMODE(1);
	driver->XTARGET(2000);
    // XTARGET = -51200 (Move one rotation left (200*256 microsteps) 
    driver->shaft(true);

    //Snapshot from Teemuatlut for positioning mode. Explicitly says step and dir and enable are wired to gnd
    // driver.toff(3);
	// driver.rms_current(800);
	// driver.en_pwm_mode(true);

	// driver.A1(1000);
	// driver.V1(50000);
	// driver.AMAX(500);
	// driver.VMAX(200000);
	// driver.DMAX(700);
	// driver.D1(1400);
	// driver.VSTOP(10);
	// driver.RAMPMODE(0);
	// driver.XTARGET(-51200);



    //Snapshot of initial commands for xtarget in positioning mode from datasheet
    //SPI send: 0xEC000100C3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
    // SPI send: 0x9000061F0A; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
    // SPI send: 0x910000000A; // TPOWERDOWN=10: Delay before power down in stand still
    // SPI send: 0x8000000004; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
    // SPI send: 0x93000001F4; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
    // SPI sample sequence to enable and initialize the motion controller and move one rotation (51200
    // microsteps) using the ramp generator. A read access querying the actual position is also shown.
    // SPI send: 0xA4000003E8; // A1 = 1 000 First acceleration
    // SPI send: 0xA50000C350; // V1 = 50 000 Acceleration threshold velocity V1
    // SPI send: 0xA6000001F4; // AMAX = 500 Acceleration above V1
    // SPI send: 0xA700030D40; // VMAX = 200 000
    // SPI send: 0xA8000002BC; // DMAX = 700 Deceleration above V1
    // SPI send: 0xAA00000578; // D1 = 1400 Deceleration below V1
    // SPI send: 0xAB0000000A; // VSTOP = 10 Stop velocity (Near to zero)
    // SPI send: 0xA000000000; // RAMPMODE = 0 (Target position move)
    // // Ready to move!
    // SPI send: 0xADFFFF3800; // XTARGET = -51200 (Move one rotation left (200*256 microsteps)
    // // Now motor 1 starts rotating
    // SPI send: 0x2100000000; // Query XACTUAL – The next read access delivers XACTUAL
    // SPI read; // Read XACTUAL

}

// class SquareWave {
// public:
//     port_t step_pin;
//     int delay_in_microseconds;
//     int numSteps;
//     int currentStep;
//     bool currentState;
//     bool enabled;
//     unsigned long lastToggleTime; // This now holds the starting reference time
//     hwtimer_t timer;

//     // Modified constructor to accept an initial starting time
//     SquareWave(port_t pin, int delayMicros, int steps, hwtimer_t hwTimer, int startTime)
//         : step_pin(pin), delay_in_microseconds(delayMicros), numSteps(steps), timer(hwTimer),
//           currentStep(0), currentState(false), enabled(true), lastToggleTime(startTime) {}

//     void update() {
//         if (enabled) {
//             int currentTime = hwtimer_get_time(timer);
//             if ((currentTime - lastToggleTime) >= delay_in_microseconds * 100) { //For some reason it seems the timer is in 10^-8, so micro, 10^-6 to 10^-8, micro * 10^2 = 10^-8 version
//                 std::cout << ((currentTime -lastToggleTime) * 100 )<< std::endl;
//                 currentState = !currentState;
//                 port_out(step_pin, currentState);
//                 std::cout << currentState << std::endl;
//                 lastToggleTime = currentTime; // Update with the current time for next toggle
                
//                 if(currentState) //Our steps are number of high's so only add if we get a high
//                     currentStep++;

//                 if (currentStep >= numSteps) {
//                     enabled = false;
//                     currentStep = 0;
//                     port_out(step_pin, 0); // Optionally set the pin to a known state (low) when disabled
//                 }
//             }
//         }
//     }

//     // Method to re-enable the square wave with a new starting time
//     void reEnable(int steps, unsigned long startTime) {
//         numSteps = steps;
//         currentStep = 0;
//         enabled = true;
//         lastToggleTime = startTime; // Set the lastToggleTime to the new starting time
//     }
// };

// // Assume global or member variable for storage
// std::vector<SquareWave> squareWaves;

// // ISR remains largely the same in structure, iterating over objects
// void ISR() {
//     for (auto& squareWave : squareWaves) {
//         squareWave.update();
//     }
// }

// class StepWaveGenerator {
// public:
//     StepWaveGenerator(port_t pin, int count, int delayUs)
//     : stepPin(pin) {}

//     void generateWave(port_t stepPin, uint32_t stepCount, uint32_t stepDelayMicroseconds) 
//     {        
        
//         for (int i = 0; i < stepCount; ++i) 
//         {
//             // if(stallDetected) //Global variable. Since PARJOB completes executions before returning control to after the PARJOB {}, we need a way to from outside the single control line, stop the motors in case of a stall
//             //     break;
//             // port_out(stepPin, 1);
//             // delay_microseconds(stepDelayMicroseconds);
//             // port_out(stepPin, 0);
//             // delay_microseconds(stepDelayMicroseconds);
//         }
//         //These will keep getting run over and over in the PAR, so we want stepCount to be 0 until updated
//         stepCount = 0;
//         // port_out(stepPin, 0); //Not needed but just in case
        
//     }

//     // // Optionally, a method to update parameters if neededS
//     // void updateParameters(int count, int delayUs) {
//     //     stepCount = count;
//     //     stepDelayMicroseconds = delayUs;
//     // }

// private:
//     port_t stepPin;
// };

DECLARE_JOB(myGenerateWave, (port_t, uint32_t, uint32_t, chanend_t));
void myGenerateWave(port_t stepPin, uint32_t stepCount, uint32_t stepDelayMicroseconds, chanend_t controlToStepper) 
{        

    chan_in_word(controlToStepper); // Weirdly works and is fine. Putting below in the loop causes issues though

    for (int i = 0; i < stepCount; ++i) 
    {
        port_out(stepPin, 1);
        // delay_microseconds(stepDelayMicroseconds);
        printf("\ndelay disabled %d\n", (int) stepPin);
        port_out(stepPin, 0);
        // delay_microseconds(stepDelayMicroseconds);
        printf("\ndelay disabled %d\n", (int) stepPin);

    //FIX ME! Weird issue when trying to use the channel down here.
        // if(chan_in_word(controlToStepper) == STALLED)
        // {
        //     stepCount = 0;
        //     port_out(stepPin, 0);
        //     return;
        // }


    }

    //These will keep getting run over and over in the PAR, so we want stepCount to be 0 until updated
    stepCount = 0;
    port_out(stepPin, 0); //Not needed but just in case
    return;
    
}

DECLARE_JOB(parCallGenerateWave, (port_t, chanend_t));  
void parCallGenerateWave(port_t pin, chanend_t controlToStepper)
{
    chan_out_word(controlToStepper,DONE_STATUS);
    uint32_t control=0;
    uint32_t stepCount=0;
    uint32_t stepDelayMicroseconds=0;
    while(1)
    {
        control = chan_in_word(controlToStepper);

        if(control == SHUTDOWN) //Our shutdown needs to available before and after the step wave is generated.
            return;
        
        
       if(control == INITIATE_ACTION)
        {
            stepCount = chan_in_word(controlToStepper);
            stepDelayMicroseconds = chan_in_word(controlToStepper);

                myGenerateWave(pin, stepCount, stepDelayMicroseconds, controlToStepper);

            if(chan_in_word(controlToStepper) == SHUTDOWN)
                return;
            else
                chan_out_word(controlToStepper, DONE_STATUS);
        }
    }
    return;
}

DECLARE_JOB(parControlLoop, (chanend_t,chanend_t));
void parControlLoop(chanend_t stepper0, chanend_t stepper1)
{
    for(int i=0; i<1; i++)
    {

        //This weirdly works with no issue
        // printf("Before");
        // delay_microseconds(100);
        // printf("After");
        
        if(chan_in_word(stepper0) == DONE_STATUS)
        {
            chan_out_word(stepper0, INITIATE_ACTION);   
            chan_out_word(stepper0, 20);
            chan_out_word(stepper0, 100);
        }

        if(chan_in_word(stepper1) == DONE_STATUS)
        {
            chan_out_word(stepper1, INITIATE_ACTION);   
            chan_out_word(stepper1, 15);
            chan_out_word(stepper1, 50);
        }
    }

    chan_out_word(stepper0, SHUTDOWN);
    printf("here");
    chan_out_word(stepper1, SHUTDOWN);

    return;
}

int main() 
{

    // spi_master_t spi_ctx;

    // spi_master_init(&spi_ctx, cb, p_ss, p_sclk, p_mosi, p_miso);

    // spi_master_device_t spi_dev_0;

    // TMC5160Stepper* driver = new TMC5160Stepper(Rsense, &spi_dev_0, &spi_ctx,
    //     0,
    //     1, 1,  //cpol, cpha
    //     spi_master_source_clock_xcore,
    //     75, //Was 75 600Mhz/(4 * 75) = 2MHz
    //     spi_master_sample_delay_0,
    //     0, 0 ,0 ,0);

    // setup_driver(driver, WRIST_ROTATION_CURRENT);
    
    // port_enable(led);
    // port_enable(p_sclk);

    // port_out(step_pin, 1);
    // hwtimer_t timer = hwtimer_alloc();
    // int delay_start = hwtimer_get_time(timer);
    // int step_delay = 100000; //in microseconds

    // SquareWave newWave(step_pin, step_delay, 10, timer, hwtimer_get_time(timer));
    // squareWaves.push_back(newWave);

    // int start = hwtimer_get_time(timer);
            //       SecMseMicNan

    // std::vector<StepWaveGenerator> generators;
    
    int stepCount = 10;
    int stepDelay = 250; //In microseconds

    port_t pin2 = XS1_PORT_1N;
    port_enable(pin2);
    port_t step_pin = XS1_PORT_1O;
    port_enable(step_pin);

    // generators.push_back(StepWaveGenerator(step_pin, stepCount, stepDelay));
    // generators.push_back(StepWaveGenerator(pin2, 20, 100));

    // StepWaveGenerator gen = StepWaveGenerator(pin2, 20, 100);
    // StepWaveGenerator gen2 = StepWaveGenerator(step_pin, stepCount, stepDelay);
    
    channel_t stepper_motors[2];
    stepper_motors[0] = chan_alloc();
    stepper_motors[1] = chan_alloc();
    
    PAR_JOBS( //Step pin wave generators
        PJOB(parControlLoop, (stepper_motors[0].end_a, stepper_motors[1].end_a)),
        PJOB(parCallGenerateWave, (pin2, stepper_motors[0].end_b)), //make sure to seperate PJOB by a ","
        PJOB(parCallGenerateWave, (step_pin, stepper_motors[1].end_b))


    );

    chan_free(stepper_motors[0]);
    chan_free(stepper_motors[1]);




       
       
        // std::cout << driver->sg_stop() << std::endl;

        // auto xactual = driver->XACTUAL();
        // auto xtarget = driver->XTARGET();

        // // //std::cout<<"ioin="<<driver->IOIN()<<" xactual="<<xactual<<"\n";
        // std::cout << "xtarget" << xtarget << std::endl;
        // std::cout << "xactual " << xactual << std::endl;

        // driver->sgt(0);
        // for(int i = 0; i < 10000; i++)
        // {
        // port_out(step_pin, 1);
        // delay_microseconds(1000);
        // port_out(step_pin, 0);
        // delay_microseconds(1000);
        
        // // std::cout << driver->sg_stop() << std::endl;
        // }
    
        // if (xactual == xtarget) 
        //     {
        //     driver->XTARGET(-xactual);
        //     }

        // port_out(led, 0b1100);
        // delay_milliseconds(100);
        // port_out(led, 0b0000);
        // delay_milliseconds(100);

        // port_out(p_sclk, 0);
        // delay_microseconds(10);
        // port_out(p_sclk, 1);
        // delay_microseconds(10);

    

    // port_disable(p_sclk);

    ////////////////////////////////////////
    //// WORKING BASIC I/O TEST         ////
    ////////////////////////////////////////

    // port_enable(p_miso);
    // port_enable(led);

    // while(1) {
    //     port_out(p_miso, 1);
    //     port_out(led, 0b1100);
    //     delay_milliseconds(1000);
    //     port_out(p_miso, 0);
    //     port_out(led, 0b0000);
    //     delay_milliseconds(1000);
    // }

    // port_disable(p_miso);
    // port_disable(led);

}