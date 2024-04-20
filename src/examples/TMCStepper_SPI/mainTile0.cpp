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

#include "KinematicPoint.h"

//xsim main.xe --vcd-tracing "-o trace.vcd -tile tile[0] -ports"

double  Rsense = 0.075;

#include "TMCStepper.h"

uint32_t DONE_STATUS_T0 = 0xD04ED04E;
uint32_t STALLED_T0 = 0xBAAAAAAD;
uint32_t GENERATINGWAVE_T0 = 0xA1A1A1A1;
uint32_t NOSTALL_T0 = 0xBADABEBA; 

bool stallDetected_T0 = false;

#define SHOULDER_SWIVEL_BELT_RATIO 10
#define SHOULDER_SWIVEL_ANGLE_TO_STEP_COEFFICIENT (1.8 / SHOULDER_SWIVEL_BELT_RATIO)
#define SHOULDER_SWIVEL_CURRENT 2000

#define SHOULDER_JOINT_BELT_RATIO 5.5
#define SHOULDER_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / SHOULDER_JOINT_BELT_RATIO)
#define SHOULDER_JOINT_CURRENT 3000

#define ELBOW_JOINT_BELT_RATIO 22.2
#define ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT (0.35 / ELBOW_JOINT_BELT_RATIO)
#define ELBOW_JOINT_CURRENT 1680

#define WRIST_SWIVEL_BELT_RATIO 1 
#define WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_SWIVEL_BELT_RATIO)
#define WRIST_SWIVEL_CURRENT 400

#define WRIST_JOINT_BELT_RATIO 4.5
#define WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_JOINT_BELT_RATIO)
#define WRIST_JOINT_CURRENT 800


port_t p_miso  = XS1_PORT_1P;
port_t p_ss = XS1_PORT_1A; //Active Low CHIP SELECT
port_t p_sclk = XS1_PORT_1C; //SPI CLK
port_t p_mosi = XS1_PORT_1D;
xclock_t cb = XS1_CLKBLK_1;
port_t led = XS1_PORT_4F;

void setup_driver(TMC5160Stepper* driver, int current) 
{
    driver->begin();
    driver->toff(4); //off time
    driver->microsteps(16); //16 microsteps
    driver->rms_current(current); 
    driver->en_pwm_mode(true);
    driver->pwm_autoscale(true);
	// driver->AMAX(500);
    // driver->VSTART(10);
	// driver->VMAX(2000);
	// driver->DMAX(700);
	// driver->VSTOP(10);
	// driver->RAMPMODE(0);
	// driver->XTARGET(2000);
    // XTARGET = -51200 (Move one rotation left (200*256 microsteps) 

    driver->shaft(true);
    // driver->sgt(4); //-64 to 63 lower being more sensitive and higher being less so
    // driver->TSTEP(); //Setting TCOOLTHRS > TSTEP for stall to be enabled???

    //Monitor sgt_result until its between 0 and 100
    // driver->sgt_result();

}

int calculateMicroSteps(double desiredAngle, double stepAngle, int stepResolution) 
{
    
    // Calculate angle per microstep
    double anglePerMicrostep = stepAngle / stepResolution;
    
    // Calculate precise microsteps needed
    double preciseMicrosteps = desiredAngle / anglePerMicrostep;

    std::cout << preciseMicrosteps << std::endl;
    
    // Round microsteps to nearest whole number
    return static_cast<int>(std::round(preciseMicrosteps));
}

extern KinematicPoint getKinematicPointAtoB(KinematicPoint currentkp, double desiredx, double desiredy, double desiredz);

extern "C" //SPI and Kinematics
void main_tile0(chanend_t c)
{
    //Setup all the drivers and SPI contexts
    port_t DUMMYFORTESTING = XS1_PORT_1L;
    spi_master_t SHOULDER_SWIVEL_SPI_CTX, SHOULDER_JOINT_SPI_CTX, ELBOW_JOINT_SPI_CTX, WRIST_SWIVEL_SPI_CTX, WRIST_JOINT_SPI_CTX;

    // port_t SHOULDER_SWIVEL_CS = XS1_PORT_1N; //X0D37 I2C_SCL, pin 5 on expansion header
    port_t SHOULDER_SWIVEL_CS = DUMMYFORTESTING;
    spi_master_init(&SHOULDER_SWIVEL_SPI_CTX, cb, SHOULDER_SWIVEL_CS, p_sclk, p_mosi, p_miso);

    port_t SHOULDER_JOINT_CS = DUMMYFORTESTING;
    spi_master_init(&SHOULDER_JOINT_SPI_CTX, cb, SHOULDER_JOINT_CS, p_sclk, p_mosi, p_miso);
    
    //Normal CS, Driver 1
    // port_t ELBOW_JOINT_CS = p_ss;
    spi_master_init(&ELBOW_JOINT_SPI_CTX, cb, p_ss, p_sclk, p_mosi, p_miso);

    //2nd driver CS
    port_t WRIST_SWIVEL_CS = XS1_PORT_1O; //X0D38, I2C_SDA, pin 3 on expansion header
    spi_master_init(&WRIST_SWIVEL_SPI_CTX, cb, WRIST_SWIVEL_CS, p_sclk, p_mosi, p_miso); //WRIST SWIVEL is using normal CS on header

    port_t WRIST_JOINT_CS = XS1_PORT_1N; //X0D37 I2C_SCL, pin 5 on expansion header
    //3rd driver CS
    spi_master_init(&WRIST_JOINT_SPI_CTX, cb, WRIST_JOINT_CS, p_sclk, p_mosi, p_miso);

    spi_master_device_t spi_device_SHOULDER_SWIVEL, spi_device_SHOULDER_JOINT, spi_device_ELBOW_JOINT, spi_device_WRIST_SWIVEL, spi_device_WRIST_JOINT;

    TMC5160Stepper* SHOULDER_SWIVEL_DRIVER = new TMC5160Stepper(Rsense, &spi_device_SHOULDER_SWIVEL, &SHOULDER_SWIVEL_SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        75, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* SHOULDER_JOINT_DRIVER = new TMC5160Stepper(Rsense, &spi_device_SHOULDER_JOINT, &SHOULDER_JOINT_SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        75, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* ELBOW_JOINT_DRIVER = new TMC5160Stepper(Rsense, &spi_device_ELBOW_JOINT, &ELBOW_JOINT_SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        75, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* WRIST_SWIVEL_DRIVER = new TMC5160Stepper(Rsense, &spi_device_WRIST_SWIVEL, &WRIST_SWIVEL_SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        75, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* WRIST_JOINT_DRIVER = new TMC5160Stepper(Rsense, &spi_device_WRIST_JOINT, &WRIST_JOINT_SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        75, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    setup_driver(SHOULDER_SWIVEL_DRIVER, SHOULDER_SWIVEL_CURRENT);
    setup_driver(SHOULDER_JOINT_DRIVER, SHOULDER_JOINT_CURRENT); 
    setup_driver(ELBOW_JOINT_DRIVER, ELBOW_JOINT_CURRENT); 
    setup_driver(WRIST_SWIVEL_DRIVER, WRIST_SWIVEL_CURRENT);
    setup_driver(WRIST_JOINT_DRIVER, WRIST_JOINT_CURRENT);
    
    // double elbow[] = {0, 45, 50, 45, 0 };
    // double wristSwiv[] = {0, 90, 0, -90, 0};
    // double wristJoint[] = {0, 45, 50, 45, 0};

    int posInArray = 0;

    double SHOULDER_SWIVEL_ANGLE_TO_MOVE, SHOULDER_JOINT_ANGLE_TO_MOVE, ELBOW_JOINT_ANGLE_TO_MOVE, WRIST_SWIVEL_ANGLE_TO_MOVE, WRIST_JOINT_ANGLE_TO_MOVE;

    while(1) //Voice Control, Kinematics, Stall checks
    {

        //Wait for voice commands, then generate kinematics to get us there

        //nextz must be greater than board to shoulder which is 0.23150

        // double currentx = 0, currenty = 0, currentz = 0.811624;
        // double nextx = -0.122, nexty = 0.002, nextz = 0.297;
        // double S1angle = 0, J1angle = 0, J2angle = 0, J3angle = 0, S3angle = 0;

        // KinematicPoint currentkp = KinematicPoint(S1angle, J1angle, J2angle, J3angle, S3angle, currentx, currenty, currentz);

        // KinematicPoint nextkp = getKinematicPointAtoB(currentkp, nextx, nexty, nextz);

        // //Bad Request Handling
        // if(nextkp.getX() == 7 && nextkp.getY() == 7 && nextkp.getZ() == 7)
        // {
        //     continue; //If we get a bad kinematic request, we go back to top of loop and skip execution and await voice commands again
        // }

        // double SHOULDER_SWIVEL_ANGLE_TO_MOVE = currentkp.getAngle1() - nextkp.getAngle1();
        // double SHOULDER_JOINT_ANGLE_TO_MOVE = currentkp.getAngle2() - nextkp.getAngle2();
        // double ELBOW_JOINT_ANGLE_TO_MOVE = currentkp.getAngle3() - nextkp.getAngle3();
        // double WRIST_SWIVEL_ANGLE_TO_MOVE = currentkp.getAngle4() - nextkp.getAngle4();
        // double WRIST_JOINT_ANGLE_TO_MOVE = currentkp.getAngle5() - nextkp.getAngle5();

        // //FIX ME AND MAKE SURE DIRECTIONS MATCH
        // if(SHOULDER_SWIVEL_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
        //     SHOULDER_SWIVEL_DRIVER->shaft(false); //false being left, true being right
        // else
        //     SHOULDER_SWIVEL_DRIVER->shaft(true);

        // if(SHOULDER_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
        //     SHOULDER_JOINT_DRIVER->shaft(false); //false being left, true being right
        // else
        //     SHOULDER_JOINT_DRIVER->shaft(true);

        // if(ELBOW_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
        //     ELBOW_JOINT_DRIVER->shaft(false); //false being left, true being right
        // else
        //     ELBOW_JOINT_DRIVER->shaft(true);

        // if(WRIST_SWIVEL_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
        //     WRIST_SWIVEL_DRIVER->shaft(false); //false being left, true being right
        // else
        //     WRIST_SWIVEL_DRIVER->shaft(true);

        // if(WRIST_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
        //     WRIST_JOINT_DRIVER->shaft(false); //false being left, true being right
        // else
        //     WRIST_JOINT_DRIVER->shaft(true);

        
        int SHOULDER_SWIVEL_FULL_STEPS, SHOULDER_JOINT_FULL_STEPS, ELBOW_JOINT_FULL_STEPS, WRIST_SWIVEL_FULL_STEPS, WRIST_JOINT_FULL_STEPS;
        int SHOULDER_SWIVEL_MICRO_STEPS, SHOULDER_JOINT_MICRO_STEPS, ELBOW_JOINT_MICRO_STEPS, WRIST_SWIVEL_MICRO_STEPS, WRIST_JOINT_MICRO_STEPS;

        int stepResolution = 16; 

        //FULL and MICRO CODE
        // calculateSteps(SHOULDER_SWIVEL_ANGLE, SHOULDER_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution, SHOULDER_SWIVEL_FULL_STEPS, SHOULDER_SWIVEL_MICRO_STEPS);
        // // calculateSteps(SHOULDER_JOINT_ANGLE, SHOULDER_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution, SHOULDER_JOINT_FULL_STEPS, SHOULDER_JOINT_MICRO_STEPS);
        // // calculateSteps(ELBOW_JOINT_ANGLE, ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution, ELBOW_JOINT_FULL_STEPS, ELBOW_JOINT_MICRO_STEPS);
        // calculateSteps(WRIST_SWIVEL_ANGLE, WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution, WRIST_SWIVEL_FULL_STEPS, WRIST_SWIVEL_MICRO_STEPS);
        // // calculateSteps(WRIST_JOINT_ANGLE, WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution, WRIST_JOINT_FULL_STEPS, WRIST_JOINT_MICRO_STEPS);
        

        // if(posInArray >= 5)
        //         posInArray = 0;

        // ELBOW_JOINT_ANGLE_TO_MOVE = elbow[posInArray];
        // WRIST_SWIVEL_ANGLE_TO_MOVE = wristSwiv[posInArray];
        // WRIST_JOINT_ANGLE_TO_MOVE = wristJoint[posInArray];

        ELBOW_JOINT_ANGLE_TO_MOVE = 0;
        WRIST_SWIVEL_ANGLE_TO_MOVE = 30;
        WRIST_JOINT_ANGLE_TO_MOVE = 15;

        // posInArray += 1;

        //FIX ME AND MAKE SURE DIRECTIONS MATCH
        if(SHOULDER_SWIVEL_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
            SHOULDER_SWIVEL_DRIVER->shaft(false); //false being left, true being right
        else
            SHOULDER_SWIVEL_DRIVER->shaft(true);

        if(SHOULDER_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
            SHOULDER_JOINT_DRIVER->shaft(false); //false being left, true being right
        else
            SHOULDER_JOINT_DRIVER->shaft(true);

        if(ELBOW_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
            ELBOW_JOINT_DRIVER->shaft(false); //false being left, true being right
        else
            ELBOW_JOINT_DRIVER->shaft(true);

        if(WRIST_SWIVEL_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
            WRIST_SWIVEL_DRIVER->shaft(false); //false being left, true being right
        else
            WRIST_SWIVEL_DRIVER->shaft(true);

        if(WRIST_JOINT_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
            WRIST_JOINT_DRIVER->shaft(false); //false being left, true being right
        else
            WRIST_JOINT_DRIVER->shaft(true);

        std::cout << ELBOW_JOINT_ANGLE_TO_MOVE << std::endl;

        //MICRO ONLY
        // SHOULDER_SWIVEL_MICRO_STEPS = calculateMicroSteps(abs(SHOULDER_SWIVEL_ANGLE_TO_MOVE), SHOULDER_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
        // SHOULDER_JOINT_MICRO_STEPS = calculateMicroSteps(abs(SHOULDER_JOINT_ANGLE_TO_MOVE), SHOULDER_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
        ELBOW_JOINT_MICRO_STEPS = calculateMicroSteps(abs(ELBOW_JOINT_ANGLE_TO_MOVE), ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
        WRIST_SWIVEL_MICRO_STEPS = calculateMicroSteps(abs(WRIST_SWIVEL_ANGLE_TO_MOVE), WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
        WRIST_JOINT_MICRO_STEPS = calculateMicroSteps(abs(WRIST_JOINT_ANGLE_TO_MOVE), WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);

        // std::cout << "ELBOW JOINT Full " << ELBOW_JOINT_MICRO_STEPS << std::endl;
        // std::cout << "Shoulder Swivel Full " << SHOULDER_SWIVEL_MICRO_STEPS << std::endl;

        std::cout << "Beginning Full Steps" << std::endl;

        std::cout << ELBOW_JOINT_MICRO_STEPS << std::endl;


        // chan_out_word(c, SHOULDER_SWIVEL_FULL_STEPS);
        // chan_out_word(c, SHOULDER_JOINT_FULL_STEPS);
        // chan_out_word(c, ELBOW_JOINT_FULL_STEPS);
        // chan_out_word(c, WRIST_SWIVEL_FULL_STEPS);
        // chan_out_word(c, WRIST_JOINT_FULL_STEPS);

        // chan_out_word(c, SHOULDER_SWIVEL_MICRO_STEPS);
        // chan_out_word(c, SHOULDER_JOINT_MICRO_STEPS);

        chan_out_word(c, ELBOW_JOINT_MICRO_STEPS);
        chan_out_word(c, WRIST_SWIVEL_MICRO_STEPS);
        chan_out_word(c, WRIST_JOINT_MICRO_STEPS);

        int fromTile1;

        // delay_microseconds_cpp(100000); //Wait a moment for steps to get started or the stall will get detected immediately

        while(1)
        {       

            // std::cout << "sg_result " << SHOULDER_SWIVEL_DRIVER->sg_result() << std::endl;
            // std::cout << "stall flag " << SHOULDER_SWIVEL_DRIVER->stallguard() << std::endl;

            if(SHOULDER_SWIVEL_DRIVER->stallguard())
            {
                stallDetected_T0 = true;
            } 

            // if(SHOULDER_JOINT_DRIVER->stallguard())
            // {
            //     stallDetected_T0 = true;
            // }

            // if(ELBOW_JOINT_DRIVER->stallguard())
            // {
            //     stallDetected_T0 = true;
            // }

            if(WRIST_SWIVEL_DRIVER->XTARGET())
            {
                stallDetected_T0 = true;
            }

            // if(WRIST_JOINT_DRIVER->XTARGET()) //There's no reason to check stall if were done
            // {
            //     stallDetected_T0 = true;
            // }

            stallDetected_T0 = false;

            if(stallDetected_T0)
            {
                // printf("Stalled\n");
                chan_out_word(c, STALLED_T0);
                break;
            }
            else
            {
                chan_out_word(c, NOSTALL_T0);
            }

            fromTile1 = chan_in_word(c);

            std::cout << "On tile 0, received from tile 1 "<<std::hex << fromTile1 << std::endl;

            if(fromTile1 == DONE_STATUS_T0)
            {   
                // printf("All waves finished\n");
                break;
            }
            
        }

        // if(stallDetected_T0) //If we stalled, handle it
        // {
        //     //TO BE DECIDED
        // }
        // else
        // {
        //     //GO BACK TO VOICE COMMANDS
        //     return;

        // }

        return;

    }


    // else
    //  {

    //     std::cout << "Starting MicroSteps" << std::endl;

    //     chan_out_word(c, SHOULDER_SWIVEL_MICRO_STEPS);
    //     // chan_out_word(c, SHOULDER_JOINT_MICRO_STEPS);
    //     // chan_out_word(c, ELBOW_JOINT_MICRO_STEPS);
    //     chan_out_word(c, WRIST_SWIVEL_MICRO_STEPS);
    //     // chan_out_word(c, WRIST_JOINT_MICRO_STEPS);


    //     while(1)
    //     {       
    //         if(SHOULDER_SWIVEL_DRIVER->stallguard())
    //         {
    //             stallDetected_T0 = true;
    //         } 

    //         // if(SHOULDER_JOINT_DRIVER->stallguard())
    //         // {
    //         //     stallDetected_T0 = true;
    //         // }

    //         // if(ELBOW_JOINT_DRIVER->stallguard())
    //         // {
    //         //     stallDetected_T0 = true;
    //         // }

    //         if(WRIST_SWIVEL_DRIVER->XTARGET())
    //         {
    //             stallDetected_T0 = true;
    //         }

    //         // if(WRIST_JOINT_DRIVER->XTARGET()) //There's no reason to check stall if were done
    //         // {
    //         //     stallDetected_T0 = true;
    //         // }

    //         if(stallDetected_T0)
    //         {
    //             printf("Stalled\n");
    //             chan_out_word(c, STALLED_T0);
    //             break;
    //         }
    //         else
    //         {
    //             chan_out_word(c, NOSTALL_T0);
    //         }

    //         uint32_t fromTile1 = chan_in_word(c);

    //         std::cout << "On tile 0, received from tile 1 "<<std::hex << fromTile1 << std::endl;


    //         if(fromTile1 == DONE_STATUS_T0)
    //         {   
    //             // printf("All waves finished\n");
    //             break;
    //         }
            
    //     }

    // }


}