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

#define ELBOW_JOINT_BELT_RATIO 22.2 //60:14 for belt, 5.18:1 for the gearbox stepper itself
#define ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / ELBOW_JOINT_BELT_RATIO)
#define ELBOW_JOINT_CURRENT 1680

#define WRIST_SWIVEL_BELT_RATIO 1 
#define WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_SWIVEL_BELT_RATIO)
#define WRIST_SWIVEL_CURRENT 400

#define WRIST_JOINT_BELT_RATIO 4.5
#define WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT (1.8 / WRIST_JOINT_BELT_RATIO)
#define WRIST_JOINT_CURRENT 800


// port_t p_miso  = XS1_PORT_1P;
// port_t p_ss = XS1_PORT_1A; //Active Low CHIP SELECT
// port_t p_sclk = XS1_PORT_1C; //SPI CLK
// port_t p_mosi = XS1_PORT_1D;

// xclock_t cb = XS1_CLKBLK_1;
// port_t led = XS1_PORT_4F;

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

    // std::cout << preciseMicrosteps << std::endl;
    
    // Round microsteps to nearest whole number
    return static_cast<int>(std::round(preciseMicrosteps));
}

extern KinematicPoint getSimpleKinematics(double desiredx, double desiredy, double desiredz);


class Point {
public:
    double x, y, z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

class AngleSet {
public:
    double S1, J1, J2, S3, J3;

    AngleSet(double S1, double J1, double J2, double S3, double J3) : S1(S1), J1(J1), J2(J2), S3(S3), J3(J3) {}
};

extern "C" //SPI and Kinematics
void main_tile0(chanend_t c)
{
    port_t p_mosi = XS1_PORT_1A; //PCB
    port_t p_miso  = XS1_PORT_1D; //
    port_t p_sclk = XS1_PORT_1L; //
    port_t p_cs = XS1_PORT_8D;

    xclock_t cb = XS1_CLKBLK_1;
    // port_t led = XS1_PORT_4F;


    // //Setup all the drivers and SPI contexts
    // port_t DUMMYFORTESTING = XS1_PORT_1B;

    // spi_master_t SHOULDER_SWIVEL_SPI_CTX, SHOULDER_JOINT_SPI_CTX1, SHOULDER_JOINT_SPI_CTX2, ELBOW_JOINT_SPI_CTX, WRIST_SWIVEL_SPI_CTX, WRIST_JOINT_SPI_CTX;

    // // port_t SHOULDER_SWIVEL_CS = XS1_PORT_1N; //X0D37 I2C_SCL, pin 5 on expansion header
    // port_t SHOULDER_SWIVEL_CS = XS1_PORT_1O; //Osprey X0D38 pin 5
    // spi_master_init(&SHOULDER_SWIVEL_SPI_CTX, cb, SHOULDER_SWIVEL_CS, p_sclk, p_mosi, p_miso);

    // port_t SHOULDER_JOINT_CS1 = XS1_PORT_1P; //pin 3
    // spi_master_init(&SHOULDER_JOINT_SPI_CTX1, cb, SHOULDER_JOINT_CS1, p_sclk, p_mosi, p_miso);
    
    // port_t SHOULDER_JOINT_CS2 = XS1_PORT_1A; //PIN 12
    // spi_master_init(&SHOULDER_JOINT_SPI_CTX2, cb, SHOULDER_JOINT_CS2, p_sclk, p_mosi, p_miso);

    // //Normal CS, Driver 1
    // port_t ELBOW_JOINT_CS = DUMMYFORTESTING;
    // spi_master_init(&ELBOW_JOINT_SPI_CTX, cb, ELBOW_JOINT_CS, p_sclk, p_mosi, p_miso);

    // //2nd driver CS
    // // port_t WRIST_SWIVEL_CS = XS1_PORT_1O; //X0D38, I2C_SDA, pin 3 on expansion header
    // port_t WRIST_SWIVEL_CS = DUMMYFORTESTING; 
    // spi_master_init(&WRIST_SWIVEL_SPI_CTX, cb, WRIST_SWIVEL_CS, p_sclk, p_mosi, p_miso); //WRIST SWIVEL is using normal CS on header

    // // port_t WRIST_JOINT_CS = XS1_PORT_1N; //X0D37 I2C_SCL, pin 5 on expansion header
    // port_t WRIST_JOINT_CS = DUMMYFORTESTING;
    // //3rd driver CS
    // spi_master_init(&WRIST_JOINT_SPI_CTX, cb, WRIST_JOINT_CS, p_sclk, p_mosi, p_miso);

    spi_master_device_t spi_device_SHOULDER_SWIVEL, spi_device_SHOULDER_JOINT1, spi_device_SHOULDER_JOINT2, spi_device_ELBOW_JOINT, spi_device_WRIST_SWIVEL, spi_device_WRIST_JOINT;


    spi_master_t SPI_CTX;
    spi_master_init(&SPI_CTX, cb, p_cs, p_sclk, p_mosi, p_miso);

    TMC5160Stepper* SHOULDER_SWIVEL_DRIVER = new TMC5160Stepper(Rsense, &spi_device_SHOULDER_SWIVEL, &SPI_CTX,
        2, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    //Clockwise is positive
    TMC5160Stepper* SHOULDER_JOINT_DRIVER1 = new TMC5160Stepper(Rsense, &spi_device_SHOULDER_JOINT1, &SPI_CTX,
        0, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* SHOULDER_JOINT_DRIVER2 = new TMC5160Stepper(Rsense, &spi_device_SHOULDER_JOINT2, &SPI_CTX,
        1, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* ELBOW_JOINT_DRIVER = new TMC5160Stepper(Rsense, &spi_device_ELBOW_JOINT, &SPI_CTX,
        3, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* WRIST_SWIVEL_DRIVER = new TMC5160Stepper(Rsense, &spi_device_WRIST_SWIVEL, &SPI_CTX,
        4, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    TMC5160Stepper* WRIST_JOINT_DRIVER = new TMC5160Stepper(Rsense, &spi_device_WRIST_JOINT, &SPI_CTX,
        5, //Chip Select bit number
        1, 1,  //cpol, cpha
        spi_master_source_clock_xcore,
        150, //Was 75 600Mhz/(4 * 75) = 2MHz
        spi_master_sample_delay_0,
        0, 0 ,0 ,0);

    setup_driver(SHOULDER_SWIVEL_DRIVER, SHOULDER_SWIVEL_CURRENT);
    setup_driver(SHOULDER_JOINT_DRIVER1, SHOULDER_JOINT_CURRENT);
    setup_driver(SHOULDER_JOINT_DRIVER2, SHOULDER_JOINT_CURRENT);
    setup_driver(ELBOW_JOINT_DRIVER, ELBOW_JOINT_CURRENT); 
    setup_driver(WRIST_SWIVEL_DRIVER, WRIST_SWIVEL_CURRENT);
    setup_driver(WRIST_JOINT_DRIVER, WRIST_JOINT_CURRENT);

    double SHOULDER_SWIVEL_ANGLE_TO_MOVE, SHOULDER_JOINT_ANGLE_TO_MOVE, ELBOW_JOINT_ANGLE_TO_MOVE, WRIST_SWIVEL_ANGLE_TO_MOVE, WRIST_JOINT_ANGLE_TO_MOVE;

    // port_t GPIObot2 = XS1_PORT_8C;
    // port_t GPIOtop2 = XS1_PORT_8D;
    // port_enable(GPIObot2);
    // port_enable(GPIOtop2);

    port_t top3 = XS1_PORT_4F;
    port_t bot1 = XS1_PORT_4E; 
    port_enable(top3);
    port_enable(bot1);

    int bit3, bit2, bit1, bit0, bit32;
    int word_id_from_voice;

    #define ASR_NUMBER_OF_COMMANDS  (16)

        typedef struct asr_lut_struct
    {
        int     asr_id;    // ASR response IDs
        const char* text;  // String output
        std::vector<Point> points; //For Kinematic Commands
        std::vector<AngleSet> angleSet; //For only angle commands
        
    } asr_lut_t;

        static asr_lut_t asr_lut[ASR_NUMBER_OF_COMMANDS] = {
        {1, "Do you believe in the afterlife?", {}, 
        {{{0}, {0}, {0}, {45}, {70}}, {{0}, {0}, {0}, {-45}, {70}}, {{0}, {0}, {0}, {45}, {70}}, {{0}, {0}, {0}, {-45}, {70}}, {{0}, {0}, {0}, {0}, {0}}} //Angle set
        }, //Answers no

        {2, "Do you like Mason more than Derek?", {},
        // {{{0}, {0}, {0}, {45}, {70}}, {{0}, {0}, {0}, {-45}, {70}}, {{0}, {0}, {0}, {45}, {70}}, {{0}, {0}, {0}, {-45}, {70}}, {{0}, {0}, {0}, {0}, {0}}} //Angle set
        {{{0}, {0}, {15}, {0}, {0}}, {{0}, {0}, {0}, {0}, {0}}}
        },

        {3, "Do you want to get out of here?", {},
        {{{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}, {{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}}
         //Angle set
        }, //Answers Yes
        {4, "Is capstone the best program ever?", {},
        {{{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}, {{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}}

        },
        {5, "Do you believe in life after love?", {},
        {{{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}, {{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}}

        },
        {6, "Does it hurt when we turn you off?", {},
        {{{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}, {{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}}

        },
        {7, "Is this on?", {}, 
        {{{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}, {{0}, {0}, {0}, {0}, {70}}, {{0}, {0}, {0}, {0}, {0}}}
        },
        {8, "You dont need no man!", {},
        {{{30}, {30}, {-15}, {0}, {-30}}, {{-30}, {45}, {-15}, {0}, {-30}}, {{30}, {60}, {-15}, {0}, {-30}}, {{0}, {0}, {0}, {0}, {0}}}
        },
        {9, "Goodbye xbot!", {},
        {{{0}, {0}, {45}, {45}, {-45}}, {{0}, {0}, {45}, {-45}, {-45}}, {{0}, {0}, {45}, {45}, {-45}}, {{0}, {0}, {45}, {-45}, {-45}}, {{0}, {0}, {0}, {0}, {0}}}

        },
        {10, "Farewell my dear xbot!", {},
        {{{45}, {65}, {-30}, {0}, {45}}, {{45}, {65}, {-30}, {45}, {45}}, {{45}, {65}, {-30}, {-45}, {45}}, {{-45}, {65}, {-30}, {0}, {45}}, {{-45}, {65}, {-30}, {45}, {45}}, {{-45}, {65}, {-30}, {-45}, {45}}, {{0}, {0}, {0}, {0}, {0}}}

        },
        {11, "Sign your name", {{0.0762, -0.254, 0.124}, {-0.0254, -0.254, 0.124}, {0.0508, -1.778, 0.124}, {0.0254, -0.2794, 0.124}, {0, -0.1778, 0.124}, {0, 0, 0},},
        {}},

        {12, "What am I?", {{0.0, 0.0, 0.0}}},
        {13, "What grade do we deserve for this project?", {{0.0, 0.0, 0.0}}},
        {14, "Where do gingers go?", {},
        {{{0}, {75}, {45}, {0}, {75}}, {{0}, {70}, {45}, {0}, {75}}, {{0}, {75}, {45}, {0}, {75}}, {{0}, {70}, {45}, {0}, {75}}, {0, 0, 0, 0, 0}}
        },

        {15, "Where do all puppies go?", {{0.0, 0.0, 0.0}}},
        {16, "Hey xbot!", {{0.0, 0.0, 0.0}}}
    };

    //Starts in upright position. 0.811 is the the highest it can reach.

    KinematicPoint currentkp = KinematicPoint(0, 0, 0, 0, 0, 0, 0, 0.811624);
    AngleSet currentAS = {0, 0, 0, 0, 0};

    // port_t test = XS1_PORT_1N;
    // port_enable(test);
    // port_out(test, 0);

    while(1) //Voice Control, Kinematics, Stall checks
    {

        std::vector<Point> points;
        std::vector<AngleSet> angleset;

        bool ifKinematic = false;

        //Wait for voice command
        while(1)
        {

            // bit32 = (port_in(GPIOtop2) >> 4) & 0xC;
            // bit0 = (port_in(GPIObot2) >> 6) & 0x1;
            // bit1 = (port_in(GPIObot2) >> 2) & 0x2;
            // word_id_from_voice = bit32 | bit1 | bit0;

            word_id_from_voice = (port_in(top3) & 0xE) | ((port_in(bot1) >> 2) & 0x1);

            // std::cout << word_id_from_voice << std::endl;

            
            if(word_id_from_voice != 0 && word_id_from_voice != 1)
            {
                const char* text = "";

                for (int i=0; i<ASR_NUMBER_OF_COMMANDS; i++) 
                {
                    if (asr_lut[i].asr_id == word_id_from_voice) 
                    {
                        text = asr_lut[i].text;

                        // std::cout << text << std::endl;

                        points = asr_lut[i].points;
                        angleset = asr_lut[i].angleSet;

                        if(points.size() > 0)
                        {
                            ifKinematic = true;
                        }
                        else
                        {
                            ifKinematic = false;
                        }
                        
                    }
                }
                // std::cout << "Voice command found" << std::endl;
                break; //Start kinematics with defined points array
            }

            delay_milliseconds(100);
        }

        int loopmax;

        if(ifKinematic)
        {
            loopmax = points.size();
        }
        else
        {
            loopmax = angleset.size();
        }

        double SHOULDER_SWIVEL_ANGLE_TO_MOVE,SHOULDER_JOINT_ANGLE_TO_MOVE, ELBOW_JOINT_ANGLE_TO_MOVE,WRIST_SWIVEL_ANGLE_TO_MOVE,WRIST_JOINT_ANGLE_TO_MOVE;

        AngleSet nextAS = {0, 0, 0, 0, 0};
        KinematicPoint nextkp;

        // int loopmax = 1;

        for (int j = 0; j < loopmax; j++) 
        {
            if(ifKinematic)
            {
                nextkp = getSimpleKinematics(points[j].x, points[j].y, points[j].z);

                // //Bad Request Handling
                if(nextkp.getX() == 7 && nextkp.getY() == 7 && nextkp.getZ() == 7)
                {
                    std::cout << "Bad Request" << std::endl;
                    break; //If we get a bad kinematic request, we go back to top of loop and skip execution and await voice commands again
                }
            
                SHOULDER_SWIVEL_ANGLE_TO_MOVE = nextkp.getAngle1() - currentkp.getAngle1();
                SHOULDER_JOINT_ANGLE_TO_MOVE = nextkp.getAngle2() - currentkp.getAngle2();
                ELBOW_JOINT_ANGLE_TO_MOVE = nextkp.getAngle3() - currentkp.getAngle3();
                WRIST_SWIVEL_ANGLE_TO_MOVE = nextkp.getAngle5() - currentkp.getAngle5();
                WRIST_JOINT_ANGLE_TO_MOVE = nextkp.getAngle4() - currentkp.getAngle4();
            }
            else //Angle set
            {
                nextAS = angleset[j];

                SHOULDER_SWIVEL_ANGLE_TO_MOVE = angleset[j].S1 - currentAS.S1;
                SHOULDER_JOINT_ANGLE_TO_MOVE = angleset[j].J1 - currentAS.J1;
                ELBOW_JOINT_ANGLE_TO_MOVE = angleset[j].J2 - currentAS.J2;
                WRIST_SWIVEL_ANGLE_TO_MOVE = angleset[j].S3 - currentAS.S3;
                WRIST_JOINT_ANGLE_TO_MOVE = angleset[j].J3 - currentAS.J3;

                // std::cout << "WS" << WRIST_SWIVEL_ANGLE_TO_MOVE << std::endl;
                // std::cout << "WJ" << WRIST_JOINT_ANGLE_TO_MOVE << std::endl;
            }

            // SHOULDER_JOINT_ANGLE_TO_MOVE = 15;

            //FIX ME AND MAKE SURE DIRECTIONS MATCH
            if(SHOULDER_SWIVEL_ANGLE_TO_MOVE < 0)  //Checking if we need to change direction
                SHOULDER_SWIVEL_DRIVER->shaft(false); //false being left, true being right
            else
                SHOULDER_SWIVEL_DRIVER->shaft(true);

            if(SHOULDER_JOINT_ANGLE_TO_MOVE < 0)
            {
                SHOULDER_JOINT_DRIVER1->shaft(false); //false being left, true being right
                SHOULDER_JOINT_DRIVER2->shaft(true); //false being left, true being right
            }  //Checking if we need to change direction
            else
            {
                SHOULDER_JOINT_DRIVER1->shaft(true);
                SHOULDER_JOINT_DRIVER2->shaft(false);
            }
                
            if(ELBOW_JOINT_ANGLE_TO_MOVE < 0) 
            {
                ELBOW_JOINT_DRIVER->shaft(false); //false being left, true being right
                // std::cout << "Switched direction for elbow" << std::endl;
            } //Checking if we need to change direction
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


            int SHOULDER_SWIVEL_MICRO_STEPS, SHOULDER_JOINT_MICRO_STEPS, ELBOW_JOINT_MICRO_STEPS, WRIST_SWIVEL_MICRO_STEPS, WRIST_JOINT_MICRO_STEPS;    
            int stepResolution = 16; 



            // std::cout << "SS " << SHOULDER_SWIVEL_ANGLE_TO_MOVE << std::endl;
            // std::cout << "SJ " << SHOULDER_JOINT_ANGLE_TO_MOVE << std::endl;
            // std::cout << "ES " << ELBOW_JOINT_ANGLE_TO_MOVE << std::endl;
            // std::cout << "WS " << WRIST_SWIVEL_ANGLE_TO_MOVE << std::endl;
            // std::cout << "WJ " << WRIST_JOINT_ANGLE_TO_MOVE << std::endl;

            // SHOULDER_SWIVEL_ANGLE_TO_MOVE = 0;
            // SHOULDER_JOINT_ANGLE_TO_MOVE = 15;
            // ELBOW_JOINT_ANGLE_TO_MOVE = 0;
            // WRIST_SWIVEL_ANGLE_TO_MOVE = 0;
            // WRIST_JOINT_ANGLE_TO_MOVE = 0;

            //MICRO ONLY
            SHOULDER_SWIVEL_MICRO_STEPS = calculateMicroSteps(abs(SHOULDER_SWIVEL_ANGLE_TO_MOVE), SHOULDER_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
            SHOULDER_JOINT_MICRO_STEPS = calculateMicroSteps(abs(SHOULDER_JOINT_ANGLE_TO_MOVE), SHOULDER_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
            ELBOW_JOINT_MICRO_STEPS = calculateMicroSteps(abs(ELBOW_JOINT_ANGLE_TO_MOVE), ELBOW_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
            WRIST_SWIVEL_MICRO_STEPS = calculateMicroSteps(abs(WRIST_SWIVEL_ANGLE_TO_MOVE), WRIST_SWIVEL_ANGLE_TO_STEP_COEFFICIENT, stepResolution);
            WRIST_JOINT_MICRO_STEPS = calculateMicroSteps(abs(WRIST_JOINT_ANGLE_TO_MOVE), WRIST_JOINT_ANGLE_TO_STEP_COEFFICIENT, stepResolution);

            // std::cout << "ELBOW JOINT Full " << ELBOW_JOINT_MICRO_STEPS << std::endl;
            // std::cout << "Shoulder Swivel Full " << SHOULDER_SWIVEL_MICRO_STEPS << std::endl;

            // std::cout << "Beginning Full Steps" << std::endl;


            // chan_out_word(c, SHOULDER_SWIVEL_FULL_STEPS);
            // chan_out_word(c, SHOULDER_JOINT_FULL_STEPS);
            // chan_out_word(c, ELBOW_JOINT_FULL_STEPS);
            // chan_out_word(c, WRIST_SWIVEL_FULL_STEPS);
            // chan_out_word(c, WRIST_JOINT_FULL_STEPS);

            chan_out_word(c, SHOULDER_SWIVEL_MICRO_STEPS);
            chan_out_word(c, SHOULDER_JOINT_MICRO_STEPS);
            chan_out_word(c, ELBOW_JOINT_MICRO_STEPS);
            chan_out_word(c, WRIST_SWIVEL_MICRO_STEPS);
            chan_out_word(c, WRIST_JOINT_MICRO_STEPS);

            int fromTile1;

            while(1) //Step Gen
            {       

                // std::cout << "sg_result " << SHOULDER_SWIVEL_DRIVER->sg_result() << std::endl;
                // std::cout << "stall flag " << SHOULDER_SWIVEL_DRIVER->stallguard() << std::endl;

                // if(SHOULDER_SWIVEL_DRIVER->stallguard())
                // {
                //     stallDetected_T0 = true;
                // } 

                // std::cout << SHOULDER_SWIVEL_DRIVER->DRV_STATUS() << std::endl;

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

                // std::cout << "On tile 0, received from tile 1 "<<std::hex << fromTile1 << std::endl;

                if(fromTile1 == DONE_STATUS_T0)
                {   
                    // printf("All waves finished\n");
                    // return;

                    break;
                }
                
            } //Step Generation loop

            if(ifKinematic)
            {
                currentkp = nextkp; //We've moved to the new position. Update our currentkp with the nextkp

            }
            else
            {
                currentAS = nextAS;
            }

        } //Point by point for loop

    }// Infinite loop for everything after setup

}//Main