#include "TMCStepper.h"
#include "TMC_MACROS.h"
#include <cstdint>
#include <iostream>


// float RS = 0.075;

//NEED TO INITIALIZE XMOS SPECIFIC CONSTANTS DOWN HERE


// int8_t TMC2130Stepper::chain_length = 0;
// uint32_t TMC2130Stepper::spi_speed = 16000000/8;



// TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS) :
//   TMCStepper(RS),
//   _pinCS(pinCS),
//   {
//     defaults();

    
//   }

//*dev represents the device, *spi is the instance already setup for SPI. CS pin represents the identity of the device


TMC2130Stepper::TMC2130Stepper(float Rsense, spi_master_device_t *dev,
        spi_master_t *spi,
        uint32_t cs_pin,
        int cpol,
        int cpha,
        spi_master_source_clock_t source_clock,
        uint32_t clock_divisor,
        spi_master_sample_delay_t miso_sample_delay,
        uint32_t miso_pad_delay,
        uint32_t cs_to_clk_delay_ticks,
        uint32_t clk_to_cs_delay_ticks,
        uint32_t cs_to_cs_delay_ticks) : TMCStepper(Rsense), dev(dev), spi(spi), cs_pin(cs_pin)
 
  {
    spi_master_device_init(dev,
        spi,
        cs_pin,
        cpol,
        cpha,
        source_clock,
        clock_divisor,
        miso_sample_delay,
        miso_pad_delay,
        cs_to_clk_delay_ticks,
        clk_to_cs_delay_ticks,
        cs_to_cs_delay_ticks);

        float RS = Rsense;

    
  }

// TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link) :
//   TMCStepper(RS),
//   _pinCS(pinCS),
//   link_index(link)
//   {
//     port_t p_miso = XS1_PORT_1A;
//     port_t p_ss[1] = {XS1_PORT_1B};
//     port_t p_sclk = XS1_PORT_1C;
//     port_t p_mosi = XS1_PORT_1D;
//     xclock_t cb = XS1_CLKBLK_1;


//     // SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
//     spi_master_t spi_ctx;

//     spi_master_init(&spi_ctx, cb, p_ss[0], p_sclk, pinMOSI, pinMISO);
    
//     TMC_SW_SPI = SW_SPI_Obj;
//     defaults();

//     if (link > chain_length)
//       chain_length = link;
//   }

void TMC2130Stepper::defaults() {
  //MSLUT0_register.sr = ???;
  //MSLUT1_register.sr = ???;
  //MSLUT2_register.sr = ???;
  //MSLUT3_register.sr = ???;
  //MSLUT4_register.sr = ???;
  //MSLUT5_register.sr = ???;
  //MSLUT6_register.sr = ???;
  //MSLUT7_register.sr = ???;
  //MSLUTSTART_register.start_sin90 = 247;
  PWMCONF_register.sr = 0x00050480;
}

// __attribute__((weak))
// void TMC2130Stepper::setSPISpeed(uint32_t speed) {
//   spi_speed = speed;
// }

// __attribute__((weak))
//   

// __attribute__((weak))
void TMC2130Stepper::beginTransaction() {
  spi_master_start_transaction(dev);
}

// __attribute__((weak))
void TMC2130Stepper::endTransaction() {
  spi_master_end_transaction(dev);
}


// uint8_t TMC2130Stepper::transfer(const uint8_t data) {
//   uint8_t out = 0;
//   if (TMC_SW_SPI != nullptr) {
//     out = TMC_SW_SPI->transfer(data);
//   }
//   else {
//     out = SPI.transfer(data);
//   }
//   return out;
// }

uint8_t TMC2130Stepper::transfer(uint8_t data) {
  uint8_t receivedData = 0; // To hold the data received back from the SPI device
  spi_master_transfer(dev, &data, &receivedData, 1); // Pass the address of `receivedData` for `data_in`
    return receivedData; // Return the data received during the SPI transfer
}

void TMC2130Stepper::transferEmptyBytes(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    transfer(0x00);
  }
}


uint32_t TMC2130Stepper::read(uint8_t addressByte) {
  uint32_t out = 0UL;
  int8_t i = 1;

  beginTransaction();

  transfer(addressByte);
  // Clear SPI
  transferEmptyBytes(4);

  //UNEEDED WE ARE NOT DOING CHAIN
  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  // while(i < link_index) {
  //   transferEmptyBytes(5);
  //   i++;
  // }

  endTransaction();
  beginTransaction();

  // switchCSpin(HIGH);
  // switchCSpin(LOW);

  // Shift data from target link into the last one...
  // while(i < chain_length) {
  //   transferEmptyBytes(5);
  //   i++;
  // }

  // ...and once more to MCU
  // status_response = transfer(addressByte); // Send the address byte again
  // out  = transfer(0x00);
  // out <<= 8;
  // out |= transfer(0x00);
  // out <<= 8;
  // out |= transfer(0x00);
  // out <<= 8;
  // out |= transfer(0x00);

  status_response = transfer(addressByte);
   // Send the address byte again
  out  = transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);

  endTransaction();
  return out;
}

void TMC2130Stepper::write(uint8_t addressByte, uint32_t config) {
  addressByte |= TMC_WRITE;
  int8_t i = 1;

  beginTransaction();
  status_response = transfer(addressByte);
  transfer(config>>24);
  transfer(config>>16);
  transfer(config>>8);
  transfer(config);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  // while(i < link_index) {
  //   transferEmptyBytes(5);
  //   i++;
  // }

  endTransaction();
  
}

void TMC2130Stepper::begin() {

  // if (TMC_SW_SPI != nullptr) TMC_SW_SPI->init();

  GCONF(GCONF_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

/**
 *  Helper functions
 */

bool TMC2130Stepper::isEnabled() { return !drv_enn_cfg6() && toff(); }

void TMC2130Stepper::push() {
  GCONF(GCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);
  TPOWERDOWN(TPOWERDOWN_register.sr);
  TPWMTHRS(TPWMTHRS_register.sr);
  TCOOLTHRS(TCOOLTHRS_register.sr);
  THIGH(THIGH_register.sr);
  XDIRECT(XDIRECT_register.sr);
  VDCMIN(VDCMIN_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  DCCTRL(DCCTRL_register.sr);
  PWMCONF(PWMCONF_register.sr);
  ENCM_CTRL(ENCM_CTRL_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t  TMC2130Stepper::IOIN()    { return read(IOIN_t::address); }
bool TMC2130Stepper::step()         { IOIN_t r{0}; r.sr = IOIN(); return r.step; }
bool TMC2130Stepper::dir()          { IOIN_t r{0}; r.sr = IOIN(); return r.dir; }
bool TMC2130Stepper::dcen_cfg4()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcen_cfg4; }
bool TMC2130Stepper::dcin_cfg5()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcin_cfg5; }
bool TMC2130Stepper::drv_enn_cfg6() { IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn_cfg6; }
bool TMC2130Stepper::dco()          { IOIN_t r{0}; r.sr = IOIN(); return r.dco; }
uint8_t TMC2130Stepper::version()   { IOIN_t r{0}; r.sr = IOIN(); return r.version; }
///////////////////////////////////////////////////////////////////////////////////////
// W: TCOOLTHRS
uint32_t TMC2130Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2130Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC2130Stepper::THIGH() { return THIGH_register.sr; }
void TMC2130Stepper::THIGH(uint32_t input) {
  THIGH_register.sr = input;
  write(THIGH_register.address, THIGH_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: XDIRECT
uint32_t TMC2130Stepper::XDIRECT() {
  return read(XDIRECT_register.address);
}
void TMC2130Stepper::XDIRECT(uint32_t input) {
  XDIRECT_register.sr = input;
  write(XDIRECT_register.address, XDIRECT_register.sr);
}
void TMC2130Stepper::coil_A(int16_t B)  { XDIRECT_register.coil_A = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
void TMC2130Stepper::coil_B(int16_t B)  { XDIRECT_register.coil_B = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
int16_t TMC2130Stepper::coil_A()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_A; }
int16_t TMC2130Stepper::coil_B()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_B; }
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC2130Stepper::VDCMIN() { return VDCMIN_register.sr; }
void TMC2130Stepper::VDCMIN(uint32_t input) {
  VDCMIN_register.sr = input;
  write(VDCMIN_register.address, VDCMIN_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: DCCTRL
void TMC2130Stepper::DCCTRL(uint32_t input) {
	DCCTRL_register.sr = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_time(uint16_t input) {
	DCCTRL_register.dc_time = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_sg(uint8_t input) {
	DCCTRL_register.dc_sg = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

uint32_t TMC2130Stepper::DCCTRL() {
	return read(DCCTRL_register.address);
}
uint16_t TMC2130Stepper::dc_time() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_time;
}
uint8_t TMC2130Stepper::dc_sg() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_sg;
}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint8_t TMC2130Stepper::PWM_SCALE() { return read(PWM_SCALE_t::address); }
///////////////////////////////////////////////////////////////////////////////////////
// W: ENCM_CTRL
uint8_t TMC2130Stepper::ENCM_CTRL() { return ENCM_CTRL_register.sr; }
void TMC2130Stepper::ENCM_CTRL(uint8_t input) {
  ENCM_CTRL_register.sr = input;
  write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr);
}
void TMC2130Stepper::inv(bool B)      { ENCM_CTRL_register.inv = B;       write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
void TMC2130Stepper::maxspeed(bool B) { ENCM_CTRL_register.maxspeed  = B; write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
bool TMC2130Stepper::inv()            { return ENCM_CTRL_register.inv; }
bool TMC2130Stepper::maxspeed()       { return ENCM_CTRL_register.maxspeed; }
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC2130Stepper::LOST_STEPS() { return read(LOST_STEPS_t::address); }

void TMC2130Stepper::sg_current_decrease(uint8_t value) {
  switch(value) {
    case 32: sedn(0b00); break;
    case  8: sedn(0b01); break;
    case  2: sedn(0b10); break;
    case  1: sedn(0b11); break;
  }
}
uint8_t TMC2130Stepper::sg_current_decrease() {
  switch(sedn()) {
    case 0b00: return 32;
    case 0b01: return  8;
    case 0b10: return  2;
    case 0b11: return  1;
  }
  return 0;
}
