

# main.xe: main.o configs.o spi.o steppers.o Stepper.o 
# 	xcc -w -std=c++11 main.o configs.o spi.o steppers.o Stepper.o -o main.xe

# main.o: main.xc
# 	xcc -w -std=c++11 -c main.xc -O2 -target=XU316-1024-QF60A-C24 -o main.o

# configs.o: CHOPCONF.cpp COOLCONF.cpp DRV_CONF.cpp DRV_STATUS.cpp DRVCONF.cpp DRVCTRL.cpp DRVSTATUS.cpp ENCMODE.cpp GCONF.cpp IHOLD_IRUN.cpp PWMCONF.cpp RAMP_STAT.cpp SGCSCONF.cpp SHORT_CONF.cpp SMARTEN.cpp
# 	xcc -w -std=c++11 -c $^  -O2 -target=XU316-1024-QF60A-C24 -o configs.o

# spi.o: spi_master.c spi.h
# 	xcc -w spi_master.c -O2 -target=XU316-1024-QF60A-C24 -o spi.o

# steppers.o: SW_MODE.cpp TMC_MACROS.h TMC2130_bitfields.h TMC2130Stepper.cpp TMC2160_bitfields.h TMC2160Stepper.cpp TMC5130_bitfields.h TMC5130Stepper.cpp TMC5160_bitfields.h TMC5160Stepper.cpp
# 	xcc -w -std=c++11 -c SW_MODE.cpp TMC2130Stepper.cpp TMC2160Stepper.cpp TMC5130Stepper.cpp TMC5160Stepper.cpp -O2 -target=XU316-1024-QF60A-C24 -o steppers.o

# Stepper.o: TMCStepper.cpp TMCStepper.h
# 	xcc -w -std=c++11 -c TMCStepper.cpp -O2 -target=XU316-1024-QF60A-C24 -o Stepper.o

# main.xe: main.o TMCStepper.o
# 	xcc -w -std=c++11 main.cpp TMCStepper.o -O2 -target=XU316-1024-QF60A-C24 -o main.xe

# TO DO: FIGURE OUT HOW TO BUILD WITH TWO C FILES (SPI_SLAVE)

spi.o:
	xcc spi_master.c -c -O2 XK_VOICE_L71.xn -w -o spi.o

# spi.o: 
# 	gcc -c spi_master.c -o spi.o

# SW_MODE.o:
# 	xcc -w -std=c++11 -c SW_MODE.cpp -O2 -target=XU316-1024-QF60A-C24 -o SW_MODE.o

main.xe: spi.o
	xcc -g -w -std=c++14 spi.o *.cpp XK_VOICE_L71.xn -O2 -o main.xe

TMCStepper.o: spi.o TMCStepper.h TMC_MACROS.h TMC2130_bitfields.h TMC2160_bitfields.h TMC5130_bitfields.h TMC5160_bitfields.h
	xcc -g -w -std=c++14 -c CHOPCONF.cpp COOLCONF.cpp DRV_CONF.cpp DRV_STATUS.cpp DRVCONF.cpp DRVCTRL.cpp DRVSTATUS.cpp ENCMODE.cpp GCONF.cpp IHOLD_IRUN.cpp PWMCONF.cpp RAMP_STAT.cpp SGCSCONF.cpp SHORT_CONF.cpp SMARTEN.cpp TMCStepper.cpp TMC2130Stepper.cpp TMC2160Stepper.cpp TMC5130Stepper.cpp TMC5160Stepper.cpp SW_MODE.cpp spi.o -O2 -target=XU316-1024-QF60A-C24 -o TMCStepper.o
