/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx.cpp
 *
 * I2C interface for Pozyx Tag
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>
#include <uORB/topics/att_pos_mocap.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include "pozyx.h"
#include "pozyx_i2c.cpp"

//#include <iostream>
//using namespace std;






/****************namespace**********************************************/
namespace pozyx
{
	#ifdef ERROR
	# undef ERROR
	#endif
	const int ERROR = -1;

	enum POZYX_BUS {
		POZYX_BUS_ALL = 0,
		POZYX_BUS_I2C_INTERNAL,
		POZYX_BUS_I2C_EXTERNAL
	};

	//list of supported bus configurations
	struct pozyx_bus_option {
		enum POZYX_BUS busid;
		const char *devpath;
		POZYX_constructor interface_constructor;
		uint8_t busnum;
		PozyxClass *dev;
	} bus_options[2] = {
		{ POZYX_BUS_I2C_EXTERNAL, "/dev/pozyx_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
		{ POZYX_BUS_I2C_INTERNAL, "/dev/pozyx_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
	};


		
	#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

	void 	start(enum POZYX_BUS busid);
	bool 	start_bus(struct pozyx_bus_option &bus);
	struct 	pozyx_bus_option &find_bus(enum POZYX_BUS busid);
	void 	test(enum POZYX_BUS busid);
	void	reset(enum POZYX_BUS busid);
	void	getposition(enum POZYX_BUS busid);
	int 	info(enum POZYX_BUS busid, bool enable);
	int 	calibrate(enum POZYX_BUS busid);
	void	usage();

	int		begin(bool print_result = false, int mode = MODE_POLLING, int interrupts = POZYX_INT_MASK_ALL);

	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus)
	{
		if (bus.dev != nullptr) {
			errx(1, "bus option already started");
		}

		device::Device *interface = bus.interface_constructor(bus.busnum);

		if (interface->init() != OK) {
			delete interface;
			warnx("no device on bus %u", (unsigned)bus.busid);
			return false;
		}

		bus.dev = new PozyxClass();

		if (bus.dev != nullptr && OK != bus.dev->init()) {
			delete bus.dev;
			bus.dev = NULL;
			return false;
		}

		int fd = open(bus.devpath, O_RDONLY);

		if (fd < 0) {
			return false;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			close(fd);
			errx(1, "failed to setup poll rate");
		}

		close(fd);

		return true;
	}

	//start the driver
	void
	start(enum POZYX_BUS busid)
	{
		bool started = false;

		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
				continue;
			}

			started |= start_bus(bus_options[i]);
		}
		if (!started) {
			exit(1);
		}
	}

	//find bus structure for a busid
	struct pozyx_bus_option &find_bus(enum POZYX_BUS busid)
	{
		for (unsigned i=0; i < NUM_BUS_OPTIONS; i++) {
			if ((busid == POZYX_BUS_ALL ||
				busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
				return bus_options[i];
			}
		}
		errx(1, "bus %u not started", (unsigned)busid);
	}

	int 
	begin(bool print_result, int mode, int interrupts)
	{
		  start(POZYX_BUS_ALL);	

		  int status = POZYX_SUCCESS;
		  struct pozyx_bus_option &bus = find_bus(POZYX_BUS_ALL);

		  if(print_result){
		    PX4_INFO("Pozyx Shield");
		    PX4_INFO("------------");
		  }

		  // check if the mode parameter is valid
		  if((mode != MODE_POLLING) && (mode != MODE_INTERRUPT)) {
		    return POZYX_FAILURE;
		  }
		  


		  // wait a bit until the pozyx board is up and running
		  sleep(0.250);

		  _mode = mode;
		  		  
		  uint8_t whoami, selftest;  
		  uint8_t regs[3];
		  regs[2] = 0x12;
		  // we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
		  if(bus.dev->regRead(POZYX_WHO_AM_I, regs, 3) != OK){
		    return POZYX_FAILURE;
		  }  
		  whoami = regs[0];
		  _fw_version = regs[1];
		  _hw_version = regs[2]; 

		  if(print_result){
		    PX4_INFO("WhoAmI: 0x%x",whoami);
		    PX4_INFO("FW ver.: v%d.%d",((_fw_version&0xF0)>>4),(_fw_version&0x0F));
		    if(fw_version < 0x10) {
		      PX4_INFO("please upgrade");
		    }
		    PX4_INFO("HW ver.: v%d.%d", ((_hw_version&0xE0)>>5),(_hw_version&0x1F));
		  }
		  // verify if the whoami is correct
		  if(whoami != 0x43) {    
		    // possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
		    status = POZYX_FAILURE;
		  }
		  
		  // readout the selftest registers to validate the proper functioning of pozyx
		  if(bus.dev->regRead(POZYX_ST_RESULT, &selftest, 1) != OK){
		    return POZYX_FAILURE;
		  } 

		  if(print_result){
		    PX4_INFO("Self Test: 0x%x", selftest);
	       	if ((selftest & POZYX_ST_RESULT_ACC) != POZYX_ST_RESULT_ACC){
	    		PX4_INFO("Self Test: Accelerometer failed");
	    	}    
	    	if ((selftest & POZYX_ST_RESULT_MAGN) != POZYX_ST_RESULT_MAGN){
	    		PX4_INFO("Self Test: Magnetometer failed");
	    	}    
	    	if ((selftest & POZYX_ST_RESULT_GYR) != POZYX_ST_RESULT_GYR){
	    		PX4_INFO("Self Test: Gyroscope failed");
	    	}    
	    	if ((selftest & POZYX_ST_RESULT_MCU) != POZYX_ST_RESULT_MCU){
	    		PX4_INFO("Self Test: IMU Microcontroller failed");
	    	}    
	    	if ((selftest & POZYX_ST_RESULT_PRES) != POZYX_ST_RESULT_PRES){
	    		PX4_INFO("Self Test: Pressure Sensor failed");
	    	}   
	    	if ((selftest & POZYX_ST_RESULT_UWB) != POZYX_ST_RESULT_UWB){
	    		PX4_INFO("Self Test: USB Transciever failed");
	    	} 
		  }

		  if((_hw_version & POZYX_TYPE) == POZYX_TAG)
		  {
		    // check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
		    if(selftest != 0b00111111) {
 		      status = POZYX_FAILURE;
		    }
		  }else if((_hw_version & POZYX_TYPE) == POZYX_ANCHOR)
		  {
		    // check if the uwb transceiver and pressure sensor are working
		    if(selftest != 0b0011000) {    
		      status = POZYX_FAILURE;
		    }
		    return status;
		  }

		if(mode == MODE_INTERRUPT){
			/*
		    // set the function that must be called upon an interrupt
		    // put your main code here, to run repeatedly:
			#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
			    // Arduino Tian
			    int tian_interrupt_pin = interrupt_pin;
			    attachInterrupt(interrupt_pin+2, IRQ, RISING);
			#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			    // Arduino UNO, Mega
			    attachInterrupt(interrupt_pin, IRQ, RISING);
			#else
			    PX4_INFO("This is not a board supported by Pozyx, interrupts may not work");
			    attachInterrupt(interrupt_pin, IRQ, RISING);
			#endif
			    // use interrupt as provided and initiate the interrupt mask
			    uint8_t int_mask = interrupts;
			    configInterruptPin(5+interrupt_pin, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 0);

			    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE){
			      return POZYX_FAILURE;
			    }
		 	*/
		  }   
		  
		  // all done
		  sleep(POZYX_DELAY_LOCAL_WRITE/1000);
		  return status;
	}

	//basic functional tests
	void
	test(enum POZYX_BUS busid)
	{
		struct pozyx_bus_option &bus = find_bus(busid);
		int testread;
		//int ret;
		const char *path = bus.devpath;

		int fd = open(path, O_RDONLY);

		if (fd < 0) {
			err(1, "%s open failed (try 'pozyx start')", path);			
		}
		uint8_t whoami = 0;
		testread = bus.dev->regRead(POZYX_WHO_AM_I, &whoami, 1);
		PX4_INFO("value of whoami is: 0x%x", whoami);

		if (testread != OK) {
			err(1, "immediate read failed");
		}
		else {
			if (whoami == POZYX_WHOAMI_EXPECTED){
				PX4_INFO("Who Am I Check Successful");
			}
			else{
				PX4_INFO("Who Am I Check Failed: 0x%x",whoami);
			}
		}
		//test a function call by blinking LED3
		uint8_t funcbuf[100];
		funcbuf[0] = 0x44;

		bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
		if (funcbuf[0] != 1) {
			err(1, "Function test failed");
		}
		PX4_INFO("LED3 turned On... ");
		sleep(2);
		funcbuf[0] = 0x40;
		bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
		if (funcbuf[0] != 1) {
			err(1, "Function test failed");
		}
		PX4_INFO("LED3 turned Off");

		errx(0, "PASS");
	}



	void
	getposition(enum POZYX_BUS busid)
	{
		struct pozyx_bus_option &bus = find_bus(busid);
		uint32_t coordinates[3];


		bus.dev->regRead(POZYX_POS_X, (uint8_t *)&coordinates[0], 4);
		bus.dev->regRead(POZYX_POS_Y, (uint8_t *)&coordinates[1], 4);
		bus.dev->regRead(POZYX_POS_Z, (uint8_t *)&coordinates[2], 4);

		PX4_INFO("Current position: %d   %d   %d", coordinates[0], coordinates[1], coordinates[2]);
		
		struct att_pos_mocap_s att;
		memset(&att, 0, sizeof(att));
		orb_advert_t att_pub = orb_advertise(ORB_ID(att_pos_mocap), &att);
		att.x = coordinates[0];
		att.y = coordinates[1];
		att.z = coordinates[2];

		orb_publish(ORB_ID(att_pos_mocap), att_pub, &att);
		
	}


	void
	reset(enum POZYX_BUS busid)
	{
		exit(0);
	}

	/*intinfo(enum POZYX_BUS busid)
	{
		struct pozyx_bus_option &bus = find_bus(busid);

		warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
		bus.dev->print_info();
		exit(0);
	}
	*/

	void
	usage()
	{
		PX4_INFO("Give description of how to use pozyx from cmd line");
	}

} //namespace

int
pozyx_main(int argc, char *argv[])
{
	
	int ch;
	enum POZYX_BUS busid = POZYX_BUS_ALL;

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
			case 'R':
			//rotation = (enum Rotation)atoi(optarg);
			default:
			pozyx::usage();
			exit(0);
		}
	}
	

	const char *verb = argv[optind];

	//start/load driver
	if (!strcmp(verb, "start")) {
		pozyx::start(busid);
		PX4_INFO("pozyx started");

		exit(0);
	}

	//test driver/device
	if (!strcmp(verb, "test")) {
		PX4_INFO("testing pozyx...");
		pozyx::test(busid);
		exit(0);
	}


	//fetch positions
	if (!strcmp(verb, "getposition")) {
		pozyx::getposition(busid);
		exit(0);
	}

	//pozyx begin
	if (!strcmp(verb, "begin")) {
		pozyx::begin(true, MODE_POLLING, POZYX_INT_MASK_ALL);
		exit(0);
	}


	errx(1, "unrecognized command, try start, test, getposition, begin");

}