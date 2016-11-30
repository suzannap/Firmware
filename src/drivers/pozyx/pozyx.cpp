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
/*
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
//#include "pozyx_i2c.cpp"

//#include <iostream>
//using namespace std;
*/

#include <uORB/uORB.h>
#include <uORB/topics/att_pos_mocap.h>
#include "pozyx.h"



enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL
};

extern "C" __EXPORT int pozyx_main(int argc, char *argv[]);
/****************namespace**********************************************/
namespace pozyx
{

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


	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus)
	{
		PX4_INFO("debug 6");
		if (bus.dev != nullptr) {
		PX4_INFO("debug 7");
			errx(1, "bus option already started");
		}

		device::Device *interface = bus.interface_constructor(bus.busnum);

		if (interface->init() != OK) {
		PX4_INFO("debug 8");
			delete interface;
			warnx("no device on bus %u", (unsigned)bus.busid);
			return false;
		}

		bus.dev = new PozyxClass(bus.busid);

		if (bus.dev != nullptr && OK != bus.dev->begin()) {
		PX4_INFO("debug 9");
			delete bus.dev;
			bus.dev = NULL;
			return false;
		}

		int fd = open(bus.devpath, O_RDONLY);

		if (fd < 0) {
		PX4_INFO("debug 10");
			return false;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_INFO("debug 11");
			close(fd);
			errx(1, "failed to setup poll rate");
		}

		PX4_INFO("debug 12");
		close(fd);

		PX4_INFO("debug 13");
		return true;
	}

	//start the driver
	void
	start(enum POZYX_BUS busid)
	{
		bool started = false;
		PX4_INFO("debug 1");

		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
		PX4_INFO("debug 2");
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
		PX4_INFO("debug 3");
				continue;
			}

			started |= start_bus(bus_options[i]);
		PX4_INFO("debug 4");
		}
		if (!started) {
		PX4_INFO("debug 5");
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
	reset(enum POZYX_BUS busid)
	{
		exit(0);
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
		//pozyx::begin(true, MODE_POLLING, POZYX_INT_MASK_ALL);
		exit(0);
	}


	errx(1, "unrecognized command, try start, test, getposition, begin");

}