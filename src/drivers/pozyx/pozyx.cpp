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

//needed to run daemon
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL
};

int pozyx_pub_main(int argc, char *argv[]);
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
	} bus_options[] = {
		{ POZYX_BUS_I2C_EXTERNAL, "/dev/pozyx_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
		{ POZYX_BUS_I2C_INTERNAL, "/dev/pozyx_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
	};
		
	//#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))
	const int NUM_BUS_OPTIONS = sizeof(bus_options)/sizeof(bus_options[0]);

	void 	start(enum POZYX_BUS busid);
	bool 	start_bus(struct pozyx_bus_option &bus);
	struct 	pozyx_bus_option &find_bus(enum POZYX_BUS busid);
	void 	test(enum POZYX_BUS busid);
	void 	config(enum POZYX_BUS busid);
	void	reset(enum POZYX_BUS busid);
	void	getposition(enum POZYX_BUS busid, bool print_result);
	int 	calibrate(enum POZYX_BUS busid);
	void	usage();


	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus)
	{
		if (bus.dev != nullptr) {
			errx(1, "bus option already started");
		}

		device::Device *interface = bus.interface_constructor(bus.busnum, bus.devpath);


		if (interface->init() != OK) {
			delete interface;
			warnx("no device on bus %u", (unsigned)bus.busid);
			return false;
		}

		bus.dev = new PozyxClass(interface);
		usleep(100000);

		/*
		if (bus.dev != nullptr && POZYX_SUCCESS != bus.dev->begin(true)){//, MODE_POLLING, POZYX_INT_MASK_ALL, POZYX_INT_PIN0)) {
			delete bus.dev;
			bus.dev = NULL;
			return false;
		}
		*/

		int fd = px4_open(bus.devpath, O_RDONLY);
		sleep(0.1);

		if (fd < 0) {
			return false;
		}

		/*
		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			close(fd);
			errx(1, "failed to setup poll rate");
		}
		*/

		close(fd);

		return true;
	}

	//start the driver
	void
	start(enum POZYX_BUS busid)
	{
		bool started = false;
		unsigned i = 0;
		for (i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
				//this device is already started
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
				//not the one that is asked for
				continue;
			}


			started |= start_bus(bus_options[i]);
		}
		if (!started) {
			//exit(1);
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
		
		const char *path = bus.devpath;


		int fd = px4_open(path, O_RDONLY);

		if (fd < 0) {
			err(1, "%s open failed (try 'pozyx start')", path);			
		}
		
		uint8_t whoami = 0;
		testread = bus.dev->regRead(POZYX_WHO_AM_I, &whoami, 1);
		PX4_INFO("value of whoami is: 0x%x", whoami);

		if (testread != POZYX_SUCCESS) {
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
	getposition(enum POZYX_BUS busid, bool print_result)
	{
		struct pozyx_bus_option &bus = find_bus(busid);

		coordinates_t coordinates;

		if (POZYX_SUCCESS == bus.dev->doPositioning(&coordinates, POZYX_3D)){
			if (print_result) {
				PX4_INFO("Current position: %d   %d   %d", coordinates.x, coordinates.y, coordinates.z);
			}
			
			struct att_pos_mocap_s pos;
			//memset(&pos, 0, sizeof(pos));
			pos.x = coordinates.x;
			pos.y = coordinates.y;
			pos.z = coordinates.z;
			orb_advert_t pos_pub = orb_advertise(ORB_ID(att_pos_mocap), &pos);

			orb_publish(ORB_ID(att_pos_mocap), pos_pub, &pos);
		}

	}

	void
	config(enum POZYX_BUS busid)
	{
		struct pozyx_bus_option &bus = find_bus(busid);

		/*
		UWB_settings_t tagconfig;
		tagconfig.channel = 5;
		tagconfig.bitrate = 0;
		tagconfig.prf = 2;
		tagconfig.plen = 0x08;
		tagconfig.gain_db = 23;
		if (bus.dev->setUWBChannel(tagconfig.channel) == POZYX_SUCCESS){
			if (bus.dev->setUWBSettings(&tagconfig) == POZYX_SUCCESS){
				PX4_INFO("UWB settings configured successfully");
			}
		}
		*/
		/*
		if (bus.dev->setUWBChannel(tagconfig.channel, 0x684E) == POZYX_SUCCESS){
			if (bus.dev->setUWBSettings(&tagconfig, 0x684E) == POZYX_SUCCESS){
				PX4_INFO("UWB settings configured successfully");
			}
		}
		//configure the anchors to match
		if (bus.dev->setUWBChannel(tagconfig.channel, 0x682E) == POZYX_SUCCESS){
			if (bus.dev->setUWBSettings(&tagconfig, 0x682E) == POZYX_SUCCESS){
				PX4_INFO("UWB settings configured successfully");
			}
		}
		if (bus.dev->setUWBChannel(tagconfig.channel, 0x6853) == POZYX_SUCCESS){
			if (bus.dev->setUWBSettings(&tagconfig, 0x6853) == POZYX_SUCCESS){
				PX4_INFO("UWB settings configured successfully");
			}
		}
		if (bus.dev->setUWBChannel(tagconfig.channel, 0x6852) == POZYX_SUCCESS){
			if (bus.dev->setUWBSettings(&tagconfig, 0x6852) == POZYX_SUCCESS){
				PX4_INFO("UWB settings configured successfully");
			}
		}
		*/

		uint8_t num_anchors =4;
		device_coordinates_t anchorlist[num_anchors] = {
			{0x684E, 1, {0, 962, 1247}},
			{0x682E, 1, {0, 4293, 2087}},
			{0x6853, 1, {6746, 4888, 1559}},
			{0x6852, 1, {4689, 0, 2491}}
		};
		if (bus.dev->clearDevices() == POZYX_SUCCESS){
			for (int i = 0; i < num_anchors; i++) {
				if (bus.dev->addDevice(anchorlist[i]) != POZYX_SUCCESS) {
					PX4_INFO("failed to add anchor");
					exit(1);
				}
				PX4_INFO("Anchor 0x%x successfully added at (%d, %d, %d)", anchorlist[i].network_id, anchorlist[i].pos.x, anchorlist[i].pos.y, anchorlist[i].pos.z);
			}
			if (bus.dev->getDeviceListSize(&num_anchors) == POZYX_SUCCESS) {
				PX4_INFO("%d anchors configured", num_anchors);
			}
			if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
				PX4_INFO("%d anchors saved", num_anchors);
			}
		}
		//exit(0);
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
		pozyx::usage();
		exit(0);
	}

	const char *verb = argv[optind];

	//start/load driver and begin cycles
	if (!strcmp(verb, "start")) {
		pozyx::start(busid);
		pozyx::config(busid);

		if (thread_running) {
			warnx("pozyx already running\n");
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("pozyx_pub", 
											SCHED_DEFAULT, 
											SCHED_PRIORITY_DEFAULT, 
											2000, 
											pozyx_pub_main,
											(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	//stop the daemon
	if (!strcmp(verb, "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(verb, "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	//test driver/device
	if (!strcmp(verb, "test")) {
		pozyx::test(busid);
		exit(0);
	}
		//configure pozyx
	if (!strcmp(verb, "config")) {
		pozyx::config(busid);
		exit(0);
	}

	//fetch positions
	if (!strcmp(verb, "getposition")) {
		pozyx::getposition(busid, true);
		exit(0);
	}

		//debug
	if (!strcmp(verb, "debug")) {
		PX4_INFO("bus options[0]: %d, %s, %d.... size: %d", pozyx::bus_options[0].busid, pozyx::bus_options[0].devpath, pozyx::bus_options[0].busnum, sizeof(pozyx::bus_options));
		PX4_INFO("num bus options: %d", pozyx::NUM_BUS_OPTIONS);
		exit(0);
	}


	errx(1, "Unrecognized command. Try start, stop, status, test, config, getposition");
}


int 
pozyx_pub_main(int argc, char *argv[])
{
	warnx("[pozyx_pub] starting\n");
	thread_running = true;

	while (!thread_should_exit) {
		pozyx::getposition(POZYX_BUS_ALL, false);
		usleep(300000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}