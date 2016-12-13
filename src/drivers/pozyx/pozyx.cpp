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

#include <uORB/uORB.h>
#include <uORB/topics/att_pos_mocap.h>
#include "pozyx.h"

//needed to run daemon
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int count = 0;

enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL,
	POZYX_BUS_I2C_ALT_INTERNAL,
	POZYX_BUS_I2C_ALT_EXTERNAL
};

int pozyx_pub_main(int argc, char *argv[]);
int pozyx_pub_main_2(int argc, char *argv[]);
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
		uint8_t devaddr;
		PozyxClass *dev;
		uint8_t index;
	} bus_options[] = {
		{ POZYX_BUS_I2C_INTERNAL, "/dev/pozyx_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, POZYX_I2C_ADDRESS, NULL , 0},
		{ POZYX_BUS_I2C_EXTERNAL, "/dev/pozyx_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, POZYX_I2C_ADDRESS, NULL, 1 },
		{ POZYX_BUS_I2C_ALT_INTERNAL, "/dev/pozyx_alt_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, POZYX_I2C_ADDRESS_ALT, NULL, 2 },
		{ POZYX_BUS_I2C_ALT_EXTERNAL, "/dev/pozyx_alt_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, POZYX_I2C_ADDRESS_ALT, NULL, 3 },
	};
		
	const int NUM_BUS_OPTIONS = sizeof(bus_options)/sizeof(bus_options[0]);

	int 	start(enum POZYX_BUS busid);
	bool 	start_bus(struct pozyx_bus_option &bus);
	struct 	pozyx_bus_option &find_bus(enum POZYX_BUS busid, unsigned startid);
	void 	test(enum POZYX_BUS busid, int count);
	void 	config(enum POZYX_BUS busid, int count);
	void	reset(enum POZYX_BUS busid, int count);
	void	getposition(enum POZYX_BUS busid, int count, bool print_result);
	int 	calibrate(enum POZYX_BUS busid);
	void	usage();


	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus)
	{
		if (bus.dev != nullptr) {
			errx(1, "bus option already started");
		}

		device::Device *interface = bus.interface_constructor(bus.busnum, bus.devpath, bus.devaddr);


		if (interface->init() != OK) {
			delete interface;
			warnx("no device on bus %u", (unsigned)bus.busid);
			return false;
		}

		bus.dev = new PozyxClass(interface);
		usleep(100000);


		int fd = px4_open(bus.devpath, O_RDONLY);
		usleep(100000);

		if (fd < 0) {
			return false;
		}

		close(fd);

		return true;
	}

	//start the driver
	int
	start(enum POZYX_BUS busid)
	{
		int started = 0;
		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
				//this device is already started
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
				//not the one that is asked for
				continue;
			}

			if (start_bus(bus_options[i])) {
				started ++;
			}
		}
		PX4_INFO("%d Pozyx tags found", started);
		return started;
	}

	//find bus structure for a busid
	struct pozyx_bus_option &find_bus(enum POZYX_BUS busid, unsigned startid)
	{
		for (unsigned i=startid; i < NUM_BUS_OPTIONS; i++) {
			if (((busid == POZYX_BUS_ALL) || (busid == bus_options[i].busid)) && (bus_options[i].dev != NULL)) {
				return bus_options[i];
			}
		}
		errx(1, "No active bus found");
	}


	//basic functional tests
	void
	test(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;
		int testread;
		

		for (int i=0; i<count; i++){	
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;	

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

			PX4_INFO("Tag %d PASS", bus.index);
		}
	}

	void
	reset(enum POZYX_BUS busid, int count)
	{
		exit(0);
	}

	void
	getposition(enum POZYX_BUS busid, int count, bool print_result)
	{

		coordinates_t poz_coordinates[count];
		quaternion_t poz_orientation;
		struct att_pos_mocap_s pos;
		unsigned startid = 0;

		pos.x = 0;
		pos.y = 0;
		pos.z = 0;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_3D)){
				if (print_result) {
					PX4_INFO("Current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
				}
			}
			pos.x += poz_coordinates[i].x;
			pos.y += poz_coordinates[i].y;
			pos.z += poz_coordinates[i].z;

			if (count == 1) {
				if (POZYX_SUCCESS == bus.dev->getQuaternion(&poz_orientation)){
					if (print_result) {
						PX4_INFO("Current orientation: %1.4f  %1.4f  %1.4f  %1.4f", (double)poz_orientation.weight, (double)poz_orientation.x, (double)poz_orientation.y, (double)poz_orientation.z);
					}
					//change orientation from NWU to NED rotate 180 degrees about x
					//[q0, q1, q2, q3] * [0, 1, 0, 0] = [-q1, q0, q3, -q2]
					pos.q[0] = -poz_orientation.x;
					pos.q[1] = poz_orientation.weight;
					pos.q[2] = poz_orientation.z;
					pos.q[3] = -poz_orientation.y;		
				}			
			}
		}	
		//change position from NWU to NED and from m to mm
		pos.x /= (count*1000);
		pos.y /= (-count*1000);
		pos.z /= (-count*1000);
		if (count > 1) {
			double yaw = atan ((poz_coordinates[1].y - poz_coordinates[0].y)/(poz_coordinates[1].x - poz_coordinates[0].x));

			if (print_result) {
				PX4_INFO("Current yaw: %f deg.", (yaw * 180 / 3.14159));
			}
			pos.q[0] = cos(yaw/2);
			pos.q[1] = 0;
			pos.q[2] = 0;
			pos.q[3] = sin(yaw/2);
		}	

		pos.timestamp = hrt_absolute_time();
		orb_advert_t pos_pub = orb_advertise(ORB_ID(att_pos_mocap), &pos);
		orb_publish(ORB_ID(att_pos_mocap), pos_pub, &pos);
	}

	void
	config(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			uint8_t num_anchors =4;
			/*device_coordinates_t anchorlist[num_anchors] = {
				{0x684E, 1, {0, 962, 1247}},
				{0x682E, 1, {0, 4293, 2087}},
				{0x6853, 1, {6746, 4888, 1559}},
				{0x6852, 1, {4689, 0, 2491}}
			};*/
			device_coordinates_t anchorlist[num_anchors] = {
				{0x684E, 1, {3479, -8725, 1479}},
				{0x682E, 1, {19025, -15030, 1603}},
				{0x6853, 1, {13334, 0, 1665}},
				{0x6852, 1, {5896, 0, 1614}}
			};
			if (bus.dev->clearDevices() == POZYX_SUCCESS){
				for (int j = 0; j < num_anchors; j++) {
					if (bus.dev->addDevice(anchorlist[j]) != POZYX_SUCCESS) {
						PX4_INFO("failed to add anchor");
						exit(1);
					}
					PX4_INFO("Anchor 0x%x successfully added at (%d, %d, %d)", anchorlist[j].network_id, anchorlist[j].pos.x, anchorlist[j].pos.y, anchorlist[j].pos.z);
				}
				if (bus.dev->getDeviceListSize(&num_anchors) == POZYX_SUCCESS) {
					PX4_INFO("%d anchors configured", num_anchors);
				}
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("%d anchors saved", num_anchors);
				}
			}
		}
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
		count = pozyx::start(busid);
		if (count > 0) {
			pozyx::config(busid, count);

			if (thread_running) {
				warnx("pozyx already running\n");
				exit(0);
			}

			thread_should_exit = false;
			if (count == 1) {
				daemon_task = px4_task_spawn_cmd("pozyx_pub", 
												SCHED_DEFAULT, 
												SCHED_PRIORITY_DEFAULT, 
												2000, 
												pozyx_pub_main,
												(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
			}
			if (count == 2) {
				daemon_task = px4_task_spawn_cmd("pozyx_pub_2", 
												SCHED_DEFAULT, 
												SCHED_PRIORITY_DEFAULT, 
												2000, 
												pozyx_pub_main_2,
												(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
			}

		}
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
		pozyx::test(busid, count);
		exit(0);
	}
	//configure pozyx
	if (!strcmp(verb, "config")) {
		pozyx::config(busid, count);
		exit(0);
	}

	//fetch positions
	if (!strcmp(verb, "getposition")) {
		pozyx::getposition(busid, count, true);
		exit(0);
	}

	//debug
	if (!strcmp(verb, "debug")) {
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
		pozyx::getposition(POZYX_BUS_ALL, 1, false);
		usleep(300000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}

int 
pozyx_pub_main_2(int argc, char *argv[])
{
	warnx("[pozyx_pub] starting\n");
	thread_running = true;

	while (!thread_should_exit) {
		pozyx::getposition(POZYX_BUS_ALL, 2, false);
		usleep(500000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}