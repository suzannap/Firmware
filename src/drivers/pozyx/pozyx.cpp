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



enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL
};

/*define error (not defined in c++)*/
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class POZYX : public device::CDev
{
public:
	POZYX(device::Device *interface, const char *path, enum Rotation rotation);
	virtual ~POZYX();

	virtual int 	init();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int 	ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver
	*/
	void			print_info();
	/*write a register*/
	int 			write_reg(uint8_t reg, uint8_t val, uint8_t len);
	/*read a register*/
	int 		 	read_reg(uint8_t reg, uint8_t &val, uint8_t len);
	int 		 	read_reg_32(uint8_t reg, uint32_t &val);
	//*function call*/
	int 			function_reg(uint8_t reg, uint8_t &val, uint8_t len);

protected:
	Device 			*_interface;

private:
	work_s			_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer *_reports;

	perf_counter_t	_sample_perf;
	perf_counter_t 	_comms_errors;
	perf_counter_t 	_buffer_overflows;
	perf_counter_t 	_range_errors;
	perf_counter_t  _conf_errors;

	/*status reporting */
	bool 			_sensor_ok; 	/*sensor was found and reports ok */
	bool 			_calibrated;	/*calibration is valid*/

	enum Rotation 	_rotation;

	//struct mag_report 	_last_report;

	//uint8_t		_range_bits;
	//uint8_t 		_conf_reg;
	//uint8_t 		_temperature_counter;
	//uint8_t 		_temperature_error_count;

	/*initialize automatic measurement state machine and start it*/
	void 			start();
	/*stop automatic measurement state machine*/
	void 			stop();
	/*reset the device */
	int 			reset();

	void			cycle();
	static void		cycle_trampoline(void *arg);

	POZYX(const POZYX &);
	POZYX operator=(const POZYX &);
};

/*Driver 'main' command.*/
extern "C" __EXPORT int pozyx_main(int argc, char *argv[]);

POZYX::POZYX(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("POZYX", path),
	_interface(interface),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "pozyx_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pozyx_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "pozyx_buf_of")),
	_range_errors(perf_alloc(PC_COUNT, "pozyx_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "pozyx_conf_err")),
	_sensor_ok(false),
	_calibrated(false),
	_rotation(rotation)
	//_last_report(0)
	//_range_bits(0),
	//_conf_reg(0)
	//_temperature_counter(0),
	//_temperature_error_count(0)
{
	_device_id.devid_s.devtype = DRV_POS_DEVTYPE_POZYX;

	//enable debug() calls
	_debug_enabled = false;

	memset(&_work, 0, sizeof(_work));
}

POZYX::~POZYX()
{
	//make sure we are inactive
	stop();

	if(_reports != nullptr) {
		delete _reports;
	}

	//free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
POZYX::init()
{
	int ret = ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	//allocate basic report buffers
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		goto out;
	}

	//reset device configuration
	reset();


	ret = OK;
	//sensor is ok but not calibrated
	_sensor_ok = true;
out:
	return ret;
}


ssize_t
POZYX::read(struct file *filp, char *buffer, size_t buflen)
{
	PX4_INFO("pozyx read attempted");
	//unsigned count = buflen;
	int ret = 0;



	return ret;
}

int
POZYX::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
		case SENSORIOCSPOLLRATE: {
			switch(arg){
				case SENSOR_POLLRATE_MANUAL: {
					stop();
					_measure_ticks = 0;
					return OK;
				}
				case SENSOR_POLLRATE_EXTERNAL:
				case 0:
					return -EINVAL;
				case SENSOR_POLLRATE_MAX:
				case SENSOR_POLLRATE_DEFAULT: {
					bool want_start = (_measure_ticks == 0);
					_measure_ticks = USEC2TICK(POZYX_CONVERSION_INTERVAL);
					if (want_start) {
						start();
					}
					return OK;
				}
				default: {
					bool want_start = (_measure_ticks == 0);
					unsigned ticks = USEC2TICK(400000 / arg);
					if (ticks < USEC2TICK(POZYX_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}
					_measure_ticks = ticks;
					if (want_start) {
						start();
					}
					return OK;
				}
			}
		}
		case SENSORIOCGPOLLRATE: {
			if (_measure_ticks == 0) {
				return SENSOR_POLLRATE_MANUAL;
			}
			return 400000 / TICK2USEC(_measure_ticks);
		}
		case SENSORIOCSQUEUEDEPTH: {
			if ((arg<1) || (arg >100)) {
				return -EINVAL;
			}
			irqstate_t flags = px4_enter_critical_section();
			if(!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}
			px4_leave_critical_section(flags);
			return OK;
		}
		case SENSORIOCGQUEUEDEPTH: 
			return _reports->size();
		case SENSORIOCRESET:
			return reset();
		case MAGIOCSSAMPLERATE:
			return ioctl(filp, SENSORIOCSPOLLRATE,arg);
		case MAGIOCGSAMPLERATE:
			return 400000 / TICK2USEC(_measure_ticks);
		case MAGIOCGEXTERNAL:
			DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
			return _interface->ioctl(cmd, dummy);
		case DEVIOCGDEVICEID:
			return _interface->ioctl(cmd, dummy);

		default:
			return CDev::ioctl(filp, cmd, arg);
	}
}


void
POZYX::start()
{
	_reports->flush();

	work_queue(HPWORK, &_work, (worker_t)&POZYX::cycle_trampoline, this, 1);
}

void
POZYX::stop()
{
	work_cancel(HPWORK, &_work);
}

int
POZYX::reset()
{
	return 0;
}

void
POZYX::cycle_trampoline(void *arg)
{
	POZYX *dev = (POZYX *)arg;

	dev->cycle();
}

void
POZYX::cycle()
{
	/*
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		start();
		return;
	}

	if(_measure_ticks > USEC2TICK(POZYX_CONVERSION_INTERVAL)) {
		work_queue(HPWORK,
			&_work,
			(worker_t)&POZYX::cycle_trampoline,
			this,
			_measure_ticks - USEC2TICK(POZYX_CONVERSION_INTERVAL));

		return;
	}

	if (OK != measure()) {
		DEVICE_DEBUG("measure_error");
	}

	work_queue(HPWORK,
		&_work,
		(worker_t)&POZYX::cycle_trampoline,
		this,
		USEC2TICK(POZYX_CONVERSION_INTERVAL));
		*/
}

int
POZYX::write_reg(uint8_t reg, uint8_t val, uint8_t len)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, len);
}

int
POZYX::read_reg(uint8_t reg, uint8_t &val, uint8_t len)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, len);
	val = buf;
	return ret;
}
int
POZYX::read_reg_32(uint8_t reg, uint32_t &val)
{
	return _interface->read(reg, (uint8_t *)&val, 4);
}

int
POZYX::function_reg(uint8_t reg, uint8_t &val, uint8_t len)
{
	uint8_t buf = val;
	int ret = _interface->write(reg, &buf, len);
	val = buf;
	return ret;
}

void
POZYX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval: %u ticks\n", _measure_ticks);
	//printf("output (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("some more stuff");
	_reports->print_info("report queue");
}

namespace pozyx
{
	#ifdef ERROR
	# undef ERROR
	#endif
	const int ERROR = -1;

	//list of supported bus configurations
	struct pozyx_bus_option {
		enum POZYX_BUS busid;
		const char *devpath;
		POZYX_constructor interface_constructor;
		uint8_t busnum;
		POZYX *dev;
	} bus_options[2] = {
		{ POZYX_BUS_I2C_EXTERNAL, "/dev/pozyx_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
		{ POZYX_BUS_I2C_INTERNAL, "/dev/pozyx_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
	};

		
		#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

	void 	start(enum POZYX_BUS busid, enum Rotation rotation);
	bool 	start_bus(struct pozyx_bus_option &bus, enum Rotation rotation);
	struct 	pozyx_bus_option &find_bus(enum POZYX_BUS busid);
	void 	test(enum POZYX_BUS busid);
	void	reset(enum POZYX_BUS busid);
	void	getposition(enum POZYX_BUS busid);
	int 	info(enum POZYX_BUS busid, bool enable);
	int 	calibrate(enum POZYX_BUS busid);
	int 	temp_enable(POZYX_BUS busid, bool enable);
	void	usage();

	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus, enum Rotation rotation)
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

		bus.dev = new POZYX(interface, bus.devpath, rotation);

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
	start(enum POZYX_BUS busid, enum Rotation rotation)
	{
		bool started = false;

		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
				continue;
			}

			started |= start_bus(bus_options[i], rotation);
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
		testread = bus.dev->read_reg(POZYX_WHO_AM_I, whoami, 1);
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

		testread = bus.dev->function_reg(POZYX_LED_CTRL, funcbuf[0], 1);
		if (funcbuf[0] != 1) {
			err(1, "Function test failed");
		}
		PX4_INFO("LED3 turned On... ");
		sleep(3);
		funcbuf[0] = 0x40;
		testread = bus.dev->function_reg(POZYX_LED_CTRL, funcbuf[0], 1);
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


		bus.dev->read_reg_32(POZYX_POS_X, coordinates[0]);
		bus.dev->read_reg_32(POZYX_POS_Y, coordinates[1]);
		bus.dev->read_reg_32(POZYX_POS_Z, coordinates[2]);

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
		struct pozyx_bus_option &bus = find_bus(busid);
		const char *path = bus.devpath;
		int fd = open(path, O_RDONLY);

		if (fd<0) {
			err(1, "failed");
		}

		if (ioctl(fd,SENSORIOCRESET, 0) < 0) {
			err(1, "driver reset failed");
		}
		if (ioctl(fd,SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			err(1, "driver poll restart failed");
		}
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
		warnx("some warnings");
		#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
		warnx(" -I only internal bus");
		#endif
	}

} //namespace

int
pozyx_main(int argc, char *argv[])
{
	
	int ch;
	enum POZYX_BUS busid = POZYX_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
			case 'R':
			rotation = (enum Rotation)atoi(optarg);
			default:
			pozyx::usage();
			exit(0);
		}
	}
	

	const char *verb = argv[optind];

	//start/load driver
	if (!strcmp(verb, "start")) {
		pozyx::start(busid, rotation);
		PX4_INFO("pozyx started");

		exit(0);
	}

	//test driver/device
	if (!strcmp(verb, "test")) {
		PX4_INFO("testing pozyx...");
		pozyx::test(busid);
		exit(0);
	}


	//reset driver
	if (!strcmp(verb, "getposition")) {
		pozyx::getposition(busid);
		exit(0);
	}


	errx(1, "unrecognized command, try start, test, reset");

}