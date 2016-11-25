/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx_i2c.h
 *
 * Shared defines for the pozyx driver.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "pozyx.h"
#include "board_config.h"

device::Device *POZYX_I2C_interface(int bus);

class POZYX_I2C : public device::POZYX_I2C
{
public:
	POZYX_I2C(int bus);
	virtual ~POZYX_I2C();

	virtual int init();
	virtual int read(unsigned address, void *data, unsigned count);
	virtual int write(unsigned address, void *data, unsigned count);

	virtual int ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int probe();

};

device::Device *
POZYX_I2C_interface(int bus)
{
	return new POZYX_I2C(bus);
}

POZYX_I2C::POZYX_I2C(int bus) :
I2C("POZYX_I2C", nullptr, bus, POZYX_I2C_ADDR, 400000)
{
	_device_id.devid_s.devtype - DRV_POS_DEVTYPE_POZYX;
}

POZYX_I2C::~POZYX_I2C()
{

}

int
POZYX_I2C::init()
{
	/*this will call probe() */
	return I2C::init();
}

int
POZYX_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;
	switch (operation) {

	case TESTOP:
		PX4_INFO("test case");
		return 1;
	case TESTOP2:
		PX4_INFO("test case 2");
		return 1
	default:
		ret = -EINVAL;
	}

	return ret;
}

int
POZYX_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;

	if (read(POZYX_WHO_AM_I, &data, 1) {
		DEVICE_DEBUG("reg_read fail");
		PX4_INFO("reg_read fail");
		return -EIO;
	}

	_retries = 2;

	if (data != POZYX_WHOAMI_EXPECTED) {
		DEVICE_DEBUG("ID byte mismatch (%02x)", data);
		PX4_INFO("ID byte mismatch");
		return -EIO;
	}

	return OK

}

int 
POZYX_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];
	if (sizeof(buf)< (count+1)) {
		return -EIO
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);	

	return transfer(&buf[0], count + 1, nullptr, 0;)
}

int
POZYX_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}