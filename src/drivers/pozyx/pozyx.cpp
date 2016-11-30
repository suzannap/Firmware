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
	POZYX(device::Device *interface, const char *path);
	virtual ~POZYX();

	virtual int 	init();
	/**
	* Diagnostics - print some basic information about the driver
	*/
	void 			IRQ();
	bool 		    waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt);
	bool 			waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt);

	void			print_info();
	/*read/write local registers, and local function calls*/
	int 			regWrite(uint8_t reg_address, uint8_t *pData, int size);
	int 			regRead(uint8_t reg_address, uint8_t *pData, int size);
	int 			regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);
	/*read/write remote registers, and remote function calls*/
	int 			remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
	int 			remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);
	int 			remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);

    static int getLastNetworkId(uint16_t *network_id, uint16_t remote_id = NULL);
    static int getLastDataLength(uint8_t *data_length, uint16_t remote_id = NULL);
    static int getNetworkId(uint16_t *network_id);
    static int setNetworkId(uint16_t network_id, uint16_t remote_id = NULL);
    static int getUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);
    static int setUWBSettings(UWB_settings_t *UWB_settings, uint16_t remote_id = NULL);
    static int setUWBChannel(int channel_num, uint16_t remote_id = NULL);
    static int getUWBChannel(int* channel_num, uint16_t remote_id = NULL);
    static int setTxPower(float txgain_dB, uint16_t remote_id = NULL);
    static int getTxPower(float* txgain_dB, uint16_t remote_id = NULL);
    static int getWhoAmI(uint8_t *whoami, uint16_t remote_id = NULL);
    static int getFirmwareVersion(uint8_t *firmware, uint16_t remote_id = NULL);
    static int getHardwareVersion(uint8_t *hardware, uint16_t remote_id = NULL);
    static int getSelftest(uint8_t *selftest, uint16_t remote_id = NULL);
   	static int getErrorCode(uint8_t *error_code, uint16_t remote_id = NULL);
   	static int getInterruptStatus(uint8_t *interrupts, uint16_t remote_id = NULL);
    static int getCalibrationStatus(uint8_t *calibration_status, uint16_t remote_id = NULL);
    static int getGPIO(int gpio_num, uint8_t *value, uint16_t remote_id = NULL);
    static int setGPIO(int gpio_num, uint8_t value, uint16_t remote_id = NULL);
    static void resetSystem(uint16_t remote_id = NULL);
    static int setLed(int led_num, bool state, uint16_t remote_id = NULL);
    static int getInterruptMask(uint8_t *mask, uint16_t remote_id = NULL);
    static int setInterruptMask(uint8_t mask, uint16_t remote_id = NULL);
    static int getConfigModeGPIO(int gpio_num, uint8_t *mode, uint16_t remote_id = NULL);
    static int getConfigPullGPIO(int gpio_num, uint8_t *pull, uint16_t remote_id = NULL);
    static int setConfigGPIO(int gpio_num, int mode, int pull, uint16_t remote_id = NULL);
    static int setLedConfig(uint8_t config = 0x0, uint16_t remote_id = NULL);
    static int configInterruptPin(int pin, int mode, int bActiveHigh, int bLatch, uint16_t remote_id=NULL);
    static int saveConfiguration(int type, uint8_t registers[] = NULL, int num_registers = 0, uint16_t remote_id = NULL);
    static int clearConfiguration(uint16_t remote_id = NULL);
    static bool isRegisterSaved(uint8_t regAddress, uint16_t remote_id = NULL);
    static int getNumRegistersSaved(uint16_t remote_id = NULL);
    static int getCoordinates(coordinates_t *coordinates, uint16_t remote_id = NULL);
    static int setCoordinates(coordinates_t coordinates, uint16_t remote_id = NULL);
    static int getPositionError(pos_error_t *pos_error, uint16_t remote_id = NULL);
    static int setPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);
    static int getPositioningAnchorIds(uint16_t anchors[], int anchor_num, uint16_t remote_id = NULL);
    static int getUpdateInterval(uint16_t *ms, uint16_t remote_id = NULL);
    static int setUpdateInterval(uint16_t ms, uint16_t remote_id = NULL);
    static int getPositionAlgorithm(uint8_t *algorithm, uint16_t remote_id = NULL);
    static int getPositionDimension(uint8_t *dimension, uint16_t remote_id = NULL);
    static int setPositionAlgorithm(int algorithm = POZYX_POS_ALG_UWB_ONLY, int dimension = 0x0, uint16_t remote_id = NULL);
    static int getAnchorSelectionMode(uint8_t *mode, uint16_t remote_id = NULL);
    static int getNumberOfAnchors(uint8_t *nr_anchors, uint16_t remote_id = NULL);
    static int setSelectionOfAnchors(int mode, int nr_anchors, uint16_t remote_id = NULL);
    static int getOperationMode(uint8_t *mode, uint16_t remote_id = NULL);
    static int setOperationMode(uint8_t mode, uint16_t remote_id = NULL);
    //static string getSystemError(uint16_t remote_id = NULL);
    static int getSensorMode(uint8_t *sensor_mode, uint16_t remote_id = NULL);
    static int setSensorMode(uint8_t sensor_mode, uint16_t remote_id = NULL);
    static int getAllSensorData(sensor_data_t *sensor_data, uint16_t remote_id = NULL);
    static int getPressure_Pa(float32_t *pressure, uint16_t remote_id = NULL);
    static int getAcceleration_mg(acceleration_t *acceleration, uint16_t remote_id = NULL);
    static int getMagnetic_uT(magnetic_t *magnetic, uint16_t remote_id = NULL);
    static int getAngularVelocity_dps(angular_vel_t *angular_vel, uint16_t remote_id = NULL);
    static int getEulerAngles_deg(euler_angles_t *euler_angles, uint16_t remote_id = NULL);
    static int getQuaternion(quaternion_t *quaternion, uint16_t remote_id = NULL);
    static int getLinearAcceleration_mg(linear_acceleration_t *linear_acceleration, uint16_t remote_id = NULL);
    static int getGravityVector_mg(gravity_vector_t *gravity_vector, uint16_t remote_id = NULL);
    static int getTemperature_c(float32_t *temperature, uint16_t remote_id = NULL);
    static int doPositioning(coordinates_t *position, uint8_t dimension = POZYX_2D, int32_t height = 0, uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY);
    static int doRemotePositioning(uint16_t remote_id, coordinates_t *coordinates, uint8_t dimension = POZYX_2D, int32_t height = 0, uint8_t algorithm = 0);
    static int doRanging(uint16_t destination, device_range_t *range);
    static int doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t *range);
    static int getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t remote_id = NULL);
    static int getDeviceListSize(uint8_t *device_list_size, uint16_t remote_id = NULL);
    static int getDeviceIds(uint16_t devices[], int size, uint16_t remote_id = NULL);
    static int getAnchorIds(uint16_t anchors[], int size, uint16_t remote_id = NULL);
    static int getTagIds(uint16_t tags[], int size, uint16_t remote_id = NULL);
    static int doDiscovery(int type = 0x0, int slots = 3, int slot_duration = 10);
    static int doAnchorCalibration(int dimension = POZYX_2D, int num_measurements = 10, int num_anchors = 0, uint16_t anchors[] = NULL,  int32_t heights[] = NULL);
    static int clearDevices(uint16_t remote_id = NULL);
    static int addDevice(device_coordinates_t device_coordinates, uint16_t remote_id = NULL);
    static int getDeviceCoordinates(uint16_t device_id, coordinates_t *coordinates, uint16_t remote_id = NULL);

/** @}*/    
protected:
	Device 		*_interface;
	//struct pozyx_bus_options _busid;
	static int _mode;               // the mode of operation, can be MODE_INTERRUPT or MODE_POLLING
    static int _interrupt;          // variable to indicate that an interrupt has occured

    static int _hw_version;         // Pozyx harware version 
    static int _fw_version;         // Pozyx software (firmware) version. (By updating the firmware on the Pozyx device, this value can change)

private:


	/*initialize automatic measurement state machine and start it*/
	void 			start();
	/*stop automatic measurement state machine*/
	void 			stop();
	/*reset the device */
	int 			reset();

	void			cycle();

	POZYX(const POZYX &);
	POZYX operator=(const POZYX &);
};

/*Driver 'main' command.*/
extern "C" __EXPORT int pozyx_main(int argc, char *argv[]);

POZYX::POZYX(device::Device *interface, const char *path) :
	CDev("POZYX", path),
	_interface(interface)
{
	//_busid = find_bus()
}

POZYX::~POZYX()
{
	//make sure we are inactive
	stop();
}

//include functions defined in pozyx_lib.cpp
#include "Pozyx_lib.h"
int
POZYX::init()
{
	return 0;
}

void
POZYX::start()
{
	return 0;
}

void
POZYX::stop()
{
	return 0;
}

int
POZYX::reset()
{
	return 0;
}

void
POZYX::cycle()
{
	return 0;
}

void POZYX::IRQ()
{  
  _interrupt = 1;  
}

boolean POZYX::waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  long timer = millis();
  int status;
  
  // stay in this loop until the event interrupt flag is set or until the the timer runs out
  while(millis()-timer < timeout_ms)
  {
    // in polling mode, we insert a small delay such that we don't swamp the i2c bus
    if( _mode == MODE_POLLING ){
      delay(1);
    }
    
    if( (_interrupt == 1) || (_mode == MODE_POLLING))
    { 
      _interrupt = 0;
      
      // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
      uint8_t interrupt_status = 0;
      status = regRead(POZYX_INT_STATUS, &interrupt_status, 1);
      if((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
      {
        // one of the interrupts we were waiting for arrived!
        if(interrupt != NULL)
          *interrupt = interrupt_status;
        return true;
      }
    }     
  } 
  // too bad, pozyx didn't respond 
  // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
  // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
  return false;  
}

boolean POZYX::waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  int tmp = _mode;
  _mode = MODE_POLLING;
  boolean result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
  _mode = tmp;
  return result;
}

int 
POZYX::regWrite(uint8_t reg_address, uint8_t *pData, int size)
{  
   
  if(!IS_REG_WRITABLE(reg_address))
    return POZYX_FAILURE;
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    if(i+1 != n_runs){
    	status = _interface->write(reg_address+offset, pData+offset, BUFFER_LENGTH);    
    }else{
    	status = _interface->write(reg_address+offset, pData+offset, size-offset);    
    }    
  }
  
  return status;
}
POZYX::remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
	return 0;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int 
POZYX::regRead(uint8_t reg_address, uint8_t *pData, int size)
{  
  if(!IS_REG_READABLE(reg_address)){
    return POZYX_FAILURE;
  }
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
  uint8_t reg;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    reg = reg_address+offset;    
    
    if(i+1 != n_runs){  
    	status = _interface->read(reg, pData+offset, BUFFER_LENGTH);    
    }else{      
    	status = _interface->read(reg, pData+offset, size-offset);    
    }    
  }
  
  return status;
}
int
POZYX::remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
	return 0;
}
/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int 
POZYX::regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{

  if(!IS_FUNCTIONCALL(reg_address)){
    	return POZYX_FAILURE;
	}	

  uint8_t status;
  
   // first write some data with i2c and then read some data
  status = _interface->write(reg_address, params, param_size);
  if(status == POZYX_FAILURE){
    return status;    
	}
  //copy returned data (which has been written to paramdata) to specified address  
  memcpy(&pData, &params, size);
  // the first byte that a function returns is always it's success indicator, so we simply pass this through
  memcpy(&status, &params, 1);
  return status;
}
POZYX::remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
	return 0;
}




/****************namespace**********************************************/
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

		bus.dev = new POZYX(interface, bus.devpath);

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