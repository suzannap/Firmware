/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx.h
 *
 * Shared defines for the pozyx driver.
 */
#pragma once
/* Board Address */
#define POZYX_I2C_ADDR					0x4B /* Default Pozyx tag address */
#define POZYX_I2C_ADDR_ALT				0x4A /* Alternate Pozyx tag address (selected by soldering a 0Ohm resistor on the back of the pozyx tag*/

/* Begin auto generated defines */ 
/* POZYX REGISTERS firmware version v1.0 */ 

/* Status registers */

#define POZYX_WHO_AM_I					0x0	 /* Returns the constant value 0x43. */
#define POZYX_FIRMWARE_VER				0x1	 /* Returns the POZYX firmware version. */
#define POZYX_HARDWARE_VER				0x2	 /* Returns the POZYX hardware version. */
#define POZYX_ST_RESULT					0x3	 /* Returns the self-test result */
#define POZYX_ERRORCODE					0x4	 /* Describes a possibly system error. */
#define POZYX_INT_STATUS				0x5	 /* Indicates the source of the interrupt. */
#define POZYX_CALIB_STATUS				0x6	 /* Returns the calibration status. */

/* Configuration registers */

#define POZYX_INT_MASK					0x10	 /* Indicates which interrupts are enabled. */
#define POZYX_INT_CONFIG				0x11	 /* Configure the interrupt pin */
#define POZYX_CONFIG_LEDS				0x15	 /* Configure the LEDs */
#define POZYX_POS_ALG					0x16	 /* Algorithm used for positioning */
#define POZYX_POS_NUM_ANCHORS			0x17	 /* Configure the number of anchors and selection procedure */
#define POZYX_POS_INTERVAL				0x18	 /* Defines the update interval in ms in continuous positioning. */
#define POZYX_NETWORK_ID				0x1A	 /* The network id.  */
#define POZYX_UWB_CHANNEL				0x1C	 /* UWB channel number. */
#define POZYX_UWB_RATES					0x1D	 /* Configure the UWB datarate and pulse repetition frequency (PRF) */
#define POZYX_UWB_PLEN					0x1E	 /* Configure the UWB preamble length.  */
#define POZYX_UWB_GAIN					0x1F	 /* Configure the power gain for the UWB transmitter */
#define POZYX_UWB_XTALTRIM				0x20	 /* Trimming value for the uwb crystal. */
#define POZYX_OPERATION_MODE			0x22	 /* Configure the mode of operation of the pozyx device */
#define POZYX_SENSORS_MODE				0x23	 /* Configure the mode of operation of the sensors */
#define POZYX_CONFIG_GPIO1				0x27	 /* Configure GPIO pin 1. */
#define POZYX_CONFIG_GPIO2				0x28	 /* Configure GPIO pin 2. */
#define POZYX_CONFIG_GPIO3				0x29	 /* Configure GPIO pin 3. */
#define POZYX_CONFIG_GPIO4				0x2A	 /* Configure GPIO pin 4. */

/* Positioning data */

#define POZYX_POS_X						0x30	 /* x-coordinate of the device in mm. */
#define POZYX_POS_Y						0x34	 /* y-coordinate of the device in mm. */
#define POZYX_POS_Z						0x38	 /* z-coordinate of the device in mm. */
#define POZYX_POS_ERR_X					0x3C	 /* estimated error covariance of x */
#define POZYX_POS_ERR_Y					0x3E	 /* estimated error covariance of y */
#define POZYX_POS_ERR_Z					0x40	 /* estimated error covariance of z */
#define POZYX_POS_ERR_XY				0x42	 /* estimated covariance of xy */
#define POZYX_POS_ERR_XZ				0x44	 /* estimated covariance of xz */
#define POZYX_POS_ERR_YZ				0x46	 /* estimated covariance of yz */

/* Sensor data */

#define POZYX_PRESSURE					0x50	 /* Pressure data */
#define POZYX_ACCEL_X					0x54	 /* Accelerometer data (in mg) */
#define POZYX_ACCEL_Y					0x56	 /*  */
#define POZYX_ACCEL_Z					0x58	 /*  */
#define POZYX_MAGN_X					0x5A	 /* Magnemtometer data */
#define POZYX_MAGN_Y					0x5C	 /*  */
#define POZYX_MAGN_Z					0x5E	 /*  */
#define POZYX_GYRO_X					0x60	 /* Gyroscope data */
#define POZYX_GYRO_Y					0x62	 /*  */
#define POZYX_GYRO_Z					0x64	 /*  */
#define POZYX_EUL_HEADING				0x66	 /* Euler angles heading (or yaw) */
#define POZYX_EUL_ROLL					0x68	 /* Euler angles roll */
#define POZYX_EUL_PITCH					0x6A	 /* Euler angles pitch */
#define POZYX_QUAT_W					0x6C	 /* Weight of quaternion. */
#define POZYX_QUAT_X					0x6E	 /* x of quaternion */
#define POZYX_QUAT_Y					0x70	 /* y of quaternion */
#define POZYX_QUAT_Z					0x72	 /* z of quaternion */
#define POZYX_LIA_X						0x74	 /* Linear acceleration in x-direction */
#define POZYX_LIA_Y						0x76	 /*  */
#define POZYX_LIA_Z						0x78	 /*  */
#define POZYX_GRAV_X					0x7A	 /* x-component of gravity vector  */
#define POZYX_GRAV_Y					0x7C	 /* y-component of gravity vector  */
#define POZYX_GRAV_Z					0x7E	 /* z-component of gravity vector  */
#define POZYX_TEMPERATURE				0x80	 /* Temperature */

/* General data */

#define POZYX_DEVICE_LIST_SIZE			0x81	 /* Returns the number of devices stored internally */
#define POZYX_RX_NETWORK_ID				0x82	 /* The network id of the latest received message */
#define POZYX_RX_DATA_LEN				0x84	 /* The length of the latest received message */
#define POZYX_GPIO1						0x85	 /* Value of the GPIO pin 1 */
#define POZYX_GPIO2						0x86	 /* Value of the GPIO pin 2 */
#define POZYX_GPIO3						0x87	 /* Value of the GPIO pin 3 */
#define POZYX_GPIO4						0x88	 /* Value of the GPIO pin 4 */

/*Functions*/

#define POZYX_RESET_SYS					0xB0	 /* Reset the Pozyx device */
#define POZYX_LED_CTRL					0xB1	 /* Control LEDS 1 to 4 on the board */
#define POZYX_TX_DATA					0xB2	 /* Write data in the UWB transmit (TX) buffer */
#define POZYX_TX_SEND					0xB3	 /* Transmit the TX buffer to some other pozyx device */
#define POZYX_RX_DATA					0xB4	 /* Read data from the UWB receive (RX) buffer */
#define POZYX_DO_RANGING				0xB5	 /* Initiate ranging measurement */
#define POZYX_DO_POSITIONING			0xB6	 /* Initiate the positioning process.  */
#define POZYX_POS_SET_ANCHOR_IDS		0xB7	 /* Set the list of anchor ID's used for positioning.  */
#define POZYX_POS_GET_ANCHOR_IDS		0xB8	 /* Read the list of anchor ID's used for positioning. */
#define POZYX_FLASH_RESET				0xB9	 /* Reset a portion of the configuration in flash memory */
#define POZYX_FLASH_SAVE				0xBA	 /* Store a portion of the configuration in flash memory */
#define POZYX_FLASH_DETAILS				0xBB	 /* Return information on what is stored in flash */

/*Device list functions*/

#define POZYX_DEVICES_GETIDS			0xC0	 /* Get all the network IDs's of devices in the device list. */
#define POZYX_DEVICES_DISCOVER			0xC1	 /* Obtain the network ID's of all pozyx devices within range. */
#define POZYX_DEVICES_CALIBRATE			0xC2	 /* Obtain the coordinates of the pozyx (anchor) devices within range. */
#define POZYX_DEVICES_CLEAR				0xC3	 /* Clear the list of all pozyx devices. */
#define POZYX_DEVICE_ADD				0xC4	 /* Add a pozyx device to the devices list */
#define POZYX_DEVICE_GETINFO			0xC5	 /* Get the stored device information for a given pozyx device */
#define POZYX_DEVICE_GETCOORDS			0xC6	 /* Get the stored coordinates of a given pozyx device */
#define POZYX_DEVICE_GETRANGEINFO		0xC7	 /* Get the stored range inforamation of a given pozyx device */
#define POZYX_CIR_DATA					0xC8	 /* Get the channel impulse response (CIR) coefficients */

/* Macro's to test if registers are readable/writable */

#define IS_REG_READABLE(x) 			 (((((x)>=0x0)&&((x)<0x7)) || (((x)>=0x10)&&((x)<0x12)) || (((x)>=0x15)&&((x)<0x21)) || (((x)>=0x22)&&((x)<0x24)) || (((x)>=0x27)&&((x)<0x2b)) || (((x)>=0x30)&&((x)<0x48)) || (((x)>=0x50)&&((x)<0x89)))?1:0) 
#define IS_REG_WRITABLE(x) 			 (((((x)>=0x10)&&((x)<0x12)) || (((x)>=0x15)&&((x)<0x21)) || (((x)>=0x22)&&((x)<0x24)) || (((x)>=0x27)&&((x)<0x2b)) || (((x)>=0x30)&&((x)<0x3c)) || (((x)>=0x85)&&((x)<0x89)))?1:0) 
#define IS_FUNCTIONCALL(x) 			 (((((x)>=0xb0)&&((x)<0xbc)) || (((x)>=0xc0)&&((x)<0xc9)))?1:0) 

/* End of auto generated defines */ 


#define DRV_POS_DEVTYPE_POZYX  		0x40
#define POZYX_WHOAMI_EXPECTED		0x43;

#define POZYX_CONVERSION_INTERVAL (400000 / 150)

/* interface factory */
extern device::Device *POZYX_I2C_interface(int bus);
typedef device::Device *(POZYX_contructor)(int);

