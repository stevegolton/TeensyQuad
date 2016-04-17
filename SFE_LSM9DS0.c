/******************************************************************************
SFE_LSM9DS0.cpp
SFE_LSM9DS0 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 14, 2014 (Happy Valentines Day!)
https://github.com/sparkfun/LSM9DS0_Breakout

This file implements all functions of the LSM9DS0 class. Functions here range
from higher level stuff, like reading/writing LSM9DS0 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	LSM9DS0 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SFE_LSM9DS0.h"

// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
static void initGyro(stLSM9DS0_t * stThis);

// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
static void initAccel(stLSM9DS0_t * stThis);

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
static void initMag(stLSM9DS0_t * stThis);

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
static uint8_t gReadByte(stLSM9DS0_t * stThis,uint8_t subAddress);

// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
static void gReadBytes(stLSM9DS0_t * stThis,uint8_t subAddress, uint8_t * dest, uint8_t count);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
static void gWriteByte(stLSM9DS0_t * stThis,uint8_t subAddress, uint8_t data);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
static uint8_t xmReadByte(stLSM9DS0_t * stThis,uint8_t subAddress);

// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
static void xmReadBytes(stLSM9DS0_t * stThis,uint8_t subAddress, uint8_t * dest, uint8_t count);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
static void xmWriteByte(stLSM9DS0_t * stThis,uint8_t subAddress, uint8_t data);

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
static void calcgRes(stLSM9DS0_t * stThis);

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
static void calcmRes(stLSM9DS0_t * stThis);

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
static void calcaRes(stLSM9DS0_t * stThis);

///////////////////
// SPI Functions //
///////////////////
// initSPI() -- Initialize the SPI hardware.
// This function will setup all SPI pins and related hardware.
static void initSPI(stLSM9DS0_t * stThis);

// SPIwriteByte() -- Write a byte out of SPI to a register in the device
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
static void SPIwriteByte(stLSM9DS0_t * stThis,uint8_t csPin, uint8_t subAddress, uint8_t data);

// SPIreadByte() -- Read a single byte from a register over SPI.
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
static uint8_t SPIreadByte(stLSM9DS0_t * stThis,uint8_t csPin, uint8_t subAddress);

// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- csPin = The chip select pin of a slave device.
//	- subAddress = The register to begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
static void SPIreadBytes(stLSM9DS0_t * stThis,uint8_t csPin, uint8_t subAddress,
						uint8_t * dest, uint8_t count);

///////////////////
// I2C Functions //
///////////////////
// initI2C() -- Initialize the I2C hardware.
// This function will setup all I2C pins and related hardware.
static void initI2C(stLSM9DS0_t * stThis);

static void wait( uint16_t millis );

/* ************************************************************************** **
** PUBLIC FUNCTIONS
** ************************************************************************** */

/* ************************************************************************** */
void LSM9DS0_Setup( stLSM9DS0_t * stThis,
					interface_mode interface,
					uint8_t gAddr,
					uint8_t xmAddr,
					write_byte_t write_byte,
					read_byte_t read_byte,
					read_bytes_t read_bytes )
{
	// interfaceMode will keep track of whether we're using SPI or I2C:
	stThis->interfaceMode = interface;

	// xmAddress and gAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	stThis->xmAddress = xmAddr;
	stThis->gAddress = gAddr;
}

/* ************************************************************************** */
uint16_t LSM9DS0_begin( stLSM9DS0_t * stThis )
{
	LSM9DS0_begin_adv( stThis,
					   G_SCALE_245DPS,
					   A_SCALE_2G,
					   M_SCALE_2GS,
					   G_ODR_95_BW_125,
					   A_ODR_50,
					   M_ODR_50 );
}

/* ************************************************************************** */
uint16_t LSM9DS0_begin_adv( stLSM9DS0_t * stThis,
							gyro_scale gScl,
							accel_scale aScl,
							mag_scale mScl,
							gyro_odr gODR,
							accel_odr aODR,
							mag_odr mODR )
{
	// Default to 0xFF to indicate status is not OK
	uint8_t gTest = 0xFF;
	uint8_t xmTest = 0xFF;
	uint8_t byDatas;

	uint8_t byAddress = 0x1D;
	uint8_t bySubAddress = 0x0F;

	// Wait for a few millis at the beginning for the chip to boot
	wait( 200 );

	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	stThis->gScale = gScl;
	stThis->aScale = aScl;
	stThis->mScale = mScl;
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(stThis); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(stThis); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(stThis); // Calculate g / ADC tick, stored in aRes variable
	
	// Now, initialize our hardware interface.
	if (stThis->interfaceMode == MODE_I2C)			// If we're using I2C
		initI2C(stThis);							// Initialize I2C
	else if (stThis->interfaceMode == MODE_SPI)		// else, if we're using SPI
		initSPI(stThis);							// Initialize SPI

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	gTest = gReadByte( stThis, WHO_AM_I_G );		// Read the gyro WHO_AM_I
	xmTest = xmReadByte( stThis, WHO_AM_I_XM );	// Read the accel/mag WHO_AM_I

	// Gyro initialization stuff:
	initGyro(stThis);	// This will "turn on" the gyro. Setting up interrupts, etc.
	LSM9DS0_setGyroODR(stThis, gODR); // Set the gyro output data rate and bandwidth.
	LSM9DS0_setGyroScale(stThis, stThis->gScale); // Set the gyro range
	
	// Accelerometer initialization stuff:
	initAccel(stThis); // "Turn on" all axes of the accel. Set up interrupts, etc.
	LSM9DS0_setAccelODR(stThis, aODR); // Set the accel data rate.
	LSM9DS0_setAccelScale(stThis, stThis->aScale); // Set the accel range.
	
	// Magnetometer initialization stuff:
	initMag(stThis); // "Turn on" all axes of the mag. Set up interrupts, etc.
	LSM9DS0_setMagODR(stThis, mODR); // Set the magnetometer output data rate.
	LSM9DS0_setMagScale(stThis, stThis->mScale); // Set the magnetometer's range.
	
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xmTest << 8) | gTest;
}

/* ************************************************************************** */
static void initGyro(stLSM9DS0_t * stThis)
{
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(stThis, CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	
	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(stThis, CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(stThis, CTRL_REG3_G, 0x88);
	
	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(stThis, CTRL_REG4_G, 0x00); // Set scale to 245 dps
	
	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(stThis, CTRL_REG5_G, 0x00);
	
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	LSM9DS0_configGyroInt(stThis, 0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

/* ************************************************************************** */
static void initAccel(stLSM9DS0_t * stThis)
{
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
	xmWriteByte(stThis, CTRL_REG0_XM, 0x00);
	
	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */	
	xmWriteByte(stThis, CTRL_REG1_XM, 0x57); // 100Hz data rate, x/y/z all enabled
	
	//Serial.println(xmReadByte(CTRL_REG1_XM));
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	ABW[1:0] - Accelerometer anti-alias filter bandwidth
		00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	AFS[2:0] - Accel full-scale selection
		000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	AST[1:0] - Accel self-test enable
		00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	SIM - SPI mode selection
		0=4-wire, 1=3-wire													 */
	xmWriteByte(stThis, CTRL_REG2_XM, 0x00); // Set scale to 2g
	
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
	xmWriteByte(stThis, CTRL_REG3_XM, 0x04);
}

/* ************************************************************************** */
static void initMag(stLSM9DS0_t * stThis)
{	
	/* CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
	Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
	TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
	M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
	M_ODR[2:0] - Magnetometer data rate select
		000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
	LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
		0=interrupt request not latched, 1=interrupt request latched
	LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
		0=irq not latched, 1=irq latched 									 */
	xmWriteByte(stThis, CTRL_REG5_XM, 0x94); // Mag data rate - 100 Hz, enable temperature sensor
	
	/* CTRL_REG6_XM sets the magnetometer full-scale
	Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
	MFS[1:0] - Magnetic full-scale selection
	00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs							 */
	xmWriteByte(stThis, CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS
	
	/* CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
	AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
	AHPM[1:0] - HPF mode selection
		00=normal (resets reference registers), 01=reference signal for filtering, 
		10=normal, 11=autoreset on interrupt event
	AFDS - Filtered acceleration data selection
		0=internal filter bypassed, 1=data from internal filter sent to FIFO
	MLP - Magnetic data low-power mode
		0=data rate is set by M_ODR bits in CTRL_REG5
		1=data rate is set to 3.125Hz
	MD[1:0] - Magnetic sensor mode selection (default 10)
		00=continuous-conversion, 01=single-conversion, 10 and 11=power-down */
	xmWriteByte(stThis, CTRL_REG7_XM, 0x00); // Continuous conversion mode
	
	/* CTRL_REG4_XM is used to set interrupt generators on INT2_XM
	Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
	*/
	xmWriteByte(stThis, CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
	
	/* INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
	Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
	XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
	PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
	IEA - Interrupt polarity for accel and magneto
		0=active-low, 1=active-high
	IEL - Latch interrupt request for accel and magneto
		0=irq not latched, 1=irq latched
	4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
	MIEN - Enable interrupt generation for magnetic data
		0=disable, 1=enable) */
	xmWriteByte(stThis, INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
}

/* ************************************************************************** */
// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void LSM9DS0_calLSM9DS0(stLSM9DS0_t * stThis, float * gbias, float * abias)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  int samples, ii;
  
  // First get gyro bias
  uint8_t c = gReadByte(stThis, CTRL_REG5_G);
  gWriteByte(stThis, CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO
  wait(20);
  //delay(20);                                 // Wait for change to take effect
  gWriteByte(stThis, FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  wait(1000);
  //delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (gReadByte(stThis, FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    gReadBytes(stThis, OUT_X_L_G,  &data[0], 6);
    gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gbias[0] = (float)gyro_bias[0]*stThis->gRes;  // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1]*stThis->gRes;
  gbias[2] = (float)gyro_bias[2]*stThis->gRes;
  
  c = gReadByte(stThis, CTRL_REG5_G);
  gWriteByte(stThis, CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO
  //delay(20);
  wait(20);
  gWriteByte(stThis, FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode
  

  //  Now get the accelerometer biases
  c = xmReadByte(stThis, CTRL_REG0_XM);
  xmWriteByte(stThis, CTRL_REG0_XM, c | 0x40);      // Enable accelerometer FIFO
  //delay(20);                                // Wait for change to take effect
  wait(20);
  xmWriteByte(stThis, FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable accelerometer FIFO stream mode and set watermark at 32 samples
  //delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  wait(1000);

  samples = (xmReadByte(stThis, FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

   for(ii = 0; ii < samples ; ii++) {          // Read the accelerometer data stored in the FIFO
    xmReadBytes(stThis, OUT_X_L_A, &data[0], 6);
    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1./stThis->aRes); // Assumes sensor facing up!
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  abias[0] = (float)accel_bias[0]*stThis->aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1]*stThis->aRes;
  abias[2] = (float)accel_bias[2]*stThis->aRes;

  c = xmReadByte(stThis, CTRL_REG0_XM);
  xmWriteByte(stThis, CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO
  //delay(20);
  wait(20);
  xmWriteByte(stThis, FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}

/* ************************************************************************** */
void LSM9DS0_readAccel(stLSM9DS0_t * stThis)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xmReadBytes(stThis, OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	stThis->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	stThis->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	stThis->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

/* ************************************************************************** */
void LSM9DS0_readMag(stLSM9DS0_t * stThis)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	xmReadBytes(stThis, OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	stThis->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	stThis->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	stThis->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

/* ************************************************************************** */
void LSM9DS0_readTemp(stLSM9DS0_t * stThis)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xmReadBytes(stThis, OUT_TEMP_L_XM, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L_M
	stThis->temperature = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

/* ************************************************************************** */
void LSM9DS0_readGyro(stLSM9DS0_t * stThis)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(stThis, OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	stThis->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	stThis->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	stThis->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

/* ************************************************************************** */
float LSM9DS0_calcGyro(stLSM9DS0_t * stThis, int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return stThis->gRes * gyro;
}

/* ************************************************************************** */
float LSM9DS0_calcAccel(stLSM9DS0_t * stThis, int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return stThis->aRes * accel;
}

/* ************************************************************************** */
float LSM9DS0_calcMag(stLSM9DS0_t * stThis, int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return stThis->mRes * mag;
}

/* ************************************************************************** */
void LSM9DS0_setGyroScale(stLSM9DS0_t * stThis, gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(stThis, CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(stThis, CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	stThis->gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes(stThis);
}

/* ************************************************************************** */
void LSM9DS0_setAccelScale(stLSM9DS0_t * stThis, accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(stThis, CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(stThis, CTRL_REG2_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	stThis->aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes(stThis);
}

/* ************************************************************************** */
void LSM9DS0_setMagScale(stLSM9DS0_t * stThis, mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(stThis, CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(stThis, CTRL_REG6_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	stThis->mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes(stThis);
}

/* ************************************************************************** */
void LSM9DS0_setGyroODR(stLSM9DS0_t * stThis, gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(stThis, CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(stThis, CTRL_REG1_G, temp);
}

/* ************************************************************************** */
void LSM9DS0_setAccelODR(stLSM9DS0_t * stThis, accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(stThis, CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(stThis, CTRL_REG1_XM, temp);
}

/* ************************************************************************** */
void LSM9DS0_setAccelABW(stLSM9DS0_t * stThis, accel_abw abwRate)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(stThis, CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 6);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 6);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(stThis, CTRL_REG2_XM, temp);
}

/* ************************************************************************** */
void LSM9DS0_setMagODR(stLSM9DS0_t * stThis, mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(stThis, CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(stThis, CTRL_REG5_XM, temp);
}

/* ************************************************************************** */
void LSM9DS0_configGyroInt( stLSM9DS0_t * stThis,
							uint8_t int1Cfg,
							uint16_t int1ThsX,
							uint16_t int1ThsY,
							uint16_t int1ThsZ,
							uint8_t duration)
{
	gWriteByte(stThis, INT1_CFG_G, int1Cfg);
	gWriteByte(stThis, INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(stThis, INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(stThis, INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(stThis, INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(stThis, INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(stThis, INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(stThis, INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(stThis, INT1_DURATION_G, 0x00);
}

/* ************************************************************************** */
static void calcgRes(stLSM9DS0_t * stThis)
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (stThis->gScale)
	{
	case G_SCALE_245DPS:
		stThis->gRes = 245.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		stThis->gRes = 500.0 / 32768.0;
		break;
	case G_SCALE_2000DPS:
		stThis->gRes = 2000.0 / 32768.0;
		break;
	}
}

/* ************************************************************************** */
static void calcaRes(stLSM9DS0_t * stThis)
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	stThis->aRes = stThis->aScale == A_SCALE_16G ? 16.0 / 32768.0 :
		   (((float) stThis->aScale + 1.0) * 2.0) / 32768.0;
}

/* ************************************************************************** */
static void calcmRes(stLSM9DS0_t * stThis)
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	stThis->mRes = stThis->mScale == M_SCALE_2GS ? 2.0 / 32768.0 :
	       (float) (stThis->mScale << 2) / 32768.0;
}

/* ************************************************************************** */
static void gWriteByte(stLSM9DS0_t * stThis, uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		stThis->write_byte(stThis, stThis->gAddress, subAddress, data);
	else if (stThis->interfaceMode == MODE_SPI)
		SPIwriteByte(stThis, stThis->gAddress, subAddress, data);
}

/* ************************************************************************** */
static void xmWriteByte(stLSM9DS0_t * stThis, uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		return stThis->write_byte(stThis, stThis->xmAddress, subAddress, data);
	else if (stThis->interfaceMode == MODE_SPI)
		return SPIwriteByte(stThis, stThis->xmAddress, subAddress, data);
}

/* ************************************************************************** */
static uint8_t gReadByte(stLSM9DS0_t * stThis, uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		return stThis->read_byte(stThis, stThis->gAddress, subAddress);
	else if (stThis->interfaceMode == MODE_SPI)
		return SPIreadByte(stThis, stThis->gAddress, subAddress);
}

/* ************************************************************************** */
static void gReadBytes(stLSM9DS0_t * stThis, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		stThis->read_bytes(stThis, stThis->gAddress, subAddress, dest, count);
	else if (stThis->interfaceMode == MODE_SPI)
		SPIreadBytes(stThis, stThis->gAddress, subAddress, dest, count);
}

/* ************************************************************************** */
static uint8_t xmReadByte(stLSM9DS0_t * stThis, uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		return stThis->read_byte(stThis, stThis->xmAddress, subAddress);
	else if (stThis->interfaceMode == MODE_SPI)
		return SPIreadByte(stThis, stThis->xmAddress, subAddress);
}

/* ************************************************************************** */
static void xmReadBytes(stLSM9DS0_t * stThis, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (stThis->interfaceMode == MODE_I2C)
		stThis->read_bytes(stThis, stThis->xmAddress, subAddress, dest, count);
	else if (stThis->interfaceMode == MODE_SPI)
		SPIreadBytes(stThis, stThis->xmAddress, subAddress, dest, count);
}

/* ************************************************************************** */
static void initSPI(stLSM9DS0_t * stThis)
{
#if 0
	pinMode(gAddress, OUTPUT);
	digitalWrite(gAddress, HIGH);
	pinMode(xmAddress, OUTPUT);
	digitalWrite(xmAddress, HIGH);
	
	SPI.begin();
	// Maximum SPI frequency is 10MHz, could divide by 2 here:
	SPI.setClockDivider(SPI_CLOCK_DIV4);
	// Data is read and written MSb first.
	SPI.setBitOrder(MSBFIRST);
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	SPI.setDataMode(SPI_MODE1);
#endif
}

/* ************************************************************************** */
static void SPIwriteByte(stLSM9DS0_t * stThis, uint8_t csPin, uint8_t subAddress, uint8_t data)
{
#if 0
	digitalWrite(csPin, LOW); // Initiate communication
	
	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	SPI.transfer(subAddress & 0x3F); // Send Address
	SPI.transfer(data); // Send data
	
	digitalWrite(csPin, HIGH); // Close communication
#endif
}

/* ************************************************************************** */
static uint8_t SPIreadByte(stLSM9DS0_t * stThis, uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte. 
	// Value is returned to `temp`.
	SPIreadBytes(stThis, csPin, subAddress, &temp, 1);
	return temp;
}

/* ************************************************************************** */
static void SPIreadBytes(stLSM9DS0_t * stThis, uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{
#if 0
	digitalWrite(csPin, LOW); // Initiate communication
	// To indicate a read, set bit 0 (msb) to 1
	// If we're reading multiple bytes, set bit 1 to 1
	// The remaining six bytes are the address to be read
	if (count > 1)
		SPI.transfer(0xC0 | (subAddress & 0x3F));
	else
		SPI.transfer(0x80 | (subAddress & 0x3F));
	for (int i=0; i<count; i++)
	{
		dest[i] = SPI.transfer(0x00); // Read into destination array
	}
	digitalWrite(csPin, HIGH); // Close communication
#endif
}

/* ************************************************************************** */
static void initI2C(stLSM9DS0_t * stThis)
{
	// I2C library initialisation already done!
}

static void wait( uint16_t millis )
{
	uint32_t tick;

	for ( tick = 0; tick < ( millis * 10000 ); tick++ );
}
