#ifndef SYSTRONIX_TMP102_h
#define SYSTRONIX_TMP102_h


/******************************************************************************/
/*!
	@file		Systronix_TMP102.h
	
	@author		B Boyes (Systronix Inc)
    @license	BSD (see license.txt)	
    @section	HISTORY

	v1.00	2016May15 bboyes, added conditional comp for use of 
			i2C_t3 vs Wire on Teensy 3.x
	v0.2	2013Nov30 bboyes Changed pointer register manifest constants
			TMP102_REG_XXXX_REG to TMP102_XXXX_REG_PTR
    v0.1	2013Nov15 bboyes Start based on working Arduino program

*/
/******************************************************************************/

/** TODO LIST

Function to get current temp in degrees C TMP102_GetTempC (uint16_t *data16)
Change constants and add a function to config so it can be something like
TMP102_Setup(TMP102_ONE_SHOT) or (TMP102_4HZ)

**/

/** --------  Description ------------------------------------------------------

See the data sheet at http://www.ti.com/product/tmp102

Max operating temperature range is –55 to +150 °C, but to read this requires 
the non-default, 13-bit data format. The "specified range" in the data sheet
is -40 to 125 C, so the extended range accuracy is not guaranteed.
Accuracy from –25°C to +85°C is 0.5/2°C typ/max.
Accuracy from –40°C to +125°C is 1/3°C typ/max.
Accuracy from –55°C to +150°C is not specified. I plan to compare to a thermocouple
and see what some typical accuracies are.

One lsb is 0.0625°C in both 12- and 13-bit formats. The 13-bit mode just gives 
you a greater range over which you can use the sensor.
Negative numbers are binary twos complement format.

Aren't there other TMP102 libs? Yes, sort of:
There are other TMP102 Arduino libs out there but all those I found were very 
limited. I needed the full 13-bit data and temperature range support, multiple
devices per I2C net and so forth.

------------------------------------------------------------------------------*/

/** -------- Arduino Device and Version Testing and Issues ---------------------

This library assumes Arduino greater than 1.00, with Wire.read() and Wire.write()

This library was developed and tested on Teensy2, Teensy++2 (with AVR controllers)
and Teensy3 (ARM Cortex-M4??? is that correct?).

------------------------------------------------------------------------------*/


/** --------  Device Addressing --------
TMP102 base address is 0x48 (B 1001 000x) where x is R/W

This is confusing because the address part is just the the upper 7 bits, so if considered
alone, and right-justified, that is  B 0100 1000 which is 0x48 which is how Arduino wants it.
if ADDR is GND, address is 0x48
if ADDR is VDD, address is 0x49
if ADDR is SDA, address is 0x4A
if ADDR is SCL, address is 0x4B
-------------------------------------*/

#include <Arduino.h>

#if defined (__MK20DX256__) || defined (__MK20DX128__) 	// Teensy 3.1 or 3.2 || Teensy 3.0
#include <i2c_t3.h>		
#else
#include <Wire.h>	// for AVR I2C library
#endif

#define		SUCCESS	0
#define		FAIL	(~SUCCESS)
#define		ABSENT	0xFD

/** --------  Register Addresses --------
The two lsb of the pointer register hold the register bits, which
are used to address one of the four directly-accessible registers.

There are four pointer addresses:
00  Temperature Register (Read Only) 12-13 bits in ms position
	bit 0 in LS byte is 1 if in 13-bit 'extended mode' (EM)
	If temp is positive (msb=0) the value is just the binary value.
	If temp is negative (msb=0) the value is in 2's complement form,
	so take the whole binary value, complement it, and add 1.
01  Configuration Register (Read/Write) 16 bits, MSB first
10  TLOW Limit Register (Read/Write)
11  THIGH Limit Register (Read/Write)
*/

#define TMP102_TEMP_REG_PTR 0x00  // 16-bit temperature register, read only
#define TMP102_CONF_REG_PTR 0x01  // 16-bit config register, read/write
#define TMP102_TLOW_REG_PTR 0x02  // 16-bit Tlow register, read/write
#define TMP102_THIGH_REG_PTR 0x03  // 16-bit Thigh register, read/write

/** --------  Configuration --------
OR these bits together appropriately, one choice from each option group
and write to the config register for the desired option
This is a a 16-bit register, bits 15..0 but bits 3..0 are not used and read as zeros
I have used the same designation as the data sheet, to be consistent,
instead of making names longer and perhaps more readable;
for example TMP102_CFG_AL = 'AL', the Alert config bit
*/

/*
  0x60A0 is the POR default read value of the configuration register.
  12-bit resolution (but these bits 14,13 are read only)
  continuous conversions at 4Hz rate
  Alert bit = 1  (but this bit 5 is read only)
  So writing to only the bits of this default value that are writeable,
  we could write 0x0080, which would still read back as 0x60A0
*/
#define TMP102_CFG_DEFAULT_RD 0x60A0
#define TMP102_CFG_DEFAULT_WR 0x0080

/* One-shot/Conversion Ready is Config bit 15
  When in shutdown mode (SD=1), setting OS starts a single conversion
  during which OS reads '0'. When conversion is complete OS = 1.
  A single conversion typically takes 26ms and a read can take
  place in less than 20ms. When using One-Shot mode,
  30 or more conversions per second are possible.
  Default is OS = 0.
*/
#define TMP102_CFG_OS 0x08000  // Start a single conversion (if in SD mode)

// Resolution is Config bits 14,13
// read-only bits, '11' = 12-bit resolution
#define TMP102_CFG_RES 0x6000  // if both bits set, 12-bit mode *default*

// Fault Queue Config bits 12,11
// how many faults generate an Alert based on T-high and T-low registers
#define TMP102_CFG_FLTQ_6 0x01800  // 6 consecutive faults
#define TMP102_CFG_FLTQ_4 0x01000  // 4 consecutive faults
#define TMP102_CFG_FLTQ_2 0x00800  // 2 consecutive faults
#define TMP102_CFG_FLTQ_1 0x00000  // 1 consecutive fault *default*

// Polarity bit is Config bit 10, adjusts polarity of Alert pin output
// POL=0 means Alert is active LOW
// POL=1 means Alert is active HIGH
#define TMP102_CFG_POL 0x0400  // 0 = Alert active LOW *default*

// TM Thermostat Mode is Config bit 9
// Tells TMP102 to operate in Comparator (TM=0) or Interrupt mode (TM=1)
#define TMP102_CFG_TM 0x0200  // 0 = Comparator mode *default*

// Shutdown mode Config bit 8
// Set this bit to be low power (0.5 uA) between single conversions
// In SD mode the conversion rate bits are ignored, and you have
// to set the OS (one shot) bit to start a single conversion.
#define TMP102_CFG_SD 0x0100  // 0 = continuous conversion state *default*

// Conversion rate is Config bits 7,6
#define TMP102_CFG_RATE_8HZ 0x00C0  // 8 Hz conversion rate
#define TMP102_CFG_RATE_4HZ 0x0080  // 4 Hz conversion rate *default*
#define TMP102_CFG_RATE_1HZ 0x0040  // 1 Hz conversion rate
#define TMP102_CFG_RATE_QHZ 0x0000  // 0.25 Hz conversion rate (Q = 1/4)

// Alert is Config bit 5, read-only, it shows comparator status
// POL bit inverts the alarm polarity
#define TMP102_CFG_AL 0x0020  // I don't fully understand this bit. Page 7 in data sheet

// Extended mode is bit 4, set for 13-bit to read temps above 128C
#define TMP102_CFG_EM 0x0010  // 0 = 12-bit mode *default*

class Systronix_TMP102
{
	protected:
		uint8_t		_base;								// base address for this instance; four possible values
		uint8_t		_pointer_reg;						// copy of the pointer register value so we know where it's pointing
		void		tally_errors (uint8_t);				// maintains the i2c_t3 error counters

	public:
		// Instance-specific properties
		/** Data for one instance of a TMP102 temp sensor.
		Extended 13-bit mode is assumed (12-bit mode is only there for compatibility with older parts)
		Error counters could be larger but then they waste more data in the typical case where they are zero.
		Errors peg at max value for the data type: they don't roll over.
		
		Maybe different structs for data values and part control
		**/
		struct
			{
			uint16_t	raw_temp;						// most recent
			uint16_t	t_high = 0xE480;				// preset to minimum temperature value (-55C in raw13 format)
			uint16_t	t_low = 0x4B00;  				// preset to max temperature value (150C in raw13 format)
			float		deg_c;        
			float		deg_f;
			bool		fresh;							// data is good and fresh TODO: how does one know that the data are not 'fresh'?
			} data;

		struct
			{
			uint8_t		ret_val;						// i2c_t3 library return value from most recent transaction
			uint32_t	incomplete_write_count;			// Wire.write failed to write all of the data to tx_buffer
			uint32_t	data_len_error_count;			// data too long
			uint32_t	rcv_addr_nack_count;			// slave did not ack address
			uint32_t	rcv_data_nack_count;			// slave did not ack data
			uint32_t	other_error_count;				// arbitration lost or timeout
			boolean		exists;							// set false after an unsuccessful i2c transaction
			} control;

		uint8_t BaseAddr;
														// i2c_t3 error counters

		void		setup (uint8_t base);				// constructor
		void		begin (void);
		uint8_t		init (uint16_t);					// device present and communicating detector

		float		raw13ToC (uint16_t raw13);			// temperature conversion functions
		float		raw13_to_F (uint16_t raw13);
		uint8_t		get_temperature_data (void);

		uint8_t		writePointer (uint8_t pointer);		// i2c bus dependent functions
		uint8_t		writeRegister (uint8_t pointer, uint16_t data);
		uint8_t		readRegister (uint16_t *data);

		uint8_t		readTempDegC (float *tempC);		// these functions not yet implemented; all return FAIL
		uint8_t		degCToRaw13 (uint16_t *raw13, float *tempC);
		uint8_t		getOneShotDegC (float *tempC);
		uint8_t		setModeOneShot (boolean mode);
		uint8_t		setModeContinuous (int8_t rate);

	private:

};

extern Systronix_TMP102 tmp102;

#endif /* SYSTRONIX_TMP102_h */
