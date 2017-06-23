/*
 * Systronix_TMP102.cpp
 *
 *  Created on: Nov 15, 2013
 *	  Author: BAB
 */

#include <Systronix_TMP102.h>	

//---------------------------< S E T U P >--------------------------------------------------------------------
/*!
	@brief  Instantiates a new TMP102 class to use the given base address
	@todo	Test base address for legal range 0x48-0x4B
			Add constructor(void) for default address of 0x48
*/

uint8_t Systronix_TMP102::setup(uint8_t base)
	{
	if ((TMP102_BASE_MIN > base) || (TMP102_BASE_MAX < base))
		{
		tally_transaction (SILLY_PROGRAMMER);
		return FAIL;
		}

	_base = base;
	return SUCCESS;
	}


//---------------------------< B E G I N >--------------------------------------------------------------------
//
// TODO: add support for Wire1, Wire2, ... alternate pins, etc
//

void Systronix_TMP102::begin(void)
	{
	Wire.begin();	// join I2C as master
	}


//---------------------------< B A S E _ G E T >--------------------------------------------------------------
//
//	return the I2C base address for this instance
//

uint8_t Systronix_TMP102::base_get(void)
	{
	return _base;
	}


//---------------------------< I N I T >----------------------------------------------------------------------
//
// Attempts to write configuration register.  If successful, sets error.exists true, else false.
//

uint8_t Systronix_TMP102::init (uint16_t config)
	{
	error.exists = true;								// here, assume that the device exists
	if (SUCCESS != register_write (TMP102_CONF_REG_PTR, config))
		{
		error.exists = false;							// only place in this file where this is set false
		return ABSENT;
		}
	return SUCCESS;
	}


//---------------------------< T A L L Y _ E R R O R S >------------------------------------------------------
//
// Here we tally errors.  This does not answer the 'what to do in the event of these errors' question; it just
// counts them.
//

void Systronix_TMP102::tally_transaction (uint8_t value)
	{
	if (value && (error.total_error_count < UINT64_MAX))
		error.total_error_count++; 			// every time here incr total error count

	error.error_val = value;

	switch (value)
		{
		case SUCCESS:
			if (error.successful_count < UINT64_MAX)
				error.successful_count++;
			break;
		case 1:								// i2c_t3 and Wire: data too long from endTransmission() (rx/tx buffers are 259 bytes - slave addr + 2 cmd bytes + 256 data)
			error.data_len_error_count++;
			break;
#if defined I2C_T3_H
		case I2C_TIMEOUT:
			error.timeout_count++;			// 4 from i2c_t3; timeout from call to status() (read)
#else
		case 4:
			error.other_error_count++;		// i2c_t3 and Wire: from endTransmission() "other error"
#endif
			break;
		case 2:								// i2c_t3 and Wire: from endTransmission()
		case I2C_ADDR_NAK:					// 5 from i2c_t3
			error.rcv_addr_nack_count++;
			break;
		case 3:								// i2c_t3 and Wire: from endTransmission()
		case I2C_DATA_NAK:					// 6 from i2c_t3
			error.rcv_data_nack_count++;
			break;
		case I2C_ARB_LOST:					// 7 from i2c_t3; arbitration lost from call to status() (read)
			error.arbitration_lost_count++;
			break;
		case I2C_BUF_OVF:
			error.buffer_overflow_count++;
			break;
		case I2C_SLAVE_TX:
		case I2C_SLAVE_RX:
			error.other_error_count++;		// 9 & 10 from i2c_t3; these are not errors, I think
			break;
		case WR_INCOMPLETE:					// 11; Wire.write failed to write all of the data to tx_buffer
			error.incomplete_write_count++;
			break;
		case SILLY_PROGRAMMER:				// 12
			error.silly_programmer_error++;
			break;
		default:
			error.unknown_error_count++;
			break;
		}
	}


//---------------------------< R A W 1 3 T O C >--------------------------------------------------------------
/*!
	@brief  Convert raw 13-bit temperature to float deg C
			handles neg and positive values specific to TMP102 extended mode 
			13-bit temperature data

	@TODO instead pass a pointer to the float variable? and return error if value out of bounds
*/
//
// receives uint16_t raw13 argument that should be an int16_t.  But, because readRegister() is a general
// purpose function for reading all of the 16 bite registers we get around that by casting the uint16_t to
// int16_t before we use the value.
//

float Systronix_TMP102::raw13ToC (uint16_t raw13)
	{
	uint8_t		shift = (raw13 & 1) ? 3 : 4;			// if extended mode shift 3, else shift 4
	return 0.0625 * ((int16_t)raw13 >> shift);
	}


//---------------------------< R A W 1 3 _ T O _ F >----------------------------------------------------------
//
// Convert raw 13-bit TMP102 temperature to degrees Fahrenheit.
//

float Systronix_TMP102::raw13_to_F (uint16_t raw13)
	{
	return (raw13ToC (raw13) * 1.8) + 32.0;
	}


//---------------------------< G E T _ T E M P E R A T U R E _ D A T A >--------------------------------------
//
// Gets current temperature and fills the data struct with the various temperature info
//

uint8_t Systronix_TMP102::get_temperature_data (void)
	{
	uint8_t ret_val;
	
	if (TMP102_TEMP_REG_PTR != _pointer_reg)			// if not pointed at temperature register
		{
		ret_val = pointer_write (TMP102_TEMP_REG_PTR);	// attempt to point it
		if (SUCCESS != ret_val)
			return ret_val;								// attempt failed; quit
		}
	
	ret_val = register_read (&data.raw_temp);			// attempt to read the temperature
	if (SUCCESS != ret_val)
		return ret_val;									// attempt failed; quit
	
	data.t_high = max((int16_t)data.raw_temp, (int16_t)data.t_high);	// keep track of min/max temperatures
	data.t_low = min((int16_t)data.t_low, (int16_t)data.raw_temp);

	data.deg_c = raw13ToC (data.raw_temp);				// convert to human-readable forms
	data.deg_f = raw13_to_F (data.raw_temp);
	
	return SUCCESS;
	}


//---------------------------< P O I N T E R _ W R I T E >----------------------------------------------------
/**
Write to the TMP102 pointer register

The last value written to the Pointer Register persists until changed.

**/

uint8_t Systronix_TMP102::pointer_write (uint8_t target_register)
	{
	uint8_t ret_val;

	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (target_register > TMP102_THIGH_REG_PTR)
		tally_transaction (SILLY_PROGRAMMER);		// upper bits not fatal. We think.

	Wire.beginTransmission (_base);
	ret_val = Wire.write (target_register);			// returns # of bytes written to i2c_t3 buffer
	if (1 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);			// only here we make 0 an error value
		return FAIL;
		}

	ret_val = Wire.endTransmission ();				// endTransmission() returns 0 if successful
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);				// increment the appropriate counter
		return FAIL;								// calling function decides what to do with the error
		}

	_pointer_reg = target_register;					// remember where the pointer register is pointing

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< R E G I S T E R _ W R I T E >--------------------------------------------------
/**
Param target_register is the TMP102 register where 'data' shall be written
data is the 16 bits to write.
returns 0 if no error
**/

uint8_t Systronix_TMP102::register_write (uint8_t target_register, uint16_t data)
	{
	uint8_t ret_val;								// number of bytes written

	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if ((TMP102_TEMP_REG_PTR == target_register) || (TMP102_THIGH_REG_PTR < target_register))
		tally_transaction (SILLY_PROGRAMMER);		// upper bits not fatal. We think; only write to writable registers

	Wire.beginTransmission (_base);					// base address
	ret_val = Wire.write (target_register);			// pointer in 2 lsb
	ret_val += Wire.write ((uint8_t)(data >> 8));	// write MSB of data
	ret_val += Wire.write ((uint8_t)(data & 0x00FF));	// write LSB of data

	if (3 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);			// increment the appropriate counter
		return FAIL;
		}
	
	ret_val = Wire.endTransmission ();				// endTransmission() returns 0 if successful
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);				// increment the appropriate counter
		return FAIL;								// calling function decides what to do with the error
		}

	_pointer_reg = target_register;					// remember where the pointer register is pointing

	if (TMP102_CONF_REG_PTR == target_register)		// and remember the data we wrote to the register
		_config_reg = data;
	else if (TMP102_TLOW_REG_PTR == target_register)
		_tlow_reg = data;
	else if (TMP102_THIGH_REG_PTR == target_register)
		_thigh_reg = data;

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< R E G I S T E R _ R E A D >----------------------------------------------------
/**
  Read the 16-bit register addressed by the current pointer value, store the data at the location passed
  
  return 0 if no error.
*/

uint8_t Systronix_TMP102::register_read (uint16_t *data)
	{
	uint8_t ret_val;
	
	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (2 != Wire.requestFrom(_base, 2, I2C_STOP))
		{
		ret_val = Wire.status();					// to get error value
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;
		}

	*data = (uint16_t)Wire.read() << 8;
	*data |= (uint16_t)Wire.read();
	return SUCCESS;
	}


//---------------------------< R E A D T E M P D E G C >------------------------------------------------------
/**
Read the most current temperature already converted and present in the TMP102 temperature registers

In continuous mode, this could be one sample interval old
In one shot mode this data is from the last-requested one shot conversion
**/
uint8_t Systronix_TMP102::readTempDegC (float *tempC) 
	{
	return FAIL;
	}



//---------------------------< D E G C T O R A W 1 3 >--------------------------------------------------------
/**
Convert deg C float to a raw 13-bit temp value in TMP102 format.
This is needed for Th and Tl registers as thermostat setpoint values

return 0 if OK, error codes if float is outside range of TMP102
**/

uint8_t Systronix_TMP102::degCToRaw13 (uint16_t *raw13, float *tempC)
	{
	return FAIL;
	}


//---------------------------< G E T O N E S H O T D E G C>---------------------------------------------------
/**
Trigger a one-shot temperature conversion, wait for the new value, about 26 msec, and update 
the variable passed.

If the TMP102 is in continuous conversion mode, this places the part in One Shot mode, 
triggers the conversion, waits for the result, updates the variable, and leaves the TMP102 in one shot mode.

returns 0 if no error
**/

uint8_t Systronix_TMP102::getOneShotDegC (float *tempC)
	{
	return FAIL;
	}


//---------------------------< S E T M O D E O N E S H O T >--------------------------------------------------
/**
Set the TMP102 mode to one-shot, with low power sleep in between

mode: set to One Shot if true. 
If false, sets to continuous sampling mode at whatever sample rate was last set.

returns: 0 if successful
**/

uint8_t Systronix_TMP102::setModeOneShot (boolean mode)
	{
	return FAIL;
	}


//---------------------------< S E T M O D E C O N T I N U O U S >--------------------------------------------
/**
Set TMP102 mode to continuous sampling at the rate given.

rate: must be one of the manifest constants such as TMP102_CFG_RATE_1HZ
if rate is not one of the four supported, it is set to the default 4 Hz

returns: 0 if successful
**/

uint8_t Systronix_TMP102::setModeContinuous (int8_t rate)
	{
	return FAIL;
	}
