
/** ---------- TMP102 Library Test Code ------------------------

Controller is Teensy++ 2.0 but will also work with Arduino
We are running at 3.3V and therefore 8 MHz

Why the TMP102? The TMP102 is two-wire- and SMBus 
interface-compatible, and is specified over a 
temperature range of –40°C to +125°C.
TINY SOT563 PACKAGE
ACCURACY: 0.5°C (–25°C to +85°C) (a bit worse up to 125C)
12-bit direct to digital conversion
Alarm output
Up to four devices on one bus.
Reported value agrees to within 0.25 deg C with Fluke 80T-150U

Copyright 2013-2016 Systronix Inc www.systronix.com

NOTES ABOUT I2C library

Wire.endTransmission() seems to be only intended for use with a master write.
Wire.requestFrom() is used to get bytes from a slave, with read().
beginTransmission() followed by read() does not work. Slave address gets sent, then nothing else. As if the read commands get ignored.
I guess reads are not a "Transmission".

So to read three bytes, do a requestFrom(address, 2, true). << insists the first param is int.  
Compiler has major whines if called as shown in the online Wire reference.

**/
 
/** ---------- REVISIONS ----------

2016 Aug 21 wsk		tweaks in support of recent Systronix_TMP102 library rewrite
2013 Nov 30 bboyes: Working very simple library
2013 Nov 25 bboyes: First try of code to use TMP102 library
2013 Nov 11 bboyes: converting to Arduino library per http://arduino.cc/en/Hacking/LibraryTutorial
2013 Jul 20 bboyes: adding Ethernet support on Teensy++2 and Wiznet 812 on adapter. Simple web server works!
2013 Feb 13 bboyes: adding more methods to set mode and sample temperature
2013 Feb 11 bboyes: reducing output and changing format to allow for easy import into spreadsheet
2013 Feb 08 bboyes: 13-bit extended mode is working. Simulated data used to test entire range from +150C to -55C
            SD (shutdown) and OS (one-shot conversion) bits work in low power shutdown mode
2013 Feb 04 bboyes: Arduino 1.0.3 and Teensy 1v12 (not beta). got readRegister to work and discovered some surprising things 
            about the Arduino Wire library.
2013 Jan 16 bboyes: start
--------------------------------**/

#include <Arduino.h>
#include <Systronix_TMP102.h>	// best version of I2C library is #included by the library. Don't include it here!


//AKW0	KEYWORD0
//AKW1	KEYWORD1
//AKW2	KEYWORD2
//AKW3	KEYWORD3
//AKW4	KEYWORD4
//AKW5	KEYWORD5
//AKW6	KEYWORD6

 /**
 * debug level
 * 0 = quiet, suppress everything
 * 3 = max, even more or less trivial message are emitted
 * 4 = emit debug info which checks very basic data conversion, etc
 */
 byte DEBUG = 1;

uint16_t rawtemp;
uint16_t faketemp;

uint16_t dtime;  // delay in loop

uint16_t configOptions;

boolean fake;    // if true use simulated temperature data

/**
Data for one instance of a TMP102 temp sensor.
Extended 13-bit mode is assumed (12-bit mode is only there for compatibility with older parts)
Error counters could be larger but then they waste more data in the typical case where they are zero.
Errors peg at max value for the data type: they don't roll over.
**/
//struct data {
//  uint8_t address;    // I2C address, only the low 7 bits matter
//  uint16_t raw_temp;  // most recent
//  float deg_c;        
//  float deg_f;
//  uint16_t t_high;
//  uint16_t t_low;  
//  uint16_t i2c_err_nak;  // total since startup
//  uint16_t i2c_err_rd;   // total read fails - data not there when expected
//};

//static data tmp48;      // TMP102 at base address 0x48

float temperature = 0.0;

//Systronix_TMP102 tmp102_48(0x48);    // We can pass constructor a value
Systronix_TMP102 tmp102_48;    // We can pass constructor a value

/* ========== SETUP ========== */
void setup(void) 
{
  uint16_t raw16=0;  // place to put what we just read
//  uint16_t wrt16=0;  // temp write variable
  int8_t stat = -1;
//  int16_t temp_int16 = 0;
  
  delay (2000);      // give some time to open monitor window
  Serial.begin(115200);     // use max baud rate
  // Teensy3 doesn't reset with Serial Monitor as do Teensy2/++2, or wait for Serial Monitor window
  // Wait here for 10 seconds to see if we will use Serial Monitor, so output is not lost
  while((!Serial) && (millis()<10000));    // wait until serial monitor is open or timeout, which seems to fall through
  
  Serial.print("TMP102 Library Test Code at 0x");
  Serial.println(tmp102_48.BaseAddr, HEX);
   
//  int8_t flag = -1;  // I2C returns 0 if no error
  
  // Teensy++2 PE7/INT7/AIN1, pin 32 on module.
  // Teensy3 Digital 3, module pin 4
  // make it input so we can use it for ALERT
  pinMode(3, INPUT_PULLUP);  
  
  // start TMP102 library
	tmp102_48.setup(0x48);
  tmp102_48.begin();
  

  // start with default config
  Serial.print ("SetCFG=");
  Serial.print (TMP102_CFG_DEFAULT_WR, HEX);
  Serial.print (" ");
//  stat = tmp102_48.writeRegister(TMP102_CONF_REG_PTR, TMP102_CFG_DEFAULT_WR);
	stat = tmp102_48.init(TMP102_CFG_DEFAULT_WR);
//  if ( 0!= stat) Serial.print (" writeReg error! ");
  if (SUCCESS != stat) Serial.print (" writeReg error! ");
  stat = tmp102_48.readRegister (&raw16);
//  if ( 2!= stat) Serial.print (" readReg error! ");
  if (SUCCESS != stat) Serial.print (" readReg error! ");
  Serial.print("CFG:");
  Serial.print(raw16, HEX);
  Serial.print(" ");    
  
  configOptions = 0x0;  // 
  configOptions |= TMP102_CFG_EM;  // set Extended Mode
  configOptions |= TMP102_CFG_RATE_1HZ;  // 1Hz conversion
  configOptions |= TMP102_CFG_SD;        // sleep between conversions
  
  Serial.print ("SetCFG=");
  Serial.print (configOptions, HEX);
  Serial.print (" ");
  stat = tmp102_48.writeRegister(TMP102_CONF_REG_PTR, configOptions);
//  if ( 0!= stat) Serial.print (" writeReg error! ");
  if (SUCCESS != stat) Serial.print (" writeReg error! ");
  stat = tmp102_48.readRegister (&raw16);
//  if ( 2!= stat) Serial.print (" readReg error! ");
  if (SUCCESS != stat) Serial.print (" readReg error! ");
  Serial.print("CFGnow:");
  Serial.print(raw16, HEX);
  Serial.print(" ");  
  
  delay(30);    // 26 msec for conversion
  stat = tmp102_48.writePointer(TMP102_CONF_REG_PTR);
  stat = tmp102_48.readRegister (&raw16);
  Serial.print("CFG:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  stat = tmp102_48.writePointer(TMP102_TLOW_REG_PTR);
  stat = tmp102_48.readRegister (&raw16);
  Serial.print("Tlo:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  stat = tmp102_48.writePointer(TMP102_THIGH_REG_PTR);
  stat = tmp102_48.readRegister (&raw16);
  Serial.print("Thi:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  // leave the pointer set to read temperature
  stat = tmp102_48.writePointer(TMP102_TEMP_REG_PTR);
  
//  // fake temp  
//  fake = true;
//  dtime = 0;  // fast loop
//  faketemp = (uint16_t)0x4B00;  // max 13-bit value in raw 16-bit format
  
  // use real temperature data
  fake = false;      // switch to real data after full cycle of simulated
  dtime = 1000;      // msec between samples, 1000 = 1 sec, 60,000 = 1 minute
  Serial.print(" Interval is ");
  Serial.print(dtime/1000);
  Serial.print(" sec, ");
 
  Serial.println("Setup Complete!");
  Serial.println(" "); 
  
  if (1 == DEBUG)
  {
    Serial.println("sec deg C");
  }
}


uint16_t good=0;
uint16_t bad=0;
uint16_t raw16;

/* ========== LOOP ========== */
void loop(void) 
{
//  int16_t temp0;
  int8_t stat=-1;  // status flag
  float temp;
  
  Serial.print("@");
  Serial.print(millis()/1000);
  Serial.print(" ");
  
  if (!fake)  // get real temperature data from sensor
  {
  //  Serial.print("good:");
  //  Serial.print(good);
  //  Serial.print(" ");
    if (bad > 0) 
    {
      Serial.print(" bad:");
      Serial.print(bad);
      Serial.print(" ");
    }
  
    if (DEBUG >=3)
    {
      Serial.print("ALpin:");
      Serial.print(digitalRead(19));
      Serial.print(" ");
    }
  
    stat = tmp102_48.writePointer(TMP102_CONF_REG_PTR);
    if (DEBUG >=3)
    {
      stat = tmp102_48.readRegister (&raw16);
      Serial.print("CFG:");
      Serial.print(raw16, HEX);
      Serial.print(" ");
    }
  
    configOptions |= TMP102_CFG_OS;        // start One Shot conversion
    stat = tmp102_48.writeRegister (TMP102_CONF_REG_PTR, configOptions);
  
    if (DEBUG >=2)
    {  
      stat = tmp102_48.readRegister (&raw16);
      Serial.print("CFG:");
      Serial.print(raw16, HEX);
      Serial.print(" ");
    }
    // pointer set to read temperature
    stat = tmp102_48.writePointer(TMP102_TEMP_REG_PTR); 
    // read two bytes of temperature
    stat = tmp102_48.readRegister (&rawtemp);
//    if (2==stat) good++;
    if (SUCCESS==stat) good++;
    else bad++;
  } // if !fake 
  else rawtemp = faketemp;    // fresh simulated value
  
  if (DEBUG >= 2)
  {
    Serial.print ("Raw16:0x");
    if (0==(rawtemp & 0xF000)) Serial.print("0");
    Serial.print(rawtemp, HEX);
    Serial.print (" ");
  }

  temp = tmp102_48.raw13ToC(rawtemp);
  
  temperature = temp;  // for Ethernet client
  
  if (DEBUG >= 2)
  {
    Serial.print("ms13:0x");
    Serial.print(rawtemp, HEX);
    Serial.print (" ");
  }
  
  if (fake) Serial.print (temp, 4);  // no rounding of simulated data
  else Serial.print (temp, 4);       // 2 dec pts good enough for real data 0.0625 deg C per count
  
  Serial.print (" C ");
  
  if (DEBUG >= 2)
  {
    Serial.print(rawtemp, DEC);
    Serial.print ("D ");
  }
  
  // test with all values of simulated rawtemp data
  if (fake) 
  {
    // faketemp raw change of 0x280 is 5 deg C
    faketemp -= 0x280;  
    // 0xE480 is min legal value
    if (temp <= -55) 
    {
      faketemp = 0x4B00;  // if min then reset to max    
      
      fake = false;      // switch to real data after full cycle of simulated
      dtime = 2000;
      Serial.println();
      Serial.print ("Changing to real data");
    }
  }
  
  Serial.println();
  
  
  delay(dtime);
}



/**
Read the most current temperature already converted and present in the TMP102 temperature registers

In continuous mode, this could be one sample interval old
In one shot mode this data is from the last-requested one shot conversion
**/
uint8_t readTempDegC (float *tempC) 
{
    return 1;
}



/**
Convert deg C float to a raw 13-bit temp value in TMP102 format.
This is needed for Th and Tl registers as thermostat setpoint values

return 0 if OK, error codes if float is outside range of TMP102
**/
int8_t degCToRaw13 (uint16_t *raw13, float *tempC)
{
    return 1;
}



/**
Trigger a one-shot temperature conversion, wait for the new value, about 26 msec, and update 
the variable passed.

If the TMP102 is in continuous conversion mode, this places the part in One Shot mode, 
triggers the conversion, waits for the result, updates the variable, and leaves the TMP102 in one shot mode.

returns 0 if no error
**/
uint8_t getOneShotDegC (float *tempC)
{
    return 1;
}


/**
Set the TMP102 mode to one-shot, with low power sleep in between

mode: set to One Shot if true. 
If false, sets to continuous sampling mode at whatever sample rate was last set.

returns: 0 if successful
**/
int8_t setModeOneShot (boolean mode)
{
    return 1;
}

/**
Set TMP102 mode to continuous sampling at the rate given.

rate: must be one of the manifest constants such as TMP102_CFG_RATE_1HZ
if rate is not one of the four supported, it is set to the default 4 Hz

returns: 0 if successful
**/
int8_t setModeContinuous (int8_t rate)
{
    return 1;
}





