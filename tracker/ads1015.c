#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <inttypes.h>

#include "gps.h"
#include "misc.h"
#include "ads1015.h"

/**************************************************************************/
/*!
    @file     ads1015.c
    @author   J. Hackney
    @license  BSD (see license.txt)

    Driver for the ADS1015/ADS1115 ADC
    
    Original driver in C++ by K. Townsend (Adafruit Industries)
    * Using example code by R. Reignier

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
    v2.0 - pcDuino version - R. Reignier
    v2.1 - pcDuino version with adjustable SPS - R. Reignier
    v3.0 - ported to C for use with PiInTheSky - J. Hackney
 
*/
/**************************************************************************/

#define I2C_DEVICE "/dev/i2c-1"


/**************************************************************************/

struct TADS 
{
  // i2c file handle
  int 		m_i2cHandle;
  
  // Instance-specific properties
  uint8_t   m_i2cAddress;
  uint8_t   m_bitShift;
  uint8_t   m_adsType;
  uint32_t  m_conversionDelay;
  adsGain_t m_gain;
  adsSps_t  m_sps;
}; 


void ADS1015_setConversionDelay(struct TADS *tads);


/**************************************************************************/
/*!
    @brief Init the i2c communication 
*/
/**************************************************************************/
static void ADS1015_beginTransmission(struct TADS *tads) 
{
  // Create the file descriptor for the i2c bus
  tads->m_i2cHandle = open(I2C_DEVICE, O_RDWR);
  if(tads->m_i2cHandle < 0)
  {
    fprintf(stderr, "Error while opening the %s device! Error: %s\n", I2C_DEVICE, strerror(errno));
  }
  // Set the slave address
  if(ioctl(tads->m_i2cHandle, I2C_SLAVE, tads->m_i2cAddress) < 0)
  {
    fprintf(stderr, "Error while configuring the slave address %d. Error: %s\n", tads->m_i2cAddress, strerror(errno));
  }
}

/**************************************************************************/
/*!
    @brief End the i2c communication 
*/
/**************************************************************************/
static void ADS1015_endTransmission(struct TADS *tads) {
  close(tads->m_i2cHandle);
}

/*************************************************************************/
/*!
	@brief initializes an ADS1015 structure
*/
/*************************************************************************/
 
void ADS1015_init(struct TADS *tads, uint8_t i2cAddress) 
{
  tads->m_i2cAddress = i2cAddress;
  tads->m_adsType = ADS1015;
  tads->m_bitShift = 4;
  tads->m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  tads->m_sps  = SPS_1600;
  ADS1015_setConversionDelay(tads);
  
  // test communication with bus
  ADS1015_beginTransmission(tads);
  ADS1015_endTransmission(tads);
}



/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register
*/
/**************************************************************************/
static void ADS1015_writeRegister(struct TADS *tads, uint8_t reg, uint16_t value) 
{
  ADS1015_beginTransmission(tads);
  uint8_t lsb = (uint8_t)(value >> 8);
  uint8_t msb = (uint8_t)value;
  uint16_t payload = (msb << 8) | lsb; 
  i2c_smbus_write_word_data(tads->m_i2cHandle, reg, payload);
  ADS1015_endTransmission(tads);
}

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register
*/
/**************************************************************************/
static uint16_t ADS1015_readRegister(struct TADS *tads, uint8_t reg) 
{
  ADS1015_beginTransmission(tads);
  uint16_t res = i2c_smbus_read_word_data(tads->m_i2cHandle, reg);
  ADS1015_endTransmission(tads);
  uint8_t msb = (uint8_t)res;
  uint8_t lsb = (uint8_t)(res >> 8);
  return (msb << 8) | lsb;
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range
*/
/**************************************************************************/
void ADS1015_setGain(struct TADS *tads, adsGain_t gain)
{
  tads->m_gain = gain;
}

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range
*/
/**************************************************************************/
adsGain_t ADS1015_getGain(struct TADS *tads)
{
  return tads->m_gain;
}

/**************************************************************************/
/*!
    @brief  Sets the sample rate in SPS (samples per second)
*/
/**************************************************************************/
void ADS1015_setSps(struct TADS *tads, adsSps_t sps)
{
  tads->m_sps = sps;
  ADS1015_setConversionDelay(tads);
}

/**************************************************************************/
/*!
    @brief  Gets the sample rate in SPS (samples per second)
*/
/**************************************************************************/
adsSps_t ADS1015_getSps(struct TADS *tads)
{
  return tads->m_sps;
}

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel
*/
/**************************************************************************/
uint16_t ADS1015_readADC_SingleEnded(struct TADS *tads, uint8_t channel) 
{
  if (channel > 3)
  {
    return 0;
  }
  
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= tads->m_gain;

  // Set the sample rate
  config |= tads->m_sps;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  ADS1015_writeRegister(tads, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(tads->m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return ADS1015_readRegister(tads, ADS1015_REG_POINTER_CONVERT) >> tads->m_bitShift;  
}

/**************************************************************************/
/*! 
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************/
int16_t ADS1015_readADC_Differential_0_1(struct TADS *tads) 
{
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= tads->m_gain;
                    
  // Set the sample rate
  config |= tads->m_sps;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  ADS1015_writeRegister(tads, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(tads->m_conversionDelay);

  // Read the conversion results
  uint16_t res = ADS1015_readRegister(tads, ADS1015_REG_POINTER_CONVERT) >> tads->m_bitShift;
  if (tads->m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}


/**************************************************************************/
/*! 
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************/
int16_t ADS1015_readADC_Differential_2_3(struct TADS *tads) 
{
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= tads->m_gain;

  // Set the sample rate
  config |= tads->m_sps;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  ADS1015_writeRegister(tads, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(tads->m_conversionDelay);

  // Read the conversion results
  uint16_t res = ADS1015_readRegister(tads, ADS1015_REG_POINTER_CONVERT) >> tads->m_bitShift;
  if (tads->m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.
*/
/**************************************************************************/
void ADS1015_startComparator_SingleEnded(struct TADS *tads, uint8_t channel, int16_t threshold)
{
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= tads->m_gain;
                    
  // Set the sample rate
  config |= tads->m_sps;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  ADS1015_writeRegister(tads, ADS1015_REG_POINTER_HITHRESH, threshold << tads->m_bitShift);

  // Write config register to the ADC
  ADS1015_writeRegister(tads, ADS1015_REG_POINTER_CONFIG, config);
}


/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
*/
/**************************************************************************/
int16_t ADS1015_getLastConversionResults(struct TADS *tads)
{
  // Wait for the conversion to complete
  usleep(tads->m_conversionDelay);

  // Read the conversion results
  uint16_t res = ADS1015_readRegister(tads, ADS1015_REG_POINTER_CONVERT) >> tads->m_bitShift;
  if (tads->m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief Compute the needed delay in microseconds for the conversion
*/
/**************************************************************************/
void ADS1015_setConversionDelay (struct TADS *tads)
{
  if(tads->m_adsType == ADS1015)
  {
    switch(tads->m_sps)
    {
      case SPS_128:
        tads->m_conversionDelay = 1000000 / 128;
        break;
      case SPS_250:
        tads->m_conversionDelay = 1000000 / 250;
        break;
      case SPS_490:
        tads->m_conversionDelay = 1000000 / 490;
        break;
      case SPS_920:
        tads->m_conversionDelay = 1000000 / 920;
        break;
      case SPS_1600:
        tads->m_conversionDelay = 1000000 / 1600;
        break;
      case SPS_2400:
        tads->m_conversionDelay = 1000000 / 2400;
        break;
      case SPS_3300:
        tads->m_conversionDelay = 1000000 / 3300;
        break;
      case SPS_860:
        tads->m_conversionDelay = 1000000 / 3300;
        break;
      default:
        tads->m_conversionDelay = 8000;
        break;
    }
  }
  else
  {
    switch(tads->m_sps)
    {
      case SPS_128:
        tads->m_conversionDelay = 1000000 / 8;
        break;
      case SPS_250:
        tads->m_conversionDelay = 1000000 / 16;
        break;
      case SPS_490:
        tads->m_conversionDelay = 1000000 / 32;
        break;
      case SPS_920:
        tads->m_conversionDelay = 1000000 / 64;
        break;
      case SPS_1600:
        tads->m_conversionDelay = 1000000 / 128;
        break;
      case SPS_2400:
        tads->m_conversionDelay = 1000000 / 250;
        break;
      case SPS_3300:
        tads->m_conversionDelay = 1000000 / 475;
        break;
      case SPS_860:
        tads->m_conversionDelay = 1000000 / 860;
        break;
      default:
        tads->m_conversionDelay = 125000;
        break;
    }
  }
  tads->m_conversionDelay += 100; // Add 100 us to be safe
}

/*************************************************************************/
/*!
 * @brief Returns the multiplier to use to convert ADC counts to a voltage
 */
/*************************************************************************/
float ADS1015_getMultiplier(struct TADS *tads)
{
  //                                                   ADS1015  ADS1115
  //                                                   -------  -------
  // GAIN_TWOTHIRDS;  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // GAIN_ONE;        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // GAIN_TWO;        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // GAIN_FOUR;       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // GAIN_EIGHT;      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // GAIN_SIXTEEN;    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
	
	switch(ADS1015_getGain(tads))
	{
		case GAIN_TWOTHIRDS:
			return 3.0F;
		case GAIN_ONE:
			return 2.0F;
		case GAIN_TWO:
			return 1.0F;
		case GAIN_FOUR:
			return 0.5F;
		case GAIN_EIGHT:
			return 0.25F;
		case GAIN_SIXTEEN:
			return 0.125F;
		default:
			printf("ADS1015 not initialized! Multiplier = 0\n");
			return 0;
	}

}


/*************************************************************************/
/*!
 * 
 */
/*************************************************************************/

void *ADS1015ADCLoop(void *some_void_ptr)
{
	struct TADS tads;
	struct TGPS *GPS;
	
	float ch0, ch1, ch2_3;

	GPS = (struct TGPS *)some_void_ptr;
	
	// initialize the structure and make sure sensor is available
	ADS1015_init(&tads, ADS1015_ADDRESS);
	
	while(tads.m_i2cHandle >= 0)
	{
		// on custom board single-ended resistor dividers are 49.9k (divide by 2)
		// VSENSE resistor = R200
		// input voltages:
		// ch0 - ADCVIN - gain 2/3 (max input V = 12.288V)
		// ch1 - ADCAUX - gain 1 (max input V = 8.192V)
		// ch2_3 - ADC_VSENSE+/- - gain 16 (max v_diff +/-0.256V)
// TODO: make resistor dividers/multipliers configurable

		ADS1015_beginTransmission(&tads);
		
		//read ch0
		ADS1015_setGain(&tads, GAIN_TWOTHIRDS);
		ch0 = ADS1015_readADC_SingleEnded(&tads, 0);
		//~ printf("ch0 = %5.2f\n", ch0);
		ch0 *= ADS1015_getMultiplier(&tads);
		GPS->BatteryVoltage = (ch0 * 2) / 1000; //multipliers return mV, need V
		
		printf("Battery Voltage: %5.2fV\n", GPS->BatteryVoltage);
		
		//read ch1
		ADS1015_setGain(&tads, GAIN_ONE);
		ch1 = ADS1015_readADC_SingleEnded(&tads, 1);
		//~ printf("ch1 = %5.2f\n", ch1);
		ch1 *= ADS1015_getMultiplier(&tads);
		GPS->AuxVoltage = (ch1 * 2) / 1000; //multipliers return mV, need V
		
		printf("Aux Voltage: %5.2fV\n", GPS->AuxVoltage);
		
		//read ch2-3 differential
		ADS1015_setGain(&tads, GAIN_SIXTEEN);
		ch2_3 = ADS1015_readADC_Differential_2_3(&tads) * ADS1015_getMultiplier(&tads);
		GPS->BoardCurrent = ch2_3 / 0.200F;
		
		printf("Board Current: %5.3fmA\n", GPS->BoardCurrent);
		
		ADS1015_endTransmission(&tads);
		
		sleep(10);
		
	}
	
	return NULL;
}
