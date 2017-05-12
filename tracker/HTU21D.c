#include <ctype.h>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>		// POSIX error

#include "wiringPi.h"
#include "wiringPiI2C.h"

#include "HTU21D.h"

#include <math.h>

#include "gps.h"

// Get temperature
double getTemperature(int fd)
{
	unsigned char buf [4];
	wiringPiI2CWrite(fd, HTU21D_TEMP);
	delay(100);
	read(fd, buf, 3);
	unsigned int temp = (buf [0] << 8 | buf [1]) & 0xFFFC;
	// Convert sensor reading into temperature.
	// See page 14 of the datasheet
	double tSensorTemp = temp / 65536.0;
	return -46.85 + (175.72 * tSensorTemp);
}

// Get humidity
double getHumidity(int fd)
{
	unsigned char buf [4];
	wiringPiI2CWrite(fd, HTU21D_HUMID);
	delay(100);
	read(fd, buf, 3);
  	unsigned int humid = (buf [0] << 8 | buf [1]) & 0xFFFC;
	// Convert sensor reading into humidity.
	// See page 15 of the datasheet
	double tSensorHumid = humid / 65536.0;
	return -6.0 + (125.0 * tSensorHumid);
}

void *HTU21Loop(void *some_void_ptr)
{
	struct TGPS *GPS;
	float Temp;
	float Hum;

	//telemetry object
	GPS = (struct TGPS *)some_void_ptr;

	while (1)
	{
		wiringPiSetup();
		int fd = wiringPiI2CSetup(HTU21D_I2C_ADDR);
		if ( 0 > fd )
		{
			fprintf (stderr, "Unable to open I2C device: %s\n", strerror (errno));
			exit (-1);
		}

		Temp = (float) getTemperature(fd);
		Hum = (float) getHumidity(fd);

		printf("%5.2fC\n", Temp);
		printf("%5.2f%%rh\n", Hum);

		GPS->HTU21DTemperature = Temp;
		GPS->ExternalHumidity = Hum;

		sleep(5);

	}


}
