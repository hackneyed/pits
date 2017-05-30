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
#include <wiringPiSPI.h>
#include <inttypes.h>

#include "gps.h"
#include "misc.h"
#include "ads1015.h"

void *ADS1015ADCLoop(void *some_void_ptr)
{

	while(1)
	{
		sleep(5);
	}
	
	return NULL;
}
