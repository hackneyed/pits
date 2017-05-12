// Types

struct TGPS
{
	// long Time;						// Time as read from GPS, as an integer but 12:13:14 is 121314
	long SecondsInDay;					// Time in seconds since midnight
	int Hours, Minutes, Seconds;
	float Longitude, Latitude;
	int32_t Altitude, MaximumAltitude;
	float AscentRate;
	unsigned int Satellites;
	int Speed;
	int Direction;
	float HTU21DTemperature;
	float ExternalHumidity;

	float BatteryVoltage;
	float BMP180Temperature;
	float Pressure;
	float BME280Humidity;
	float BoardCurrent;
	//int DS18B20Count;
	float PredictedLongitude, PredictedLatitude;
	int FlightMode;
	int PowerMode;
	int Lock;
	unsigned int MessageCount;
} GPS;


// functions

void *GPSLoop(void *some_void_ptr);


