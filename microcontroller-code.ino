/*
	Geospatial Meteorological Transmitter Module
	Transmits BME280 and GNSS data over BlueTooth (HC-05)
	Author: Deep Pradhan, 2021

	Update on 04 Dec 2021:
	* Changed to DOP from accuracy, as calculated accuracy may be incorrect
	* Fix corrupted altitude for four digit altitude
	* Possible fix for 0,0,0 longitude,latitude,altitude
	* Possible fix for incorrect/corrupted humidity
	* Output satellites visible

	Output format:
		BME280 & GNSS data:
			$MTGNO,<Temperature (celsius)>,<Pressure (hPa)>,<Humidity (%)>,
			<Barometric altitude (m)>,<longitude (decimal degrees)>,
			<latitude (decimal degrees)>,<altitude (m, GNSS)>,<DOP>,
			<seconds since UNIX epoch>,<satellites in use>,<satellites visible>
			Note:	In case GNSS sensor does not give longitude/latitude/
			altitude, a value of 'NAN' is returned.
		Status:
			$MTGNS,ERR,BME	Error with BME280
			$MTGNS,ERR,GNSS	Error with GNSS
			$MTGNS,INTERVAL,<integer>	New interval of transmissions

	Input format:
		$MTGNI,<command>,<option>:
			$MTGNI,0,1	Force transmission disregarding interval
			$MTGNI,1,<integer>	Change interval (milliseconds) to option integer (>= 1000)

	Wiring:
		BME280 SDA  <-->  Arduino Uno/Nano A4
		BME280 SCL  <-->  Arduino Uno/Nano A5
		GNSS RXD    <---  Arduino Uno/Nano D2 (Software Serial TX)
		GNSS TXD    --->  Arduino Uno/Nano D3 (Software Serial RX)
		HC-05 RX    <---  Arduino Uno/Nano D0/TX
		HC-05 TX    --->  Arduino Uno/Nano D1/RX
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <Wire.h>

#define size_array(x) (sizeof(x) / sizeof(x[0]))

// GNSS Pins
#define PIN_GNSS_RX 2
#define PIN_GNSS_TX 3

// Duration of LED
#define DURATION_LED 100

// Default interval of transmission: 10 seconds
#define INTERVAL_TRANSMIT 10 * 1000

// Maximum time to await GNSS
#define MAXIMUM_AWAIT_GNSS 5000

// Sea level standard atmospheric pressure
#define PRESSURE_SEA_LEVEL_HPA (1013.25)

// Decimal point precision for geodetic coordinates (longitude/latitude)
#define PRECISION_GEODETIC 6

#define SIZE_BUFFER_IN 16

#define PIN_RED A1
#define PIN_GREEN A2
#define PIN_BLUE A3

// Prefix for input messages/commands 
#define PREFIX_IN "$MTGNI"

// Prefix for output messages
#define PREFIX_OUT "$MTGNO"

// Prefix for status messages
#define PREFIX_STATUS "$MTGNS"

// String representation of NAN
#define STR_NAN "NAN"

// Has a GNSS fix being achieved
bool gnss_location_available = false;

// Time of last transmission in milliseconds
unsigned long last_transmit = 0;

int interval_transmit = INTERVAL_TRANSMIT,
		param_in[2], _satellites_visible;

char temperature[6] = STR_NAN,
		pressure[8] = STR_NAN,
		humidity[7] = STR_NAN,
		altitudeBarometric[7] = STR_NAN,
		longitude[PRECISION_GEODETIC + 6] = STR_NAN,
		latitude[PRECISION_GEODETIC + 5] = STR_NAN,
		altitudeGNSS[7] = STR_NAN,
		dop[6] = STR_NAN,
		date_time[11] = STR_NAN,
		satellites[4] = STR_NAN,
		satellites_visible[4] = STR_NAN,
		buffer_out[84],
		buffer_in[SIZE_BUFFER_IN]; // '$MTGNI,11,10000'

// Create a BME280 object
Adafruit_BME280 bme;

// Create our Software Serial object
SoftwareSerial serial_gnss(PIN_GNSS_RX, PIN_GNSS_TX);

// Create a TinyGPS++ object
TinyGPSPlus gnss;

TinyGPSCustom gnss_visible[] = {
		TinyGPSCustom(gnss, "GPGSV", 3), // GPS/Navstar
		TinyGPSCustom(gnss, "GLGSV", 3), // GLONASS
		TinyGPSCustom(gnss, "GNGSV", 3), // GLONASS (Multi-constilation)
		TinyGPSCustom(gnss, "GBGSV", 3), // BeiDou
		TinyGPSCustom(gnss, "GAGSV", 3), // Galileo
		TinyGPSCustom(gnss, "GIGSV", 3) // IRNSS/NavIC
};

const int size_gnss_visible = size_array(gnss_visible);

// Time struct
tmElements_t _date_time;

void setup() {
	Serial.begin(9600);
	serial_gnss.begin(9600);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  digitalWrite(PIN_RED, HIGH); // Default LED colour: Red
 
	if (!bme.begin(0x77) && !bme.begin(0x76)) // Default address '0x77', some may have set as '0x76'
		while (true) {
			Serial.println(PREFIX_STATUS ",ERR,BME");
			delay(1000);
		}
}

void loop() {
	// Read BME280 data and create char arrays
	dtostrf(bme.readTemperature(), 3, 1, temperature);
	dtostrf(bme.readPressure() / 100.0F, 3, 1, pressure);
	dtostrf(bme.readHumidity(), 3, 1, humidity);
	dtostrf(bme.readAltitude(PRESSURE_SEA_LEVEL_HPA), 3, 1, altitudeBarometric);

	// Read GNSS data
	while (serial_gnss.available())
		if (gnss.encode(serial_gnss.read())) {
			_satellites_visible = 0;
			if (gnss.date.isValid() && gnss.time.isValid()) {
				_date_time.Year = gnss.date.year() - 1970;
				_date_time.Month = gnss.date.month();
				_date_time.Day = gnss.date.day();
				_date_time.Hour = gnss.time.hour();
				_date_time.Minute = gnss.time.minute();
				_date_time.Second = gnss.time.second();
				sprintf(date_time, "%lu", (unsigned long) makeTime(_date_time)); // Get time with reference to Unix epoch
			} else
				strcpy(date_time, STR_NAN);
			if (gnss.location.isValid() && gnss.location.isUpdated()) { // Valid location received - create char arrays
				dtostrf(gnss.location.lng(), PRECISION_GEODETIC + 2, PRECISION_GEODETIC, longitude);
				dtostrf(gnss.location.lat(), PRECISION_GEODETIC + 2, PRECISION_GEODETIC, latitude);
				if (gnss.altitude.isValid())
					dtostrf(gnss.altitude.meters(), 2, 1, altitudeGNSS);
				else
					strcpy(altitudeGNSS, STR_NAN);
				gnss_location_available = true;
			}
			/* DOP: https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)#Interpretation
				<01: Ideal, 01-02: Excellent, 02-05: Good, 05-10: Moderate, 10-20: Fair, >20: Poor*/
			if (gnss.hdop.isValid()) {
				dtostrf(gnss.hdop.hdop(), 2, 1, dop);
				if (gnss.hdop.hdop() == 0 || gnss.hdop.hdop() == 100) // Location not available if DOP is 0 / 100
					gnss_location_available = false;
			} else
				strcpy(dop, STR_NAN);
			for (int i = 0; i < size_gnss_visible; i++)
				_satellites_visible += atoi(gnss_visible[i].value());
			if (gnss.satellites.isValid()) {
				sprintf(satellites, "%d", gnss.satellites.value());
				/*if (_satellites_visible < gnss.satellites.value())
					_satellites_visible = gnss.satellites.value();*/
				if (gnss.satellites.value() == 0) // Location not available if satellites are 0
					gnss_location_available = false;
			} else
				strcpy(satellites, STR_NAN);
			sprintf(satellites_visible, "%d", _satellites_visible);
		}

	if (millis() > MAXIMUM_AWAIT_GNSS && gnss.charsProcessed() < 10)
		while (true) {
			Serial.println(PREFIX_STATUS ",ERR,GNSS");
			delay(interval_transmit);
		}

	if (!gnss_location_available) { // Fill char arrays with NAN if no GNSS fix
		strcpy(longitude, STR_NAN);
		strcpy(latitude, STR_NAN);
		strcpy(altitudeGNSS, STR_NAN);
	}

	if (Serial.available() > 0) { // Command received
		Serial.readStringUntil(',').toCharArray(buffer_in, SIZE_BUFFER_IN);
		if (strcmp(buffer_in, PREFIX_IN) == 0) { // Correct prefix
			// Make LED magenta
			digitalWrite(PIN_RED, HIGH);
			digitalWrite(PIN_GREEN, LOW);
			digitalWrite(PIN_BLUE, HIGH);

			Serial.read(); // Skip comma
			param_in[0] = Serial.readStringUntil(',').toInt();
			Serial.read(); // Skip comma
			param_in[1] = Serial.readStringUntil('\n').toInt();
			switch (param_in[0]) {
				case 0: // Force transmit command
					if (param_in[1] == 1) {
						delay(DURATION_LED); // Let the LED remain magenta for at least specifeid duration
						transmit();
					}
					break;
				case 1: // Change transmission interval command
					if (param_in[1] >= 1000) {
						interval_transmit = param_in[1];
						sprintf(buffer_out, "%s,INTERVAL,%d\n", PREFIX_STATUS, interval_transmit);
						Serial.print(buffer_out);
						delay(DURATION_LED); // Let the LED remain magenta for at least specifeid duration
					}
					break;
			}
		}
	}

	// If GNSS available then make LED green otherwise red
	digitalWrite(PIN_RED, gnss_location_available ? LOW : HIGH);
	digitalWrite(PIN_GREEN, gnss_location_available ? HIGH : LOW);
	digitalWrite(PIN_BLUE, LOW);

	if (last_transmit + interval_transmit <= millis()) // Transmit after set interval
		transmit();
}

/** Function to transmit data over Bluetooth*/
void transmit() {
	last_transmit = millis(); // Save time of transmission for timing
	// Make LED blue
	digitalWrite(PIN_RED, LOW);
	digitalWrite(PIN_GREEN, LOW);
	digitalWrite(PIN_BLUE, HIGH);
	sprintf(buffer_out, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", PREFIX_OUT, temperature, pressure, humidity, altitudeBarometric,
			longitude, latitude, altitudeGNSS, dop, date_time, satellites, satellites_visible);
	Serial.print(buffer_out);
	delay(DURATION_LED); // Let the LED remain blue for at least specifeid duration
}
