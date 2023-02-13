/**
 * This sketch will use a HTU21D (SHT20/SHT21 compatible) sensor to measure temperature and htuHumidity.
 * It will also use a BMP280 sensor to measure temperature and barometric pressure.
 * My HTU21D sensor uses address 0x40, which is the default for this class of sensor.
 * My BMP280 sensor uses address 0x76, which is NOT the default, and which I have overridden in the setupBMP280() function.
 * One unconventional thing about this devkit is the LED is on when the pin is set to low (0), rather than high (1).
 * My MQTT topic formats are:
 *    <location>/<device>/<device reading>
 *    <location>/<device>/<sensor type>/<sensor reading>
 *
 * @copyright   Copyright © 2022 Adam Howell
 * @license     The MIT License (MIT)
 */
#include "ESP_BMP280_HTU21D_OTA.h"


/**
 * @brief onMessage() handles callback operations for MQTT.
 * This function will react to JSON messages containing the "command" property.
 * The "publishTelemetry" and "publishStatus" commands will immediately perform those operations.
 * The "changeTelemetryInterval" expects another property named "value" to have an integer value, which represents the new update interval in milliseconds.
 */
void onMessage( char *topic, byte *payload, unsigned int length )
{
	callbackCount++;
	Serial.printf( "\nMessage arrived on Topic: '%s'\n", topic );

	StaticJsonDocument<BUFFER_SIZE> callbackJsonDoc;
	deserializeJson( callbackJsonDoc, payload, length );

	// The command can be: publishTelemetry, publishStatus, changeTelemetryInterval, or changePublishInterval.
	const char *command = callbackJsonDoc["command"];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values because MQTT received the 'publishTelemetry' command." );
		readTelemetry();
		lastPollTime = millis();
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval because MQTT received the 'changeTelemetryInterval' command." );
		unsigned long tempValue = callbackJsonDoc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mix-up.
		if( tempValue > 4 * MILLIS_IN_SEC )
			publishInterval = tempValue;
		Serial.printf( "MQTT publish interval has been updated to %lu\n", publishInterval );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "changeSeaLevelPressure" ) == 0 )
	{
		Serial.println( "Changing the publish interval because MQTT received the 'changeTelemetryInterval' command." );
		unsigned long tempValue = callbackJsonDoc["value"];
		// Only update the value if it is greater than 400 hPa and less than 1500 hPa.
		if( tempValue > 400 && tempValue < 1500 )
			seaLevelPressure = tempValue;
		Serial.printf( "Sea level pressure has been updated to %f\n", seaLevelPressure );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStats" ) == 0 )
	{
		Serial.println( "Publishing stats because MQTT received the 'publishStats' command." );
		readTelemetry();
		lastPollTime = millis();
		publishTelemetry();
	}
	else
		Serial.printf( "Unknown command: %s\n", command );
}  // End of onMessage() function.


/**
 * @brief setup() runs once, every time the device is booted, and then loop() takes over.
 */
void setup()
{
	// Wait one second before starting serial communication, to give the user time to connect the terminal.
	delay( MILLIS_IN_SEC );
	Serial.begin( 115200 );
	// If the serial port has not connected yet, wait one more second before printing that this function has begun.
	if( !Serial )
		delay( MILLIS_IN_SEC );
	Serial.println( "\nSetup is initiating..." );

	// Start I2C communication.
	Wire.begin();

	// Set up the onboard LED, and turn it on (this devkit uses 0, or LOW, to turn the LED on).
	pinMode( LED_PIN, OUTPUT );
	digitalWrite( LED_PIN, 0 );

	Serial.println( __FILE__ );

	// Set up the environmental sensors.
	setupHTU21D();
	setupBMP280();

	// Set ipAddress to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	Serial.println( "Connecting WiFi..." );
	wifiMultiConnect();

	// The networkIndex variable is initialized to 2112.  If it is still 2112 at this point, then WiFi failed to connect.
	if( networkIndex != 2112 )
	{
		const char *mqttBroker = mqttBrokerArray[networkIndex];
		const int mqttPort     = mqttPortArray[networkIndex];
		// Set the MQTT client parameters.
		mqttClient.setServer( mqttBroker, mqttPort );
		// Assign the onMessage() function to handle MQTT callbacks.
		mqttClient.setCallback( onMessage );
		Serial.printf( "Using MQTT broker: %s\n", mqttBroker );
		Serial.printf( "Using MQTT port: %d\n", mqttPort );
	}
	else
	{
		Serial.println( "\n\n---------------------------------" );
		Serial.println( "Failed to connect to the network!" );
		Serial.println( "---------------------------------\n\n" );
	}

	// Configure Over-The-Air update functionality.
	configureOTA();

	Serial.printf( "IP address: %s\n", ipAddress );

	// Read the telemetry twice to populate the temperature, humidity, and pressure arrays.
	readTelemetry();
	readTelemetry();

	Serial.println( "Setup has completed.\n" );
}  // End of setup() function.


/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 * An excellent OTA guide can be found here:
 * https://randomnerdtutorials.com/esp8266-ota-updates-with-arduino-ide-over-the-air/
 * The setPort(), setHostname(), and setPassword() functions are optional.
 */
void configureOTA()
{
	// Port defaults to 8266, but can be overridden here:
	// ArduinoOTA.setPort( 8266 );

	// Hostname defaults to esp8266-[ChipID], but can be overridden here:
	//	ArduinoOTA.setHostname( hostname );

	// No authentication by default, but a password can be set here:
	// ArduinoOTA.setPassword( ( const char * )"abc123" );

	// OTA callbacks are required:
	ArduinoOTA.onStart( []() { Serial.println( "Starting OTA communication." ); } );
	ArduinoOTA.onEnd( []() { Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error ) {
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR )
			Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR )
			Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR )
			Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR )
			Serial.println( "OTA transmission failed to terminate properly!" );
	} );
	ArduinoOTA.begin();
	Serial.println( "OTA is configured and ready." );
}  // End of the configureOTA() function.


/**
 * @brief setupHTU21D() will initialize the sensor and check its status.
 */
void setupHTU21D()
{
	Serial.println( "Initializing the HTU21D sensor." );

	if( !htu21d.begin() )
	{
		Serial.println( "Failed to initialize the HTU21D!" );
		Serial.println( "Resetting the sensor and attempting to reinitialize..." );
		htu21d.reset();
		delay( 3000 );
		if( !htu21d.begin() )
		{
			Serial.println( "Failed again to initialize the HTU21D!" );
			Serial.println( "Check the wiring and I2C bus settings." );
			Serial.println( "Going into an infinite loop..." );
			while( 1 )
				delay( 3 );
		}
	}
	else
		Serial.println( "The HTU21D has been configured." );

}  // End of setupHTU21D function.


/**
 * @brief setupBMP280() will initialize the sensor and check its status.
 */
void setupBMP280()
{
	Serial.println( "Attempting to connect to the BMP280 at address 0x76..." );
	if( !bmp280.begin( 0x76 ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor!" );
		Serial.println( "Check the wiring and I2C bus settings." );
		Serial.println( "Going into an infinite loop..." );
		while( 1 )
			delay( 3 );
	}
	/* Default settings from datasheet. */
	bmp280.setSampling( Adafruit_BMP280::MODE_NORMAL,      /* Operating Mode. */
	                    Adafruit_BMP280::SAMPLING_X2,      /* Temp. oversampling */
	                    Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
	                    Adafruit_BMP280::FILTER_X16,       /* Filtering. */
	                    Adafruit_BMP280::STANDBY_MS_500 ); /* Standby time. */

	Serial.println( "Connected to the BMP280!\n" );
}  // End of setupBMP280 function.


/**
 * @brief wifiMultiConnect() will iterate through 'wifiSsidArray[]', attempting to connect with the password stored at the same index in 'wifiPassArray[]'.
 */
void wifiMultiConnect()
{
	digitalWrite( LED_PIN, 1 );  // Turn the LED off to show a connection is being made.

	Serial.println( "\nEntering wifiMultiConnect()" );
	for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
	{
		wifiConnectCount++;
		// Get the details for this connection attempt.
		const char *wifiSsid     = wifiSsidArray[networkArrayIndex];
		const char *wifiPassword = wifiPassArray[networkArrayIndex];

		// Announce the WiFi parameters for this connection attempt.
		Serial.print( "Attempting to connect to SSID \"" );
		Serial.print( wifiSsid );
		Serial.println( "\"" );

		// Don't even try to connect if the SSID cannot be found.
		if( checkForSSID( wifiSsid ) )
		{
			// Attempt to connect to this WiFi network.
			Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
			WiFi.begin( wifiSsid, wifiPassword );

			unsigned long wifiConnectionStartTime = millis();
			// Wait up to 10 seconds for a connection.
			Serial.print( "Waiting up to " );
			Serial.print( wifiConnectionTimeout / MILLIS_IN_SEC );
			Serial.print( " seconds for a connection" );
			/*
			WiFi.status() return values:
			WL_IDLE_STATUS      = 0,
			WL_NO_SSID_AVAIL    = 1,
			WL_SCAN_COMPLETED   = 2,
			WL_CONNECTED        = 3,
			WL_CONNECT_FAILED   = 4,
			WL_CONNECTION_LOST  = 5,
			WL_WRONG_PASSWORD   = 6,
			WL_DISCONNECTED     = 7
			*/
			while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
			{
				Serial.print( "." );
				delay( MILLIS_IN_SEC );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				digitalWrite( LED_PIN, 0 );  // Turn the LED on to show the connection was successful.
				Serial.print( "IP address: " );
				snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
				Serial.println( ipAddress );
				networkIndex = networkArrayIndex;
				// Print that WiFi has connected.
				Serial.println( "\nWiFi connection established!" );
				return;
			}
			else
				Serial.println( "Unable to connect to WiFi!" );
		}
		else
			Serial.println( "That network was not found!" );
	}
	Serial.println( "Exiting wifiMultiConnect()\n" );
}  // End of wifiMultiConnect() function.


/**
 * @brief checkForSSID() is used by wifiMultiConnect() to avoid attempting to connect to SSIDs which are not in range.
 * Returns 1 if 'ssidName' can be found.
 * Returns 0 if 'ssidName' cannot be found.
 */
int checkForSSID( const char *ssidName )
{
	byte networkCount = WiFi.scanNetworks();
	if( networkCount == 0 )
		Serial.println( "No WiFi SSIDs are in range!" );
	else
	{
		Serial.print( networkCount );
		Serial.println( " networks found." );
		for( int i = 0; i < networkCount; ++i )
		{
			// Check to see if this SSID matches the parameter.
			if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
				return 1;
			// Alternately, the String compareTo() function can be used like this: if( WiFi.SSID( i ).compareTo( ssidName ) == 0 )
		}
	}
	Serial.print( "SSID '" );
	Serial.print( ssidName );
	Serial.println( "' was not found!" );
	return 0;
}  // End of checkForSSID() function.


/**
 * @brief mqttMultiConnect() will:
 * 1. Check the WiFi connection, and reconnect WiFi as needed.
 * 2. Attempt to connect the MQTT client designated in 'mqttBrokerArray[networkIndex]' up to 'maxAttempts' number of times.
 * 3. Subscribe to the topic defined in 'MQTT_COMMAND_TOPIC'.
 * If the broker connection cannot be made, an error will be printed to the serial port.
 */
void mqttMultiConnect( int maxAttempts )
{
	unsigned long time = millis();
	// Connect the first time.  Avoid subtraction overflow.  Connect every interval.
	if( lastMqttConnectionTime == 0 || ( ( time > mqttCoolDownInterval ) && ( time - mqttCoolDownInterval ) > lastMqttConnectionTime ) )
	{
		mqttConnectCount++;
		Serial.print( "Current time: " );
		Serial.println( time );
		Serial.print( "Last MQTT connection time: " );
		Serial.println( lastMqttConnectionTime );
		Serial.print( "MQTT boolean: " );
		Serial.println( ( ( time > mqttCoolDownInterval ) && ( time - mqttCoolDownInterval ) > lastMqttConnectionTime ) );

		Serial.println( "Function mqttMultiConnect() has initiated.\n" );
		if( WiFi.status() != WL_CONNECTED )
			wifiMultiConnect();

		digitalWrite( LED_PIN, 1 );  // Turn the LED off to show a connection is being made.

		/*
		 * The networkIndex variable is initialized to 2112.
		 * If it is still 2112 at this point, then WiFi failed to connect.
		 * This is only needed to display the name and port of the broker being used.
		 */
		if( networkIndex != 2112 )
		{
			Serial.print( "Attempting to connect to the MQTT broker at '" );
			Serial.print( mqttBrokerArray[networkIndex] );
			Serial.print( ":" );
			Serial.print( mqttPortArray[networkIndex] );
			Serial.print( "' up to " );
			Serial.print( maxAttempts );
			Serial.println( " times." );
		}
		else
		{
			Serial.print( "Attempting to connect to the MQTT broker up to " );
			Serial.print( maxAttempts );
			Serial.println( " times." );
		}

		int attemptNumber = 0;
		// Loop until MQTT has connected.
		while( !mqttClient.connected() && attemptNumber < maxAttempts )
		{
			// Put the macAddress and random number into clientId.
			char clientId[22];
			// Optionally add random( 999 ) to this to create a pseudo-random client ID.
			//			snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ) );
			snprintf( clientId, 22, "%s", macAddress );
			// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
			Serial.printf( "Connecting with client ID '%s'.\n", clientId );
			Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ) );
			if( mqttClient.connect( clientId ) )
			{
				lastMqttConnectionTime = millis();
				digitalWrite( LED_PIN, 0 );  // Turn the LED on to show the connection was successful.
				Serial.println( " connected." );
				if( !mqttClient.setBufferSize( BUFFER_SIZE ) )
				{
					Serial.printf( "Unable to create a buffer %d bytes long!\n", BUFFER_SIZE );
					Serial.println( "Restarting the device!" );
					ESP.restart();
				}

				// Subscribe to the command topic.
				if( mqttClient.subscribe( MQTT_COMMAND_TOPIC ) )
					Serial.printf( "Successfully subscribed to topic '%s'.\n", MQTT_COMMAND_TOPIC );
				else
					Serial.printf( "Failed to subscribe to topic '%s'!\n", MQTT_COMMAND_TOPIC );
			}
			else
			{
				lastMqttConnectionTime = millis();
				int mqttState          = mqttClient.state();
				/*
					Possible values for client.state():
					#define MQTT_CONNECTION_TIMEOUT     -4		// Note: This also comes up when the clientID is already in use.
					#define MQTT_CONNECTION_LOST        -3
					#define MQTT_CONNECT_FAILED         -2
					#define MQTT_DISCONNECTED           -1
					#define MQTT_CONNECTED               0
					#define MQTT_CONNECT_BAD_PROTOCOL    1
					#define MQTT_CONNECT_BAD_CLIENT_ID   2
					#define MQTT_CONNECT_UNAVAILABLE     3
					#define MQTT_CONNECT_BAD_CREDENTIALS 4
					#define MQTT_CONNECT_UNAUTHORIZED    5
				*/
				Serial.printf( " failed!  Return code: %d", mqttState );
				if( mqttState == -4 )
					Serial.println( " - MQTT_CONNECTION_TIMEOUT" );
				else if( mqttState == 2 )
					Serial.println( " - MQTT_CONNECT_BAD_CLIENT_ID" );
				else
					Serial.println( "" );

				if( maxAttempts > 1 )
				{
					Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectDelay / MILLIS_IN_SEC );
					delay( mqttReconnectDelay );
				}

				// This block increments the broker connection "cooldown" time by 10 seconds after every failed connection, and resets it once it is over 2 minutes.
				if( mqttCoolDownInterval > 120000 )
					mqttCoolDownInterval = 0;
				mqttCoolDownInterval += 10000;
			}
			attemptNumber++;
		}

		if( !mqttClient.connected() )
		{
			Serial.println( "\n" );
			Serial.println( "*************************************" );
			Serial.println( "Unable to connect to the MQTT broker!" );
			Serial.println( "*************************************" );
			Serial.println( "\n" );
			delay( MILLIS_IN_SEC );
		}
	}
}  // End of mqttMultiConnect() function.


/**
 * @brief cToF() will convert Celsius to Fahrenheit.
 */
float cToF( float value )
{
	return value * 1.8 + 32;
}  // End of the cToF() function.


/**
 * @brief mToF() will convert meters to feet.
 */
float mToF( float value )
{
	return value * 3.28084;
}  // End of the mToF() function.


/**
 * @brief averageArray() will return the average of values in the passed array.
 */
float averageArray( float valueArray[] )
{
	const unsigned int arraySize = 3;
	float tempValue              = 0;
	for( int i = 0; i < arraySize; ++i )
	{
		tempValue += valueArray[i];
	}
	return tempValue / arraySize;
}  // End of the averageArray() function.


/**
 * @brief findMaximum() will return the largest value in the passed array.
 */
float findMaximum( float valueArray[], unsigned int size )
{
	float maxValue = valueArray[0];
	for( int i = 1; i < size; ++i )
	{
		if( valueArray[i] > maxValue )
			maxValue = valueArray[i];
	}
	return maxValue;
}  // End of the findMaximum() function.


/**
 * @brief findMinimum() will return the smallest value in the passed array.
 */
float findMinimum( float valueArray[], unsigned int size )
{
	float minValue = valueArray[0];
	for( int i = 1; i < size; ++i )
	{
		if( valueArray[i] < minValue )
			minValue = valueArray[i];
	}
	return minValue;
}  // End of the findMinimum() function.


/**
 * @brief addValue() will add the passed value to the 0th element of the passed array, after moving the existing array values to higher indexes.
 * If value is less than minValue, or greater than maxValue, it will be discarded and nothing will be added to valueArray.
 */
void addValue( float valueArray[], unsigned int size, float value, float minValue, float maxValue )
{
	// Prevent sensor anomalies from getting into the array.
	if( value < minValue || value > maxValue )
	{
		Serial.printf( "\n\nValue %f is not between %f and %f!\n\n", value, minValue, maxValue );
		invalidValueCount++;
		return;
	}

	// Detect outliers.
	float minArrayValue = findMinimum( valueArray, size );
	float maxArrayValue = findMaximum( valueArray, size );
	if( value < ( minArrayValue / 2 ) )
	{
		Serial.printf( "\n\nValue %f is less than half the smallest existing value of %f!\n\n", value, minArrayValue );
		invalidValueCount++;
		return;
	}
	if( value > ( maxArrayValue * 2 ) )
	{
		Serial.printf( "\n\nValue %f is more than double the largest existing value of %f!\n\n", value, maxArrayValue );
		invalidValueCount++;
		return;
	}

	valueArray[2] = valueArray[1];
	valueArray[1] = valueArray[0];
	valueArray[0] = value;
}  // End of the addValue() function.


/**
 * @brief readTelemetry() will:
 * 1. Read from all available sensors.
 * 2. Store legitimate values in global variables.
 * 3. Increment a counter if any value is invalid.
 */
void readTelemetry()
{
	rssi = WiFi.RSSI();

	// Get readings from the HTU21D.
	addValue( htuTempCArray, 3, htu21d.readTemperature(), -40, 125 );
	addValue( htuHumidityArray, 3, htu21d.readHumidity(), 0, 100 );

	// Get readings from the BMP280.
	addValue( bmpTempCArray, 3, bmp280.readTemperature(), -30, 80 );
	addValue( bmpPressureHPaArray, 3, bmp280.readPressure() / 100, 300, 1500 );
	addValue( bmpAltitudeMArray, 3, bmp280.readAltitude( seaLevelPressure ), 0, 10000 );
}  // End of readTelemetry() function.


/**
 * @brief printTelemetry() will print the sensor and device data to the serial port.
 */
void printTelemetry()
{
	Serial.println();
	printCount++;
	Serial.println( __FILE__ );
	Serial.printf( "Print count %ld\n", printCount );
	Serial.println();

	Serial.println( "Network stats:" );
	Serial.printf( "  MAC address: %s\n", macAddress );
	int wifiStatusCode = WiFi.status();
	char buffer[29];
	lookupWifiCode( wifiStatusCode, buffer );
	if( wifiStatusCode == 3 )
	{
		Serial.printf( "  IP address: %s\n", ipAddress );
		Serial.printf( "  RSSI: %ld\n", rssi );
	}
	Serial.printf( "  wifiConnectCount: %u\n", wifiConnectCount );
	Serial.printf( "  wifiCoolDownInterval: %lu\n", wifiCoolDownInterval );
	Serial.printf( "  Wi-Fi status text: %s\n", buffer );
	Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
	Serial.println();

	Serial.println( "MQTT stats:" );
	Serial.printf( "  mqttConnectCount: %u\n", mqttConnectCount );
	Serial.printf( "  mqttCoolDownInterval: %lu\n", mqttCoolDownInterval );
	if( networkIndex != 2112 )
		Serial.printf( "  Broker: %s:%d\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex] );
	lookupMQTTCode( mqttClient.state(), buffer );
	Serial.printf( "  MQTT state: %s\n", buffer );
	Serial.printf( "  Publish count: %lu\n", publishCount );
	Serial.printf( "  Callback count: %lu\n", callbackCount );
	Serial.println();

	Serial.println( "Environmental stats:" );
	Serial.printf( "  HTU21D temperature: %.2f C\n", averageArray( htuTempCArray ) );
	Serial.printf( "  HTU21D temperature: %.2f F\n", cToF( averageArray( htuTempCArray ) ) );
	Serial.printf( "  HTU21D humidity: %.2f %%\n", averageArray( htuHumidityArray ) );
	Serial.printf( "  BMP280 temperature: %.2f C\n", averageArray( bmpTempCArray ) );
	Serial.printf( "  BMP280 temperature: %.2f F\n", cToF( averageArray( bmpTempCArray ) ) );
	Serial.printf( "  BMP280 pressure: %.2f hPa\n", averageArray( bmpPressureHPaArray ) );
	Serial.printf( "  BMP280 altitude: %.2f m\n", averageArray( bmpAltitudeMArray ) );
	Serial.printf( "  BMP280 altitude: %.2f f\n", mToF( averageArray( bmpAltitudeMArray ) ) );
	Serial.printf( "  Sea level pressure: %.2f hPa\n", seaLevelPressure );
	Serial.printf( "  Invalid reading count from all sensors: %u\n", invalidValueCount );
	Serial.println();

	Serial.printf( "Next telemetry poll in %lu seconds\n", telemetryInterval / MILLIS_IN_SEC );
	Serial.println( "\n" );
}  // End of printTelemetry() function.


/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
	switch( code )
	{
		case 0:
			snprintf( buffer, 26, "%s", "Idle" );
			break;
		case 1:
			snprintf( buffer, 26, "%s", "No SSID" );
			break;
		case 2:
			snprintf( buffer, 26, "%s", "Scan completed" );
			break;
		case 3:
			snprintf( buffer, 26, "%s", "Connected" );
			break;
		case 4:
			snprintf( buffer, 26, "%s", "Connection failed" );
			break;
		case 5:
			snprintf( buffer, 26, "%s", "Connection lost" );
			break;
		case 6:
			snprintf( buffer, 26, "%s", "Disconnected" );
			break;
		default:
			snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
	}
}  // End of lookupWifiCode() function.


/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
	switch( code )
	{
		case -4:
			snprintf( buffer, 29, "%s", "Connection timeout" );
			break;
		case -3:
			snprintf( buffer, 29, "%s", "Connection lost" );
			break;
		case -2:
			snprintf( buffer, 29, "%s", "Connect failed" );
			break;
		case -1:
			snprintf( buffer, 29, "%s", "Disconnected" );
			break;
		case 0:
			snprintf( buffer, 29, "%s", "Connected" );
			break;
		case 1:
			snprintf( buffer, 29, "%s", "Bad protocol" );
			break;
		case 2:
			snprintf( buffer, 29, "%s", "Bad client ID" );
			break;
		case 3:
			snprintf( buffer, 29, "%s", "Unavailable" );
			break;
		case 4:
			snprintf( buffer, 29, "%s", "Bad credentials" );
			break;
		case 5:
			snprintf( buffer, 29, "%s", "Unauthorized" );
			break;
		default:
			snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
	}
}  // End of lookupMQTTCode() function.


/**
 * @brief publishTelemetry() will publish the sensor and device data over MQTT.
 * It is also called by the callback when the "publishTelemetry" command is received.
 */
void publishTelemetry()
{
	char mqttString[BUFFER_SIZE];  // A String to hold the JSON.

	// Create a JSON Document on the stack.
	StaticJsonDocument<BUFFER_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, temperature, htuTempF, htuHumidity, rssi, publishCount, notes
	doc["sketch"]       = __FILE__;
	doc["mac"]          = macAddress;
	doc["ip"]           = ipAddress;
	doc["rssi"]         = rssi;
	doc["htuTempC"]     = averageArray( htuTempCArray );
	doc["htuTempF"]     = cToF( averageArray( htuTempCArray ) );
	doc["htuHumidity"]  = averageArray( htuHumidityArray );
	doc["bmpTempC"]     = averageArray( bmpTempCArray );
	doc["bmpTempF"]     = cToF( averageArray( bmpTempCArray ) );
	doc["bmpPressure"]  = averageArray( bmpPressureHPaArray );
	doc["publishCount"] = publishCount;

	// Serialize the JSON into mqttString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttString );

	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( MQTT_TOPIC, mqttString, false );
	if( success )
	{
		Serial.println( "Successfully published to:" );
		Serial.printf( "  %s\n", MQTT_TOPIC );

		char buffer[20];
		// Device topic format: <location>/<device>/<metric>
		// Sensor topic format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( SKETCH_TOPIC, __FILE__, false ) )
			Serial.printf( "  %s\n", SKETCH_TOPIC );
		if( mqttClient.publish( MAC_TOPIC, macAddress, false ) )
			Serial.printf( "  %s\n", MAC_TOPIC );
		if( mqttClient.publish( IP_TOPIC, ipAddress, false ) )
			Serial.printf( "  %s\n", IP_TOPIC );
		if( mqttClient.publish( RSSI_TOPIC, ltoa( rssi, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", RSSI_TOPIC );
		if( mqttClient.publish( PUBLISH_COUNT_TOPIC, ltoa( publishCount, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", PUBLISH_COUNT_TOPIC );

		snprintf( buffer, 25, "%f", averageArray( htuTempCArray ) );
		if( mqttClient.publish( HTU_TEMP_C_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", HTU_TEMP_C_TOPIC );
		snprintf( buffer, 25, "%f", cToF( averageArray( htuTempCArray ) ) );
		if( mqttClient.publish( HTU_TEMP_F_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", HTU_TEMP_F_TOPIC );
		snprintf( buffer, 25, "%f", averageArray( htuHumidityArray ) );
		if( mqttClient.publish( HTU_HUMIDITY_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", HTU_HUMIDITY_TOPIC );

		snprintf( buffer, 25, "%f", averageArray( bmpTempCArray ) );
		if( mqttClient.publish( BMP_TEMP_C_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", BMP_TEMP_C_TOPIC );
		snprintf( buffer, 25, "%f", cToF( averageArray( bmpTempCArray ) ) );
		if( mqttClient.publish( BMP_TEMP_F_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", BMP_TEMP_F_TOPIC );
		snprintf( buffer, 25, "%f", averageArray( bmpPressureHPaArray ) );
		if( mqttClient.publish( BMP_PRESSURE_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", BMP_PRESSURE_TOPIC );
		snprintf( buffer, 25, "%f", averageArray( bmpAltitudeMArray ) );
		if( mqttClient.publish( BMP_ALTITUDE_M_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", BMP_ALTITUDE_M_TOPIC );
		snprintf( buffer, 25, "%f", mToF( averageArray( bmpAltitudeMArray ) ) );
		if( mqttClient.publish( BMP_ALTITUDE_F_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", BMP_ALTITUDE_F_TOPIC );
		snprintf( buffer, 25, "%f", seaLevelPressure );
		if( mqttClient.publish( SEA_LEVEL_PRESSURE_TOPIC, buffer, false ) )
			Serial.printf( "  %s\n", SEA_LEVEL_PRESSURE_TOPIC );
	}
	else
		Serial.printf( "Failed to publish to '%s'.\n", MQTT_TOPIC );

	Serial.printf( "Next MQTT publish in %lu seconds.\n", publishInterval / MILLIS_IN_SEC );
	Serial.println( "\n" );
}  // End of publishTelemetry() function.


/**
 * @brief toggleLED() will change the state of the LED.
 * This function does not manage any timings.
 */
void toggleLED()
{
	if( digitalRead( LED_PIN ) != 1 )
		digitalWrite( LED_PIN, 1 );
	else
		digitalWrite( LED_PIN, 0 );
}  // End of toggleLED() function.


/**
 * @brief The main loop function, which continues execution after setup() finishes.
 */
void loop()
{
	// Check the WiFi and MQTT client connection state.
	if( !mqttClient.connected() )
		mqttMultiConnect( 1 );
	// The MQTT loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();
	// The OTA handle() function broadcasts this device's presence to compatible clients on the same Wi-Fi network.
	ArduinoOTA.handle();

	unsigned long currentTime = millis();
	// Print the first currentTime.  Avoid subtraction overflow.  Print every interval.
	if( lastPollTime == 0 || ( currentTime > telemetryInterval && ( currentTime - telemetryInterval ) > lastPollTime ) )
	{
		readTelemetry();
		printTelemetry();
		lastPollTime = millis();
	}

	currentTime = millis();
	// Publish the first currentTime.  Avoid subtraction overflow.  Publish every interval.
	if( lastPublishTime == 0 || ( currentTime > publishInterval && ( currentTime - publishInterval ) > lastPublishTime ) )
	{
		publishCount++;
		publishTelemetry();
		lastPublishTime = millis();
	}

	currentTime = millis();
	// Process the first currentTime.  Avoid subtraction overflow.  Process every interval.
	if( lastLedBlinkTime == 0 || ( ( currentTime > ledBlinkInterval ) && ( currentTime - ledBlinkInterval ) > lastLedBlinkTime ) )
	{
		// If Wi-Fi is connected, but MQTT is not, blink the LED.
		if( WiFi.status() == WL_CONNECTED )
		{
			if( mqttClient.state() != 0 )
				toggleLED();
			else
				digitalWrite( LED_PIN, 0 );  // Turn the LED on to show both Wi-Fi and MQTT are connected.
		}
		else
			digitalWrite( LED_PIN, 1 );  // Turn the LED off to show that Wi-Fi is not connected.
		lastLedBlinkTime = millis();
	}

	// Reset the device if the number of invalid sensor readings is too high.
	if( invalidValueCount > 10 )
	{
		Serial.println( "\n\n\n" );
		Serial.println( "Too many invalid sensor readings have occurred!" );
		Serial.println( "The device will reset in 5 seconds!" );
		delay( 5 * MILLIS_IN_SEC );
		Serial.println( "\n\n\n" );
		ESP.restart();
	}
}  // End of loop() function.
