//
// Created by Adam Howell on 2022-11-29.
//

#ifndef ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H
#define ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H


#include "PubSubClient.h"      // My fork of the PubSubClient MQTT API by Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"       // I use this file to hide my network information from random people browsing my GitHub repo.
#include <Adafruit_BMP280.h>   // The Adafruit library for BMP280 sensor.
#include <Adafruit_HTU21DF.h>  // The Adafruit library for HTU21D sensors.
#include <ArduinoJson.h>       // The JSON parsing library used.  Author: Benoît Blanchon  https://arduinojson.org/
#include <ArduinoOTA.h>        // OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.


#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h"         // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h>         // OTA - mDNSResponder (Multicast DNS) for the ESP8266 family.
const unsigned int LED_ON  = 0;  // For this DevKit, 0 turns the LED on.
const unsigned int LED_OFF = 1;  // For this DevKit, 1 turns the LED off.
#else                            // !ESP8266
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"                // ESP32 WiFi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h>             // OTA - Multicast DNS for the ESP32.
const unsigned int LED_ON  = 1;  // 1 turns the LED on.
const unsigned int LED_OFF = 0;  // 0 turns the LED off.
#endif                           // ESP8266


/**
 * @brief Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Defined in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Defined in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Defined in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Defined in "privateInfo.h".
// const char * otaPass = "nunya";																				// Defined in "privateInfo.h".

const char *MQTT_COMMAND_TOPIC        = "AdamsDesk/8266/command";           // The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *SKETCH_TOPIC              = "AdamsDesk/8266/sketch";            // The topic used to publish the sketch name.
const char *MAC_TOPIC                 = "AdamsDesk/8266/mac";               // The topic used to publish the MAC address.
const char *IP_TOPIC                  = "AdamsDesk/8266/ip";                // The topic used to publish the IP address.
const char *RSSI_TOPIC                = "AdamsDesk/8266/rssi";              // The topic used to publish the WiFi Received Signal Strength Indicator.
const char *PUBLISH_COUNT_TOPIC       = "AdamsDesk/8266/publishCount";      // The topic used to publish the loop count.
const char *SEA_LEVEL_PRESSURE_TOPIC  = "AdamsDesk/8266/seaLevelPressure";  // The topic used to publish the current sea level pressure value.
const char *HTU_TEMP_C_TOPIC          = "AdamsDesk/8266/HTU21D/tempC";      // The topic used to publish the HTU21D temperature in Celsius.
const char *HTU_TEMP_F_TOPIC          = "AdamsDesk/8266/HTU21D/tempF";      // The topic used to publish the HTU21D temperature in Fahrenheit.
const char *HTU_HUMIDITY_TOPIC        = "AdamsDesk/8266/HTU21D/humidity";   // The topic used to publish the HTU21D relative humidity.
const char *BMP_TEMP_C_TOPIC          = "AdamsDesk/8266/BMP280/tempC";      // The topic used to publish the BMP280 temperature in Celsius.
const char *BMP_TEMP_F_TOPIC          = "AdamsDesk/8266/BMP280/tempF";      // The topic used to publish the BMP280 temperature in Fahrenheit.
const char *BMP_PRESSURE_TOPIC        = "AdamsDesk/8266/BMP280/pressure";   // The topic used to publish the BMP280 barometric pressure in hectopascals.
const char *BMP_ALTITUDE_M_TOPIC      = "AdamsDesk/8266/BMP280/altitudeM";  // The topic used to publish the BMP280 altitude in meters.
const char *BMP_ALTITUDE_F_TOPIC      = "AdamsDesk/8266/BMP280/altitudeF";  // The topic used to publish the BMP280 altitude in feet.
const String HOSTNAME                 = "AdamsDesk8266";                    // The network hostname.
const unsigned int BUFFER_SIZE        = 1024;                               // The maximum packet size MQTT should transfer.
const unsigned int MILLIS_IN_SEC      = 1000;                               // The number of milliseconds in one second.
const unsigned int LED_PIN            = 2;                                  // The blue LED on the Freenove devkit.
const unsigned int LED_BLINK_INTERVAL = 200;                                // The interval between LED state changes (when MQTT is not connected).
unsigned int invalidValueCount        = 0;                                  // A counter of how many times invalid values have been measured.
unsigned long wifiCoolDownInterval    = 10 * MILLIS_IN_SEC;                 // The interval between Wi-Fi connection attempts.
unsigned long mqttCoolDownInterval    = 10 * MILLIS_IN_SEC;                 // The interval between MQTT connection attempts.
unsigned long publishInterval         = 20 * MILLIS_IN_SEC;                 // The interval between MQTT publishes.  This prevents "flooding" the broker.
unsigned long telemetryInterval       = 15 * MILLIS_IN_SEC;                 // The interval between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long lastLedBlinkTime        = 0;                                  // The time of the last LED state change.
unsigned long lastWifiConnectTime     = 0;                                  // The time of the last Wi-Fi connection attempt.
unsigned long lastMqttConnectionTime  = 0;                                  // The time of the last MQTT connection attempt.
unsigned long lastPublishTime         = 0;                                  // The time of the last MQTT publish.
unsigned long lastPollTime            = 0;                                  // The time of the last sensor poll.
unsigned long publishCount            = 0;                                  // A count of how many publishes have taken place.
unsigned long printCount              = 0;                                  // A count of how many times the stats have been printed.
unsigned long wifiConnectCount        = 0;                                  // A count for how many times the wifiConnect() function has been called.
unsigned long mqttConnectCount        = 0;                                  // A count for how many times the mqttConnect() function has been called.
unsigned long callbackCount           = 0;                                  // A count for how many times a callback was received.
unsigned long wifiConnectionTimeout   = 10 * MILLIS_IN_SEC;                 // The maximum amount of time in milliseconds to wait for a WiFi connection before trying a different SSID.
long rssi;                                                                  // A global to hold the Received Signal Strength Indicator.
char macAddress[18];                                                        // The MAC address of the WiFi NIC.
char ipAddress[16];                                                         // The IP address given to the device.
float seaLevelPressure      = 1026.1;                                       // The local sea-level pressure. Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
float htuTempCArray[]       = { -21.12, 21.12, 42.42 };                     // An array to hold the 3 most recent Celsius values, initialized to reasonable levels.
float htuHumidityArray[]    = { 1.1, 21.12, 99.99 };                        // An array to hold the 3 most recent humidity values, initialized to reasonable levels.
float bmpTempCArray[]       = { -21.12, 21.12, 42.42 };                     // An array to hold the 3 most recent Celsius values, initialized to reasonable levels.
float bmpPressureHPaArray[] = { 8.7, 882.64, 1083.8 };                      // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.
float bmpAltitudeMArray[]   = { -413.0, 1337.0, 3108.0 };                   // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.


// Create class objects.
WiFiClient espClient;                  // Network client.
PubSubClient mqttClient( espClient );  // MQTT client.
Adafruit_HTU21DF htu21d = Adafruit_HTU21DF();
Adafruit_BMP280 bmp280;


void onMessage( char *topic, byte *payload, unsigned int length );
void configureOTA();
void setupHTU21D();
void setupBMP280();
void wifiMultiConnect();
int checkForSSID( const char *ssidName );
void mqttConnect();
void readTelemetry();
void printTelemetry();
void lookupWifiCode( int code, char *buffer );
void lookupMQTTCode( int code, char *buffer );
void publishTelemetry();
void toggleLED();
float cToF( float value );
float mToF( float value );

#endif  //ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H
