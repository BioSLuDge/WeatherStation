#include <Math.h>
#include <ESP8266WiFi.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_ADS1015.h>
#include <WeatherStation.h>
#include <Adafruit_AM2315.h>
#include <Arduino.h>

#include "weatherStation_config.h"
#include "floatToString.h"

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 13;
const byte RAIN = 12;

const byte CHARGE = 16;
const byte DONE = 14;

// analog I/O pins
const byte BATTERY = A0;

// analog I/O pins for the ADS1015
const byte WDIR = 1;
const byte SOLAR = 4;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
Adafruit_SI1145 uv;      //light sensor
Adafruit_MPL3115A2 baro;       //altitude and pressure
Adafruit_AM2315 temp;          //temp
Adafruit_ADS1015 ads1015;      //ADC

boolean tempFound = false;
boolean baroFound = false;
boolean uvFound = false;

int16_t get_wind_direction();

WeatherStation ws(get_wind_direction);

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup a feed called 'data' for publishing.
const char DATA_WINDSPEEDMPH_FEED[] PROGMEM = AIO_WINDSPEEDMPH_FEED;
Adafruit_MQTT_Publish data_windspeedmph = Adafruit_MQTT_Publish(&mqtt, DATA_WINDSPEEDMPH_FEED);

const char DATA_WINDGUSTMPH_FEED[] PROGMEM = AIO_WINDGUSTMPH_FEED;
Adafruit_MQTT_Publish data_windgustmph = Adafruit_MQTT_Publish(&mqtt, DATA_WINDGUSTMPH_FEED);

const char DATA_WINDSPDMPH_AVG2M_FEED[] PROGMEM = AIO_WINDSPDMPH_AVG2M_FEED;
Adafruit_MQTT_Publish data_windspdmph_avg2m = Adafruit_MQTT_Publish(&mqtt, DATA_WINDSPDMPH_AVG2M_FEED);

const char DATA_WINDGUSTMPH_10M_FEED[] PROGMEM = AIO_WINDGUSTMPH_10M_FEED;
Adafruit_MQTT_Publish data_windgustmph_10m = Adafruit_MQTT_Publish(&mqtt, DATA_WINDGUSTMPH_10M_FEED);

const char DATA_RAININ_FEED[] PROGMEM = AIO_RAININ_FEED;
Adafruit_MQTT_Publish data_rainin = Adafruit_MQTT_Publish(&mqtt, DATA_RAININ_FEED);

const char DATA_DAILYRAININ_FEED[] PROGMEM = AIO_DAILYRAININ_FEED;
Adafruit_MQTT_Publish data_dailyrainin = Adafruit_MQTT_Publish(&mqtt, DATA_DAILYRAININ_FEED);

const char DATA_WINDDIR_FEED[] PROGMEM = AIO_WINDDIR_FEED;
Adafruit_MQTT_Publish data_winddir = Adafruit_MQTT_Publish(&mqtt, DATA_WINDDIR_FEED);

const char DATA_WINDGUSTDIR_FEED[] PROGMEM = AIO_WINDGUSTDIR_FEED;
Adafruit_MQTT_Publish data_windgustdir = Adafruit_MQTT_Publish(&mqtt, DATA_WINDGUSTDIR_FEED);

const char DATA_WINDDIR_AVG2M_FEED[] PROGMEM = AIO_WINDDIR_AVG2M_FEED;
Adafruit_MQTT_Publish data_winddir_avg2m = Adafruit_MQTT_Publish(&mqtt, DATA_WINDDIR_AVG2M_FEED);

const char DATA_WINDGUSTDIR_10M_FEED[] PROGMEM = AIO_WINDGUSTDIR_10M_FEED;
Adafruit_MQTT_Publish data_windgustdir_10m = Adafruit_MQTT_Publish(&mqtt, DATA_WINDGUSTDIR_10M_FEED);

const char DATA_BATTERY_FEED[] PROGMEM = AIO_BATTERY_FEED;
Adafruit_MQTT_Publish data_battery = Adafruit_MQTT_Publish(&mqtt, DATA_BATTERY_FEED);

const char DATA_DONE_FEED[] PROGMEM = AIO_DONE_FEED;
Adafruit_MQTT_Publish data_done = Adafruit_MQTT_Publish(&mqtt, DATA_DONE_FEED);

const char DATA_CHARGE_FEED[] PROGMEM = AIO_CHARGE_FEED;
Adafruit_MQTT_Publish data_charge = Adafruit_MQTT_Publish(&mqtt, DATA_CHARGE_FEED);

const char DATA_SOLAR_FEED[] PROGMEM = AIO_SOLAR_FEED;
Adafruit_MQTT_Publish data_solar = Adafruit_MQTT_Publish(&mqtt, DATA_SOLAR_FEED);

const char DATA_VISIBILITY_FEED[] PROGMEM = AIO_VISIBILITY_FEED;
Adafruit_MQTT_Publish data_visibility = Adafruit_MQTT_Publish(&mqtt, DATA_VISIBILITY_FEED);

const char DATA_IR_FEED[] PROGMEM = AIO_IR_FEED;
Adafruit_MQTT_Publish data_ir = Adafruit_MQTT_Publish(&mqtt, DATA_IR_FEED);

const char DATA_UV_FEED[] PROGMEM = AIO_UV_FEED;
Adafruit_MQTT_Publish data_uv = Adafruit_MQTT_Publish(&mqtt, DATA_UV_FEED);

const char DATA_DEWPT_FEED[] PROGMEM = AIO_DEWPT_FEED;
Adafruit_MQTT_Publish data_dewpt = Adafruit_MQTT_Publish(&mqtt, DATA_DEWPT_FEED);

const char DATA_HUMIDITY_FEED[] PROGMEM = AIO_HUMIDITY_FEED;
Adafruit_MQTT_Publish data_humidity = Adafruit_MQTT_Publish(&mqtt, DATA_HUMIDITY_FEED);

const char DATA_BAROMIN_FEED[] PROGMEM = AIO_BAROMIN_FEED;
Adafruit_MQTT_Publish data_baromin = Adafruit_MQTT_Publish(&mqtt, DATA_BAROMIN_FEED);

const char DATA_TEMP_FEED[] PROGMEM = AIO_TEMP_FEED;
Adafruit_MQTT_Publish data_temp = Adafruit_MQTT_Publish(&mqtt, DATA_TEMP_FEED);

const char DATA_TEMP2_FEED[] PROGMEM = AIO_TEMP2_FEED;
Adafruit_MQTT_Publish data_temp2 = Adafruit_MQTT_Publish(&mqtt, DATA_TEMP2_FEED);

const char DATA_PRESSURE_FEED[] PROGMEM = AIO_PRESSURE_FEED;
Adafruit_MQTT_Publish data_pressure = Adafruit_MQTT_Publish(&mqtt, DATA_PRESSURE_FEED);

const char DATA_ALTITUDE_FEED[] PROGMEM = AIO_ALTITUDE_FEED;
Adafruit_MQTT_Publish data_altitude = Adafruit_MQTT_Publish(&mqtt, DATA_ALTITUDE_FEED);

// Setup a feed called 'reset' for subscribing to changes.
const char RESET_FEED[] PROGMEM = AIO_RESET_FEED;
Adafruit_MQTT_Subscribe reset = Adafruit_MQTT_Subscribe(&mqtt, RESET_FEED);

// Setup a feed called 'reset' for subscribing to changes.
const char READ_FEED[] PROGMEM = AIO_READ_FEED;
Adafruit_MQTT_Subscribe read = Adafruit_MQTT_Subscribe(&mqtt, READ_FEED);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void powerSaver() {

}

void rainIRQ() {
  ws.rainIRQ();
}

void wspeedIRQ() {
  ws.wspeedIRQ();
}

void setup() {
  powerSaver();

  Serial.begin(115200);
  Serial.println("");
  Serial.println("Hello");

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  // ws.setup();
  Serial.print("Setup UV: ");
  uvFound = uv.begin();
  Serial.println(uvFound);

  Serial.print("Baro Setup: ");
  baroFound = baro.begin();
  Serial.println(baroFound);

  Serial.print("Temp Setup: ");
  tempFound = temp.begin();
  Serial.println(tempFound);

  Serial.println("ADC Setup");
  ads1015.begin();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  Serial.println("Connecting to WIFI");
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  // Setup MQTT subscriptions.
  Serial.println("Subscribing to reset");
  mqtt.subscribe(&reset);
  Serial.println("Subscribing to read");
  mqtt.subscribe(&read);

  // Setup charger pins
  Serial.println("Setting up DONE and CHARGE pins");
  pinMode(DONE, INPUT);
  pinMode(CHARGE, INPUT);
}

void resetFunc() {
  ESP.reset();
}

void loop() {
  ws.update();

  if (WiFi.status() == WL_CONNECTED) {
    if (mqtt.connected()) {

      Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqtt.readSubscription(5000))) {
        if (subscription == &reset) {
          Serial.print(F("Got: "));
          Serial.println((char *)reset.lastread);
          resetFunc();
        }
        else if (subscription == &read) {
          Serial.print(F("Got: "));
          Serial.println((char *)read.lastread);
          reportData();
        }
      }
    }
    else {
      int8_t ret;

      if ((ret = mqtt.connect()) != 0) {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(4000);
      }
    }
  }

  delay(1000);
}

void reportData() {
  Serial.println("Reporting data");
  
  char buffer[25]; 

  String weatherString = "";

  if (baroFound) {
    Serial.println("Reading Baro");
    Serial.println("Reading Temp");
    float tempc2 = baro.getTemperature();
    Serial.println("Reading pressure");
    float pressure = baro.getPressure();
    Serial.println("Reading Altitude");
    float altitude = baro.getAltitude();

    float temp2f = tempToF(tempc2);
    float baromin = pressureToIM(pressure, altitude);

    data_altitude.publish((uint8_t *)floatToString(buffer, altitude, FLOAT_PERSISION), strlen(buffer));
    data_temp2.publish((uint8_t *)floatToString(buffer, temp2f, FLOAT_PERSISION), strlen(buffer));
    data_baromin.publish((uint8_t *)floatToString(buffer, baromin, FLOAT_PERSISION), strlen(buffer));
    data_pressure.publish((uint8_t *)floatToString(buffer, pressure, FLOAT_PERSISION), strlen(buffer));
  }

  if (tempFound) {
    Serial.println("Reading TEMP");
    float tempc, humidity;
    temp.readTemperatureAndHumidity(tempc, humidity);
    float tempf = tempToF(tempc);
    float dewptf = calcDewPoint(humidity, tempf);
    
    data_dewpt.publish((uint8_t *)floatToString(buffer, dewptf, FLOAT_PERSISION), strlen(buffer));
    data_temp.publish((uint8_t *)floatToString(buffer, tempf, FLOAT_PERSISION), strlen(buffer));
    data_humidity.publish((uint8_t *)floatToString(buffer, humidity, FLOAT_PERSISION), strlen(buffer));
  }

  if (uvFound) {
    Serial.println("Reading UV");
    String UV = String(uv.readUV());
    String IR = String(uv.readIR());
    String visibility = String(uv.readVisible());

    UV.toCharArray(buffer, sizeof(buffer));
    data_uv.publish((uint8_t *)buffer, strlen(buffer));
    IR.toCharArray(buffer, sizeof(buffer));
    data_ir.publish((uint8_t *)buffer, strlen(buffer));
    visibility.toCharArray(buffer, sizeof(buffer));
    data_visibility.publish((uint8_t *)buffer, strlen(buffer));
  }

  String winddir = String(ws.getWindDir());
  float windspeedmph = ws.getWindSpeedMPH();
  float windgustmph = ws.getWindGustMPH();
  String windgustdir = String(ws.getWindGustDir());
  float windspdmph_avg2m = ws.getWindSpeedMPH_Avg2m();
  String winddir_avg2m = String(ws.getWindDir_Avg2m());
  float windgustmph_10m = ws.getWindGustMPH_10m();
  String windgustdir_10m = String(ws.getWindGustDir_10m());
  float rainin = ws.getRainIn();
  float dailyrainin = ws.getDailyRainIn();

  data_dailyrainin.publish((uint8_t *)floatToString(buffer, dailyrainin, FLOAT_PERSISION), strlen(buffer));
  data_rainin.publish((uint8_t *)floatToString(buffer, rainin, FLOAT_PERSISION), strlen(buffer));  
  data_windgustmph_10m.publish((uint8_t *)floatToString(buffer, windgustmph_10m, FLOAT_PERSISION), strlen(buffer));
  data_windspdmph_avg2m.publish((uint8_t *)floatToString(buffer, windspdmph_avg2m, FLOAT_PERSISION), strlen(buffer));  
  data_windgustmph.publish((uint8_t *)floatToString(buffer, windgustmph, FLOAT_PERSISION), strlen(buffer));
  data_windspeedmph.publish((uint8_t *)floatToString(buffer, windspeedmph, FLOAT_PERSISION), strlen(buffer)); 

  windgustdir_10m.toCharArray(buffer, sizeof(buffer));
  data_windgustdir_10m.publish((uint8_t *)buffer, strlen(buffer));
  winddir_avg2m.toCharArray(buffer, sizeof(buffer));
  data_winddir_avg2m.publish((uint8_t *)buffer, strlen(buffer));
  windgustdir.toCharArray(buffer, sizeof(buffer));
  data_windgustdir.publish((uint8_t *)buffer, strlen(buffer));
  winddir.toCharArray(buffer, sizeof(buffer));
  data_winddir.publish((uint8_t *)buffer, strlen(buffer));

  String done = String(!digitalRead(DONE));
  String charge = String(!digitalRead(CHARGE));

  data_solar.publish((uint8_t *)floatToString(buffer, getSolarLevel(), FLOAT_PERSISION), strlen(buffer));
  data_battery.publish((uint8_t *)floatToString(buffer, getBatteryLevel(), FLOAT_PERSISION), strlen(buffer));
  done.toCharArray(buffer, sizeof(buffer));
  data_done.publish((uint8_t *)buffer, strlen(buffer));
  charge.toCharArray(buffer, sizeof(buffer));
  data_charge.publish((uint8_t *)buffer, strlen(buffer));

  /*
    winddir - [0-360 instantaneous wind direction]
    windspeedmph - [mph instantaneous wind speed]
    windgustmph - [mph current wind gust, using software specific time period]
    windgustdir - [0-360 using software specific time period]

    windspdmph_avg2m  - [mph 2 minute average wind speed mph]
    winddir_avg2m - [0-360 2 minute average wind direction]
    windgustmph_10m - [mph past 10 minutes wind gust mph ]
    windgustdir_10m - [0-360 past 10 minutes wind gust direction]

    humidity - [% outdoor humidity 0-100%]
    dewptf- [F outdoor dewpoint F]
    tempf - [F outdoor temperature]
     for extra outdoor sensors use temp2f, temp3f, and so on
    rainin - [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
    dailyrainin - [rain inches so far today in local time]
    baromin - [barometric pressure inches]
    UV - [index]
    indoortempf - [F indoor temperature F]
    indoorhumidity - [% indoor humidity 0-100]
  */
}

//Takes an average of readings on a given pin
//Returns the average
int16_t averageAnalogRead() {
  uint8_t numberOfReadings = 8;
  int16_t runningValue = 0;

  for (int16_t x = 0 ; x < numberOfReadings ; x++)
    runningValue += (ads1015.readADC_SingleEnded(WDIR) >> 2);
  runningValue /= numberOfReadings;

  return (runningValue);
}

int16_t get_wind_direction() {
  // read the wind direction sensor, return heading in degrees
  int16_t adc, result;

  adc = averageAnalogRead(); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) result = (113);
  else if (adc < 393) result = (68);
  else if (adc < 414) result = (90);
  else if (adc < 456) result = (158);
  else if (adc < 508) result = (135);
  else if (adc < 551) result = (203);
  else if (adc < 615) result = (180);
  else if (adc < 680) result = (23);
  else if (adc < 746) result = (45);
  else if (adc < 801) result = (248);
  else if (adc < 833) result = (225);
  else if (adc < 878) result = (338);
  else if (adc < 913) result = (0);
  else if (adc < 940) result = (293);
  else if (adc < 967) result = (315);
  else if (adc < 990) result = (270);
  else return (-1); // error, disconnected?

  result += WIND_DIR_OFFSET;
  while (result < 0) result += 360;
  while (result > 360) result -= 360;

  return result;
}

float getSolarLevel() {
  int level = ads1015.readADC_SingleEnded(SOLAR);

  // convert solar to percent
  level = map(level, 580, 774, 314, 420);

  return (float)level / (100.0);
}

float getBatteryLevel() {
  int level = analogRead(A0);

  // convert battery level to percent
  level = map(level, 580, 774, 314, 420);

  return (float)level / (100.0);
}

//With relative humidity and temp, calculate a dew point
//From: http://ag.arizona.edu/azmet/dewpoint.html
float calcDewPoint(float humidity, float tempF) {
  float tempC = (tempF - 32) * 5 / 9.0;

  float L = log(humidity / 100.0);
  float M = 17.27 * tempC;
  float N = 237.3 + tempC;
  float B = (L + (M / N)) / 17.27;
  float dewPoint = (237.3 * B) / (1.0 - B);

  //Result is in C
  //Convert back to F
  dewPoint = dewPoint * 9 / 5.0 + 32;

  return (dewPoint);
}

//From Nathan https://github.com/sparkfun/Wimp_Weather_Station/blob/master/agent.nut
//Thank you spark fun
float pressureToIM(float pressure, float altitude) {
  float pressure_mb = pressure / 100; //pressure is now in millibars, 1 pascal = 0.01 millibars

  float part1 = pressure_mb - 0.3; //Part 1 of formula
  float part2 = 8.42288 / 100000.0;
  float part3 = pow((pressure_mb - 0.3), 0.190284);
  float part4 = altitude / part3;
  float part5 = (1.0 + (part2 * part4));
  float part6 = pow(part5, (1.0 / 0.190284));
  float altimeter_setting_pressure_mb = part1 * part6; //Output is now in adjusted millibars
  float baromin = altimeter_setting_pressure_mb * 0.02953;

  return (baromin);
}

float tempToF(float c) {
  return c * 1.8 + 32;
}
