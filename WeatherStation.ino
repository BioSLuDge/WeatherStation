#include <Math.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_ADS1015.h>
#include <WeatherStation.h>
#include <Adafruit_AM2315.h>
#include <Arduino.h>

#include "config.h"
#include "weatherStation_config.h"

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 12;
const byte RAIN = 13;

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

unsigned long lastUpdate = 0;

int16_t get_wind_direction();

WeatherStation ws(get_wind_direction);

/****************************** Feeds ***************************************/

// Setup a feed called 'data' for publishing.
AdafruitIO_Feed *data_windspeedmph = io.feed(IO_WINDSPEEDMPH_FEED);
AdafruitIO_Feed *data_windgustmph = io.feed(IO_WINDGUSTMPH_FEED);
AdafruitIO_Feed *data_windspdmph_avg2m = io.feed(IO_WINDSPDMPH_AVG2M_FEED);
AdafruitIO_Feed *data_windgustmph_10m = io.feed(IO_WINDGUSTMPH_10M_FEED);
AdafruitIO_Feed *data_rainin = io.feed(IO_RAININ_FEED);
AdafruitIO_Feed *data_dailyrainin = io.feed(IO_DAILYRAININ_FEED);
AdafruitIO_Feed *data_winddir = io.feed(IO_WINDDIR_FEED);
AdafruitIO_Feed *data_windgustdir = io.feed(IO_WINDGUSTDIR_FEED);
AdafruitIO_Feed *data_winddir_avg2m = io.feed(IO_WINDDIR_AVG2M_FEED);
AdafruitIO_Feed *data_windgustdir_10m = io.feed(IO_WINDGUSTDIR_10M_FEED);
AdafruitIO_Feed *data_battery = io.feed(IO_BATTERY_FEED);
AdafruitIO_Feed *data_done = io.feed(IO_DONE_FEED);
AdafruitIO_Feed *data_charge = io.feed(IO_CHARGED_FEED);
AdafruitIO_Feed *data_solar = io.feed(IO_SOLAR_FEED);
AdafruitIO_Feed *data_visibility = io.feed(IO_VISIBILITY_FEED);
AdafruitIO_Feed *data_ir = io.feed(IO_IR_FEED);
AdafruitIO_Feed *data_uv = io.feed(IO_UV_FEED);
AdafruitIO_Feed *data_dewpt = io.feed(IO_DEWPT_FEED);
AdafruitIO_Feed *data_humidity = io.feed(IO_HUMIDITY_FEED);
AdafruitIO_Feed *data_baromin = io.feed(IO_BAROMIN_FEED);
AdafruitIO_Feed *data_temp = io.feed(IO_TEMP_FEED);
AdafruitIO_Feed *data_temp2 = io.feed(IO_TEMP2_FEED);
AdafruitIO_Feed *data_pressure = io.feed(IO_PRESSURE_FEED);
AdafruitIO_Feed *data_altitude = io.feed(IO_ALTITUDE_FEED);

// Setup a feed called 'reset' for subscribing to changes.
AdafruitIO_Feed *reset = io.feed(IO_RESET_FEED);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void ICACHE_RAM_ATTR rainIRQ() {
  ws.rainIRQ();
}

void ICACHE_RAM_ATTR wspeedIRQ() {
  ws.wspeedIRQ();
}

void setup() {
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

  Serial.println("Setting up interrupts");
  // attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();
 
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  //setup reset trigger
  reset->onMessage(resetFunc);

  // Setup charger pins
  Serial.println("Setting up DONE and CHARGE pins");
  pinMode(DONE, INPUT);
  pinMode(CHARGE, INPUT);
}

void resetFunc(AdafruitIO_Data *data) {
  if(data->isTrue()) {
    ESP.reset();
  }
}

void loop() {
  Serial.println("Weather Station Update.");
  ws.update();

  Serial.println("Adafruit IO Run.");
  io.run();

  if(millis() > (lastUpdate + IO_LOOP_DELAY)) {
    reportData();
    lastUpdate = millis();
  }

  delay(1000);
}

void reportData() {
  Serial.println("Reporting data");
  
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

    data_altitude->save(altitude);
    data_temp2->save(temp2f);
    data_baromin->save(baromin);
    data_pressure->save(pressure);
  }

  if (tempFound) {
    Serial.println("Reading TEMP");
    float tempc, humidity;
    temp.readTemperatureAndHumidity(&tempc, &humidity);
    float tempf = tempToF(tempc);
    float dewptf = calcDewPoint(humidity, tempf);
    
    data_dewpt->save(dewptf);
    data_temp->save(tempf);
    data_humidity->save(humidity);
  }

  if (uvFound) {
    Serial.println("Reading UV");

    data_uv->save(uv.readUV());
    data_ir->save(uv.readIR());
    data_visibility->save(uv.readVisible());
  }

  data_dailyrainin->save(ws.getDailyRainIn());
  data_rainin->save(ws.getRainIn());  
  data_windgustmph_10m->save(ws.getWindGustMPH_10m());
  data_windspdmph_avg2m->save(ws.getWindSpeedMPH_Avg2m());  
  data_windgustmph->save(ws.getWindGustMPH());
  data_windspeedmph->save(ws.getWindSpeedMPH()); 
  data_windgustdir_10m->save(ws.getWindGustDir_10m());
  data_winddir_avg2m->save(ws.getWindDir_Avg2m());
  data_windgustdir->save(ws.getWindGustMPH());
  data_winddir->save(ws.getWindDir());
  data_solar->save(getSolarLevel());
  data_battery->save(getBatteryLevel());
  data_done->save(digitalRead(DONE));
  data_charge->save(digitalRead(CHARGE));

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
