#include <Adafruit_SI1145.h>
#include <WeatherStation.h>
#include <Adafruit_AM2315.h>
#include <Arduino.h>
#include "RTC_AdafruitIO.h"

#include "config.h"
#include "weatherStation_config.h"

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 12;
const byte RAIN = 11;

const byte STAT1 = 5;
const byte STAT2 = 6;


// analog I/O pins
const byte BATTERY = A1;
const byte WDIR = A3;
const byte SOLAR = A2;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
Adafruit_SI1145 uv;      //light sensor
Adafruit_AM2315 temp;          //temp

boolean tempFound = false;
boolean uvFound = false;

unsigned long lastUpdate = 0;

int16_t get_wind_direction();

RTC_AdafruitIO rtc(MOUNTAIN, true);
WeatherStation ws(get_wind_direction, &rtc);

/****************************** Feeds ***************************************/

// Setup a feed called 'data' for publishing.
AdafruitIO_Feed *data_windspeedmph = io.feed(IO_WINDSPEEDMPH_FEED);
AdafruitIO_Feed *data_windgustmph = io.feed(IO_WINDGUSTMPH_FEED);
AdafruitIO_Feed *data_windgustmph_10m = io.feed(IO_WINDGUSTMPH_10M_FEED);
AdafruitIO_Feed *data_rainin = io.feed(IO_RAININ_FEED);
AdafruitIO_Feed *data_dailyrainin = io.feed(IO_DAILYRAININ_FEED);
AdafruitIO_Feed *data_winddir = io.feed(IO_WINDDIR_FEED);
AdafruitIO_Feed *data_windgustdir = io.feed(IO_WINDGUSTDIR_FEED);
AdafruitIO_Feed *data_windgustdir_10m = io.feed(IO_WINDGUSTDIR_10M_FEED);
AdafruitIO_Feed *data_battery = io.feed(IO_BATTERY_FEED);
AdafruitIO_Feed *data_charger_status = io.feed(IO_CHARGER_STATUS_FEED);
AdafruitIO_Feed *data_solar = io.feed(IO_SOLAR_FEED);
AdafruitIO_Feed *data_visibility = io.feed(IO_VISIBILITY_FEED);
AdafruitIO_Feed *data_ir = io.feed(IO_IR_FEED);
AdafruitIO_Feed *data_uv = io.feed(IO_UV_FEED);
AdafruitIO_Feed *data_humidity = io.feed(IO_HUMIDITY_FEED);
AdafruitIO_Feed *data_temp = io.feed(IO_TEMP_FEED);


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void rainIRQ() {
  ws.rainIRQ();
}

void wspeedIRQ() {
  ws.wspeedIRQ();
}

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Hello");

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.setAPN(F("h2g2"));
  io.connect();
 
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  // Begin RTC
  RTC_AdafruitIO::begin(&io);
  
  Serial.print("Setup UV: ");
  uvFound = uv.begin(UV_ADDRESS);
  Serial.println(uvFound);

  Serial.print("Temp Setup: ");
  tempFound = temp.begin();
  Serial.println(tempFound);

  Serial.println("Setting up interrupts");
  // attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  // Setup charger pins
  Serial.println("Setting up DONE and CHARGE pins");
  pinMode(STAT1, INPUT_PULLUP);
  pinMode(STAT2, INPUT_PULLUP);
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

  if(!uvFound) {
    Serial.print("Searching for UV: ");
    uvFound = uv.begin(UV_ADDRESS);
    Serial.println(uvFound);    
  }

  if(!tempFound) {
    Serial.println("Searching for Temp: ");
    tempFound = temp.begin();
    Serial.println(tempFound);    
  }
  
  delay(700);
}

float tempToF(float c) {
  return c * 1.8 + 32;
}

void reportData() {
  Serial.println("Reporting data");
  ws.calcWindGust();

  if (tempFound) {
    Serial.println("Reading TEMP");
    float tempc, humidity;
    temp.readTemperatureAndHumidity(&tempc, &humidity);
    float tempf = tempToF(tempc);
    
    data_temp->save(tempf);
    data_humidity->save(humidity);
  }

  if (uvFound) {
    Serial.println("Reading UV");

    data_uv->save(uv.readUV());
    data_ir->save(uv.readIR());
    data_visibility->save(uv.readVisible());
  }

  data_rainin->save(ws.getRainIn());  

  data_windgustmph_10m->save(ws.getWindGustMPH_10m());
  data_windgustdir_10m->save(ws.getWindGustDir_10m());

  data_winddir->save(ws.getWindDir_Avg2m());
  data_windspeedmph->save(ws.getWindSpeedMPH_Avg2m()); 
  
  data_solar->save(getSolarLevel());
  data_battery->save(getBatteryLevel());
  data_charger_status->save(getChargeCircuitStatus());

  data_windgustmph->save(ws.getWindGustMPH());
  data_windgustdir->save(ws.getWindGustDir());
  data_dailyrainin->save(ws.getDailyRainIn());

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

String chargeStatus[2][2] = {{"Temperature Fault","Charged"},{"Charging","No Input Power"}};
//Gets charge circuit status
String getChargeCircuitStatus() {
  int stat1 = digitalRead(STAT1);
  int stat2 = digitalRead(STAT2);

  return chargeStatus[stat1][stat2];
}

//Takes an average of readings on a given pin
//Returns the average
int16_t averageAnalogRead() {
  uint8_t numberOfReadings = 8;
  int16_t runningValue = 0;

  for (int16_t x = 0 ; x < numberOfReadings ; x++) {
    int windir = analogRead(WDIR);
    runningValue += (windir >> 2);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

int16_t get_wind_direction() {
  // read the wind direction sensor, return heading in degrees
  int16_t adc, result;

  adc = averageAnalogRead(); // get the current reading from the sensor

  //Calc voltage
  int volts = (adc * 2 * 3.3);

  if (volts < 490) result = (113);
  else if (volts < 508) result = (68);
  else if (volts < 525) result = (90);
  else if (volts < 570) result = (158);
  else if (volts < 630) result = (135);
  else if (volts < 700) result = (203);
  else if (volts < 740) result = (180);
  else if (volts < 865) result = (23);
  else if (volts < 915) result = (45);
  else if (volts < 1015) result = (248);
  else if (volts < 1035) result = (225);
  else if (volts < 1090) result = (315);
  else if (volts < 1165) result = (0);
  else if (volts < 1230) result = (293);
  else if (volts < 1250) result = (270);
  else return (-1); // error, disconnected?

  //normalize result
  result += WIND_DIR_OFFSET;
  result = result % 360;
  
  return result;
}

float getSolarLevel() {
  int level = analogRead(BATTERY);
  return convertToVolts(analogRead(SOLAR));
}

float getBatteryLevel() {
  int level = analogRead(BATTERY);
  return convertToVolts(analogRead(BATTERY));
}

float convertToVolts(int level){
  return (level * 2 * 3.3) / 1024;
}
