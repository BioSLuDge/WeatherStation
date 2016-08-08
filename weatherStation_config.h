#ifndef WEATHERSTATION_CONF
#define WEATHERSTATION_CONF

#define WIND_DIR_OFFSET 0

#define WLAN_SSID       "YOUR WIFI SSID"
#define WLAN_PASS       "YOUR WIFI PASSWORD"

#define AIO_SERVER      "MQTT HOSTNAME"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "MQTT USERNAME"
#define AIO_KEY         "MQTT PASSWORD"

#define AIO_RESET_FEED       "myfeed/reset"
#define AIO_READ_FEED        "myfeed/read"
#define AIO_WINDSPEEDMPH_FEED "myfeed/windspeedmph";
#define AIO_WINDGUSTMPH_FEED "myfeed/windgustmph";
#define AIO_WINDSPDMPH_AVG2M_FEED "myfeed/windspdmph_avg2m";
#define AIO_WINDGUSTMPH_10M_FEED "myfeed/windgustmph_10m";
#define AIO_RAININ_FEED "myfeed/rainin";
#define AIO_DAILYRAININ_FEED "myfeed/dailyrainin";
#define AIO_WINDDIR_FEED "myfeed/winddir";
#define AIO_WINDGUSTDIR_FEED "myfeed/windgustdir";
#define AIO_WINDDIR_AVG2M_FEED "myfeed/winddir_avg2m";
#define AIO_WINDGUSTDIR_10M_FEED "myfeed/windgustdir_10m";
#define AIO_BATTERY_FEED "myfeed/battery";
#define AIO_DONE_FEED "myfeed/done";
#define AIO_CHARGE_FEED "myfeed/charge";
#define AIO_SOLAR_FEED "myfeed/solar";
#define AIO_VISIBILITY_FEED "myfeed/visibility";
#define AIO_IR_FEED "myfeed/ir";
#define AIO_DEWPT_FEED "myfeed/dewpt";
#define AIO_HUMIDITY_FEED "myfeed/humidity";
#define AIO_UV_FEED "myfeed/uv";
#define AIO_BAROMIN_FEED "myfeed/baromin";
#define AIO_TEMP_FEED "myfeed/temp";
#define AIO_TEMP2_FEED "myfeed/temp2";
#define AIO_PRESSURE_FEED "myfeed/pressure";
#define AIO_ALTITUDE_FEED "myfeed/altitude";

#define FLOAT_PERSISION 4
#endif
