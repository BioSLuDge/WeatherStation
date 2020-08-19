#ifndef WEATHERSTATION_CONF
#define WEATHERSTATION_CONF

#define WIND_DIR_OFFSET 0

#define IO_LOOP_DELAY (1000*60*2) //every 2 minute

#define IO_RESET_FEED       "reset"
#define IO_WINDSPEEDMPH_FEED "wind-speed"
#define IO_WINDGUSTMPH_FEED "wind-gust"
#define IO_WINDSPDMPH_AVG2M_FEED "wind-speed-2-minute-average"
#define IO_WINDGUSTMPH_10M_FEED "wind-gust-10-minute-average"
#define IO_RAININ_FEED "rain"
#define IO_DAILYRAININ_FEED "rain-daily"
#define IO_WINDDIR_FEED "wind-direction"
#define IO_WINDGUSTDIR_FEED "wind-gust-direction"
#define IO_WINDDIR_AVG2M_FEED "wind-direction-2-minute-average"
#define IO_WINDGUSTDIR_10M_FEED "wind-gust-direction-10-minute-average"
#define IO_BATTERY_FEED "battery-power"
#define IO_DONE_FEED "battery-charged"
#define IO_CHARGED_FEED "charged"
#define IO_SOLAR_FEED "solar"
#define IO_VISIBILITY_FEED "visibility"
#define IO_IR_FEED "ir"
#define IO_DEWPT_FEED "dew-point"
#define IO_HUMIDITY_FEED "humidity"
#define IO_UV_FEED "uv"
#define IO_BAROMIN_FEED "baromin"
#define IO_TEMP_FEED "temperature"
#define IO_TEMP2_FEED "temperature-2"
#define IO_PRESSURE_FEED "pressure"
#define IO_ALTITUDE_FEED "altitude"

#define IO_LATITUDE 1
#define IO_LONGITUDINAL 1
#define IO_ELEVATION 1

#define IO_PERSISION 4
#endif
