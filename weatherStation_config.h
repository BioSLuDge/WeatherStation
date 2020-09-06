#ifndef WEATHERSTATION_CONF
#define WEATHERSTATION_CONF

//203
#define WIND_DIR_OFFSET (360-203)

#define UV_ADDRESS 0x60

#define IO_LOOP_DELAY (1000*60*2) //every 2 minute

#define IO_WINDDIR_FEED "wind-direction"
#define IO_WINDSPEEDMPH_FEED "wind-speed"

#define IO_WINDGUSTMPH_10M_FEED "wind-gust-10-minute"
#define IO_WINDGUSTDIR_10M_FEED "wind-gust-direction-10-minute"

#define IO_WINDGUSTMPH_FEED "daily-wind-gust"
#define IO_WINDGUSTDIR_FEED "daily-wind-gust-direction"

#define IO_RAININ_FEED "rain-last-hour"
#define IO_DAILYRAININ_FEED "rain-daily"

#define IO_BATTERY_FEED "battery"
#define IO_CHARGER_STATUS_FEED "charger-status"
#define IO_SOLAR_FEED "solar"

#define IO_VISIBILITY_FEED "visibility"
#define IO_UV_FEED "uv"
#define IO_IR_FEED "ir"

#define IO_TEMP_FEED "temperature"
#define IO_HUMIDITY_FEED "humidity"

#define IO_LATITUDE 1
#define IO_LONGITUDINAL 1
#define IO_ELEVATION 1

#define IO_PERSISION 4
#endif
