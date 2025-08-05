/*******************************************************************************
*
*       Copyright (c) PEAK Techno, 2021
*         http://www.peak-techno.com
*
********************************************************************************
*@file          : Gps.h
*@project       : Tracker
*@version       : $Id: Gps.h
*@brief         :
*******************************************************************************/
#ifndef _GPS_H
#define _GPS_H


#ifndef NMEA_FLOAT_T
#define NMEA_FLOAT_T float ///< let float be overidden on command line
#endif

//typedef uint8_t bool;
#define bool	_Bool

typedef NMEA_FLOAT_T
    nmea_float_t; ///< the type of variables to use for floating point

typedef enum
{
  GPS_RESET = 0U,
  GPS_SET,
  GPS_STARTING,
  GPS_POS_FOUND, //Position found
  GPS_POS_LOST,   //Position lost
  GPS_ERROR
} gps_state_t;



typedef struct
{
  uint8_t hour;          ///< GMT hours
  uint8_t minute;        ///< GMT minutes
  uint8_t seconds;       ///< GMT seconds
  uint16_t milliseconds; ///< GMT milliseconds
  uint8_t year;          ///< GMT year
  uint8_t month;         ///< GMT month
  uint8_t day;           ///< GMT day


  /** Fixed point latitude and longitude value with degrees stored in units of
    1/10000000 of a degree. See pull #13 for more details:
    https://github.com/adafruit/Adafruit-GPS-Library/pull/13 */
  int32_t latitude_fixed;  ///< Fixed point latitude in decimal degrees.
                           ///< Divide by 10000000.0 to get a double.
  int32_t longitude_fixed; ///< Fixed point longitude in decimal degrees
                           ///< Divide by 10000000.0 to get a double.

  nmea_float_t geoidheight;      ///< Diff between geoid height and WGS84 height
  nmea_float_t altitude;         ///< Altitude in meters above MSL
  nmea_float_t speed;            ///< Current speed over ground in knots
  nmea_float_t angle;            ///< Course in degrees from true north
  nmea_float_t magvariation; ///< Magnetic variation in degrees (vs. true north)
  nmea_float_t HDOP; ///< Horizontal Dilution of Precision - relative accuracy
                     ///< of horizontal position
  char lat;    ///< N/S
  char lon;    ///< E/W
  char mag;    ///< Magnetic variation direction
  bool fix;          ///< Have a fix?
  uint8_t fixquality;    ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  uint8_t satellites;    ///< Number of satellites in use
  uint8_t sat_in_view;    ///< Number of satellites in view
  uint32_t timestamp;
}gps_data_t;


extern gps_data_t tnl_tmp_gps_data;
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif


bool 	TRK_GpsEnable(uint8_t enable);
bool 	TRK_GpsReadProcessing(uint8_t *buffer, uint16_t buffer_size);
bool 	TRK_GpsRead();
bool 	TRK_GpsConfigure();
bool 	TRK_GpsNewPos();
uint8_t TRK_SearchSatellite();

#define MAXLINELENGTH 120 ///< how long are max NMEA lines to parse?
#define NMEA_MAX_SENTENCE_ID                                                   \
  20 ///< maximum length of a sentence ID name, including terminating 0
#define NMEA_MAX_SOURCE_ID                                                     \
  3 ///< maximum length of a source ID name, including terminating 0


// type for resulting code from running check()
typedef enum {
  NMEA_BAD = 0, // passed none of the checks
  NMEA_HAS_DOLLAR =
      1, ///< has a dollar sign or exclamation mark in the first position
  NMEA_HAS_CHECKSUM = 2,   ///< has a valid checksum at the end
  NMEA_HAS_NAME = 4,       ///< there is a token after the $ followed by a comma
  NMEA_HAS_SOURCE = 10,    ///< has a recognized source ID
  NMEA_HAS_SENTENCE = 20,  ///< has a recognized sentence ID
  NMEA_HAS_SENTENCE_P = 40 ///< has a recognized parseable sentence ID
} nmea_check_t;


  size_t TRK_GpsAvailable(void);
  size_t TRK_GpsWrite(uint8_t);
  void TRK_GpsSendCommand(const char *);
  bool TRK_GpsNewNMEAreceived();
  void TRK_GpsPause(bool b);
  char *TRK_GpsLastNMEA(void);
  bool TRK_GpsStandby(void);
  bool TRK_GpsWakeup(void);
  nmea_float_t TRK_GpsSecondsSinceFix();
  nmea_float_t TRK_GpsSecondsSinceTime();
  nmea_float_t TRK_GpsSecondsSinceDate();
  void TRK_GpsResetSentTime();

  // NMEA_parse.cpp
  bool TRK_GpsParse(char *nmea);
  bool TRK_GpsCheck(char *nmea);
  bool TRK_GpsOnList(char *nmea, const char **list);
  uint8_t TRK_GpsParseHex(char c);

  // NMEA_parse.cpp
  const char *TRK_GpsTokenOnList(char *token, const char **list);
  bool TRK_GpsParseCoord(char *p, int32_t *angle_fixed, char *dir);
  char *TRK_GpsParseStr(char *buff, char *p, int n);
  bool TRK_GpsParseTime(char *);
  bool TRK_GpsParseSV(char *p);
  bool TRK_GpsParseFix(char *);
  bool TRK_GpsIsEmpty(char *pStart);
  void TRK_GpsError();

float TRK_GpsDistanceTo(long lat_source, long lon_source, long lat_dest, long lon_dest) ;

extern uint8_t GPS_FIX;

uint32_t TRK_GpsMillis();
/**************************************************************************/


#endif //_GPS_H
