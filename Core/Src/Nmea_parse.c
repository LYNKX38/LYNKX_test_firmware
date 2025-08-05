/**************************************************************************/
/*!
  @file NMEA_parse.cpp
  @mainpage Adafruit Ultimate GPS Breakout
  @section intro Introduction
  This is the Adafruit GPS library - the ultimate GPS library
  for the ultimate GPS module!
  Tested and works great with the Adafruit Ultimate GPS module
  using MTK33x9 chipset
  ------> http://www.adafruit.com/products/746
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  @section author Author
  Written by Limor Fried/Ladyada for Adafruit Industries.
  @section license License
  BSD license, check license.txt for more information
  All text above must be included in any redistribution
*/
/**************************************************************************/
#include "main.h"
//#include "tapnlink.h"
#include "Nmea_parse.h"
#include "gps.h"
#include "stdlib.h"

  int thisCheck = 0; ///< the results of the check on the current sentence
  char thisSource[NMEA_MAX_SOURCE_ID] = {
      0}; ///< the first two letters of the current sentence, e.g. WI, GP
  char thisSentence[NMEA_MAX_SENTENCE_ID] = {
      0}; ///< the next three letters of the current sentence, e.g. GLL, RMC
  char lastSource[NMEA_MAX_SOURCE_ID] = {
      0}; ///< the results of the check on the most recent successfully parsed
          ///< sentence
  char lastSentence[NMEA_MAX_SENTENCE_ID] = {
      0}; ///< the next three letters of the most recent successfully parsed
          ///< sentence, e.g. GLL, RMC




  char txtTXT[63] = {0}; ///< text content from most recent TXT sentence
  int txtTot = 0;        ///< total TXT sentences in group
  int txtID = 0;         ///< id of the text message
  int txtN = 0;          ///< the TXT sentence number

// used by check() for validity tests, room for future expansion
  const char *sources[8] = {"II", "WI", "GP", "GN", "BD", "GA", "P", "ZZZ"}; ///< valid source ids

  const char *sentences_parsed[7] = {"GGA", "GLL", "GSA", "RMC", "TXT", "GSV",
                                     "ZZZ"}; ///< parseable sentence ids
  const char *sentences_known[4] = {"DBT", "HDM", "HDT",
                                    "ZZZ"}; ///< known, but not parseable


  // Make all of these times far in the past by setting them near the middle of
  // the millis() range. Timing assumes that sentences are parsed promptly.
  uint32_t lastUpdate =
      2000000000L; ///< millis() when last full sentence successfully parsed
  uint32_t lastFix = 2000000000L;  ///< millis() when last fix received
  uint32_t lastTime = 2000000000L; ///< millis() when last time received
  uint32_t lastDate = 2000000000L; ///< millis() when last date received
  uint32_t recvdTime =
      2000000000L; ///< millis() when last full sentence received
  uint32_t sentTime = 2000000000L; ///< millis() when first character of last
                                   ///< full sentence received
  bool paused;

  uint8_t TRK_GpsParseResponse(char *response);

  bool noComms = false;

  int8_t _buff_max = -1, _buff_idx = 0;
  char last_char = 0;

  volatile char line1[MAXLINELENGTH]; ///< We double buffer: read one line in
                                      ///< and leave one for the main program
  volatile char line2[MAXLINELENGTH]; ///< Second buffer
  volatile uint8_t lineidx = 0; ///< our index into filling the current line
  volatile char *currentline;   ///< Pointer to current line buffer
  volatile char *lastline;      ///< Pointer to previous line buffer
  volatile bool recvdflag;      ///< Received flag
  volatile bool inStandbyMode;  ///< In standby flag

/**************************************************************************/
/*!
    @brief Parse a standard NMEA string and update the relevant variables.
   Sentences start with a $, then a two character source identifier, then a
   three character sentence identifier that defines the format, then a comma and
   more comma separated fields defined by the sentence name. There are many
   sentences listed that are not yet supported, including proprietary sentences
   that start with P, like the $PMTK commands to the GPS modules. See the
   build() function and http://fort21.ru/download/NMEAdescription.pdf for
   sentence descriptions.
   Encapsulated data sentences are supported by NMEA-183, and start with !
   instead of $. https://gpsd.gitlab.io/gpsd/AIVDM.html provides details
   about encapsulated data sentences used in AIS.
    parse() permits, but does not require Carriage Return and Line Feed at the
   end of sentences. The end of the sentence is recognized by the * for the
   checksum. parse() will not recognize a sentence without a valid checksum.
   NMEA_EXTENSIONS must be defined in order to parse more than basic
   GPS module sentences.
    @param nmea Pointer to the NMEA string
    @return True if successfully parsed, false if fails check or parsing
*/
/**************************************************************************/

/*******************************************************************************
* Function Name  : TRK_GpsParse
* Description    : Search first lin in the buffer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool TRK_GpsParse(char *nmea)
    {
  if (!TRK_GpsCheck(nmea))
    return false;
  // passed the check, so there's a valid source in thisSource and a valid
  // sentence in thisSentence
  char *p = nmea; // Pointer to move through the sentence -- good parsers are
                  // non-destructive
  p = strchr(p, ',') + 1; // Skip to char after the next comma, then check.

  // This may look inefficient, but an M0 will get down the list in about 1 us /
  // strcmp()! Put the GPS sentences from Adafruit_GPS at the top to make
  // pruning excess code easier. Otherwise, keep them alphabetical for ease of
  // reading.
  if (!strcmp(thisSentence, "GGA")) { //************************************GGA
    // Adafruit from Actisense NGW-1 from SH CP150C
    if (!TRK_GpsParseTime(p))
        return false;
    p = strchr(p, ',') + 1; // parse time with specialized function
//    // parse out both latitude and direction, then go to next field, or fail
//    if (!TRK_GpsParseCoord(p, &tnl_tmp_gps_data.latitude_fixed, &tnl_tmp_gps_data.lat))
//      return false;
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
//    // parse out both longitude and direction, then go to next field, or fail
//    if (!TRK_GpsParseCoord(p, &tnl_tmp_gps_data.longitude_fixed, &tnl_tmp_gps_data.lon))
//      return false;
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p)) { // if it's a , (or a * at end of sentence) the value is
                       // not included
      tnl_tmp_gps_data.fixquality = atoi(p); // needs additional processing
      if (tnl_tmp_gps_data.fixquality > 0)
        {
        tnl_tmp_gps_data.fix = true;
        lastFix = sentTime;
        }
      else
        tnl_tmp_gps_data.fix = false;
    }
    p = strchr(p, ',') + 1; // then move on to the next
    // Most can just be parsed with atoi() or atof(), then move on to the next.
    if (!TRK_GpsIsEmpty(p))
      tnl_tmp_gps_data.satellites = atoi(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      tnl_tmp_gps_data.HDOP = atof(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      tnl_tmp_gps_data.altitude = atof(p);
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1; // skip the units
    if (!TRK_GpsIsEmpty(p))
      tnl_tmp_gps_data.geoidheight = atof(p); // skip the rest

    return tnl_tmp_gps_data.fix;

  } else if (!strcmp(thisSentence, "RMC")) { //*****************************RMC

    //parse time
//    TRK_GpsParseTime(p);
    p = strchr(p, ',') + 1;
    //parse fix
    TRK_GpsParseFix(p);
    p = strchr(p, ',') + 1;
    // parse out both latitude and direction, then go to next field, or fail
    if (!TRK_GpsParseCoord(p, &tnl_tmp_gps_data.latitude_fixed, &tnl_tmp_gps_data.lat))
      {}
//     { return false;}
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    // parse out both longitude and direction, then go to next field, or fail
    if (!TRK_GpsParseCoord(p, &tnl_tmp_gps_data.longitude_fixed, &tnl_tmp_gps_data.lon))
      {}
//     { return false;}
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    //parse speed
//    if (!TRK_GpsIsEmpty(p))
//      tnl_tmp_gps_data.speed = atof(p);
    p = strchr(p, ',') + 1;
    //parse angle
//    if (!TRK_GpsIsEmpty(p))
//      tnl_tmp_gps_data.angle = atof(p);
    p = strchr(p, ',') + 1;
    //parse date
    if (!TRK_GpsIsEmpty(p))
      {
      uint32_t fulldate = atof(p);
      tnl_tmp_gps_data.day = fulldate / 10000;
      tnl_tmp_gps_data.month = (fulldate % 10000) / 100;
      tnl_tmp_gps_data.year = (fulldate % 100);
      lastDate = sentTime;
      } // skip the rest

    return tnl_tmp_gps_data.fix;
    }

  else if (!strcmp(thisSentence, "GSV")) { //*****************************RMC
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
      // parse out number of sat in view
      TRK_GpsParseSV(p);
      return 0;
      }


  #if 0
  else if (!strcmp(thisSentence, "GLL")) { //*****************************GLL
    // in Adafruit from Actisense NGW-1 from SH CP150C
    // parse out both latitude and direction, then go to next field, or fail
    if (!TRK_GpsParseCoord(p, &latitudeDegrees, &latitude, &latitude_fixed, &lat))
      return false;
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    // parse out both longitude and direction, then go to next field, or fail
    if (!TRK_GpsParseCoord(p, &longitudeDegrees, &longitude, &longitude_fixed, &lon))
      return false;
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    TRK_GpsParseTime(p);
    p = strchr(p, ',') + 1;
    TRK_GpsParseFix(p); // skip the rest

  } else if (!strcmp(thisSentence, "GSA")) { //*****************************GSA
    // in Adafruit from Actisense NGW-1
    p = strchr(p, ',') + 1; // skip selection mode
    if (!TRK_GpsIsEmpty(p))
      fixquality_3d = atoi(p);
    p = strchr(p, ',') + 1;
    // skip 12 Satellite PDNs without interpreting them
    for (int i = 0; i < 12; i++)
      p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      PDOP = atof(p);
    p = strchr(p, ',') + 1;
    // parse out HDOP, we also parse this from the GGA sentence. Chipset should
    // report the same for both
    if (!TRK_GpsIsEmpty(p))
      HDOP = atof(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      VDOP = atof(p); // last before checksum
    return false;
  }

  else if (!strcmp(thisSentence, "TXT")) { //*****************************TXT
    if (!TRK_GpsIsEmpty(p))
      txtTot = atoi(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      txtN = atoi(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      txtID = atoi(p);
    p = strchr(p, ',') + 1;
    if (!TRK_GpsIsEmpty(p))
      TRK_GpsParseStr(txtTXT, p, 61); // copy the text to NMEA TXT max of 61 characters
    return false;
  }
  else {
    return false; // didn't find the required sentence definition
  }
  #endif
  // Record the successful parsing of where the last data came from and when
  //strcpy(lastSource, thisSource);
  //strcpy(lastSentence, thisSentence);
  //lastUpdate = TRK_GpsMillis();
  return false;
}

/**************************************************************************/
/*!
    @brief Check an NMEA string for basic format, valid source ID and valid
    and valid sentence ID. Update the values of thisCheck, thisSource and
    thisSentence.
    @param nmea Pointer to the NMEA string
    @return True if well formed, false if it has problems
*/
/**************************************************************************/
bool TRK_GpsCheck(char *nmea) {
  thisCheck = 0; // new check
  *thisSentence = *thisSource = 0;

  //Verify that the first car is '$' or '!'
  if (*nmea != '$' && *nmea != '!')
    return false; // doesn't start with $ or !
  else
    thisCheck += NMEA_HAS_DOLLAR;

  //Search the end of line
  char *ast = nmea; // not strchr(nmea,'*'); for first *
  while (*ast)
    ast++; // go to the end

  //Search the car '*' to extract the checksum
  while (*ast != '*' && ast > nmea)
    ast--; // then back to * if it's there
  if (*ast != '*')
    return false; // there is no asterisk
  else
    {
    //Extract checksum
    uint16_t sum = TRK_GpsParseHex(*(ast + 1)) * 16; // extract checksum
    sum += TRK_GpsParseHex(*(ast + 2));

    //Verify checksum
    char *p = nmea; // check checksum
    for (char *p1 = p + 1; p1 < ast; p1++)
      sum ^= *p1;
    if (sum != 0)
      return false; // bad checksum :(
    else
      thisCheck += NMEA_HAS_CHECKSUM;
    }
  // extract source of variable length
  char *p = nmea + 1;
  const char *src = TRK_GpsTokenOnList(p, sources);
  if (src)
    {
    strcpy(thisSource, src);
    thisCheck += NMEA_HAS_SOURCE;
    }
  else
    return false;

  p += strlen(src);
  // extract sentence id and check if parsed
  const char *snc = TRK_GpsTokenOnList(p, sentences_parsed);
  if (snc)
    {
    strcpy(thisSentence, snc);
    thisCheck += NMEA_HAS_SENTENCE_P + NMEA_HAS_SENTENCE;
    }
  else
    { // check if known
    snc = TRK_GpsTokenOnList(p, sentences_known);
    if (snc)
      {
      strcpy(thisSentence, snc);
      thisCheck += NMEA_HAS_SENTENCE;
      return false; // known but not parsed
      }
    else
      {
      TRK_GpsParseStr(thisSentence, p, NMEA_MAX_SENTENCE_ID);
      return false; // unknown
      }
  }
  return true; // passed all the tests
}

/**************************************************************************/
/*!
    @brief Check if a token at the start of a string is on a list.
    @param token Pointer to the string
    @param list A list of strings, with the final entry starting "ZZ"
    @return Pointer to the found token, or NULL if it fails
*/
/**************************************************************************/
const char *TRK_GpsTokenOnList(char *token, const char **list)
{
  int i = 0; // index in the list
  while (strncmp(list[i], "ZZ", 2) &&
         i < 1000) { // stop at terminator and don't crash without it
    // test for a match on the sentence name
    if (!strncmp((const char *)list[i], (const char *)token, strlen(list[i])))
      return (char*)list[i];
    i++;
  }
  return NULL; // couldn't find a match
}

/**************************************************************************/
/*!
    @brief Check if an NMEA string is valid and is on a list, perhaps to
    decide if it should be passed to a particular NMEA device.
    @param nmea Pointer to the NMEA string
    @param list A list of strings, with the final entry "ZZ"
    @return True if on the list, false if it fails check or is not on the list
*/
/**************************************************************************/
bool TRK_GpsOnList(char *nmea, const char **list) {
  if (!TRK_GpsCheck(nmea)) // sets thisSentence if valid
    return false;   // not a valid sentence
  // stop at terminator with first two letters ZZ and don't crash without it
  for (int i = 0; strncmp(list[i], "ZZ", 2) && i < 1000; i++) {
    // test for a match on the sentence name
    if (!strcmp((const char *)list[i], (const char *)thisSentence))
      return true;
  } 
  return false; // couldn't find a match
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for lat or lon angle and direction.
    Works for either DDMM.mmmm,N (latitude) or DDDMM.mmmm,W (longitude) format.
    Insensitive to number of decimal places present. Only fills the variables
    if it succeeds and the variable pointer is not NULL. This allows calling
    to fill only the variables of interest. Does rudimentary validation on
    angle range.
    Supersedes private functions parseLat(), parseLon(), parseLatDir(),
    parseLonDir(), all previously called from parse().
    @param pStart Pointer to the location of the token in the NMEA string
    @param angle Pointer to the angle to fill with value in degrees/minutes as
      received from the GPS (DDDMM.MMMM), unsigned
    @param angle_fixed Pointer to the fix point version latitude in decimal
      degrees * 10000000, signed
    @param angleDegrees Pointer to the angle to fill with decimal degrees,
      signed. As actual double on SAMD, etc. resolution is better than the
      fixed point version.
    @param dir Pointer to character to fill the direction N/S/E/W
    @return true if successful, false if failed or no value
*/
/**************************************************************************/
bool TRK_GpsParseCoord(char *pStart, int32_t *angle_fixed, char *dir)
  {
  char *p = pStart;
  if (!TRK_GpsIsEmpty(p)) {
    // get the number in DDDMM.mmmm format and break into components
    char degreebuff[10] = {0}; // Ensure string is terminated after strncpy
    char *e = strchr(p, '.');
    if (e == NULL || e - p > 6)
      return false;                // no decimal point in range
    strncpy(degreebuff, p, e - p); // get DDDMM
    long dddmm = atol(degreebuff);
    long degrees = (dddmm / 100);         // truncate the minutes
    long minutes = dddmm - degrees * 100; // remove the degrees
    p = e;                                // start from the decimal point
    nmea_float_t decminutes = atof(e); // the fraction after the decimal point
    p = strchr(p, ',') + 1;            // go to the next field

    // get the NSEW direction as a character
    char nsew = 'X';
    if (!TRK_GpsIsEmpty(p))
      nsew = *p; // field is not empty
    else
      return false; // no direction provided

    // set the various numerical formats to their values
    long fixed = degrees * 10000000 + (minutes * 10000000) / 60 +
                 (decminutes * 10000000) / 60;
//    nmea_float_t ang = degrees * 100 + minutes + decminutes;
    nmea_float_t deg = fixed / (nmea_float_t)10000000.;
    if (nsew == 'S' ||
        nsew == 'W') { // fixed and deg are signed, but DDDMM.mmmm is not
      fixed = -fixed;
      deg = -deg;
    }

    // reject directions that are not NSEW
    if (nsew != 'N' && nsew != 'S' && nsew != 'E' && nsew != 'W')
      return false;

    // reject angles that are out of range
    if (nsew == 'N' || nsew == 'S')
      if (abs(deg) > 90)
        return false;
    if (abs(deg) > 180)
      return false;

    // store in locations passed as args
    if (angle_fixed != NULL)
      *angle_fixed = fixed;
    if (dir != NULL)
      *dir = nsew;
  } else
    return false; // no number
  return true;
}

/**************************************************************************/
/*!
    @brief Parse a string token from pointer p to the next comma, asterisk
    or end of string.
    @param buff Pointer to the buffer to store the string in
    @param p Pointer into a string
    @param n Max permitted size of string including terminating 0
    @return Pointer to the string buffer
*/
/**************************************************************************/
char *TRK_GpsParseStr(char *buff, char *p, int n) {
  char *e = strchr(p, ',');
  int len = 0;
  if (e) {
    len = MIN(e - p, n - 1);
    strncpy(buff, p, len); // copy up to the comma
    buff[len] = 0;
  } else {
    e = strchr(p, '*');
    if (e) {
      len = MIN(e - p, n - 1);
      strncpy(buff, p, len); // or up to the *
      buff[e - p] = 0;
    } else {
      len = MIN((int)strlen(p), n - 1);
      strncpy(buff, p, len); // or to the end or max capacity
    } 
  }
  return buff;
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for time. Independent of number
    of decimal places after the '.'
    @param p Pointer to the location of the token in the NMEA string
    @return true if successful, false otherwise
*/
/**************************************************************************/
bool TRK_GpsParseTime(char *p) {
  if (!TRK_GpsIsEmpty(p)) { // get time
    uint32_t time = atol(p);
    tnl_tmp_gps_data.hour = time / 10000;
    tnl_tmp_gps_data.minute = (time % 10000) / 100;
    tnl_tmp_gps_data.seconds = (time % 100);
    char *dec = strchr(p, '.');
    char *comstar = MIN(strchr(p, ','), strchr(p, '*'));
    if (dec != NULL && comstar != NULL && dec < comstar)
      tnl_tmp_gps_data.milliseconds = atof(dec) * 1000;
    else
      tnl_tmp_gps_data.milliseconds = 0;
    lastTime = sentTime;
    return true;
  }
  return false;
}

/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for number of sat in view
    @param p Pointer to the location of the token in the NMEA string
    @return true if successful, false otherwise
*/
/**************************************************************************/
bool TRK_GpsParseSV(char *p) {
  if (!TRK_GpsIsEmpty(p)) { // get SV
    uint32_t SV = atol(p);
    tnl_tmp_gps_data.sat_in_view = tnl_tmp_gps_data.sat_in_view + SV;
    return true;
  }
  return false;
}


/**************************************************************************/
/*!
    @brief Parse a part of an NMEA string for whether there is a fix
    @param p Pointer to the location of the token in the NMEA string
    @return True if we parsed it, false if it has invalid data
*/
/**************************************************************************/
bool TRK_GpsParseFix(char *p) {
  if (!TRK_GpsIsEmpty(p)) {
    if (p[0] == 'A')
      {
      tnl_tmp_gps_data.fix = true;
      lastFix = sentTime;
      }
    else if (p[0] == 'V')
      tnl_tmp_gps_data.fix = false;
    else
      return false;
    return true;
  }
  return false;
}

/**************************************************************************/
/*!
    @brief Is the field empty, or should we try conversion? Won't work
    for a text field that starts with an asterisk or a comma, but that
    probably violates the NMEA-183 standard.
    @param pStart Pointer to the location of the token in the NMEA string
    @return true if empty field, false if something there
*/
/**************************************************************************/
bool TRK_GpsIsEmpty(char *pStart) {
  if (',' != *pStart && '*' != *pStart && pStart != NULL)
    return false;
  else
    return true;
}

/**************************************************************************/
/*!
    @brief Parse a hex character and return the appropriate decimal value
    @param c Hex character, e.g. '0' or 'B'
    @return Integer value of the hex character. Returns 0 if c is not a proper
   character
*/
/**************************************************************************/
// read a Hex value and return the decimal equivalent
uint8_t TRK_GpsParseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A') + 10;
  // if (c > 'F')
  return 0;
}

void TRK_GpsNewDataValue(nmea_index_t idx, nmea_float_t v)
    {
    } 

uint32_t TRK_GpsMillis()
{
return HAL_GetTick();
}
