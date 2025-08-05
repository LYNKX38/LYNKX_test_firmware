/*******************************************************************************
*
*       Copyright (c) PEAK Techno, 2021
*         http://www.peak-techno.com
*
********************************************************************************
*@file          : GPS.c
*@project       : Tracker
*@version       : $Id: GPS.c
*@brief         :
*******************************************************************************/

#include "main.h"
#include <math.h>
#include "utilities_conf.h"
#include "sys_app.h"
#include "gps.h"

extern gps_data_t gps_data;
gps_data_t tnl_tmp_gps_data;
bool gps_configured = false;
gps_state_t gps_state = GPS_RESET;
bool new_gps_pos = false;

//float distance;
//extern uint8_t master;
//extern uint8_t request_beep;

///*******************************************************************************
//* Function Name  : TRK_GPSEnable
//* Description    : Power and Enable GPS
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//bool TRK_GpsEnable(uint8_t enable)
//{
//    if (enable)
//        {
//        gps_data.hour = gps_data.minute = gps_data.seconds = gps_data.year = gps_data.month = gps_data.day =
//            gps_data.fixquality = gps_data.satellites = 0;
//
//        gps_data.geoidheight = gps_data.altitude =
//            gps_data.speed = gps_data.angle = gps_data.magvariation = gps_data.HDOP;
//        gps_data.latitude_fixed = gps_data.longitude_fixed = 0;
//        gps_data.lat = 'X';    ///< N/S
//        gps_data.lon = 'X';    ///< E/W
//        gps_data.mag = 'X';    ///< Magnetic variation direction
//        gps_data.fix = tnl_tmp_gps_data.fix = false;
//
//        gps_state = GPS_SET;
//        }
//    else
//        {
//        gps_state = GPS_RESET;
//        }
//  //Activate GPS
//  return TRK_GK9501Enable(enable);
//}

///*******************************************************************************
//* Function Name  : TRK_GpsConfigure
//* Description    : Configure the GPS when it is started
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//bool TRK_GpsConfigure()
//{
//    if (gps_state == GPS_ERROR)
//        return false;
//    if (!TRK_GK9501EnableSatellite(1, 1, 0, 1)) //GPS, GLONASS, GALILEO
//        return false;
//    if (!TRK_GK9501NmeaOutputMessage()) //Keep only RMC and GGA
//        return false;
//
//    gps_configured = true;
//    return true;
//}


///*******************************************************************************
//* Function Name  : TRK_GpsRead
//* Description    : Search first lin in the buffer
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//bool TRK_GpsRead()
//    {
//    if (gps_state != GPS_ERROR)
//        return TRK_GK9501StartNmeaReception();
//    return true;
//    }

/*******************************************************************************
* Function Name  : TRK_GpsReadProcessing
* Description    : Search first lin in the buffer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool TRK_GpsReadProcessing(uint8_t *buffer, uint16_t buffer_size)
    {
    uint16_t line_start = 0xFFFF;
    uint16_t line_end = 0xFFFF;

    //Search first line
    for (uint16_t i=0; i<buffer_size; i++)
        {
        //Search first NMEA character
        if ((line_start==0xFFFF) && (buffer[i] == '$'))
            {
            line_start = i;
            }
        //Search first end of line
        else if ((line_start!=0xFFFF) && (buffer[i] == '\n'))
            {
            line_end = i;
            }
        //Line found, parse the line
        if (line_end != 0xFFFF)
            {
            buffer[line_end] = 0;//To create a string
            if (TRK_GpsParse((char *) buffer+line_start))
                {
                gps_state = GPS_POS_FOUND;
                TRK_GpsNewPos();
                }
            else if (gps_state == GPS_POS_FOUND)
                {
                gps_state = GPS_POS_LOST;
                }
            else
                {
                gps_state = GPS_STARTING;
                }
            line_start = line_end+1; //+1 for '\r' carac
            line_end = 0xFFFF;
            }
        }
    return true;
    }

/*******************************************************************************
* Function Name  : TRK_GpsNewPos
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool TRK_GpsNewPos()
    {
    new_gps_pos = true;
    gps_data.hour = tnl_tmp_gps_data.hour;
    gps_data.minute = tnl_tmp_gps_data.minute;
    gps_data.seconds = tnl_tmp_gps_data.seconds;
    gps_data.year = tnl_tmp_gps_data.year;
    gps_data.month = tnl_tmp_gps_data.month;
    gps_data.day = tnl_tmp_gps_data.day;
    gps_data.fixquality = tnl_tmp_gps_data.fixquality;
    gps_data.satellites = tnl_tmp_gps_data.satellites;

    gps_data.geoidheight = tnl_tmp_gps_data.geoidheight;
    gps_data.altitude = tnl_tmp_gps_data.altitude;
    gps_data.speed = tnl_tmp_gps_data.speed;
    gps_data.angle = tnl_tmp_gps_data.angle;
    gps_data.HDOP = tnl_tmp_gps_data.HDOP;

    gps_data.latitude_fixed = tnl_tmp_gps_data.latitude_fixed;
    gps_data.longitude_fixed = tnl_tmp_gps_data.longitude_fixed;
    gps_data.lat = tnl_tmp_gps_data.lat;
    gps_data.lon = tnl_tmp_gps_data.lon;
    gps_data.mag = tnl_tmp_gps_data.mag;
    gps_data.fix = tnl_tmp_gps_data.fix;

//    tnl_msg_tx.latitude = gps_data.latitude_fixed;
//    tnl_msg_tx.longitude = gps_data.longitude_fixed;
//    tnl_msg_tx.altitude = gps_data.altitude;

    //Save the point
//    if (gps_data.fix)
//        {
//        record_gps_t data;
//        data.latitude = gps_data.latitude_fixed;
//        data.longitude = gps_data.longitude_fixed;
//        data.altitude = gps_data.altitude;
//        TRK_LogAddRecord(RECORD_GPS, (uint8_t*)&data);
//        }

    //If local GPS coord found and received GPS coord, compute the distance
//    if (gps_data.fix && tnl_msg_rx.latitude && tnl_msg_rx.longitude)
//        {
//        distance = TRK_GpsDistanceTo(gps_data.latitude_fixed, gps_data.longitude_fixed, 452087221, 57930484);
//        TRK_DisplayDistance(distance);
//        if (distance <= 200)
//	{
//	  HAL_GPIO_TogglePin(LED_RED1_GPIO_Port, LED_RED1_Pin);
//            if (master==1)
//                {
//                request_beep = 1;
//                tnl_msg_tx.evt |= EVT_REQUEST_BEEP;
//                }
//            TRK_Tone(TONE_LA, 300);
//            HAL_Delay(200); //ms
//            TRK_Tone(TONE_LA, 300);
//            HAL_Delay(200); //ms
//            TRK_Tone(TONE_LA, 300);
//            HAL_Delay(200); //ms
//	  }

//        }

    return true;
    }

uint8_t TRK_SearchSatellite(uint8_t *buffer){
	uint8_t nb = 0;
	if (!strncmp(buffer, "$GPGSV", 6) ||
	    !strncmp(buffer, "$GLGSV", 6) ||
	    !strncmp(buffer, "$GAGSV", 6) ||
	    !strncmp(buffer, "$GBGSV", 6) ||
	    !strncmp(buffer, "$GNGSV", 6))
	{
	    // Format: $GxGSV,<total_sentences>,<sentence_number>,<sat_in_view>,...
	    int total_sentences = 0, sentence_number = 0, sat_in_view = 0;
	    if (sscanf((char*)buffer, "$%*[^,],%d,%d,%d", &total_sentences, &sentence_number, &sat_in_view) == 3)
	    {
	        nb = sat_in_view;
	    }
	}
	return nb;

}

#if 0
uint8_t TRK_GpsBob()
{
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }

}

#endif

     /**
     * Returns the distance along the surface of the earth from �this� point to destination point.
     *
     * Uses haversine formula: a = sin�(?f/2) + cosf1�cosf2 � sin�(??/2); d = 2 � atan2(va, v(a-1)).
     *
     * @param   {LatLon} point - Latitude/longitude of destination point.
     * @param   {number} [radius=6371e3] - Radius of earth (defaults to mean radius in metres).
     * @returns {number} Distance between this point and destination point, in same units as radius.
     * @throws  {TypeError} Invalid radius.
     *
     * @example
     *   const p1 = new LatLon(52.205, 0.119);
     *   const p2 = new LatLon(48.857, 2.351);
     *   const d = p1.distanceTo(p2);       // 404.3�10� m
     *   const m = p1.distanceTo(p2, 3959); // 251.2 miles
     */
float TRK_GpsDistanceTo(long lat_source, long lon_source, long lat_dest, long lon_dest)
    {
    long R = (long)6371000; // metres
    float phi_source = (lat_source * M_PI/180)/10000000;
    float phi_dest = (lat_dest * M_PI/180)/10000000;
    float delta_phi = ((lat_source - lat_dest) * M_PI/180)/10000000;
    float delta_teta = ((lon_dest-lon_source) * M_PI/180)/10000000;

    float a = sin(delta_phi/2) * sin(delta_phi/2) +
          cos(phi_source) * cos(phi_dest) *
          sin(delta_teta/2) * sin(delta_teta/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    float d = R * c; // in metres
    return d;

    }

/*******************************************************************************
* Function Name  : TRK_GPSError
* Description    : Error received
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TRK_GpsError()
    {
    gps_state = GPS_ERROR;
    }


