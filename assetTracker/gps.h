

//  AmpleGPS.h
//
//
//  Created by dial on 4/16/16.
//
//
//

#ifndef AMPLEGPS_H
#define AMPLEGPS_H

#endif /* AmpleGPS_h */

#include "Particle.h"


// char *nmeaPrefix = "$GP";// keep in mind that 'GP" may be changed
// The prefix will be removed asap in order to save some bytes
#define GP_NMEA_PREFIX "$GN"

#define GPRMC_NMEA "GNRMC"
#define GPVTG_NMEA "VTG"
#define GPGGA_NMEA "GGA"
#define GPGSA_NMEA "GSA"
#define GPGSV_NMEA "GSV"
#define GPGLL_NMEA "GLL"

#define GNTXT       "TXT"   //this is used for config the ublox module
#define ANT_STATUS  "ANTSTATUS=OK"       //the GPS module should print out this info after received the antenna configure command


#define MAXLINELENGTH 87
#define MAXSATELLITES 16 // supposed to be 16, but some are carried on while not in sight


//those are the start message
#define SYNC_CHAR_1 0xb5
#define SYNC_CHAR_2 0x62
//those are used for config the gps module
#define CLASS_NAV 0X01
#define CLASS_RXM 0x02
#define CLASS_CFG 0x06        //only use this for swith the antenna

#define CFG_ANT 0x13            //antenna control setting

#define LENGTH_HEADER   2
#define LENGTH_CLASS    1
#define LENGTH_ID       1
#define LENGTH_DATALENGTH   2   //two byte to store the info of datalength issue
#define LENGTH_CHECKSUM 2
#define CONSTANT_LENGTH 8       //the total abvoe length is always the same.

#define LENGTH_CONFIG_ANT  4




const float half_piRad  = 0.017453293;//  = π/180
const uint16_t doubleEarth  = 12742;//  = 2 * 6371// diameter of the Earth

// typedef is removed
enum {//GPS_Mode
    GPS_PAUSED = 0,
    GPS_DO_NOT_PARSE = (1 << 0),
    GPS_NEW_COORDS = (1 << 1),
    GPS_NEW_TIME = (1 << 2),
    GPS_UPDATE_SATS = (1 << 3),
    GPS_RESERVED = (1 << 4),
    GPS_COMPLETE_DATA = (1 << 5),
    GPS_VIEW_UNPARSED = (1 << 6),
} ;//GPS_Mode_t;

typedef int (*callBackFunction) (uint8_t callBackMode);

typedef struct gsv_satellite {  //inserted for GPGSV @dial
    uint8_t number;         //4    = SV PRN number
    uint8_t elevation;      //5    = Elevation in degrees, 90 maximum
    uint16_t azimuth;        //6    = Azimuth, degrees from true north, 000 to 359
    uint8_t strength;       //7    = SNR, 00-99 dB (null when not tracking)
}GSV_Satellite;


class AmpleGPS {
public:
    uint16_t milliseconds;

    char readBuffer[MAXLINELENGTH];
    uint8_t bufferIndex=0;

    char nmeaSentence[81];//82 -'GP' +'/0';
    char *checksumString;
    uint16_t nmeaCheckSum;

    uint8_t hour, minute, seconds, year, month, day;


    uint8_t mode;
    callBackFunction callBack;

    float latitudeDegrees, longitudeDegrees, altitude;
    float speedOnGround, bearing, HDOP;
    boolean fix,extAntenna;
    uint8_t fixquality, satellitesCount;

    GSV_Satellite satellitesBuffer[MAXSATELLITES];
    boolean gsv_isDirty;

    char *unparsedSentence;

    char readSentence(char c);
    boolean parseSentence(char *nmeaSentence);
    void begin();
    float getDistanceInKm( float targetLat, float targetLon);
    void loadSentence(uint8_t ubxClass,uint8_t ubxID,uint8_t dataBuf[],uint8_t length,uint8_t *buf);
    void sendSentence(uint8_t *buf,uint8_t length);

private:
    char *parseSingleSatellite(char *p, gsv_satellite *aSatellite);
    char *parseSingleCoord (char *p, float *degreeValue);
    char *parsePairOfCoords (char *p);
    char *parseTime(char *p);
    int performCallBack(uint8_t suggestedMode);

};
