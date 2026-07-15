//***********************************************************************//**********Calculation of Azimuth and Elevation of the sun****************
//*************Tobias Sommer****30.06.2007*********************************
//***********************************************************************
//The program calculates azimut and elevation by first transforming the actual date (ut time !!!) into julian days
//and using information about the position (latitude and longitude)
//The algorithm are taken from
//The Astronomical Almanac For the Year 2000, The Stationery Office, London, 1999 (calculating alpha and delta)
//Jean Meeus: Astronomische Algorithmen, Barth, Leipzig-Berlin-Heidelberg, 1192 (azimut and elevation)
//It is valid for the years 1950-2050 with an error of about 0.01 degrees.
//Coordinates are for a geocentrical observer, it differs in elevation for a real observer on the earth's surface
//up to 0.0024 degrees.

//azimut is counted from south direction clockwise!!!!  

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//!!!!all trigonometric functions need angle in rad!!!!

//input variables


//variables for julian day calculation
double JD;                                              //Julian Day (see info below)
double D0;
double M0;
double Y0;
double H0;
double A;
double B;

//output variables
double azim;                                            //azimuth angle
double elev;                                            //elevation angle

//constants
const double PI = 3.1415926535897932;                         //double can store 15 digits!

double const e = 0.0167;                               //excentricity of elipse earth around sun

//intermediate variables
double n;                                               //number of days that passed since junuary 1st 2000, 12 UT (Greenwich)
double L;                                               //middle ecliptical length 
double g;                                               //middle anomaly
double EL;                                              //ecliptical length
double eps;                                             //slope of elipse (schiefe der ekliptik)
double alpha;                                           //rektaszension
double delta;                                           //declination
double JD0;                                             //Julian Day, but refering to 0 h UT of the concerning date!!!! 
double T0;                                              //julian centuries since 1.1.2000 0 h UT
double SWGH;                                            //middle star time greenwich for time T
double SWG;                                             //middle star time converted to degrees
double SW;                                              //hour angle of spring point
double S;                                               //hour angle of sun only dependend on time and longitude!!!!


double integer;                                         //needed in the modulo functions            
double newinteger;
short input = 0;                                          //asking for input in the end of program









double moduloday(double hours)                         //modulo 24 h
{
    integer = (short) (hours / 24);
    newinteger = integer * 24;
    return (hours - newinteger);
}

double dtr(double degree)                               //transfers degrees to rad
{
    return (degree * PI / 180);
}

double rtd(double rad)                                  //transfers rad to degree
{
    return (rad * 180 / PI);
}

double modulocircle(double degree)                     //modulo 360 degrees
{
    integer = (short) (degree / 360);
    newinteger = integer * 360;

    return (degree - newinteger);
}


void CalcSunAngles(double lon, double lat, double D, double M, double Y, double h, double min, double s) {


    if (M > 2)                   //Julian Day, this number is counting days continuosly since 1. Januar 4713 BC, always
    {                         //refering to 12 h UT as a full day!!!!!!! So eg 0.75 means 12 h + (0.75h *24 h)
        Y0 = Y;                  //= 6 h (in the morning)
        M0 = M;
    } else {
        Y0 = Y - 1;
        M0 = M + 12;
    }
    D0 = D;
    H0 = h / 24.0 + min / 1440.0 + s / 86400.0;
    A = (short) (Y0 / 100);
    B = 2 - A + (short) (A / 4);

    JD = (long) (365.25 * (Y0 + 4716)) + (short) (30.6001 * (M0 + 1)) + D0 + H0 + B -
         1524.5;     //JulianDays refering to current date

    n = JD - 2451545.0;                                 //days since 1.1.2000 12h UT

    L = 280.460 + 0.9856474 * n;                        //middle ecliptical length
    L = modulocircle(L);
    g = 357.528 + 0.9856003 * n;                        //middle anomaly

    g = modulocircle(g);

    EL = L + (2 * e * sin(dtr(g)) + 5.0 / 4.0 * pow(e, 2) * sin(2 * dtr(g))) * 180 / PI;  //ecliptical length

    eps = 23.439 - 0.0000004 * n;                         //schiefe der ekliptik

    alpha = rtd(atan(cos(dtr(eps)) * sin(dtr(EL)) / cos(dtr(EL)))); //rektraszension
    if (cos(dtr(EL)) < 0) alpha = alpha + 180;

    delta = rtd(asin(sin(dtr(eps) * sin(dtr(EL)))));      //declination




    JD0 = JD - 0.5;                                    //*************************shouldn't it be 0.5???

    T0 = (JD0 - 2451545.0) / 36525;                       //julian centuries since 1.1.2000 0 h UT

    SWGH = 6.697376 + 2400.05134 * T0 + 1.002738 * (h + min / 60 + s / 3600);       //it's all leading to calculate S,
    SWGH = moduloday(SWGH);                                             //which is the hour angle

    SWG = SWGH * 15;
    SW = SWG + lon;
    S = SW - alpha;                                      //hour angle for specific longitude


    //CALCULATION OF AZIMUT AND ELEVATION:

    //Azimut:
    azim = rtd(atan(sin(dtr(S)) / (cos(dtr(S)) * sin(dtr(lat)) - tan(dtr(delta)) * cos(dtr(lat)))));
    if ((cos(dtr(S)) * sin(dtr(lat)) - tan(dtr(delta)) * cos(dtr(lat))) < 0) azim = azim + 180;
    azim = azim - 360;
    if (azim > 180) azim = azim - 360;
    if (azim < -180) azim = azim + 360;

    //Elevation:
    elev = rtd(asin(cos(dtr(delta)) * cos(dtr(S)) * cos(dtr(lat)) + sin(dtr(delta)) * sin(dtr(lat))));

}


double GetSunAngle() {
    long t, d;
    short second, minute, hour, year, month, day;
    t = gpstime / 100;
    second = t % 100;
    t /= 100;
    minute = t % 100;
    t /= 100;
    hour = t;
    d = gpsdate;
    year = 2000 + d % 100;
    d /= 100;
    month = d % 100;
    d /= 100;
    day = d;
    CalcSunAngles(gpslon, gpslat, day, month, year, hour, minute, second);

    if (debugflag) {
        syslog(LOG, "SunAngle: azim=%lf elev=%lf\n", azim, elev);
    }
    return elev;
}



