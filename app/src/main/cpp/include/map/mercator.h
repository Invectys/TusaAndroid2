//
// Created by Artem on 18.05.2024.
//

#ifndef TUSA_ANDROID_MERCATOR_H
#define TUSA_ANDROID_MERCATOR_H

#include <math.h>
#define DEG2RAD(a)   ((a) / (180 / M_PI))
#define RAD2DEG(a)   ((a) * (180 / M_PI))
#define EARTH_RADIUS 6378137

class Mercator {
public:
    /* The following functions take their parameter and return their result in degrees */

    static double y2lat_d(double y)   { return RAD2DEG( atan(exp( DEG2RAD(y) )) * 2 - M_PI/2 ); }
    static double x2lon_d(double x)   { return x; }

    static double lat2y_d(double lat) { return RAD2DEG( log(tan( DEG2RAD(lat) / 2 +  M_PI/4 )) ); }
    static double lon2x_d(double lon) { return lon; }

    /* The following functions take their parameter in something close to meters, along the equator, and return their result in degrees */

    static double y2lat_m(double y)   { return RAD2DEG(2 * atan(exp( y/EARTH_RADIUS)) - M_PI/2); }
    static double x2lon_m(double x)   { return RAD2DEG(x / EARTH_RADIUS); }

    /* The following functions take their parameter in degrees, and return their result in something close to meters, along the equator */

    static double lat2y_m(double lat) { return log(tan( DEG2RAD(lat) / 2 + M_PI/4 )) * EARTH_RADIUS; }
    static double lon2x_m(double lon) { return DEG2RAD(lon) * EARTH_RADIUS; }
};


#endif //TUSA_ANDROID_MERCATOR_H
