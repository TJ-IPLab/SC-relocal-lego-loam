#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cassert>
#include <math.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

struct GPS_structure
{
    double longitude;
    double latitude;
};

struct UTM_structure
{
    double x;
    double y;
    double zone;
};

/*
* ArcLengthOfMeridian
*
* Computes the footpoint latitude for use in converting transverse
* Mercator coordinates to ellipsoidal coordinates.
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
*   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
* Inputs:
*   y - The UTM northing coordinate, in meters.
* Returns:
*   The footpoint latitude, in radians.
*/
double ArcLengthOfMeridian(double phi);

/*
* MapLatLonToXY
*
* Converts a latitude/longitude pair to x and y coordinates in the
* Transverse Mercator projection.  Note that Transverse Mercator is not
* the same as UTM; a scale factor is required to convert between them.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
* GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*    phi - Latitude of the point, in radians.
*    lambda - Longitude of the point, in radians.
*    lambda0 - Longitude of the central meridian to be used, in radians.
*
* Outputs:
*    xy - A 2-element array containing the x and y coordinates
*         of the computed point.
*
* Returns:
*    The function does not return a value.
*
*/
double FootpointLatitude(double y);
UTM_structure MapLatLonToXY(double phi, double lambda, double lambda0);
double UTMCentralMeridian(double zone);
UTM_structure LatLonToUTMXY(double lon, double lat);
GPS_structure MapXYToLatLOn(double x, double y, double lambda0);
GPS_structure UTMXYTOLatLon(double x, double y, int zone, bool southhemi);

