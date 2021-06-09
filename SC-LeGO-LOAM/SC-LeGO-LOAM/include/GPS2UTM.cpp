#include "GPS2UTM.h"

double pi = 3.1415926535898;
double sm_a = 6378137.0;  //椭球体长半轴长度
double sm_b = 6356752.314; //椭球体短半轴长度
double sm_EccSquared = 0.00669437999013;
double UTMScaleFactor = 0.9996; //UTM投影的比例系数

inline double RadToDeg(double rad)
{
    return (rad / pi * 180.0);
}

/*
 48 * ArcLengthOfMeridian
 49 *
 50 * Computes the ellipsoidal distance from the equator to a point at a
 51 * given latitude.
 52 *
 53 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 54 * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 55 *
 56 * Inputs:
 57 *     phi - Latitude of the point, in radians.
 58 *
 59 * Globals:
 60 *     sm_a - Ellipsoid model major axis.
 61 *     sm_b - Ellipsoid model minor axis.
 62 *
 63 * Returns:
 64 *     The ellipsoidal distance of the point from the equator, in meters.
 65 *
 66 */

double ArcLengthOfMeridian(double phi)
{
    double alpha, beta, gamma, delta, epsilon, n;
    double result;

    /* Precalculate n */
    n = (sm_a - sm_b) / (sm_a + sm_b);

    /* Precalculate alpha */
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));

    return result;
}

double UTMCentralMeridian(double zone)
{
    double cmeridian;

    double deg = -183.0 + (zone * 6.0);

    cmeridian = deg / 180.0 * pi;

    return cmeridian;
}

/*
117 * FootpointLatitude
118 *
119 * Computes the footpoint latitude for use in converting transverse
120 * Mercator coordinates to ellipsoidal coordinates.
121 *
122 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
123 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
124 *
125 * Inputs:
126 *   y - The UTM northing coordinate, in meters.
127 *
128 * Returns:
129 *   The footpoint latitude, in radians.
130 *
131 */

double FootpointLatitude(double y)
{
    double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
    double result;

    /* Precalculate n (Eq. 10.18) */
    n = (sm_a - sm_b) / (sm_a + sm_b);

    /* Precalculate alpha_ (Eq. 10.22) */
    /* (Same as alpha in Eq. 10.17) */
    alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));

    /* Precalculate y_ (Eq. 10.23) */
    y_ = y / alpha_;

    /* Precalculate beta_ (Eq. 10.22) */
    beta_ = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);

    /* Precalculate gamma_ (Eq. 10.22) */
    gamma_ = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta_ (Eq. 10.22) */
    delta_ = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);

    /* Precalculate epsilon_ (Eq. 10.22) */
    epsilon_ = (1097.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series (Eq. 10.21) */
    result = y_ + (beta_ * sin(2.0 * y_)) + (gamma_ * sin(4.0 * y_)) + (delta_ * sin(6.0 * y_)) + (epsilon_ * sin(8.0 * y_));

    return result;
}

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
UTM_structure MapLatLonToXY(double phi, double lambda, double lambda0)
{

    UTM_structure utm;

    double N, nu2, ep2, t, t2, l;
    double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
    double tmp;

    /* Precalculate ep2 */
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

    /* Precalculate nu2 */
    nu2 = ep2 * pow(cos(phi), 2.0);

    /* Precalculate N */
    N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));

    /* Precalculate t */
    t = tan(phi);
    t2 = t * t;
    tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    l = lambda - lambda0;

    /* Precalculate coefficients for l**n in the equations below
	so a normal human being can read the expressions for easting
	and northing
	-- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;

    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */

    utm.x = N * cos(phi) * l + (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    utm.y = ArcLengthOfMeridian(phi) + (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));

    return utm;
}

/*
243 * MapXYToLatLon
244 *
245 * Converts x and y coordinates in the Transverse Mercator projection to
246 * a latitude/longitude pair.  Note that Transverse Mercator is not
247 * the same as UTM; a scale factor is required to convert between them.
248 *
249 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
250 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
251 *
252 * Inputs:
253 *   x - The easting of the point, in meters.
254 *   y - The northing of the point, in meters.
255 *   lambda0 - Longitude of the central meridian to be used, in radians.
256 *
257 * Outputs:
258 *   philambda - A 2-element containing the latitude and longitude
259 *               in radians.
260 *
261 * Returns:
262 *   The function does not return a value.
263 *
264 * Remarks:
265 *   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
266 *   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
267 *   to the footpoint latitude phif.
268 *
269 *   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
270 *   to optimize computations.
271 *
272 */

GPS_structure MapXYToLatLOn(double x, double y, double lambda0)
{
    GPS_structure GPS;
    double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
    double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
    double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

    /* Get the value of phif, the footpoint latitude. */
    phif = FootpointLatitude(y);

    /* Precalculate ep2 */
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

    /* Precalculate cos (phif) */
    cf = cos(phif);

    /* Precalculate nuf2 */
    nuf2 = ep2 * pow(cf, 2.0);

    /* Precalculate Nf and initialize Nfpow */
    Nf = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nuf2));
    Nfpow = Nf;

    /* Precalculate tf */
    tf = tan(phif);
    tf2 = tf * tf;
    tf4 = tf2 * tf2;

    /* Precalculate fractional coefficients for x**n in the equations
     below to simplify the expressions for latitude and longitude. */
    x1frac = 1.0 / (Nfpow * cf);

    Nfpow *= Nf; /* now equals Nf**2) */
    x2frac = tf / (2.0 * Nfpow);

    Nfpow *= Nf; /* now equals Nf**3) */
    x3frac = 1.0 / (6.0 * Nfpow * cf);

    Nfpow *= Nf; /* now equals Nf**4) */
    x4frac = tf / (24.0 * Nfpow);

    Nfpow *= Nf; /* now equals Nf**5) */
    x5frac = 1.0 / (120.0 * Nfpow * cf);

    Nfpow *= Nf; /* now equals Nf**6) */
    x6frac = tf / (720.0 * Nfpow);

    Nfpow *= Nf; /* now equals Nf**7) */
    x7frac = 1.0 / (5040.0 * Nfpow * cf);

    Nfpow *= Nf; /* now equals Nf**8) */
    x8frac = tf / (40320.0 * Nfpow);

    /* Precalculate polynomial coefficients for x**n.
     -- x**1 does not have a polynomial coefficient. */
    x2poly = -1.0 - nuf2;

    x3poly = -1.0 - 2 * tf2 - nuf2;

    x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

    x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

    x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;
    x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

    x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

    /* Calculate latitude */
    GPS.latitude = RadToDeg(phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) + x6frac * x6poly * pow(x, 6.0) + x8frac * x8poly * pow(x, 8.0));

    /* Calculate longitude */
    GPS.longitude = RadToDeg(lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) + x5frac * x5poly * pow(x, 5.0) + x7frac * x7poly * pow(x, 7.0));

    return GPS;
}
/*
 * LatLonToUTMXY
 *
 * Converts a latitude/longitude pair to x and y coordinates in the
 * Universal Transverse Mercator projection.
 *
 * Inputs:
 *   lat - Latitude of the point, in radians.
 *   lon - Longitude of the point, in radians.
 *   zone - UTM zone to be used for calculating values for x and y.
 *          If zone is less than 1 or greater than 60, the routine
 *          will determine the appropriate zone from the value of lon.
 *
 * Outputs:
 *   xy - A 2-element array where the UTM x and y values will be stored.
 *
 * Returns:
 *   void
 *
*/
UTM_structure LatLonToUTMXY(double lon, double lat)
{

    UTM_structure utm = {0, 0, 0};
    utm.zone = floor((lon + 180.0) / 6) + 1;
    double cm = UTMCentralMeridian(utm.zone); //中央子午线

    utm = MapLatLonToXY(lat / 180.0 * pi, lon / 180 * pi, cm);

    utm.zone = floor((lon + 180.0) / 6) + 1;
    //cout << "zone "<< utm.zone<<endl;
    /* Adjust easting and northing for UTM system. */
    utm.x = utm.x * UTMScaleFactor + 500000.0; //为了方便计算， 将负数全都转换为正数
    utm.y = utm.y * UTMScaleFactor;

    if (utm.y < 0.0)
    {
        utm.y = utm.y + 10000000.0;
    }

    return utm;
}

/*
 * UTMXYToLatLon
 *
 * Converts x and y coordinates in the Universal Transverse Mercator
 * projection to a latitude/longitude pair.
 *
 * Inputs:
 *    x - The easting of the point, in meters.
 *    y - The northing of the point, in meters.
 *    zone - The UTM zone in which the point lies.
 *    southhemi - True if the point is in the southern hemisphere;
*               false otherwise.
 *
 * Outputs:
 *    latlon - A 2-element array containing the latitude and
 *            longitude of the point, in radians.
 *
 * Returns:
 *    The function does not return a value.
 *
 */

GPS_structure UTMXYTOLatLon(double x, double y, int zone, bool southhemi)
{
    GPS_structure output_gps;
    double cmeridian;
    x -= 500000.0;
    x /= UTMScaleFactor;
    if (southhemi)
    {
        y -= 10000000.0;
    }
    y /= UTMScaleFactor;
    cmeridian = UTMCentralMeridian(zone);
    output_gps = MapXYToLatLOn(x, y, cmeridian); //弧度
    return output_gps;
}

// int main()
// {
// 	ifstream infile;
// 	infile.open("gpsdata_test.txt", ios::in);
// 	char[70] linechar = { 0 };
// 	string latstr = "", lonstr = "";
// 	double lat = 0, lon = 0;
// 	while (!infile.eof())
// 	{
// 		infile.getline(linechar, sizeof(linechar));
// 		stringstream latlonstr(linestr);
// 		latlonstr >> latstr;
// 		latlonstr >> lonstr;
// 		lat = Float.parseFloat(latstr);
// 		lon = Float.parseFloat(lonstr);
// 		cout << lat << "-----------" << lon << std::endl;
// 	}
// }