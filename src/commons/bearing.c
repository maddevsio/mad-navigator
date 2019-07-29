#include <math.h>
#include "commons/bearing.h"
#include "commons/commons.h"


//android Location.bearingTo()
#ifdef BEARING_HIGH_PRECISION
static float initialBearingHP(float lat1,
                              float lon1,
                              float lat2,
                              float lon2) {
#define MAXITERS 20
#define majAxis 6378137.0f
#define smajAxis 6356752.3142f // WGS84 semi-major axis
#define fraction (majAxis - smajAxis) / majAxis
#define EXP 1.0e-6f

  float L, U1, U2;
  float cosU1, cosU2, sinU1, sinU2;
  float cosU1cosU2, sinU1sinU2;
  float sigma, cosSqAlpha, cos2SM, cosSigma, sinSigma, cosLambda, sinLambda;
  float lambda, bearing;

  // Convert lat/long to radians
  lat1 *= deg2rad_coeff;
  lat2 *= deg2rad_coeff;
  lon1 *= deg2rad_coeff;
  lon2 *= deg2rad_coeff;

  L = lon2 - lon1;
  U1 = atanf((1.0f - fraction) * tanf(lat1));
  U2 = atanf((1.0f - fraction) * tanf(lat2));

  cosU1 = cosf(U1);
  cosU2 = cosf(U2);
  sinU1 = sinf(U1);
  sinU2 = sinf(U2);
  cosU1cosU2 = cosU1 * cosU2;
  sinU1sinU2 = sinU1 * sinU2;

  sigma = 0.0;
  cosSqAlpha = 0.0;
  cos2SM = 0.0;
  cosSigma = 0.0;
  sinSigma = 0.0;
  cosLambda = 0.0;
  sinLambda = 0.0;
  lambda = L; // initial guess

  for (int iter = 0; iter < MAXITERS; iter++) {
    float lambdaOrig, t1, t2, sinSqSigma, sinAlpha, C, delta;
    lambdaOrig = lambda;
    cosLambda = cosf(lambda);
    sinLambda = sinf(lambda);
    t1 = cosU2 * sinLambda;
    t2 = cosU1 * sinU2 - sinU1 * cosU2 * cosLambda;
    sinSqSigma = t1 * t1 + t2 * t2;
    sinSigma = sqrtf(sinSqSigma);
    cosSigma = sinU1sinU2 + cosU1cosU2 * cosLambda;
    sigma = atan2f(sinSigma, cosSigma);
    sinAlpha = (sinSigma == 0.0f) ? 0.0f : cosU1cosU2 * sinLambda / sinSigma;
    cosSqAlpha = 1.0f - sinAlpha * sinAlpha;
    cos2SM = (cosSqAlpha == 0.0f) ? 0.0f : cosSigma - 2.0f * sinU1sinU2 / cosSqAlpha;
    C = (fraction / 16.0f) * cosSqAlpha * (4.0f + fraction * (4.0f - 3.0f * cosSqAlpha));
    lambda = L + (1.0f - C) * fraction * sinAlpha * (sigma + C * sinSigma * (cos2SM + C * cosSigma * (-1.0f + 2.0f * cos2SM * cos2SM))); // (11)
    delta = (lambda - lambdaOrig) / lambda;
    if (fabsf(delta) < EXP)
      break;
  }
  bearing = atan2f(cosU2 * sinLambda,
                   cosU1 * sinU2 - sinU1 * cosU2 * cosLambda);
  return bearing * rad2deg_coeff;
}
///////////////////////////////////////////////////////
#endif

#ifndef BEARING_HIGH_PRECISION
//https://software.intel.com/en-us/blogs/2012/11/30/calculating-a-bearing-between-points-in-location-aware-apps
static float initialBearingLP(float lat1,
                              float lon1,
                              float lat2,
                              float lon2) {
  float phi1 = lat1 * deg2rad_coeff;
  float phi2 = lat2 * deg2rad_coeff;
  float lam1 = lon1 * deg2rad_coeff;
  float lam2 = lon2 * deg2rad_coeff;
  float deltaL = lam2 - lam1;
  float cosPhi2 = cosf(phi2);
  float bearing = atan2f(sinf(deltaL) * cosPhi2,
                         cosf(phi1) * sinf(phi2) - sinf(phi1) * cosPhi2 * cosf(deltaL));
  return bearing * rad2deg_coeff;
}
///////////////////////////////////////////////////////
#endif

float initialBearing(float lat1,
                     float lon1,
                     float lat2,
                     float lon2) {
  float ib;
#ifdef BEARING_HIGH_PRECISION
  ib = initialBearingHP(lat1, lon1, lat2, lon2);
#else
  ib = initialBearingLP(lat1, lon1, lat2, lon2);
#endif
  return fmodf(ib+360.f, 360.0f); //+360 % 360
}
///////////////////////////////////////////////////////
