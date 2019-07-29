#ifndef MAGNETIC_H
#define MAGNETIC_H

///
/// \brief calculate
/// \param alt - altitude in kilometers
/// \param lat - decimal degrees
/// \param lon - decimal degrees
/// \param year - decimal (2015.0 - 2020.0)
/// \param declination
/// \param inclination
/// \param totalIntensity
/// \param gridVariation
///

void magnetic_calculate(float alt, float lat, float lon,
                        float year, float *declination, float *inclination,
                        float *totalIntensity, float *gridVariation);

#endif // MAGNETIC_H
