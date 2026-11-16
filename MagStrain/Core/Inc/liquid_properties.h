#ifndef LIQUID_PROPERTIES_H
#define LIQUID_PROPERTIES_H

/* Density in kg/m³. A non-positive reference density selects water.
 * A missing measured temperature is represented by NAN. */
float Liquid_CalculateDensity(float temperature_c, float reference_density_kg_m3,
                              float reference_temperature_c, float expansion_per_mille);

/* Volume in m³, density in kg/m³, result in kg. */
float Liquid_CalculateMass(float volume_m3, float density_kg_m3);
float Liquid_CalculateVolume(float mass_kg, float density_kg_m3);

#endif
