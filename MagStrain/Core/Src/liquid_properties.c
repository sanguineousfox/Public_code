#include "liquid_properties.h"

#include <math.h>

float Liquid_CalculateDensity(float temperature_c, float reference_density_kg_m3,
                              float reference_temperature_c, float expansion_per_mille)
{
    if (isfinite(reference_density_kg_m3) && reference_density_kg_m3 > 0.0f) {
        if (!isfinite(temperature_c) || !isfinite(reference_temperature_c) ||
            !isfinite(expansion_per_mille) || expansion_per_mille == 0.0f) {
            return reference_density_kg_m3;
        }
        float volume_ratio = 1.0f +
            expansion_per_mille * 0.001f * (temperature_c - reference_temperature_c);
        float density = reference_density_kg_m3 / volume_ratio;
        return volume_ratio > 0.0f && isfinite(density) ? density : NAN;
    }

    /* NISTIR 7383, GLP 10, equations 1–2 for air-saturated water. */
    float water_temperature_c = isfinite(temperature_c) ? temperature_c : 20.0f;
    if (water_temperature_c < 0.0f || water_temperature_c > 40.0f) {
        return NAN;
    }
    float delta_c = water_temperature_c - 3.983035f;
    return 999.974950f * (1.0f -
        delta_c * delta_c * (water_temperature_c + 301.797f) /
        (522528.9f * (water_temperature_c + 69.34881f))) +
        (-4.612f + 0.106f * water_temperature_c) * 0.001f;
}

float Liquid_CalculateMass(float volume_m3, float density_kg_m3)
{
    if (!isfinite(volume_m3) || volume_m3 < 0.0f ||
        !isfinite(density_kg_m3) || density_kg_m3 <= 0.0f) {
        return NAN;
    }
    float mass_kg = volume_m3 * density_kg_m3;
    return isfinite(mass_kg) ? mass_kg : NAN;
}

float Liquid_CalculateVolume(float mass_kg, float density_kg_m3)
{
    if (!isfinite(mass_kg) || mass_kg < 0.0f ||
        !isfinite(density_kg_m3) || density_kg_m3 <= 0.0f) {
        return NAN;
    }
    float volume_m3 = mass_kg / density_kg_m3;
    return isfinite(volume_m3) ? volume_m3 : NAN;
}
