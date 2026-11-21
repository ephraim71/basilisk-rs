//! ITU-R P.835 mean annual global reference atmosphere: port of upstream
//! Basilisk `architecture/utilities/ItuRefAtmosphere`.
//!
//! Temperature, pressure and water-vapour density as functions of altitude,
//! for gaseous-attenuation calculations along Earth-space paths. Upstream's
//! singleton class with static getters is a set of free functions here.
//!
//! The input altitude is geopotential, in metres; above 84.852 km the
//! formulas switch to geometric height in kilometres (ITU-R P.835-7 eq. 1b).
//!
//! Upstream's troposphere pressure (eq. 3a) divides by `288.15 - L h` with
//! the signed lapse rate `L = -6.5` K/km, where ITU-R P.835 uses its
//! magnitude, so below 11 km the pressure falls more slowly than the
//! standard's and jumps at 11 km. The port keeps upstream's formula: the
//! link-budget reference attenuations are generated from it.

/// One layer of the piecewise temperature profile (upstream
/// `AtmosphereLayer`).
struct AtmosphereLayer {
    /// Upstream `h` \[m\]: altitude of the layer base.
    base_m: f64,
    /// Upstream `dT_dh` \[K/km\]: temperature gradient in the layer.
    lapse_k_per_km: f64,
}

/// Upstream `ATMOSPHERE_DATA` (ITU-R P.835-7 Table 1). The gradient of the
/// last layer is unused; the temperature is held above it.
const ATMOSPHERE_DATA: [AtmosphereLayer; 8] = [
    AtmosphereLayer {
        base_m: 0.0e3,
        lapse_k_per_km: -6.5,
    },
    AtmosphereLayer {
        base_m: 11.0e3,
        lapse_k_per_km: 0.0,
    },
    AtmosphereLayer {
        base_m: 20.0e3,
        lapse_k_per_km: 1.0,
    },
    AtmosphereLayer {
        base_m: 32.0e3,
        lapse_k_per_km: 2.8,
    },
    AtmosphereLayer {
        base_m: 47.0e3,
        lapse_k_per_km: 0.0,
    },
    AtmosphereLayer {
        base_m: 51.0e3,
        lapse_k_per_km: -2.8,
    },
    AtmosphereLayer {
        base_m: 71.0e3,
        lapse_k_per_km: -2.0,
    },
    AtmosphereLayer {
        base_m: 84.852e3,
        lapse_k_per_km: 0.0,
    },
];

/// Upstream `itu_press_const` \[K/km\]: `g M / R` of the barometric formula
/// (ITU-R P.835-7).
const ITU_PRESS_CONST: f64 = 34.1632;
/// Upstream `T0` \[K\]: sea-level temperature (eq. 5).
const T0_K: f64 = 288.15;
/// Upstream `rho0` \[g/m^3\]: sea-level water-vapour density (eq. 7).
const RHO0_G_PER_M3: f64 = 7.5;
/// Upstream `h0_water` \[km\]: water-vapour scale height (eq. 6).
const H0_WATER_KM: f64 = 2.0;
/// Upstream `R_EARTH_ITU` \[km\]: Earth radius of the ITU height conversion.
const R_EARTH_ITU_KM: f64 = 6356.766;
/// Upstream `TEMP_RISE_START` \[km\]: geometric height where the
/// upper-atmosphere temperature starts to rise.
const TEMP_RISE_START_KM: f64 = 91.0;
/// Upstream `MODEL_TOP` \[km\]: geometric height above which the pressure is
/// zero.
const MODEL_TOP_KM: f64 = 100.0;

/// Upstream `findLayerIndex`: the layer containing `altitude_m`, the last
/// one above its base.
fn layer_index(altitude_m: f64) -> usize {
    ATMOSPHERE_DATA
        .windows(2)
        .position(|pair| altitude_m >= pair[0].base_m && altitude_m < pair[1].base_m)
        .unwrap_or(ATMOSPHERE_DATA.len() - 1)
}

/// Upstream `geometricHeight` \[km\] of the geopotential altitude
/// `altitude_m` (ITU-R P.835-7 eq. 1b).
fn geometric_height_km(altitude_m: f64) -> f64 {
    (R_EARTH_ITU_KM * altitude_m * 1e-3) / (R_EARTH_ITU_KM - altitude_m * 1e-3)
}

/// Upstream `getTempISA` \[K\]: temperature at `altitude_m` (ITU-R P.835-7
/// eqs. 2 and 4).
pub fn temperature_k(altitude_m: f64) -> f64 {
    let top = &ATMOSPHERE_DATA[ATMOSPHERE_DATA.len() - 1];
    if altitude_m >= top.base_m {
        let z_km = geometric_height_km(altitude_m);
        return if z_km < TEMP_RISE_START_KM {
            186.8673
        } else if z_km < MODEL_TOP_KM {
            263.1905 - 76.3232 * (1.0 - ((z_km - TEMP_RISE_START_KM) / 19.9429).powi(2)).sqrt()
        } else {
            195.08134
        };
    }

    let index = layer_index(altitude_m);
    // Temperature at the base of the layer, accumulated over the ones below.
    let base_temperature_k =
        ATMOSPHERE_DATA
            .windows(2)
            .take(index)
            .fold(T0_K, |temperature, pair| {
                temperature + pair[0].lapse_k_per_km * (pair[1].base_m - pair[0].base_m) * 1e-3
            });
    let layer = &ATMOSPHERE_DATA[index];
    base_temperature_k + layer.lapse_k_per_km * (altitude_m - layer.base_m) * 1e-3
}

/// Upstream `getPresISA` \[hPa\]: pressure at `altitude_m` (ITU-R P.835-7
/// eqs. 3a-3g and 5); zero above 100 km geometric height.
pub fn pressure_hpa(altitude_m: f64) -> f64 {
    let index = layer_index(altitude_m);
    let lapse = ATMOSPHERE_DATA[index].lapse_k_per_km;
    let h_km = altitude_m * 1e-3;
    match index {
        0 => 1013.25 * (288.15 / (288.15 - lapse * h_km)).powf(-ITU_PRESS_CONST / lapse),
        1 => 226.3226 * (-0.157688 * (h_km - 11.0)).exp(),
        2 => 54.7498 * (216.65 / (216.65 + (h_km - 20.0))).powf(-ITU_PRESS_CONST),
        3 => 8.680422 * (228.65 / (228.65 + lapse * (h_km - 32.0))).powf(-ITU_PRESS_CONST / lapse),
        4 => 1.109106 * (-0.126226 * (h_km - 47.0)).exp(),
        5 => 0.6694167 * (270.65 / (270.65 - lapse * (h_km - 51.0))).powf(-ITU_PRESS_CONST / lapse),
        6 => {
            0.03956649 * (214.65 / (214.65 - lapse * (h_km - 71.0))).powf(-ITU_PRESS_CONST / lapse)
        }
        _ => {
            let z = geometric_height_km(altitude_m);
            if z < MODEL_TOP_KM {
                (95.571899 - 4.011801 * z + 6.424731 * z * z * 1e-2 - 4.789660 * z * z * z * 1e-4
                    + 1.340543 * z * z * z * z * 1e-6)
                    .exp()
            } else {
                0.0
            }
        }
    }
}

/// Upstream `getWaterVapDensityISA` \[g/m^3\]: water-vapour density at
/// `altitude_m` (ITU-R P.835-7 eq. 6), floored at a mixing ratio of 2e-6
/// (eq. 8).
pub fn water_vapour_density_g_per_m3(altitude_m: f64) -> f64 {
    let z_km = geometric_height_km(altitude_m);
    let rho = RHO0_G_PER_M3 * (-z_km / H0_WATER_KM).exp();
    let temperature = temperature_k(altitude_m);
    let partial_pressure_hpa = (rho * temperature) / 216.7;
    let pressure = pressure_hpa(altitude_m);
    if partial_pressure_hpa / pressure < 2.0e-6 {
        2.0e-6 * (pressure * 216.7) / temperature
    } else {
        rho
    }
}

/// Upstream `getLayerBoundaries` \[m\]: every altitude where a formula
/// changes branch, ascending: the layer bases above sea level, then the 91 km
/// and 100 km geometric heights converted to the input altitude.
pub fn layer_boundaries_m() -> Vec<f64> {
    ATMOSPHERE_DATA[1..]
        .iter()
        .map(|layer| layer.base_m)
        .chain(
            [TEMP_RISE_START_KM, MODEL_TOP_KM]
                .map(|height_km| R_EARTH_ITU_KM * height_km / (R_EARTH_ITU_KM + height_km) * 1e3),
        )
        .collect()
}

#[cfg(test)]
mod tests {
    //! Upstream has no unit test of its own for `ItuRefAtmosphere`; the
    //! boundary check is `LinkBudgetAtmosphere.ExposesAtmosphericBoundaries`
    //! of `linkBudget/_UnitTest/test_linkBudgetAtmosphere.cpp`, and these
    //! check the profile against ITU-R P.835's published values.

    use super::*;

    #[test]
    fn exposes_atmospheric_boundaries() {
        let itu_earth_radius_km = 6356.766;
        let expected = [
            11000.0,
            20000.0,
            32000.0,
            47000.0,
            51000.0,
            71000.0,
            84852.0,
            itu_earth_radius_km * 91.0 / (itu_earth_radius_km + 91.0) * 1e3,
            itu_earth_radius_km * 100.0 / (itu_earth_radius_km + 100.0) * 1e3,
        ];
        let boundaries = layer_boundaries_m();
        assert_eq!(boundaries.len(), expected.len());
        for (boundary, expected) in boundaries.iter().zip(expected) {
            assert!(
                (boundary - expected).abs() <= 1.0e-8,
                "{boundary} vs {expected}"
            );
        }
    }

    #[test]
    fn sea_level_and_tropopause_match_the_standard() {
        assert_eq!(temperature_k(0.0), 288.15);
        assert!((pressure_hpa(0.0) - 1013.25).abs() < 1e-12);
        assert!((water_vapour_density_g_per_m3(0.0) - 7.5).abs() < 1e-12);
        // 11 km: 288.15 - 6.5 * 11 = 216.65 K, and the eq. 3b base pressure.
        assert!((temperature_k(11.0e3) - 216.65).abs() < 1e-9);
        assert!((pressure_hpa(11.0e3) - 226.3226).abs() < 1e-9);
        // Upstream's eq. 3a, with the signed lapse rate.
        let expected = 1013.25 * (288.15_f64 / (288.15 + 6.5 * 5.0)).powf(34.1632 / 6.5);
        assert!((pressure_hpa(5.0e3) - expected).abs() < 1e-9);
    }

    #[test]
    fn upper_atmosphere_branches() {
        assert_eq!(temperature_k(85.0e3), 186.8673);
        assert_eq!(temperature_k(200.0e3), 195.08134);
        assert_eq!(pressure_hpa(200.0e3), 0.0);
        let below_top = pressure_hpa(95.0e3);
        assert!(below_top > 0.0 && below_top < 1e-3);
    }
}
