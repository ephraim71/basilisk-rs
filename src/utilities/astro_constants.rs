//! Physical constants, unit conversions and planetary data, ported from
//! upstream Basilisk `architecture/utilities/astroConstants.h`.
//!
//! Every identifier keeps its upstream spelling (including the historical
//! `G_UNIVERSIAL` typo) so ports read like the C they come from.
//!
//! # Units — read this before using a constant
//!
//! **Most planetary constants are kilometre based**, exactly as upstream
//! defines them: `MU_*` are **km^3/s^2**, `REQ_*`, `RP_*`, `SMA_*` and [`AU`]
//! are **km**, and [`G_UNIVERSIAL`] is **km^3/(s^2 kg)**. Basilisk messages
//! and most flight-software configuration are SI (m, m/s), so convert before
//! mixing — for example `MU_EARTH * 1e9` (m^3/s^2) and `REQ_EARTH * 1e3` (m),
//! which is what upstream Python scripts do. The few SI constants ([`AU2M`],
//! [`SPEED_LIGHT`], [`EARTH_GRAV`], [`SOLAR_FLUX_EARTH`], ...) say so in their
//! docs. Angles are radians and rates are rad/s.
//!
//! # Upstream macros not repeated here
//!
//! Upstream also defines `MPI`, `MPI_2`, `M_E` and `NAN` as fall-backs for
//! platforms without them. Use [`std::f64::consts::PI`],
//! [`std::f64::consts::FRAC_PI_2`], [`std::f64::consts::E`] and [`f64::NAN`]
//! instead; `MPI`, `MPI_2` and `M_E` are bit-identical to the `std` values.
//!
//! The derived constants below are fully evaluated values. Upstream's
//! unparenthesized macros (`R2D`, `SEC2DAY`, `SMA_*`, `I_*`) can parse
//! differently inside a larger C expression — `x / R2D` expands to
//! `x / 180. / MPI` — so when porting such an expression, reproduce the C
//! parse rather than substituting the constant blindly.

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Universal gravitational constant, **km^3/(s^2 kg)** (km based). Upstream
/// `G_UNIVERSIAL` (the spelling is upstream's).
pub const G_UNIVERSIAL: f64 = 6.67259e-20;

/// Astronomical unit, **km**. Upstream `AU`.
pub const AU: f64 = 149_597_870.693;

/// Astronomical unit, **km**. Upstream `AU2KM`, a deprecated alias of [`AU`]
/// that upstream keeps until 2027-04-24.
#[deprecated(
    note = "AU2KM is deprecated in favor of AU (upstream keeps both names until Apr 24, 2027)"
)]
pub const AU2KM: f64 = AU;

/// Astronomical unit, **m**. Upstream `AU2M`.
pub const AU2M: f64 = 149_597_870_693.0;

/// Speed of light in vacuum, **m/s**. Upstream `SPEED_LIGHT` (an integer
/// literal upstream; `f64` here because every use is floating point).
pub const SPEED_LIGHT: f64 = 299_792_458.0;

/// Solar flux at Earth's distance (1 AU), **W/m^2**. Upstream
/// `SOLAR_FLUX_EARTH`.
pub const SOLAR_FLUX_EARTH: f64 = 1361.0;

/// Elementary charge, **C**. Upstream `Q_CHARGE`.
pub const Q_CHARGE: f64 = 1.602_176_634e-19;

/// Electron mass, **kg**. Upstream `MASS_ELECTRON`.
pub const MASS_ELECTRON: f64 = 9.109_383_701_5e-31;

/// Proton mass, **kg**. Upstream `MASS_PROTON`.
pub const MASS_PROTON: f64 = 1.672_621_9e-27;

/// Cosmic microwave background temperature, **K**. Upstream
/// `CMB_TEMPERATURE`.
pub const CMB_TEMPERATURE: f64 = 2.725;

/// Boltzmann's constant, **J/K**. Upstream `K_BOLTZMANN`.
pub const K_BOLTZMANN: f64 = 1.380_648_52e-23;

/// Standard gravitational acceleration at Earth's surface, **m/s^2**.
/// Upstream `EARTH_GRAV`.
pub const EARTH_GRAV: f64 = 9.80665;

// ---------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------

/// Degrees to radians, **rad/deg** (`PI / 180`). Upstream `D2R`.
pub const D2R: f64 = PI / 180.0;

/// Radians to degrees, **deg/rad** (`180 / PI`). Upstream `R2D`
/// (unparenthesized upstream: `x / R2D` in C means `x / 180 / PI`).
pub const R2D: f64 = 180.0 / PI;

/// Revolutions per minute to radians per second, **(rad/s)/rpm**
/// (`2 PI / 60`). Upstream `RPM`.
pub const RPM: f64 = 2.0 * PI / 60.0;

/// Seconds to days, **day/s** (`1 / 86400`). Upstream `SEC2DAY` (whose C
/// macro carries a stray trailing `;`).
pub const SEC2DAY: f64 = 1.0 / (60.0 * 60.0 * 24.0);

// ---------------------------------------------------------------------------
// Gravitational parameters mu = G m, **km^3/s^2**, from the NAIF SPICE
// generic kernels.
// ---------------------------------------------------------------------------

/// Sun gravitational parameter, **km^3/s^2** (x 1e9 for m^3/s^2). Upstream
/// `MU_SUN`.
pub const MU_SUN: f64 = 132_712_440_023.310;
/// Mercury gravitational parameter, **km^3/s^2**. Upstream `MU_MERCURY`.
pub const MU_MERCURY: f64 = 22_032.080;
/// Venus gravitational parameter, **km^3/s^2**. Upstream `MU_VENUS`.
pub const MU_VENUS: f64 = 324_858.599;
/// Earth gravitational parameter, **km^3/s^2** (x 1e9 for m^3/s^2).
/// Upstream `MU_EARTH`.
pub const MU_EARTH: f64 = 398_600.436;
/// Moon gravitational parameter, **km^3/s^2**. Upstream `MU_MOON`.
pub const MU_MOON: f64 = 4902.799;
/// Mars gravitational parameter, **km^3/s^2**. Upstream `MU_MARS`.
pub const MU_MARS: f64 = 42_828.314;
/// Jupiter gravitational parameter, **km^3/s^2**. Upstream `MU_JUPITER`.
pub const MU_JUPITER: f64 = 126_712_767.881;
/// Saturn gravitational parameter, **km^3/s^2**. Upstream `MU_SATURN`.
pub const MU_SATURN: f64 = 37_940_626.068;
/// Uranus gravitational parameter, **km^3/s^2**. Upstream `MU_URANUS`.
pub const MU_URANUS: f64 = 5_794_559.128;
/// Neptune gravitational parameter, **km^3/s^2**. Upstream `MU_NEPTUNE`.
pub const MU_NEPTUNE: f64 = 6_836_534.065;
/// Pluto gravitational parameter, **km^3/s^2**. Upstream `MU_PLUTO`.
pub const MU_PLUTO: f64 = 983.055;

// ---------------------------------------------------------------------------
// Planetary data, **km** for every length (NSSDC planetary fact sheets).
// `J*` zonal coefficients and `E_*` eccentricities are dimensionless, `I_*`
// inclinations (to the ecliptic) are rad and `OMEGA_*` rotation rates are
// rad/s (negative for retrograde rotation).
// ---------------------------------------------------------------------------

/// Sun equatorial radius, **km**. Upstream `REQ_SUN`.
pub const REQ_SUN: f64 = 695_000.0;

/// Mercury equatorial radius, **km**. Upstream `REQ_MERCURY`.
pub const REQ_MERCURY: f64 = 2439.7;
/// Mercury J2 zonal coefficient, dimensionless. Upstream `J2_MERCURY`.
pub const J2_MERCURY: f64 = 60.0e-6;
/// Mercury heliocentric semi-major axis, **km** (`0.38709893 AU`). Upstream
/// `SMA_MERCURY`.
pub const SMA_MERCURY: f64 = 0.387_098_93 * AU;
/// Mercury orbit inclination, **rad**. Upstream `I_MERCURY`.
pub const I_MERCURY: f64 = 7.00487 * D2R;
/// Mercury orbit eccentricity, dimensionless. Upstream `E_MERCURY`.
pub const E_MERCURY: f64 = 0.205_630_69;
/// Mercury rotation rate, **rad/s** (one turn per 1407.6 h). Upstream
/// `OMEGA_MERCURY`.
pub const OMEGA_MERCURY: f64 = 2.0 * PI / 1407.6 / 3600.0;

/// Venus equatorial radius, **km**. Upstream `REQ_VENUS`.
pub const REQ_VENUS: f64 = 6051.8;
/// Venus J2 zonal coefficient, dimensionless. Upstream `J2_VENUS`.
pub const J2_VENUS: f64 = 4.458e-6;
/// Venus heliocentric semi-major axis, **km** (`0.72333199 AU`). Upstream
/// `SMA_VENUS`.
pub const SMA_VENUS: f64 = 0.723_331_99 * AU;
/// Venus orbit inclination, **rad**. Upstream `I_VENUS`.
pub const I_VENUS: f64 = 3.39471 * D2R;
/// Venus orbit eccentricity, dimensionless. Upstream `E_VENUS`.
pub const E_VENUS: f64 = 0.006_773_23;
/// Venus rotation rate, **rad/s**, negative (retrograde, one turn per
/// 5832.6 h). Upstream `OMEGA_VENUS`.
pub const OMEGA_VENUS: f64 = 2.0 * PI / (-5832.6) / 3600.0;

/// Earth equatorial radius, **km** (from SPICE; x 1e3 for m). Upstream
/// `REQ_EARTH`.
pub const REQ_EARTH: f64 = 6378.1366;
/// Earth polar radius, **km** (from SPICE). Upstream `RP_EARTH`.
pub const RP_EARTH: f64 = 6356.7519;
/// Earth J2 zonal coefficient, dimensionless. Upstream `J2_EARTH`.
pub const J2_EARTH: f64 = 1082.616e-6;
/// Earth J3 zonal coefficient, dimensionless. Upstream `J3_EARTH`.
pub const J3_EARTH: f64 = -2.53881e-6;
/// Earth J4 zonal coefficient, dimensionless. Upstream `J4_EARTH`.
pub const J4_EARTH: f64 = -1.65597e-6;
/// Earth J5 zonal coefficient, dimensionless. Upstream `J5_EARTH`.
pub const J5_EARTH: f64 = -0.15e-6;
/// Earth J6 zonal coefficient, dimensionless. Upstream `J6_EARTH`.
pub const J6_EARTH: f64 = 0.57e-6;
/// Earth heliocentric semi-major axis, **km** (`1.00000011 AU`). Upstream
/// `SMA_EARTH`.
pub const SMA_EARTH: f64 = 1.000_000_11 * AU;
/// Earth orbit inclination, **rad**. Upstream `I_EARTH`.
pub const I_EARTH: f64 = 0.00005 * D2R;
/// Earth orbit eccentricity, dimensionless. Upstream `E_EARTH`.
pub const E_EARTH: f64 = 0.016_710_22;
/// Earth rotation rate, **rad/s** (one turn per 23.9345 h). Upstream
/// `OMEGA_EARTH`.
pub const OMEGA_EARTH: f64 = 2.0 * PI / 23.9345 / 3600.0;

/// Moon equatorial radius, **km**. Upstream `REQ_MOON`.
pub const REQ_MOON: f64 = 1737.4;
/// Moon J2 zonal coefficient, dimensionless. Upstream `J2_MOON`.
pub const J2_MOON: f64 = 202.7e-6;
/// Moon geocentric semi-major axis, **km**. Upstream `SMA_MOON`.
pub const SMA_MOON: f64 = 0.3844e6;
/// Moon orbit eccentricity, dimensionless. Upstream `E_MOON`.
pub const E_MOON: f64 = 0.0549;
/// Moon rotation rate, **rad/s** (one turn per 655.728 h). Upstream
/// `OMEGA_MOON`.
pub const OMEGA_MOON: f64 = 2.0 * PI / 655.728 / 3600.0;

/// Mars equatorial radius, **km**. Upstream `REQ_MARS`.
pub const REQ_MARS: f64 = 3396.19;
/// Mars polar radius, **km**. Upstream `RP_MARS`.
pub const RP_MARS: f64 = 3376.2;
/// Mars J2 zonal coefficient, dimensionless. Upstream `J2_MARS`.
pub const J2_MARS: f64 = 1960.45e-6;
/// Mars heliocentric semi-major axis, **km** (`1.52366231 AU`). Upstream
/// `SMA_MARS`.
pub const SMA_MARS: f64 = 1.523_662_31 * AU;
/// Mars orbit inclination, **rad**. Upstream `I_MARS`.
pub const I_MARS: f64 = 1.85061 * D2R;
/// Mars orbit eccentricity, dimensionless. Upstream `E_MARS`.
pub const E_MARS: f64 = 0.093_412_33;
/// Mars rotation rate, **rad/s** (one turn per 24.6229 h). Upstream
/// `OMEGA_MARS`.
pub const OMEGA_MARS: f64 = 2.0 * PI / 24.6229 / 3600.0;

/// Phobos equatorial radius, **km**. Upstream `REQ_PHOBOS`.
pub const REQ_PHOBOS: f64 = 11.2;

/// Deimos equatorial radius, **km**. Upstream `REQ_DEIMOS`.
pub const REQ_DEIMOS: f64 = 6.1;

/// Jupiter equatorial radius, **km**. Upstream `REQ_JUPITER`.
pub const REQ_JUPITER: f64 = 71_492.0;
/// Jupiter J2 zonal coefficient, dimensionless. Upstream `J2_JUPITER`.
pub const J2_JUPITER: f64 = 14_736.0e-6;
/// Jupiter heliocentric semi-major axis, **km** (`5.20336301 AU`). Upstream
/// `SMA_JUPITER`.
pub const SMA_JUPITER: f64 = 5.203_363_01 * AU;
/// Jupiter orbit inclination, **rad**. Upstream `I_JUPITER`.
pub const I_JUPITER: f64 = 1.30530 * D2R;
/// Jupiter orbit eccentricity, dimensionless. Upstream `E_JUPITER`.
pub const E_JUPITER: f64 = 0.048_392_66;
/// Jupiter rotation rate, **rad/s** (one turn per 9.9250 h). Upstream
/// `OMEGA_JUPITER`.
pub const OMEGA_JUPITER: f64 = 2.0 * PI / 9.9250 / 3600.0;

/// Saturn equatorial radius, **km**. Upstream `REQ_SATURN`.
pub const REQ_SATURN: f64 = 60_268.0;
/// Saturn J2 zonal coefficient, dimensionless. Upstream `J2_SATURN`.
pub const J2_SATURN: f64 = 16_298.0e-6;
/// Saturn heliocentric semi-major axis, **km** (`9.53707032 AU`). Upstream
/// `SMA_SATURN`.
pub const SMA_SATURN: f64 = 9.537_070_32 * AU;
/// Saturn orbit inclination, **rad**. Upstream `I_SATURN`.
pub const I_SATURN: f64 = 2.48446 * D2R;
/// Saturn orbit eccentricity, dimensionless. Upstream `E_SATURN`.
pub const E_SATURN: f64 = 0.054_150_60;
/// Saturn rotation rate, **rad/s** (one turn per 10.656 h). Upstream
/// `OMEGA_SATURN`.
pub const OMEGA_SATURN: f64 = 2.0 * PI / 10.656 / 3600.0;

/// Uranus equatorial radius, **km**. Upstream `REQ_URANUS`.
pub const REQ_URANUS: f64 = 25_559.0;
/// Uranus J2 zonal coefficient, dimensionless. Upstream `J2_URANUS`.
pub const J2_URANUS: f64 = 3343.43e-6;
/// Uranus heliocentric semi-major axis, **km** (`19.19126393 AU`). Upstream
/// `SMA_URANUS`.
pub const SMA_URANUS: f64 = 19.191_263_93 * AU;
/// Uranus orbit inclination, **rad**. Upstream `I_URANUS`.
pub const I_URANUS: f64 = 0.76986 * D2R;
/// Uranus orbit eccentricity, dimensionless. Upstream `E_URANUS`.
pub const E_URANUS: f64 = 0.047_167_71;
/// Uranus rotation rate, **rad/s**, negative (retrograde, one turn per
/// 17.24 h). Upstream `OMEGA_URANUS`.
pub const OMEGA_URANUS: f64 = 2.0 * PI / (-17.24) / 3600.0;

/// Neptune equatorial radius, **km**. Upstream `REQ_NEPTUNE`.
pub const REQ_NEPTUNE: f64 = 24_746.0;
/// Neptune J2 zonal coefficient, dimensionless. Upstream `J2_NEPTUNE`.
pub const J2_NEPTUNE: f64 = 3411.0e-6;
/// Neptune heliocentric semi-major axis, **km** (`30.06896348 AU`). Upstream
/// `SMA_NEPTUNE`.
pub const SMA_NEPTUNE: f64 = 30.068_963_48 * AU;
/// Neptune orbit inclination, **rad**. Upstream `I_NEPTUNE`.
pub const I_NEPTUNE: f64 = 1.76917 * D2R;
/// Neptune orbit eccentricity, dimensionless. Upstream `E_NEPTUNE`.
pub const E_NEPTUNE: f64 = 0.008_585_87;
/// Neptune rotation rate, **rad/s** (one turn per 16.11 h). Upstream
/// `OMEGA_NEPTUNE`.
pub const OMEGA_NEPTUNE: f64 = 2.0 * PI / 16.11 / 3600.0;

/// Pluto equatorial radius, **km**. Upstream `REQ_PLUTO`.
pub const REQ_PLUTO: f64 = 1137.0;
/// Pluto heliocentric semi-major axis, **km** (`39.48168677 AU`). Upstream
/// `SMA_PLUTO`.
pub const SMA_PLUTO: f64 = 39.481_686_77 * AU;
/// Pluto orbit inclination, **rad**. Upstream `I_PLUTO`.
pub const I_PLUTO: f64 = 17.14175 * D2R;
/// Pluto orbit eccentricity, dimensionless. Upstream `E_PLUTO`.
pub const E_PLUTO: f64 = 0.248_807_66;
/// Pluto rotation rate, **rad/s**, negative (retrograde, one turn per
/// 153.2928 h). Upstream `OMEGA_PLUTO`.
pub const OMEGA_PLUTO: f64 = 2.0 * PI / (-153.2928) / 3600.0;

#[cfg(test)]
mod tests {
    use super::*;

    /// The upstream `MPI`, `MPI_2` and `M_E` fall-back literals parse to
    /// exactly the `std` constants this port tells callers to use instead.
    #[test]
    #[allow(clippy::approx_constant)]
    fn std_constants_replace_upstream_fallback_macros_bit_for_bit() {
        assert_eq!(3.141592653589793_f64, std::f64::consts::PI);
        assert_eq!(1.5707963267948966_f64, std::f64::consts::FRAC_PI_2);
        assert_eq!(2.718281828459045_f64, std::f64::consts::E);
    }

    /// Spot-check that literals match the header exactly (these mirror the
    /// tests of upstream's own generated Rust bindings).
    #[test]
    fn literals_match_upstream_header() {
        assert_eq!(MU_EARTH, 398600.436);
        assert_eq!(J2_EARTH, 1082.616e-6);
        assert_eq!(REQ_EARTH, 6378.1366);
        assert_eq!(AU, 149597870.693);
        assert_eq!(AU2M, 149597870693.0);
        assert_eq!(MU_SUN, 132712440023.310);
        assert_eq!(SPEED_LIGHT, 299792458.0);
    }

    #[test]
    #[allow(deprecated)]
    fn au2km_is_an_alias_of_au() {
        assert_eq!(AU2KM, AU);
    }

    #[test]
    fn derived_constants_evaluate_like_the_c_macros() {
        let pi = std::f64::consts::PI;
        assert_eq!(D2R, pi / 180.0);
        assert_eq!(R2D, 180.0 / pi);
        assert_eq!(RPM, 2.0 * pi / 60.0);
        assert_eq!(SEC2DAY, 1.0 / 86400.0);
        assert_eq!(SMA_EARTH, 1.00000011 * AU);
        assert_eq!(I_MARS, 1.85061 * (pi / 180.0));
        assert_eq!(OMEGA_EARTH, 2.0 * pi / 23.9345 / 3600.0);
        // Sidereal-day sanity check: about 7.2921e-5 rad/s.
        assert!((OMEGA_EARTH - 7.2921e-5).abs() < 1e-8);
        // Retrograde rotators spin negatively.
        const { assert!(OMEGA_VENUS < 0.0 && OMEGA_URANUS < 0.0 && OMEGA_PLUTO < 0.0) };
        assert!((D2R * R2D - 1.0).abs() < 1e-15);
        assert!((AU2M - AU * 1000.0).abs() < 1e-3);
    }
}
