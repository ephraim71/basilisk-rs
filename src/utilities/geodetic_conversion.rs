//! Conversions among planet-centred inertial (PCI), planet-centred
//! planet-fixed (PCPF), latitude/longitude/altitude (LLA) and site-local
//! south-east-zenith (SEZ) coordinates. Port of upstream Basilisk
//! `architecture/utilities/geodeticConversion.cpp`.
//!
//! | upstream | here |
//! |---|---|
//! | `PCI2PCPF` | [`pci_to_pcpf`] |
//! | `PCPF2LLA` | [`pcpf_to_lla`] |
//! | `PCI2LLA` | [`pci_to_lla`] |
//! | `LLA2PCPF` | [`lla_to_pcpf`] |
//! | `PCPF2PCI` | [`pcpf_to_pci`] |
//! | `LLA2PCI` | [`lla_to_pci`] |
//! | `C_PCPF2SEZ` | [`dcm_pcpf_to_sez`] |
//!
//! # Conventions
//!
//! * Upstream's `J20002Pfix` argument (a row-major `double[3][3]`) is the
//!   `[PN]` DCM `dcm_pn`, mapping inertial components to planet-fixed ones:
//!   the `inertial_to_fixed` of a [`crate::messages::PlanetOrientation`].
//! * Upstream's optional `planetPoRad` (default `-1.0`, a spherical planet)
//!   is `polar_radius: Option<f64>`. `None`, or a negative value as upstream,
//!   selects the sphere of the equatorial radius.
//! * Distances are in the unit of the radii (metres in every caller); angles
//!   are radians.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use crate::utilities::rigid_body_kinematics::{euler2, euler3};

/// Upstream `nr_kapp_iter` in `PCPF2LLA`: iteration budget of the ellipsoidal
/// latitude solution.
const KAPPA_ITERATIONS: usize = 10;
/// Upstream `good_kappa_acc` in `PCPF2LLA`: the iteration stops once `kappa`
/// changes by less than this.
const KAPPA_TOLERANCE: f64 = 1.0e-13;

/// Planet-centred latitude, longitude and altitude, upstream's
/// `llaPosition` (`[0]` latitude, `[1]` longitude, `[2]` altitude).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Lla {
    /// Latitude above the planet equator \[rad\].
    pub latitude_rad: f64,
    /// Longitude east of the planet prime meridian \[rad\].
    pub longitude_rad: f64,
    /// Altitude above the planet surface, in the unit of the radii \[m\].
    pub altitude_m: f64,
}

impl Lla {
    /// The point at `latitude_rad`, `longitude_rad` and `altitude_m`.
    pub const fn new(latitude_rad: f64, longitude_rad: f64, altitude_m: f64) -> Self {
        Self {
            latitude_rad,
            longitude_rad,
            altitude_m,
        }
    }
}

/// Planet-fixed components of an inertial position (upstream `PCI2PCPF`).
pub fn pci_to_pcpf(pci_position: Vector3<f64>, dcm_pn: &Matrix3<f64>) -> Vector3<f64> {
    dcm_pn * pci_position
}

/// Inertial components of a planet-fixed position (upstream `PCPF2PCI`).
pub fn pcpf_to_pci(pcpf_position: Vector3<f64>, dcm_pn: &Matrix3<f64>) -> Vector3<f64> {
    dcm_pn.transpose() * pcpf_position
}

/// Latitude, longitude and altitude of a planet-fixed position (upstream
/// `PCPF2LLA`).
///
/// A sphere of radius `equatorial_radius` gives the geocentric latitude and
/// the height above that sphere. An ellipsoid solves the geodetic latitude by
/// upstream's fixed-point iteration on `kappa`, and switches the altitude to
/// `z/sin(phi) - N(1 - e^2)` beyond 45 degrees of latitude, where
/// `p/cos(phi) - N` is ill-conditioned.
pub fn pcpf_to_lla(
    pcpf_position: Vector3<f64>,
    equatorial_radius: f64,
    polar_radius: Option<f64>,
) -> Lla {
    let [x, y, z] = [pcpf_position.x, pcpf_position.y, pcpf_position.z];
    let longitude_rad = y.atan2(x);
    let Some(polar_radius) = polar_radius.filter(|radius| *radius >= 0.0) else {
        return Lla {
            latitude_rad: z.atan2((x * x + y * y).sqrt()),
            longitude_rad,
            altitude_m: pcpf_position.norm() - equatorial_radius,
        };
    };

    let ecc2 = 1.0 - polar_radius * polar_radius / (equatorial_radius * equatorial_radius);
    let mut kappa = 1.0 / (1.0 - ecc2);
    let p2 = x * x + y * y;
    let z2 = z * z;
    for _ in 0..KAPPA_ITERATIONS {
        let c_i = p2 + (1.0 - ecc2) * z2 * kappa * kappa;
        let c_i = (c_i * c_i * c_i).sqrt() / (equatorial_radius * ecc2);
        let kappa_next = 1.0 + (p2 + (1.0 - ecc2) * z2 * kappa * kappa * kappa) / (c_i - p2);
        if (kappa_next - kappa).abs() < KAPPA_TOLERANCE {
            break;
        }
        kappa = kappa_next;
    }

    let p = p2.sqrt();
    let latitude_rad = (kappa * z).atan2(p);
    let sin_lat = latitude_rad.sin();
    let cos_lat = latitude_rad.cos();
    let n = equatorial_radius / (1.0 - ecc2 * sin_lat * sin_lat).sqrt();
    let altitude_m = if sin_lat.abs() > cos_lat.abs() {
        z / sin_lat - n * (1.0 - ecc2)
    } else {
        p / cos_lat - n
    };
    Lla {
        latitude_rad,
        longitude_rad,
        altitude_m,
    }
}

/// Latitude, longitude and altitude of an inertial position (upstream
/// `PCI2LLA`).
pub fn pci_to_lla(
    pci_position: Vector3<f64>,
    dcm_pn: &Matrix3<f64>,
    equatorial_radius: f64,
    polar_radius: Option<f64>,
) -> Lla {
    pcpf_to_lla(
        pci_to_pcpf(pci_position, dcm_pn),
        equatorial_radius,
        polar_radius,
    )
}

/// Planet-fixed position of a latitude, longitude and altitude (upstream
/// `LLA2PCPF`).
pub fn lla_to_pcpf(lla: Lla, equatorial_radius: f64, polar_radius: Option<f64>) -> Vector3<f64> {
    let ecc2 = match polar_radius {
        Some(polar_radius) if polar_radius >= 0.0 => {
            1.0 - polar_radius * polar_radius / (equatorial_radius * equatorial_radius)
        }
        _ => 0.0,
    };
    let sin_lat = lla.latitude_rad.sin();
    let n = equatorial_radius / (1.0 - ecc2 * sin_lat * sin_lat).sqrt();
    Vector3::new(
        (n + lla.altitude_m) * lla.latitude_rad.cos() * lla.longitude_rad.cos(),
        (n + lla.altitude_m) * lla.latitude_rad.cos() * lla.longitude_rad.sin(),
        ((1.0 - ecc2) * n + lla.altitude_m) * sin_lat,
    )
}

/// Inertial position of a latitude, longitude and altitude (upstream
/// `LLA2PCI`).
pub fn lla_to_pci(
    lla: Lla,
    dcm_pn: &Matrix3<f64>,
    equatorial_radius: f64,
    polar_radius: Option<f64>,
) -> Vector3<f64> {
    pcpf_to_pci(lla_to_pcpf(lla, equatorial_radius, polar_radius), dcm_pn)
}

/// DCM `[LP]` from planet-fixed components to the south-east-zenith frame of
/// the site at `latitude_rad`, `longitude_rad` (upstream `C_PCPF2SEZ`):
/// `Euler2(π/2 - lat) · Euler3(lon)`.
pub fn dcm_pcpf_to_sez(latitude_rad: f64, longitude_rad: f64) -> Matrix3<f64> {
    euler2(FRAC_PI_2 - latitude_rad) * euler3(longitude_rad)
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `architecture/utilities/tests/test_geodeticConversion.cpp`, with its
    //! tolerances.

    use std::f64::consts::FRAC_PI_2;

    use nalgebra::{Matrix3, Vector3};

    use super::*;

    /// Upstream `planetEqRad` of the LLA cases \[m\].
    const EQUATORIAL_RADIUS_M: f64 = 6378.1363e3;
    /// Upstream `planetPoRad` of the LLA cases \[m\].
    const POLAR_RADIUS_M: f64 = 6356.7523e3;

    /// Upstream `m33Set(1, 0, 0, 0, 0, 1, 0, -1, 0, J20002Pfix)`.
    fn quarter_turn_about_x() -> Matrix3<f64> {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    }

    #[track_caller]
    fn assert_near(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() < tolerance,
            "{actual} vs {expected}, tolerance {tolerance}"
        );
    }

    #[test]
    fn pci_to_pcpf_rotates_into_the_planet_frame() {
        let answer = pci_to_pcpf(Vector3::new(1.0, 2.0, 3.0), &quarter_turn_about_x());
        assert_eq!(answer, Vector3::new(1.0, 3.0, -2.0));
    }

    #[test]
    fn pcpf_to_pci_rotates_back_to_inertial() {
        let answer = pcpf_to_pci(Vector3::new(1.0, 2.0, 3.0), &quarter_turn_about_x());
        assert_eq!(answer, Vector3::new(1.0, -3.0, 2.0));
    }

    #[test]
    fn lla_to_pcpf_matches_the_reference_ellipsoid_point() {
        let lla = Lla::new(0.6935805104270613, 1.832425562445269, 1596.668);
        let answer = lla_to_pcpf(lla, EQUATORIAL_RADIUS_M, Some(POLAR_RADIUS_M));
        let expected = Vector3::new(-1270640.01, 4745322.63, 4056784.62);
        assert!((expected - answer).norm() < 100.0);
    }

    #[test]
    fn pcpf_to_lla_matches_the_reference_ellipsoid_point() {
        let pcpf = Vector3::new(-1270640.01, 4745322.63, 4056784.62);
        let answer = pcpf_to_lla(pcpf, EQUATORIAL_RADIUS_M, Some(POLAR_RADIUS_M));
        assert_near(answer.latitude_rad, 0.6935805104270613, 0.0001);
        assert_near(answer.longitude_rad, 1.832425562445269, 0.0001);
        assert_near(answer.altitude_m, 1596.668, 100.0);
    }

    #[test]
    fn pcpf_to_lla_is_well_conditioned_at_the_poles() {
        let altitude_m = 400.0e3;
        let north = pcpf_to_lla(
            Vector3::new(0.0, 0.0, POLAR_RADIUS_M + altitude_m),
            EQUATORIAL_RADIUS_M,
            Some(POLAR_RADIUS_M),
        );
        assert!((north.latitude_rad - FRAC_PI_2).abs() <= 1.0e-12);
        assert!((north.altitude_m - altitude_m).abs() <= 1.0e-3);

        let south = pcpf_to_lla(
            Vector3::new(0.0, 0.0, -(POLAR_RADIUS_M + altitude_m)),
            EQUATORIAL_RADIUS_M,
            Some(POLAR_RADIUS_M),
        );
        assert!((south.latitude_rad + FRAC_PI_2).abs() <= 1.0e-12);
        assert!((south.altitude_m - altitude_m).abs() <= 1.0e-3);
    }

    #[test]
    fn pcpf_to_lla_round_trips_from_equator_to_pole() {
        let longitude_rad = 1.0;
        for altitude_m in [-1000.0, 0.0, 400.0e3, 36000.0e3] {
            let mut latitude_deg = -90.0_f64;
            while latitude_deg <= 90.0 {
                let lla = Lla::new(latitude_deg.to_radians(), longitude_rad, altitude_m);
                let answer = pcpf_to_lla(
                    lla_to_pcpf(lla, EQUATORIAL_RADIUS_M, Some(POLAR_RADIUS_M)),
                    EQUATORIAL_RADIUS_M,
                    Some(POLAR_RADIUS_M),
                );
                assert!(
                    (answer.latitude_rad - lla.latitude_rad).abs() <= 1.0e-10,
                    "latitude [deg] {latitude_deg}"
                );
                assert!(
                    (answer.altitude_m - lla.altitude_m).abs() <= 1.0e-3,
                    "latitude [deg] {latitude_deg}"
                );
                latitude_deg += 0.25;
            }
        }
    }

    /// Not an upstream case: the spherical branches (`planetPoRad < 0`) that
    /// every location module uses invert each other, and the inertial forms
    /// compose the planet-fixed ones with `[PN]`.
    #[test]
    fn spherical_and_inertial_forms_round_trip() {
        let radius_m = 6378.1366e3;
        let dcm_pn = euler3(0.3) * euler2(-0.2);
        let lla = Lla::new(0.4, -1.9, 1200.0);
        for polar_radius in [None, Some(-1.0)] {
            let pcpf = lla_to_pcpf(lla, radius_m, polar_radius);
            assert_near(pcpf.norm(), radius_m + lla.altitude_m, 1.0e-6);
            let answer = pcpf_to_lla(pcpf, radius_m, polar_radius);
            assert_near(answer.latitude_rad, lla.latitude_rad, 1.0e-14);
            assert_near(answer.longitude_rad, lla.longitude_rad, 1.0e-14);
            assert_near(answer.altitude_m, lla.altitude_m, 1.0e-6);

            let pci = lla_to_pci(lla, &dcm_pn, radius_m, polar_radius);
            assert!((pci - dcm_pn.transpose() * pcpf).norm() < 1.0e-6);
            let answer = pci_to_lla(pci, &dcm_pn, radius_m, polar_radius);
            assert_near(answer.latitude_rad, lla.latitude_rad, 1.0e-12);
            assert_near(answer.longitude_rad, lla.longitude_rad, 1.0e-12);
        }
    }

    /// Not an upstream case: `[LP]` maps the local south, east and zenith
    /// directions of the site onto the SEZ axes.
    #[test]
    fn sez_frame_axes_point_south_east_and_up() {
        let (lat, lon) = (0.6_f64, -2.1_f64);
        let dcm_lp = dcm_pcpf_to_sez(lat, lon);
        let zenith = Vector3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
        let east = Vector3::new(-lon.sin(), lon.cos(), 0.0);
        let south = east.cross(&zenith);
        assert!((dcm_lp * south - Vector3::x()).norm() < 1.0e-15);
        assert!((dcm_lp * east - Vector3::y()).norm() < 1.0e-15);
        assert!((dcm_lp * zenith - Vector3::z()).norm() < 1.0e-15);
    }
}
