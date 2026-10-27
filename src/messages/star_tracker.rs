//! Star-tracker attitude sample in the case frame (upstream
//! `STSensorMsgPayload`).

use nalgebra::{Matrix3, Quaternion, UnitQuaternion};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One star-tracker attitude sample: upstream `STSensorMsg`.
///
/// # Quaternion convention
///
/// Upstream `qInrtl2Case` is the Euler-parameter set β_CN of the case frame
/// `C` relative to inertial `N`, **scalar first** (`[β0, β1, β2, β3]`), with
/// `EP2C(qInrtl2Case) = [CN]`.
///
/// `attitude_inertial_to_sensor` holds the same attitude as a nalgebra
/// rotation, stored **scalar last** (`i, j, k, w`), whose rotation matrix *is*
/// that DCM: `attitude_inertial_to_sensor.to_rotation_matrix() == [CN]`, mapping
/// inertial components to case components. Basilisk DCMs are passive and
/// nalgebra rotations active, so the two quaternions are conjugates:
/// `(w, i, j, k) = (β0, -β1, -β2, -β3)` up to an overall sign. Convert with
/// [`Self::from_q_inrtl2case`], [`Self::q_inrtl2case`] and [`Self::dcm_cn`]
/// rather than by copying components.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StarTrackerMsg {
    /// Upstream `timeTag` \[ns\]: time of the spacecraft state the sample was
    /// taken from.
    pub time_tag_nanos: u64,
    /// Upstream `qInrtl2Case`: inertial-to-case attitude, see the type docs for
    /// the ordering and conjugation.
    pub attitude_inertial_to_sensor: UnitQuaternion<f64>,
}

impl Default for StarTrackerMsg {
    /// Identity attitude rather than upstream's all-zero `qInrtl2Case`, which
    /// is not a rotation; unchanged from before the time tag was added.
    fn default() -> Self {
        Self {
            time_tag_nanos: 0,
            attitude_inertial_to_sensor: UnitQuaternion::identity(),
        }
    }
}

impl StarTrackerMsg {
    /// Builds the message from upstream's scalar-first `qInrtl2Case` (β_CN).
    ///
    /// The set is normalized; upstream `EP2C` uses it as given.
    pub fn from_q_inrtl2case(time_tag_nanos: u64, q_inrtl2case: [f64; 4]) -> Self {
        let [b0, b1, b2, b3] = q_inrtl2case;
        Self {
            time_tag_nanos,
            attitude_inertial_to_sensor: UnitQuaternion::new_normalize(Quaternion::new(
                b0, -b1, -b2, -b3,
            )),
        }
    }

    /// Upstream's scalar-first `qInrtl2Case` (β_CN), with `β0 >= 0` as
    /// upstream `C2EP` produces it.
    pub fn q_inrtl2case(&self) -> [f64; 4] {
        let q = self.attitude_inertial_to_sensor.quaternion();
        let sign = if q.w < 0.0 { -1.0 } else { 1.0 };
        [sign * q.w, -sign * q.i, -sign * q.j, -sign * q.k]
    }

    /// `[CN]`, the inertial-to-case DCM upstream gets from
    /// `EP2C(qInrtl2Case)`.
    pub fn dcm_cn(&self) -> Matrix3<f64> {
        self.attitude_inertial_to_sensor
            .to_rotation_matrix()
            .into_inner()
    }
}

impl_telemetry!(StarTrackerMsg {
    time_tag_nanos,
    attitude_inertial_to_sensor,
});
