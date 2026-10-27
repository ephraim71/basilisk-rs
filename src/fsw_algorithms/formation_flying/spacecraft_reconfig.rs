//! Impulsive formation-reconfiguration planner for a deputy spacecraft: port of
//! upstream Basilisk `spacecraftReconfig`.
//!
//! At its first update after a reset the module plans, from the chief's and the
//! deputy's osculating classical elements, the three impulsive burns of Schaub &
//! Junkins, *Analytical Mechanics of Space Systems* §14.8.3, that take the
//! deputy to the target element difference within one chief orbit: a radial and
//! along-track burn at the chief's perigee, another at its apogee, and an
//! out-of-plane burn at the argument of latitude that corrects `δi` and `δΩ`.
//! On top of the textbook scheme the plan
//! * compensates the mean-anomaly drift that a nonzero `δa` causes until the
//!   in-plane burns are done,
//! * merges the out-of-plane burn into the perigee or apogee burn when the two
//!   are less than `att_control_time_s` apart,
//! * skips burns that would last less than one second per thruster,
//!
//! and then sorts the burns by time. The deputy has one-axis thrusters, so every
//! burn also has a target attitude, the one that aligns the first thruster with
//! the burn direction.
//!
//! Every update publishes a reference attitude (the optional `att_ref_in_msg`
//! while no burn is near or running, the burn attitude otherwise), the burn plan,
//! and, at the update where a burn starts (half its duration before the impulse
//! time), the thruster on-time command. Once the time since the reset exceeds
//! one chief orbital period the module resets itself and plans again.

use std::cmp::Ordering;
use std::f64::consts::{PI, TAU};

use nalgebra::{Matrix3, Vector3, Vector4, Vector6};

use super::mean_oe_feedback::adjust_range;
use crate::messages::{
    AttitudeReferenceMsg, Input, MAX_BURN_CNT, NavigationTranslationMsg, Output,
    ReconfigBurnArrayInfoMsg, ReconfigBurnInfoMsg, ReconfigBurnStatus, ThrusterArrayConfigMsg,
    ThrusterArrayOnTimeCommandMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::{normalize_or_zero, safe_acos};
use crate::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
    hill_frame, mean_to_eccentric_anomaly, state_to_elements, true_to_eccentric_anomaly,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, ep_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`SpacecraftReconfig`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct SpacecraftReconfigConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body.
    pub mu_m3ps2: f64,
    /// Upstream `attControlTime` \[s\]: time the deputy needs to slew to a
    /// burn attitude. Burns closer together than this are merged.
    pub att_control_time_s: f64,
    /// Upstream `targetClassicOED`: target deputy-minus-chief classical element
    /// difference `(δa, δe, δi, δΩ, δω, δM)`.
    ///
    /// Upstream documents `δa` as normalized by the chief's semi-major axis but
    /// subtracts it from the semi-major-axis difference in metres; that unit
    /// mismatch is kept.
    pub target_classic_oed: Vector6<f64>,
}

impl Default for SpacecraftReconfigConfig {
    fn default() -> Self {
        Self {
            name: "spacecraftReconfig".to_string(),
            mu_m3ps2: 0.0,
            att_control_time_s: 0.0,
            target_classic_oed: Vector6::zeros(),
        }
    }
}

/// Burn planner and attitude/thruster sequencer for formation reconfiguration.
#[derive(Clone, Debug)]
pub struct SpacecraftReconfig {
    /// User-settable parameters.
    pub config: SpacecraftReconfigConfig,
    /// Upstream `chiefTransInMsg`: chief's inertial position and velocity.
    pub chief_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `deputyTransInMsg`: deputy's inertial position and velocity.
    pub deputy_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `thrustConfigInMsg`: deputy's thrusters. The first thruster's
    /// direction and maximum thrust are used for every burn.
    pub thrust_config_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `attRefInMsg` (optional): nominal reference attitude, followed
    /// whenever no burn needs the thrusters pointed.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `vehicleConfigInMsg`: deputy's mass properties.
    pub vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `attRefOutMsg`: reference attitude for the deputy.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `onTimeOutMsg`: thruster on-times, written only at the update
    /// where a burn starts.
    pub on_time_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    /// Upstream `burnArrayInfoOutMsg`: the current burn plan.
    pub burn_array_info_out_msg: Output<ReconfigBurnArrayInfoMsg>,
    /// Upstream `resetPeriod` \[s\]; see [`Self::reset_period_s`].
    reset_period_s: f64,
    /// Upstream `tCurrent` \[s\]: time since the last reset.
    t_current_s: f64,
    /// Upstream `prevCallTime` \[ns\]; 0 means "no update since the reset".
    prev_call_time_nanos: u64,
    /// Upstream `attRefInIsLinked`, sampled at reset.
    att_ref_in_is_linked: bool,
    /// Upstream `burnArrayInfoOutMsgBuffer`: the burn plan, all
    /// [`ReconfigBurnStatus::NotScheduled`] until the first update after a
    /// reset.
    burns: ReconfigBurnArrayInfoMsg,
}

/// What one update commands.
struct ManeuverCommand {
    attitude: AttitudeReferenceMsg,
    /// Thruster on-times when a burn starts at this update.
    on_time: Option<ThrusterArrayOnTimeCommandMsg>,
}

impl SpacecraftReconfig {
    /// Creates the module with `config`, unconnected ports and no burn plan.
    pub fn new(config: SpacecraftReconfigConfig) -> Self {
        Self {
            config,
            chief_trans_in_msg: Input::default(),
            deputy_trans_in_msg: Input::default(),
            thrust_config_in_msg: Input::default(),
            att_ref_in_msg: Input::default(),
            vehicle_config_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            on_time_out_msg: Output::default(),
            burn_array_info_out_msg: Output::default(),
            reset_period_s: 0.0,
            t_current_s: 0.0,
            prev_call_time_nanos: 0,
            att_ref_in_is_linked: false,
            burns: ReconfigBurnArrayInfoMsg::default(),
        }
    }

    /// Upstream `resetPeriod` \[s\]: one orbital period of the chief, set when
    /// the burns are planned. The module resets itself once the time since its
    /// last reset exceeds it. A reset does not clear it.
    pub fn reset_period_s(&self) -> f64 {
        self.reset_period_s
    }

    /// Upstream `Reset_spacecraftReconfig`, also run by the module itself after
    /// one chief orbit: clears the plan and the timer.
    fn reset_state(&mut self) {
        if !self.chief_trans_in_msg.is_connected() {
            log::error!("spacecraftReconfig.chief_trans_in_msg is not connected");
        }
        if !self.deputy_trans_in_msg.is_connected() {
            log::error!("spacecraftReconfig.deputy_trans_in_msg is not connected");
        }
        if !self.thrust_config_in_msg.is_connected() {
            log::error!("spacecraftReconfig.thrust_config_in_msg is not connected");
        }
        if !self.vehicle_config_in_msg.is_connected() {
            log::error!("spacecraftReconfig.vehicle_config_in_msg is not connected");
        }
        self.burns = ReconfigBurnArrayInfoMsg::default();
        self.prev_call_time_nanos = 0;
        self.t_current_s = 0.0;
        self.att_ref_in_is_linked = self.att_ref_in_msg.is_connected();
    }

    fn advance_timer(&mut self, call_time_nanos: u64) {
        // Upstream's zero-time sentinel: a previous call time of 0 means "no
        // update yet", so an update at t = 0 also leaves the next update's
        // elapsed time at zero.
        if self.prev_call_time_nanos == 0 {
            self.prev_call_time_nanos = call_time_nanos;
        }
        self.t_current_s += diff_nanos_to_seconds(call_time_nanos, self.prev_call_time_nanos);
        self.prev_call_time_nanos = call_time_nanos;
    }

    /// Upstream `UpdateManeuver`: plans the burns at the first update after a
    /// reset, then steers towards the next pending burn and starts it when its
    /// time has come. Resets the module after one chief orbit.
    fn update_maneuver(
        &mut self,
        chief: &NavigationTranslationMsg,
        deputy: &NavigationTranslationMsg,
        att_ref_in: &AttitudeReferenceMsg,
        thrusters: &ThrusterArrayConfigMsg,
        vehicle: &VehicleConfigMsg,
    ) -> ManeuverCommand {
        if self.burns.burn_array[0].status == ReconfigBurnStatus::NotScheduled {
            self.plan(chief, deputy, thrusters, vehicle);
        }

        let num_thrusters = thrusters.num_thrusters;
        let command = match self
            .burns
            .burn_array
            .iter()
            .position(|burn| burn.status == ReconfigBurnStatus::Pending)
        {
            Some(next) => self.approach_burn(next, att_ref_in, num_thrusters),
            None => ManeuverCommand {
                attitude: self.attitude_after_burns(att_ref_in, num_thrusters),
                on_time: None,
            },
        };

        if self.t_current_s > self.reset_period_s {
            self.reset_state();
            // Upstream's reset also clears thrustOnFlag, so a burn that starts
            // at this update is never commanded.
            return ManeuverCommand {
                on_time: None,
                ..command
            };
        }
        command
    }

    /// Plans the burns from the current orbits and sorts them by time.
    fn plan(
        &mut self,
        chief: &NavigationTranslationMsg,
        deputy: &NavigationTranslationMsg,
        thrusters: &ThrusterArrayConfigMsg,
        vehicle: &VehicleConfigMsg,
    ) {
        let mu = self.config.mu_m3ps2;
        let chief_oe = state_to_elements(mu, chief.position_bn_n_m, chief.velocity_bn_n_mps);
        let deputy_oe = state_to_elements(mu, deputy.position_bn_n_m, deputy.velocity_bn_n_mps);
        // powf calls libm's pow like upstream; powi(3) would round differently.
        self.reset_period_s = TAU / (mu / chief_oe.semi_major_axis.powf(3.0)).sqrt();
        let mut burns = plan_burns(&self.config, &chief_oe, &deputy_oe, thrusters, vehicle);
        // Upstream qsort with CompareTime, which calls NaN times equal.
        burns.sort_by(|first, second| {
            first
                .time_s
                .partial_cmp(&second.time_s)
                .unwrap_or(Ordering::Equal)
        });
        self.burns.burn_array = burns;
    }

    /// Attitude and thruster command while burn `next` is the first pending
    /// one: keep the attitude of an earlier burn that is still running, else
    /// follow the reference while there is time to slew, else point for burn
    /// `next`. The burn starts half its duration before its impulse time.
    fn approach_burn(
        &mut self,
        next: usize,
        att_ref_in: &AttitudeReferenceMsg,
        num_thrusters: usize,
    ) -> ManeuverCommand {
        let t_current_s = self.t_current_s;
        let burns = &mut self.burns.burn_array;
        let t_left_s = burns[next].time_s - t_current_s;
        let running = burns[..next].iter().rev().find(|burn| {
            burn.status == ReconfigBurnStatus::Burned
                && t_current_s < burn_end_s(burn, num_thrusters)
        });
        let attitude = match running {
            Some(burn) => burn_attitude(burn),
            None if t_left_s > self.config.att_control_time_s && self.att_ref_in_is_linked => {
                att_ref_in.clone()
            }
            None => burn_attitude(&burns[next]),
        };

        let starts = truncated_half_on_time_s(burns[next].thrust_on_time_s, num_thrusters)
            .is_some_and(|half_on_time_s| t_left_s < half_on_time_s);
        let on_time = starts.then(|| {
            burns[next].status = ReconfigBurnStatus::Burned;
            on_time_command(burns[next].thrust_on_time_s, num_thrusters)
        });
        ManeuverCommand { attitude, on_time }
    }

    /// Attitude once no burn is pending: hold the last burn's attitude until
    /// that burn has finished, then follow the reference (zero when the
    /// reference input is not connected).
    fn attitude_after_burns(
        &self,
        att_ref_in: &AttitudeReferenceMsg,
        num_thrusters: usize,
    ) -> AttitudeReferenceMsg {
        let last_burned = self
            .burns
            .burn_array
            .iter()
            .rfind(|burn| burn.status == ReconfigBurnStatus::Burned);
        match last_burned {
            // Strictly after the end, whereas approach_burn already releases
            // the attitude at the end time itself (upstream asymmetry).
            Some(burn)
                if self.t_current_s > burn_end_s(burn, num_thrusters)
                    && self.att_ref_in_is_linked =>
            {
                att_ref_in.clone()
            }
            Some(burn) => burn_attitude(burn),
            None => att_ref_in.clone(),
        }
    }
}

impl Module for SpacecraftReconfig {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.burn_array_info_out_msg
            .write(ReconfigBurnArrayInfoMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let chief = self.chief_trans_in_msg.read();
        let deputy = self.deputy_trans_in_msg.read();
        let thrusters = self.thrust_config_in_msg.read();
        let vehicle = self.vehicle_config_in_msg.read();
        let att_ref_in = if self.att_ref_in_is_linked {
            self.att_ref_in_msg.read()
        } else {
            AttitudeReferenceMsg::default()
        };

        self.advance_timer(context.current_sim_nanos);
        let command = self.update_maneuver(&chief, &deputy, &att_ref_in, &thrusters, &vehicle);

        self.att_ref_out_msg.write(command.attitude);
        self.burn_array_info_out_msg.write(self.burns.clone());
        if let Some(on_time) = command.on_time {
            self.on_time_out_msg.write(on_time);
        }
    }
}

/// Reference attitude of a burn: its `sigma_RN` with zero rates.
fn burn_attitude(burn: &ReconfigBurnInfoMsg) -> AttitudeReferenceMsg {
    AttitudeReferenceMsg {
        sigma_rn: burn.sigma_rn,
        ..Default::default()
    }
}

/// End time \[s\] of a burn centred on its impulse time:
/// `t + thrustOnTime / (2 numThrusters)`, in floating point.
fn burn_end_s(burn: &ReconfigBurnInfoMsg, num_thrusters: usize) -> f64 {
    burn.time_s + burn.thrust_on_time_s / (2.0 * num_thrusters as f64)
}

/// Upstream's start threshold `(int)thrustOnTime / (2 * numThrusters)`, which
/// is C `int` arithmetic: the half duration is truncated to whole seconds
/// before the comparison (quirk kept). `None` without thrusters, where
/// upstream divides by zero; no burn then starts.
fn truncated_half_on_time_s(on_time_s: f64, num_thrusters: usize) -> Option<f64> {
    let divisor = i32::try_from(num_thrusters).ok()?.checked_mul(2)?;
    // Saturating cast; upstream's out-of-range conversion is undefined.
    (on_time_s as i32).checked_div(divisor).map(f64::from)
}

/// Every thruster fires for an equal share of the burn duration.
fn on_time_command(on_time_s: f64, num_thrusters: usize) -> ThrusterArrayOnTimeCommandMsg {
    let mut command = ThrusterArrayOnTimeCommandMsg::default();
    let share_s = on_time_s / num_thrusters as f64;
    command
        .on_time_request_s
        .iter_mut()
        .take(num_thrusters)
        .for_each(|request| *request = share_s);
    command
}

/// Burn slots of [`plan_burns`] before sorting, in upstream's order.
const PERIGEE: usize = 0;
const APOGEE: usize = 1;
const NORMAL: usize = 2;

/// Mean anomaly in `[0, 2π]` of the true anomaly `f` (upstream `f2E`, `E2M`,
/// `AdjustRange(0, 2π)`).
fn mean_anomaly_rad(true_anomaly_rad: f64, e: f64) -> f64 {
    let eccentric_anomaly = true_to_eccentric_anomaly(true_anomaly_rad, e);
    adjust_range(0.0, TAU, eccentric_to_mean_anomaly(eccentric_anomaly, e))
}

/// Time \[s\] for the chief to move from mean anomaly `now` to `burn`, a full
/// orbit later if `burn` has already passed.
fn time_until_s(burn_rad: f64, now_rad: f64, mean_motion_radps: f64) -> f64 {
    if burn_rad > now_rad {
        (burn_rad - now_rad) / mean_motion_radps
    } else {
        (TAU + burn_rad - now_rad) / mean_motion_radps
    }
}

/// Upstream `ScheduleDV`: the perigee, apogee and out-of-plane burns, in that
/// order and not yet sorted.
fn plan_burns(
    config: &SpacecraftReconfigConfig,
    chief: &ClassicElements,
    deputy: &ClassicElements,
    thrusters: &ThrusterArrayConfigMsg,
    vehicle: &VehicleConfigMsg,
) -> [ReconfigBurnInfoMsg; MAX_BURN_CNT] {
    let target = &config.target_classic_oed;
    let mu = config.mu_m3ps2;
    let att_control_time_s = config.att_control_time_s;

    // Element errors still to be corrected; the target is removed after the
    // angle differences are wrapped.
    let a = chief.semi_major_axis;
    let e = chief.eccentricity;
    let i = chief.inclination_rad;
    let omega = chief.arg_periapsis_rad;
    let da = deputy.semi_major_axis - a - target[0];
    let de = deputy.eccentricity - e - target[1];
    let di = adjust_range(-PI, PI, deputy.inclination_rad - i) - target[2];
    let d_raan = adjust_range(-PI, PI, deputy.raan_rad - chief.raan_rad) - target[3];
    let d_argp = adjust_range(-PI, PI, deputy.arg_periapsis_rad - omega) - target[4];
    let m_chief = mean_anomaly_rad(chief.true_anomaly_rad, e);
    let m_deputy = mean_anomaly_rad(deputy.true_anomaly_rad, deputy.eccentricity);
    let mut dm = adjust_range(-PI, PI, m_deputy - m_chief) - target[5];

    let n = (mu / (a * a * a)).sqrt();
    let eta = (1.0 - e * e).sqrt();
    let p = a * (1.0 - e * e);
    let h = n * a * a * eta;
    let rp = a * (1.0 - e);
    let ra = a * (1.0 + e);

    // Burn times: perigee (f = 0), apogee (f = π), and the argument of latitude
    // where a +z (orbit-normal) burn corrects δi and δΩ together.
    let mut times_s = [0.0; MAX_BURN_CNT];
    times_s[PERIGEE] = time_until_s(mean_anomaly_rad(0.0, e), m_chief, n);
    times_s[APOGEE] = time_until_s(mean_anomaly_rad(PI, e), m_chief, n);
    let mut theta_normal = ((-d_raan) * i.sin()).atan2(-di);
    if (-di) * theta_normal.cos() < 0.0 && (-d_raan) * i.sin() * theta_normal.sin() < 0.0 {
        theta_normal += PI;
    }
    let f_normal = adjust_range(0.0, TAU, theta_normal) - omega;
    times_s[NORMAL] = time_until_s(mean_anomaly_rad(f_normal, e), m_chief, n);

    // Burn magnitudes [m/s]; the along-track burns are needed first, to account
    // for the δM drift until the in-plane corrections are complete.
    let dvtp = n * a * eta / 4.0 * ((-da) / a + (-de) / (1.0 + e));
    let dvta = n * a * eta / 4.0 * ((-da) / a - (-de) / (1.0 - e));
    if times_s[PERIGEE] < times_s[APOGEE] {
        dm = dm
            - 1.5 * n / a * da * times_s[PERIGEE]
            - 1.5 * n / a
                * (da + 2.0 * a * a / h * p / rp * dvtp)
                * (times_s[APOGEE] - times_s[PERIGEE]);
    } else {
        dm = dm
            - 1.5 * n / a * da * times_s[APOGEE]
            - 1.5 * n / a
                * (da + 2.0 * a * a / h * p / ra * dvta)
                * (times_s[PERIGEE] - times_s[APOGEE]);
    }
    let apse_term = (-d_argp) + (-d_raan) * i.cos();
    let dvrp = -n * a / 4.0 * ((1.0 + e).powi(2) / eta * apse_term + (-dm));
    let dvra = -n * a / 4.0 * ((1.0 - e).powi(2) / eta * apse_term + (-dm));
    let r_normal = p / (1.0 + e * f_normal.cos());
    let dvn = h / r_normal * ((-di).powi(2) + ((-d_raan) * i.sin()).powi(2)).sqrt();

    // A normal burn within att_control_time_s of an in-plane burn is merged
    // into it. Both checks are independent for the apogee attitude below.
    let merges_with = |slot: usize| (times_s[NORMAL] - times_s[slot]).abs() < att_control_time_s;
    let normal_into_perigee = merges_with(PERIGEE);
    let normal_into_apogee = merges_with(APOGEE);

    let first_thruster = &thrusters.thrusters[0];
    let duration_s = |dv: f64| dv * vehicle.mass_kg / first_thruster.max_thrust_n;
    let mut thrust_on_times_s = [0.0; MAX_BURN_CNT];
    if normal_into_perigee {
        thrust_on_times_s[PERIGEE] = duration_s((dvrp * dvrp + dvtp * dvtp + dvn * dvn).sqrt());
        thrust_on_times_s[APOGEE] = duration_s((dvra * dvra + dvta * dvta).sqrt());
    } else if normal_into_apogee {
        thrust_on_times_s[PERIGEE] = duration_s((dvrp * dvrp + dvtp * dvtp).sqrt());
        thrust_on_times_s[APOGEE] = duration_s((dvra * dvra + dvta * dvta + dvn * dvn).sqrt());
    } else {
        thrust_on_times_s[PERIGEE] = duration_s((dvrp * dvrp + dvtp * dvtp).sqrt());
        thrust_on_times_s[APOGEE] = duration_s((dvra * dvra + dvta * dvta).sqrt());
        thrust_on_times_s[NORMAL] = duration_s(dvn);
    }
    let normal_merged = normal_into_perigee || normal_into_apogee;

    // Target attitudes: [TN] = [TR][RN], where R has the burn direction on +z.
    let dcm_tr = thruster_alignment_dcm(first_thruster.t_hat_thrust_b);
    let deputy_state_at = |burn_time_s: f64| {
        // Upstream advances the deputy with the chief's mean motion.
        let mean_anomaly = m_deputy + burn_time_s * n;
        let eccentric_anomaly = mean_to_eccentric_anomaly(mean_anomaly, deputy.eccentricity);
        let elements = ClassicElements {
            true_anomaly_rad: eccentric_to_true_anomaly(eccentric_anomaly, deputy.eccentricity),
            ..*deputy
        };
        elements_to_state(mu, &elements)
    };
    let (r_perigee, v_perigee) = deputy_state_at(times_s[PERIGEE]);
    let dcm_tn_perigee = dcm_tr
        * in_plane_burn_dcm(
            r_perigee,
            v_perigee,
            dvrp,
            dvtp,
            normal_into_perigee.then_some(dvn),
        );
    let (r_apogee, v_apogee) = deputy_state_at(times_s[APOGEE]);
    let dcm_tn_apogee = dcm_tr
        * in_plane_burn_dcm(
            r_apogee,
            v_apogee,
            dvra,
            dvta,
            normal_into_apogee.then_some(dvn),
        );
    let (r_normal, v_normal) = deputy_state_at(times_s[NORMAL]);
    let dcm_tn_normal = if normal_into_perigee {
        dcm_tn_perigee
    } else if normal_into_apogee {
        dcm_tn_apogee
    } else {
        // The Hill frame's +z is the orbit normal, the burn direction.
        dcm_tr * hill_frame(r_normal, v_normal)
    };
    let dcms_tn = [dcm_tn_perigee, dcm_tn_apogee, dcm_tn_normal];

    std::array::from_fn(|slot| {
        let merged_away = slot == NORMAL && normal_merged;
        let too_short = thrust_on_times_s[slot] / (thrusters.num_thrusters as f64) < 1.0;
        ReconfigBurnInfoMsg {
            status: if merged_away || too_short {
                ReconfigBurnStatus::Skipped
            } else {
                ReconfigBurnStatus::Pending
            },
            time_s: times_s[slot],
            thrust_on_time_s: thrust_on_times_s[slot],
            sigma_rn: dcm_to_mrp(dcms_tn[slot]),
        }
    })
}

/// `[TR]`: rotates the burn frame R, whose +z axis is the planned thrust
/// direction, so that +z becomes the first thruster's direction.
fn thruster_alignment_dcm(thrust_direction_b: Vector3<f64>) -> Matrix3<f64> {
    let thruster_dir = normalize_or_zero(thrust_direction_b);
    let ez = Vector3::z();
    let axis = if thruster_dir.x == 0.0 && thruster_dir.y == 0.0 {
        // Thruster along ±z: any axis in the xy plane will do.
        Vector3::x()
    } else {
        thruster_dir.cross(&ez)
    };
    let axis = normalize_or_zero(axis);
    let half_angle = safe_acos(thruster_dir.dot(&ez)) / 2.0;
    let (sin_half, cos_half) = (half_angle.sin(), half_angle.cos());
    ep_to_dcm(Vector4::new(
        cos_half,
        axis.x * sin_half,
        axis.y * sin_half,
        axis.z * sin_half,
    ))
}

/// `[RN]` of an in-plane burn: +z along the radial plus along-track burn
/// (plus the merged normal burn, if any), +y along the deputy's orbit normal or,
/// when a normal burn is merged in, along `thrust x normal burn`.
fn in_plane_burn_dcm(
    r_n: Vector3<f64>,
    v_n: Vector3<f64>,
    dv_radial: f64,
    dv_along_track: f64,
    dv_normal: Option<f64>,
) -> Matrix3<f64> {
    let h_n = r_n.cross(&v_n);
    let along_track_n = h_n.cross(&r_n);
    let mut thrust_n =
        normalize_or_zero(r_n) * dv_radial + normalize_or_zero(along_track_n) * dv_along_track;
    let mut y_axis_n = h_n;
    if let Some(dv_normal) = dv_normal {
        let normal_burn_n = normalize_or_zero(h_n) * dv_normal;
        thrust_n += normal_burn_n;
        y_axis_n = thrust_n.cross(&normal_burn_n);
    }
    let z_n = normalize_or_zero(thrust_n);
    let y_n = normalize_or_zero(y_axis_n);
    let x_n = y_n.cross(&z_n);
    Matrix3::from_rows(&[x_n.transpose(), y_n.transpose(), z_n.transpose()])
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Vector3, Vector6};

    use super::super::mean_oe_feedback::test_orbits::{MU_EARTH_M3PS2, chief, deputy, navigation};
    use super::{SpacecraftReconfig, SpacecraftReconfigConfig, truncated_half_on_time_s};
    use crate::messages::{
        AttitudeReferenceMsg, MAX_EFF_COUNT, NavigationTranslationMsg, Output,
        ReconfigBurnArrayInfoMsg, ReconfigBurnStatus, ThrusterArrayConfigMsg,
        ThrusterArrayOnTimeCommandMsg, ThrusterConfigMsg, VehicleConfigMsg,
    };
    use crate::utilities::linear_algebra::normalize_or_zero;
    use crate::utilities::orbital_motion::ClassicElements;
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    /// Step of the reconfiguration scenarios below, 10 s.
    const STEP_NANOS: u64 = 10_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Producers of every module input.
    struct Inputs {
        chief: Output<NavigationTranslationMsg>,
        deputy: Output<NavigationTranslationMsg>,
        thrusters: Output<ThrusterArrayConfigMsg>,
        vehicle: Output<VehicleConfigMsg>,
        att_ref: Output<AttitudeReferenceMsg>,
    }

    impl Inputs {
        /// Builds the module with every input connected, the reference
        /// attitude only if `with_reference`, and runs its initialization.
        fn module(
            &self,
            config: SpacecraftReconfigConfig,
            with_reference: bool,
        ) -> SpacecraftReconfig {
            let mut module = SpacecraftReconfig::new(config);
            self.chief.connect_to(&mut module.chief_trans_in_msg);
            self.deputy.connect_to(&mut module.deputy_trans_in_msg);
            self.thrusters.connect_to(&mut module.thrust_config_in_msg);
            self.vehicle.connect_to(&mut module.vehicle_config_in_msg);
            if with_reference {
                self.att_ref.connect_to(&mut module.att_ref_in_msg);
            }
            module.init();
            module
        }
    }

    fn thrusters(
        direction_b: Vector3<f64>,
        max_thrust_n: f64,
        count: usize,
    ) -> ThrusterArrayConfigMsg {
        let thruster = ThrusterConfigMsg {
            t_hat_thrust_b: direction_b,
            max_thrust_n,
            ..Default::default()
        };
        ThrusterArrayConfigMsg::from_active(&vec![thruster; count])
    }

    /// The inputs of `test_spacecraftReconfig.py`.
    fn upstream_inputs() -> Inputs {
        Inputs {
            chief: Output::new(chief()),
            deputy: Output::new(deputy()),
            // fswSetupThrusters.create([0, 0, 0], [1, 1, 1], 22.6), twice.
            thrusters: Output::new(thrusters(Vector3::new(1.0, 1.0, 1.0), 22.6, 2)),
            vehicle: Output::new(VehicleConfigMsg {
                mass_kg: 500.0,
                ..Default::default()
            }),
            att_ref: Output::new(AttitudeReferenceMsg {
                sigma_rn: Vector3::new(1.0, 0.0, 0.0),
                ..Default::default()
            }),
        }
    }

    /// A deputy a few kilometres from a 10 000 km chief, with two 1 N thrusters
    /// along body +x on a 100 kg deputy, and a reference attitude with nonzero
    /// rates. `error_scale` scales the deputy's element offsets.
    fn reconfiguration_inputs(error_scale: f64) -> Inputs {
        let chief = ClassicElements {
            semi_major_axis: 10000e3,
            eccentricity: 0.2,
            inclination_rad: 0.6,
            raan_rad: 0.8,
            arg_periapsis_rad: 1.0,
            true_anomaly_rad: 0.3,
            ..Default::default()
        };
        let deputy = ClassicElements {
            semi_major_axis: 10000e3 + 2000.0 * error_scale,
            eccentricity: 0.2 + 2e-4 * error_scale,
            inclination_rad: 0.6 + 3e-4 * error_scale,
            raan_rad: 0.8 - 2e-4 * error_scale,
            arg_periapsis_rad: 1.0 + 1e-4 * error_scale,
            true_anomaly_rad: 0.3 - 1e-4 * error_scale,
            ..Default::default()
        };
        Inputs {
            chief: Output::new(navigation(&chief)),
            deputy: Output::new(navigation(&deputy)),
            thrusters: Output::new(thrusters(Vector3::x(), 1.0, 2)),
            vehicle: Output::new(VehicleConfigMsg {
                mass_kg: 100.0,
                ..Default::default()
            }),
            att_ref: Output::new(reference()),
        }
    }

    fn reference() -> AttitudeReferenceMsg {
        AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, -0.2, 0.3),
            omega_rn_n_radps: Vector3::new(0.01, 0.02, -0.03),
            domega_rn_n_radps2: Vector3::new(1e-4, -2e-4, 3e-4),
        }
    }

    fn reconfiguration_config(att_control_time_s: f64) -> SpacecraftReconfigConfig {
        SpacecraftReconfigConfig {
            mu_m3ps2: MU_EARTH_M3PS2,
            att_control_time_s,
            ..Default::default()
        }
    }

    /// One expected burn: `(status, t [s], thrustOnTime [s], sigma_RN)`.
    type BurnTruth = (ReconfigBurnStatus, f64, f64, [f64; 3]);

    // Burn plan of the reconfiguration scenario, from upstream's C module
    // compiled with gcc and run on the same inputs.
    const NORMAL_BURN_TIME_S: f64 = 1_874.534_676_819_596_1;
    const APOGEE_BURN_TIME_S: f64 = 4_664.069_538_476_145;
    const PERIGEE_BURN_TIME_S: f64 = 9_640.076_599_924_454;
    const NORMAL_ON_TIME_S: f64 = 197.909_528_537_425_1;
    const APOGEE_ON_TIME_S: f64 = 334.439_443_494_729_3;
    const PERIGEE_ON_TIME_S: f64 = 347.394_552_537_897_7;
    const NORMAL_SIGMA: [f64; 3] = [
        -0.585_113_493_298_517_1,
        0.020_759_773_793_578_608,
        -0.411_897_184_117_731_1,
    ];
    const APOGEE_SIGMA: [f64; 3] = [
        0.520_848_089_267_587_5,
        0.415_271_167_382_305,
        0.445_630_456_920_770_8,
    ];
    const PERIGEE_SIGMA: [f64; 3] = [
        0.393_427_960_456_991_4,
        -0.111_321_974_356_969_9,
        -0.381_183_769_595_779_2,
    ];
    /// Apogee attitude once the normal burn is merged into it.
    const APOGEE_WITH_NORMAL_SIGMA: [f64; 3] = [
        -0.388_052_829_509_182_46,
        -0.430_659_343_236_931_45,
        -0.124_532_376_506_598_28,
    ];
    /// Perigee attitude once the normal burn is merged into it.
    const PERIGEE_WITH_NORMAL_SIGMA: [f64; 3] = [
        0.776_931_776_367_817,
        -0.485_817_846_679_281_2,
        -0.071_678_549_996_027_55,
    ];

    fn assert_close(actual: f64, expected: f64, tolerance: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{what}: {actual:.17e}, expected {expected:.17e}"
        );
    }

    fn assert_plan(plan: &ReconfigBurnArrayInfoMsg, truth: &[BurnTruth; 3]) {
        for (index, (burn, (status, time_s, on_time_s, sigma))) in
            plan.burn_array.iter().zip(truth).enumerate()
        {
            assert_eq!(burn.status, *status, "burn {index} status");
            assert_close(burn.time_s, *time_s, 1e-9, &format!("burn {index} time"));
            assert_close(
                burn.thrust_on_time_s,
                *on_time_s,
                1e-9,
                &format!("burn {index} on-time"),
            );
            for (actual, expected) in burn.sigma_rn.iter().zip(sigma) {
                assert_close(*actual, *expected, 1e-12, &format!("burn {index} sigma"));
            }
        }
    }

    fn first_plan(inputs: &Inputs, att_control_time_s: f64) -> ReconfigBurnArrayInfoMsg {
        let mut module = inputs.module(reconfiguration_config(att_control_time_s), true);
        module.update(&context(0));
        module.burn_array_info_out_msg.read()
    }

    /// `test_spacecraftReconfig`, both `useRefAttitude` parametrizations, with
    /// upstream's hard-coded truth and `accuracy = 1e-9`. Upstream runs a
    /// 0.1 s task to t = 0.1 s and checks the first recorded `sigma_RN`
    /// (absolute) and the first logged `resetPeriod` (relative).
    #[test]
    fn matches_upstream_spacecraft_reconfig_unit_test() {
        let accuracy = 1e-9;
        // Upstream writes 28148.5466910579925752244889736; this is the same f64.
        let true_reset_period_s = 28_148.546_691_057_993;
        let inputs = upstream_inputs();
        for (use_ref_attitude, true_sigma) in [
            (true, [1.0, 0.0, 0.0]),
            (
                false,
                [
                    0.385_326_972_092_485_95,
                    -0.701_634_909_083_973_2,
                    -0.402_619_457_244_006_9,
                ],
            ),
        ] {
            let mut module = inputs.module(
                SpacecraftReconfigConfig {
                    mu_m3ps2: MU_EARTH_M3PS2,
                    att_control_time_s: 400.0,
                    target_classic_oed: Vector6::new(0.0, 0.0, 0.0, 0.0001, 0.0002, 0.0003),
                    ..Default::default()
                },
                use_ref_attitude,
            );
            module.update(&context(0));
            let sigma = module.att_ref_out_msg.read().sigma_rn;
            let reset_period_s = module.reset_period_s();
            module.update(&context(100_000_000));

            for axis in 0..3 {
                assert_close(
                    sigma[axis],
                    true_sigma[axis],
                    accuracy,
                    &format!("useRefAttitude={use_ref_attitude} sigma_RN"),
                );
            }
            assert!(
                ((true_reset_period_s - reset_period_s) / true_reset_period_s).abs() <= accuracy,
                "resetPeriod {reset_period_s}"
            );
        }
    }

    #[test]
    fn plans_the_three_burns_like_the_upstream_c_module() {
        let inputs = reconfiguration_inputs(1.0);
        let mut module = inputs.module(reconfiguration_config(300.0), true);
        module.update(&context(0));

        // One chief orbit, 2 pi sqrt(a^3 / mu).
        assert_close(
            module.reset_period_s(),
            9_952.014_122_896_619,
            1e-9,
            "resetPeriod",
        );
        // Sorted by time: the out-of-plane burn comes first here, then apogee,
        // then perigee.
        assert_plan(
            &module.burn_array_info_out_msg.read(),
            &[
                (
                    ReconfigBurnStatus::Pending,
                    NORMAL_BURN_TIME_S,
                    NORMAL_ON_TIME_S,
                    NORMAL_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    APOGEE_BURN_TIME_S,
                    APOGEE_ON_TIME_S,
                    APOGEE_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    PERIGEE_BURN_TIME_S,
                    PERIGEE_ON_TIME_S,
                    PERIGEE_SIGMA,
                ),
            ],
        );
    }

    /// What the module points at, classified per update.
    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Pointing {
        Reference,
        Burn(usize),
    }

    /// What one run of the module published.
    struct Sequence {
        /// The burn plan published at the first update.
        plan: ReconfigBurnArrayInfoMsg,
        /// Steps where the published attitude changed, and what it points at.
        pointing_changes: Vec<(usize, Pointing)>,
        /// Steps where a burn started, with `tCurrent` and the on-time command.
        burn_starts: Vec<(usize, f64, ThrusterArrayOnTimeCommandMsg)>,
        /// Steps where the module reset itself and published the cleared plan.
        self_resets: Vec<usize>,
    }

    /// Updates the module every `STEP_NANOS` from `first_nanos` on, `steps`
    /// times, with `reference()` as the reference attitude, and records what
    /// it publishes. The inputs are constant, so every plan after a self-reset
    /// must repeat the first one.
    fn run_sequence(module: &mut SpacecraftReconfig, first_nanos: u64, steps: usize) -> Sequence {
        let marker = ThrusterArrayOnTimeCommandMsg::from_active(&[-1.0]);
        let mut sequence = Sequence {
            plan: ReconfigBurnArrayInfoMsg::default(),
            pointing_changes: Vec::new(),
            burn_starts: Vec::new(),
            self_resets: Vec::new(),
        };
        for step in 0..steps {
            // Upstream writes the on-time message only when a burn starts.
            module.on_time_out_msg.write(marker.clone());
            module.update(&context(first_nanos + step as u64 * STEP_NANOS));

            let published_plan = module.burn_array_info_out_msg.read();
            if step == 0 {
                sequence.plan = published_plan;
            } else if published_plan == ReconfigBurnArrayInfoMsg::default() {
                assert_eq!(module.t_current_s, 0.0, "step {step}");
                sequence.self_resets.push(step);
            } else if sequence.self_resets.last() == Some(&(step - 1)) {
                assert_eq!(
                    published_plan, sequence.plan,
                    "step {step}: plan after reset"
                );
            }

            let attitude = module.att_ref_out_msg.read();
            let pointing = if attitude.sigma_rn == reference().sigma_rn {
                assert_eq!(attitude.omega_rn_n_radps, reference().omega_rn_n_radps);
                assert_eq!(attitude.domega_rn_n_radps2, reference().domega_rn_n_radps2);
                Pointing::Reference
            } else {
                let burn = sequence
                    .plan
                    .burn_array
                    .iter()
                    .position(|burn| burn.sigma_rn == attitude.sigma_rn)
                    .unwrap_or_else(|| panic!("step {step}: unexpected attitude {attitude:?}"));
                // Burn attitudes carry no rates.
                assert_eq!(attitude.omega_rn_n_radps, Vector3::zeros());
                assert_eq!(attitude.domega_rn_n_radps2, Vector3::zeros());
                Pointing::Burn(burn)
            };
            if sequence.pointing_changes.last().map(|(_, last)| *last) != Some(pointing) {
                sequence.pointing_changes.push((step, pointing));
            }

            let on_time = module.on_time_out_msg.read();
            if on_time != marker {
                sequence
                    .burn_starts
                    .push((step, module.t_current_s, on_time));
            }
        }
        sequence
    }

    /// Checks the recorded burn starts: `(step, tCurrent [s], on-time per
    /// thruster [s])`, shared equally by the first `num_thrusters` thrusters.
    fn assert_burn_starts(
        sequence: &Sequence,
        num_thrusters: usize,
        expected_starts: &[(usize, f64, f64)],
    ) {
        assert_eq!(sequence.burn_starts.len(), expected_starts.len());
        for ((step, t_current_s, on_time), (true_step, true_t_current_s, true_on_time_s)) in
            sequence.burn_starts.iter().zip(expected_starts)
        {
            assert_eq!(step, true_step);
            assert_close(*t_current_s, *true_t_current_s, 1e-9, "tCurrent");
            for (index, request) in on_time.on_time_request_s.iter().enumerate() {
                let expected = if index < num_thrusters {
                    *true_on_time_s
                } else {
                    0.0
                };
                assert_close(*request, expected, 1e-9, &format!("OnTimeRequest[{index}]"));
            }
        }
    }

    /// Steps through one chief orbit and the module's self-reset, checking the
    /// attitude switches and burn starts against the upstream C module.
    #[test]
    fn sequences_attitude_and_thrusters_like_the_upstream_c_module() {
        let inputs = reconfiguration_inputs(1.0);
        let mut module = inputs.module(reconfiguration_config(300.0), true);
        let sequence = run_sequence(&mut module, 0, 1000);

        // One orbit has passed: the module resets itself, publishes the
        // cleared plan, and plans the same burns again at the next update.
        assert_eq!(sequence.self_resets, [997]);
        // Each burn attitude is taken att_control_time_s before the impulse
        // and held until the burn, centred on the impulse, has ended.
        assert_eq!(
            sequence.pointing_changes,
            [
                (0, Pointing::Reference),
                (159, Pointing::Burn(0)),
                (194, Pointing::Reference),
                (438, Pointing::Burn(1)),
                (476, Pointing::Reference),
                (936, Pointing::Burn(2)),
                (974, Pointing::Reference),
            ]
        );
        // The burn starts once the time left is below half its duration,
        // truncated to whole seconds; the timer lags by one step because the
        // run starts at t = 0. Both thrusters share the burn equally.
        assert_burn_starts(
            &sequence,
            2,
            &[
                (184, 1830.0, 98.954_764_268_712_55),
                (460, 4590.0, 167.219_721_747_364_64),
                (957, 9560.0, 173.697_276_268_948_85),
            ],
        );
    }

    /// A target element difference (with the semi-major-axis entry in metres,
    /// as upstream uses it), three thrusters along body -z and a run that
    /// starts after t = 0. Truth from the upstream C module compiled with gcc
    /// and run on the same inputs.
    #[test]
    fn plans_and_sequences_a_target_difference_like_the_upstream_c_module() {
        let mut inputs = reconfiguration_inputs(-1.5);
        inputs.thrusters = Output::new(thrusters(-Vector3::z(), 0.5, 3));
        inputs.vehicle = Output::new(VehicleConfigMsg {
            mass_kg: 150.0,
            ..Default::default()
        });
        let mut module = inputs.module(
            SpacecraftReconfigConfig {
                target_classic_oed: Vector6::new(150.0, 1e-4, -2e-4, 3e-4, -1e-4, 2e-4),
                ..reconfiguration_config(600.0)
            },
            true,
        );
        let sequence = run_sequence(&mut module, STEP_NANOS, 1000);

        assert_close(
            module.reset_period_s(),
            9_952.014_122_896_619,
            1e-9,
            "resetPeriod",
        );
        // Sorted: apogee, out-of-plane, perigee.
        assert_plan(
            &sequence.plan,
            &[
                (
                    ReconfigBurnStatus::Pending,
                    4_664.069_538_476_145,
                    1_529.728_399_229_136_8,
                    [
                        -0.281_376_989_232_214_7,
                        0.042_706_685_873_610_11,
                        0.106_599_376_743_311_4,
                    ],
                ),
                (
                    ReconfigBurnStatus::Pending,
                    8_546.666_782_015_194,
                    535.498_168_538_097_8,
                    [
                        -0.691_654_348_918_575_4,
                        -0.292_838_244_178_258_9,
                        0.090_361_661_959_907_05,
                    ],
                ),
                (
                    ReconfigBurnStatus::Pending,
                    9_640.076_599_924_454,
                    1_537.693_608_401_462_1,
                    [
                        -0.251_774_826_118_249_14,
                        0.811_912_940_759_886_4,
                        -0.473_454_541_237_613_16,
                    ],
                ),
            ],
        );
        // Starting after t = 0, the timer runs from the first update on.
        assert_eq!(sequence.self_resets, [996]);
        assert_eq!(
            sequence.pointing_changes,
            [
                (0, Pointing::Reference),
                (407, Pointing::Burn(0)),
                (492, Pointing::Reference),
                (795, Pointing::Burn(1)),
                (864, Pointing::Reference),
                (905, Pointing::Burn(2)),
                (990, Pointing::Reference),
            ]
        );
        assert_burn_starts(
            &sequence,
            3,
            &[
                (442, 4420.0, 509.909_466_409_712_25),
                (846, 8460.0, 178.499_389_512_699_28),
                (939, 9390.0, 512.564_536_133_820_8),
            ],
        );
    }

    /// The thruster, mapped to inertial axes through each burn attitude,
    /// points along the orbit normal for the out-of-plane burn and inside the
    /// orbit plane for the in-plane burns.
    #[test]
    fn burn_attitudes_point_the_first_thruster_along_the_burn() {
        for direction_b in [
            Vector3::x(),
            Vector3::z(),
            -Vector3::z(),
            Vector3::new(1.0, -2.0, 0.5),
        ] {
            let mut inputs = reconfiguration_inputs(1.0);
            inputs.thrusters = Output::new(thrusters(direction_b, 1.0, 2));
            let deputy = inputs.deputy.read();
            let orbit_normal_n =
                normalize_or_zero(deputy.position_bn_n_m.cross(&deputy.velocity_bn_n_mps));
            let plan = first_plan(&inputs, 300.0);
            for burn in &plan.burn_array {
                let thrust_n =
                    mrp_to_dcm(burn.sigma_rn).transpose() * normalize_or_zero(direction_b);
                let normal_component = thrust_n.dot(&orbit_normal_n);
                if (burn.time_s - NORMAL_BURN_TIME_S).abs() < 1e-6 {
                    assert_close(normal_component, 1.0, 1e-9, "out-of-plane burn direction");
                } else {
                    assert_close(normal_component, 0.0, 1e-9, "in-plane burn direction");
                }
            }
        }
    }

    #[test]
    fn merges_the_out_of_plane_burn_into_the_nearby_apogee_burn() {
        // The normal burn is 2790 s before the apogee burn and 7766 s before
        // the perigee burn.
        let plan = first_plan(&reconfiguration_inputs(1.0), 3000.0);
        assert_plan(
            &plan,
            &[
                (
                    ReconfigBurnStatus::Skipped,
                    NORMAL_BURN_TIME_S,
                    0.0,
                    APOGEE_WITH_NORMAL_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    APOGEE_BURN_TIME_S,
                    388.610_245_427_176_15,
                    APOGEE_WITH_NORMAL_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    PERIGEE_BURN_TIME_S,
                    PERIGEE_ON_TIME_S,
                    PERIGEE_SIGMA,
                ),
            ],
        );
        // Durations scale with the delta-v, so the merged burn is the
        // root-sum-square of the two separate ones.
        assert_close(
            plan.burn_array[1].thrust_on_time_s,
            APOGEE_ON_TIME_S.hypot(NORMAL_ON_TIME_S),
            1e-9,
            "merged duration",
        );
    }

    /// When the normal burn is close to both in-plane burns, upstream merges
    /// its delta-v into the perigee burn's duration, but tilts both the
    /// perigee and the apogee attitudes towards it (quirk kept).
    #[test]
    fn a_double_merge_tilts_the_apogee_attitude_but_not_its_duration() {
        let plan = first_plan(&reconfiguration_inputs(1.0), 8000.0);
        assert_plan(
            &plan,
            &[
                (
                    ReconfigBurnStatus::Skipped,
                    NORMAL_BURN_TIME_S,
                    0.0,
                    PERIGEE_WITH_NORMAL_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    APOGEE_BURN_TIME_S,
                    APOGEE_ON_TIME_S,
                    APOGEE_WITH_NORMAL_SIGMA,
                ),
                (
                    ReconfigBurnStatus::Pending,
                    PERIGEE_BURN_TIME_S,
                    399.813_902_483_282_9,
                    PERIGEE_WITH_NORMAL_SIGMA,
                ),
            ],
        );
        assert_close(
            plan.burn_array[2].thrust_on_time_s,
            PERIGEE_ON_TIME_S.hypot(NORMAL_ON_TIME_S),
            1e-9,
            "merged duration",
        );
    }

    /// Burns under one second per thruster are skipped, so the module only
    /// ever relays the reference attitude (zero without one) and never fires.
    #[test]
    fn skips_burns_shorter_than_one_second_per_thruster() {
        let inputs = reconfiguration_inputs(1e-3);
        for with_reference in [true, false] {
            let mut module = inputs.module(reconfiguration_config(300.0), with_reference);
            let expected_attitude = if with_reference {
                reference()
            } else {
                AttitudeReferenceMsg::default()
            };
            for step in 0..1000 {
                module.update(&context(step as u64 * STEP_NANOS));
                let plan = module.burn_array_info_out_msg.read();
                if step != 997 {
                    for burn in &plan.burn_array {
                        assert_eq!(burn.status, ReconfigBurnStatus::Skipped, "step {step}");
                        assert!(burn.thrust_on_time_s / 2.0 < 1.0);
                    }
                }
                let attitude = module.att_ref_out_msg.read();
                assert_eq!(attitude.sigma_rn, expected_attitude.sigma_rn);
                assert_eq!(
                    attitude.omega_rn_n_radps,
                    expected_attitude.omega_rn_n_radps
                );
                assert_eq!(
                    attitude.domega_rn_n_radps2,
                    expected_attitude.domega_rn_n_radps2
                );
            }
            assert_eq!(
                module.on_time_out_msg.read(),
                ThrusterArrayOnTimeCommandMsg::default()
            );
        }
    }

    /// Upstream's zero-time sentinel: an update at t = 0 does not start the
    /// timer, so the update after it also adds no elapsed time.
    #[test]
    fn the_timer_starts_one_update_late_when_run_from_time_zero() {
        let inputs = reconfiguration_inputs(1.0);
        let mut module = inputs.module(reconfiguration_config(300.0), true);
        for (time_nanos, t_current_s) in [(0, 0.0), (STEP_NANOS, 0.0), (2 * STEP_NANOS, 10.0)] {
            module.update(&context(time_nanos));
            assert_eq!(module.t_current_s, t_current_s);
        }

        let mut module = inputs.module(reconfiguration_config(300.0), true);
        for (time_nanos, t_current_s) in [(STEP_NANOS, 0.0), (2 * STEP_NANOS, 10.0)] {
            module.update(&context(time_nanos));
            assert_eq!(module.t_current_s, t_current_s);
        }
        // A reset restarts the timer.
        module.reset(&context(2 * STEP_NANOS));
        module.update(&context(3 * STEP_NANOS));
        assert_eq!(module.t_current_s, 0.0);
    }

    /// Upstream compares the time left with `(int)thrustOnTime / (2 *
    /// numThrusters)` in C integer arithmetic.
    #[test]
    fn the_burn_start_threshold_is_truncated_to_whole_seconds() {
        assert_eq!(
            truncated_half_on_time_s(197.909_528_537_425_1, 2),
            Some(49.0)
        );
        assert_eq!(truncated_half_on_time_s(7.9, 2), Some(1.0));
        assert_eq!(truncated_half_on_time_s(3.9, 2), Some(0.0));
        assert_eq!(truncated_half_on_time_s(7.9, 1), Some(3.0));
        assert_eq!(truncated_half_on_time_s(-7.9, 2), Some(-1.0));
        // Upstream divides by zero here.
        assert_eq!(truncated_half_on_time_s(7.9, 0), None);
    }

    /// Without thrusters upstream divides by zero; here no burn ever starts.
    #[test]
    fn no_burn_starts_without_thrusters() {
        let mut inputs = reconfiguration_inputs(1.0);
        inputs.thrusters = Output::new(ThrusterArrayConfigMsg::default());
        let mut module = inputs.module(reconfiguration_config(300.0), true);
        for step in 0..1000 {
            module.update(&context(step as u64 * STEP_NANOS));
        }
        assert!(
            module
                .burn_array_info_out_msg
                .read()
                .burn_array
                .iter()
                .all(|burn| burn.status != ReconfigBurnStatus::Burned)
        );
        assert_eq!(
            module.on_time_out_msg.read(),
            ThrusterArrayOnTimeCommandMsg::default()
        );
    }

    #[test]
    fn the_on_time_command_covers_every_configured_thruster() {
        let mut inputs = reconfiguration_inputs(1.0);
        inputs.thrusters = Output::new(thrusters(Vector3::x(), 1.0, MAX_EFF_COUNT));
        let mut module = inputs.module(reconfiguration_config(300.0), true);
        let mut step = 0;
        while module.on_time_out_msg.read() == ThrusterArrayOnTimeCommandMsg::default() {
            module.update(&context(step * STEP_NANOS));
            step += 1;
            assert!(step < 1000, "no burn started");
        }
        let share_s = NORMAL_ON_TIME_S / MAX_EFF_COUNT as f64;
        for request in module.on_time_out_msg.read().on_time_request_s {
            assert_close(request, share_s, 1e-12, "OnTimeRequest");
        }
    }
}
