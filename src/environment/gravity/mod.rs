use std::cell::RefCell;
use std::fmt;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::{Duration, Epoch};
use nalgebra::{Matrix3, Vector3};

use crate::messages::{Input, PlanetStateMsg};

#[derive(Clone, Debug)]
pub struct PointMassGravityModel {
    pub mu_m3ps2: f64,
}

impl PointMassGravityModel {
    pub fn compute_field(&self, position_body_fixed_m: Vector3<f64>) -> Vector3<f64> {
        let radius = position_body_fixed_m.norm();
        if radius == 0.0 {
            return Vector3::zeros();
        }

        -self.mu_m3ps2 * position_body_fixed_m / radius.powi(3)
    }
}

#[derive(Clone, Debug)]
pub struct SphericalHarmonicsGravityModel {
    pub rad_equator_m: f64,
    pub mu_body_m3ps2: f64,
    pub body_rotation_rate_radps: f64,
    pub max_deg: usize,
    pub c_bar: Vec<Vec<f64>>,
    pub s_bar: Vec<Vec<f64>>,
    orientation_model: Option<AniseOrientationModel>,
    a_bar_seed: Vec<Vec<f64>>,
    n1: Vec<Vec<f64>>,
    n2: Vec<Vec<f64>>,
    n_quot1: Vec<Vec<f64>>,
    n_quot2: Vec<Vec<f64>>,
    scratch: RefCell<SphericalHarmonicsScratch>,
}

#[derive(Clone, Debug)]
struct SphericalHarmonicsScratch {
    a_bar: Vec<Vec<f64>>,
    r_e: Vec<f64>,
    i_m: Vec<f64>,
    rho_l: Vec<f64>,
}

impl SphericalHarmonicsGravityModel {
    pub fn from_file(path: impl AsRef<Path>, max_deg: usize) -> Self {
        let resolved_path = resolve_repo_relative_path(path.as_ref());
        let file = File::open(&resolved_path).unwrap_or_else(|error| {
            panic!(
                "failed to open spherical harmonics gravity file '{}': {error}",
                resolved_path.display()
            )
        });
        let mut lines = BufReader::new(file).lines();
        let header = lines
            .next()
            .expect("gravity file must contain a header line")
            .expect("failed to read gravity file header");
        let header_fields: Vec<_> = header.split(',').map(str::trim).collect();
        assert!(
            header_fields.len() >= 8,
            "gravity file header must have at least 8 comma-separated fields"
        );

        let rad_equator_m = header_fields[0]
            .parse::<f64>()
            .expect("failed to parse gravity reference radius");
        let mu_body_m3ps2 = header_fields[1]
            .parse::<f64>()
            .expect("failed to parse gravity mu");
        let body_rotation_rate_radps = header_fields[2]
            .parse::<f64>()
            .expect("failed to parse gravity rotation rate");
        let max_degree_file = header_fields[3]
            .parse::<usize>()
            .expect("failed to parse gravity max degree");
        let max_order_file = header_fields[4]
            .parse::<usize>()
            .expect("failed to parse gravity max order");
        let coefficients_normalized = header_fields[5]
            .parse::<u8>()
            .expect("failed to parse gravity normalization flag")
            == 1;
        let ref_long = header_fields[6]
            .parse::<f64>()
            .expect("failed to parse gravity reference longitude");
        let ref_lat = header_fields[7]
            .parse::<f64>()
            .expect("failed to parse gravity reference latitude");

        assert!(
            max_degree_file >= max_deg && max_order_file >= max_deg,
            "requested degree {max_deg}, but file only supports degree/order {}",
            max_degree_file.min(max_order_file)
        );
        assert!(
            coefficients_normalized,
            "only normalized spherical harmonics coefficients are supported"
        );
        assert!(
            ref_long == 0.0 && ref_lat == 0.0,
            "reference longitude/latitude must both be zero"
        );

        let mut c_bar = vec![Vec::<f64>::new(); max_deg + 1];
        let mut s_bar = vec![Vec::<f64>::new(); max_deg + 1];
        for degree in 0..=max_deg {
            c_bar[degree] = vec![0.0; degree + 1];
            s_bar[degree] = vec![0.0; degree + 1];
        }

        for line in lines {
            let line = line.expect("failed to read gravity coefficient line");
            if line.trim().is_empty() {
                continue;
            }

            let fields: Vec<_> = line.split(',').map(str::trim).collect();
            if fields.len() < 4 {
                continue;
            }

            let degree = fields[0]
                .parse::<usize>()
                .expect("failed to parse gravity coefficient degree");
            let order = fields[1]
                .parse::<usize>()
                .expect("failed to parse gravity coefficient order");
            if degree > max_deg || order > max_deg || order > degree {
                continue;
            }

            c_bar[degree][order] = fields[2]
                .parse::<f64>()
                .expect("failed to parse normalized C coefficient");
            s_bar[degree][order] = fields[3]
                .parse::<f64>()
                .expect("failed to parse normalized S coefficient");
        }

        let (a_bar_seed, n1, n2, n_quot1, n_quot2) = initialize_pines_parameters(max_deg);

        let scratch_a_bar = a_bar_seed.clone();

        Self {
            rad_equator_m,
            mu_body_m3ps2,
            body_rotation_rate_radps,
            max_deg,
            c_bar,
            s_bar,
            orientation_model: None,
            a_bar_seed,
            n1,
            n2,
            n_quot1,
            n_quot2,
            scratch: RefCell::new(SphericalHarmonicsScratch {
                a_bar: scratch_a_bar,
                r_e: vec![0.0; max_deg + 2],
                i_m: vec![0.0; max_deg + 2],
                rho_l: vec![0.0; max_deg + 2],
            }),
        }
    }

    pub fn with_anise_orientation(
        mut self,
        first_kernel_path: impl AsRef<Path>,
        additional_kernel_paths: &[PathBuf],
        inertial_frame: Frame,
        fixed_frame: Frame,
    ) -> Self {
        self.orientation_model = Some(AniseOrientationModel::new(
            first_kernel_path,
            additional_kernel_paths,
            inertial_frame,
            fixed_frame,
        ));
        self
    }

    pub fn compute_field(
        &self,
        position_body_fixed_m: Vector3<f64>,
        degree: usize,
        include_zero_degree: bool,
    ) -> Vector3<f64> {
        assert!(
            degree <= self.max_deg,
            "requested spherical harmonics degree {degree}, but maximum available is {}",
            self.max_deg
        );

        let x = position_body_fixed_m.x;
        let y = position_body_fixed_m.y;
        let z = position_body_fixed_m.z;
        let radius = position_body_fixed_m.norm();
        if radius == 0.0 {
            return Vector3::zeros();
        }

        let s = x / radius;
        let t = y / radius;
        let u = z / radius;
        let order = degree;

        let mut scratch = self.scratch.borrow_mut();
        scratch.a_bar.clone_from(&self.a_bar_seed);
        scratch.r_e[..=order + 1].fill(0.0);
        scratch.i_m[..=order + 1].fill(0.0);
        scratch.rho_l[..=degree + 1].fill(0.0);

        let SphericalHarmonicsScratch {
            a_bar,
            r_e,
            i_m,
            rho_l,
        } = &mut *scratch;

        for l in 1..=degree + 1 {
            a_bar[l][l - 1] = (((2 * l) as f64 * get_k(l - 1)) / get_k(l)).sqrt() * a_bar[l][l] * u;
        }

        r_e[0] = 1.0;

        for m in 0..=order + 1 {
            for l in m + 2..=degree + 1 {
                a_bar[l][m] = u * self.n1[l][m] * a_bar[l - 1][m] - self.n2[l][m] * a_bar[l - 2][m];
            }

            if m > 0 {
                r_e[m] = s * r_e[m - 1] - t * i_m[m - 1];
                i_m[m] = s * i_m[m - 1] + t * r_e[m - 1];
            }
        }

        let rho = self.rad_equator_m / radius;
        rho_l[0] = self.mu_body_m3ps2 / radius;
        rho_l[1] = rho_l[0] * rho;

        let mut a1 = 0.0;
        let mut a2 = 0.0;
        let mut a3 = 0.0;
        let mut a4 = 0.0;

        if include_zero_degree {
            a4 = -rho_l[1] / self.rad_equator_m;
        }

        for l in 1..=degree {
            rho_l[l + 1] = rho * rho_l[l];

            let mut sum_a1 = 0.0;
            let mut sum_a2 = 0.0;
            let mut sum_a3 = 0.0;
            let mut sum_a4 = 0.0;

            for m in 0..=l {
                let d = self.c_bar[l][m] * r_e[m] + self.s_bar[l][m] * i_m[m];
                let (e, f) = if m == 0 {
                    (0.0, 0.0)
                } else {
                    (
                        self.c_bar[l][m] * r_e[m - 1] + self.s_bar[l][m] * i_m[m - 1],
                        self.s_bar[l][m] * r_e[m - 1] - self.c_bar[l][m] * i_m[m - 1],
                    )
                };

                sum_a1 += m as f64 * a_bar[l][m] * e;
                sum_a2 += m as f64 * a_bar[l][m] * f;
                if m < l {
                    sum_a3 += self.n_quot1[l][m] * a_bar[l][m + 1] * d;
                }
                sum_a4 += self.n_quot2[l][m] * a_bar[l + 1][m + 1] * d;
            }

            a1 += rho_l[l + 1] / self.rad_equator_m * sum_a1;
            a2 += rho_l[l + 1] / self.rad_equator_m * sum_a2;
            a3 += rho_l[l + 1] / self.rad_equator_m * sum_a3;
            a4 -= rho_l[l + 1] / self.rad_equator_m * sum_a4;
        }

        Vector3::new(a1 + s * a4, a2 + t * a4, a3 + u * a4)
    }
}

#[derive(Clone, Debug)]
pub enum GravityModel {
    PointMass(PointMassGravityModel),
    SphericalHarmonics(SphericalHarmonicsGravityModel),
}

impl GravityModel {
    pub fn compute_field(&self, position_body_fixed_m: Vector3<f64>) -> Vector3<f64> {
        match self {
            Self::PointMass(model) => model.compute_field(position_body_fixed_m),
            Self::SphericalHarmonics(model) => {
                model.compute_field(position_body_fixed_m, model.max_deg, true)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct GravBodyData {
    pub planet_name: String,
    pub is_central_body: bool,
    pub gravity_model: GravityModel,
    pub initial_position_m: Vector3<f64>,
    pub initial_velocity_mps: Vector3<f64>,
    pub planet_body_in_msg: Input<PlanetStateMsg>,
}

#[derive(Clone, Debug)]
struct CachedOrientationState {
    inertial_to_fixed: Matrix3<f64>,
    inertial_to_fixed_dot: Matrix3<f64>,
}

#[derive(Clone, Debug)]
struct CachedGravBodyState {
    position_inertial_m: Vector3<f64>,
    velocity_inertial_mps: Vector3<f64>,
    cached_at_sim_nanos: u64,
    orientation: Option<CachedOrientationState>,
}

#[derive(Clone, Debug, Default)]
pub struct GravityTimingStats {
    pub update_cache_calls: u64,
    pub update_cache_total_nanos: u128,
    pub update_cache_state_read_nanos: u128,
    pub update_cache_orientation_current_nanos: u128,
    pub update_cache_orientation_previous_nanos: u128,
    pub compute_gravity_field_calls: u64,
    pub compute_gravity_field_total_nanos: u128,
    pub compute_gravity_position_step_nanos: u128,
    pub compute_gravity_accel_eval_nanos: u128,
}

impl GravBodyData {
    pub fn point_mass(
        planet_name: impl Into<String>,
        mu_m3ps2: f64,
        is_central_body: bool,
        initial_position_m: Vector3<f64>,
        initial_velocity_mps: Vector3<f64>,
    ) -> Self {
        Self {
            planet_name: planet_name.into(),
            is_central_body,
            gravity_model: GravityModel::PointMass(PointMassGravityModel { mu_m3ps2 }),
            initial_position_m,
            initial_velocity_mps,
            planet_body_in_msg: Input::default(),
        }
    }

    pub fn spherical_harmonics_from_file(
        planet_name: impl Into<String>,
        file_path: impl AsRef<Path>,
        max_deg: usize,
        is_central_body: bool,
        initial_position_m: Vector3<f64>,
        initial_velocity_mps: Vector3<f64>,
    ) -> Self {
        Self {
            planet_name: planet_name.into(),
            is_central_body,
            gravity_model: GravityModel::SphericalHarmonics(
                SphericalHarmonicsGravityModel::from_file(file_path, max_deg),
            ),
            initial_position_m,
            initial_velocity_mps,
            planet_body_in_msg: Input::default(),
        }
    }

    pub fn with_anise_orientation(
        mut self,
        first_kernel_path: impl AsRef<Path>,
        additional_kernel_paths: &[PathBuf],
        inertial_frame: Frame,
        fixed_frame: Frame,
    ) -> Self {
        if let GravityModel::SphericalHarmonics(model) = self.gravity_model {
            self.gravity_model = GravityModel::SphericalHarmonics(model.with_anise_orientation(
                first_kernel_path,
                additional_kernel_paths,
                inertial_frame,
                fixed_frame,
            ));
        }

        self
    }

    pub fn compute_gravity_inertial(
        &self,
        position_inertial_m: Vector3<f64>,
        current_epoch: Epoch,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        match &self.gravity_model {
            GravityModel::PointMass(model) => model.compute_field(position_inertial_m),
            GravityModel::SphericalHarmonics(model) => {
                if let Some(orientation_model) = &model.orientation_model {
                    let inertial_to_fixed = orientation_model.rotation_matrix(current_epoch);
                    let position_body_fixed_m = inertial_to_fixed * position_inertial_m;
                    let acceleration_body_fixed_mps2 =
                        model.compute_field(position_body_fixed_m, model.max_deg, true);
                    inertial_to_fixed.transpose() * acceleration_body_fixed_mps2
                } else {
                    let theta = model.body_rotation_rate_radps * current_sim_nanos as f64 * 1.0e-9;
                    let position_body_fixed_m = rotate_about_z(position_inertial_m, -theta);
                    let acceleration_body_fixed_mps2 =
                        model.compute_field(position_body_fixed_m, model.max_deg, true);
                    rotate_about_z(acceleration_body_fixed_mps2, theta)
                }
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct GravityEffector {
    pub grav_bodies: Vec<GravBodyData>,
    cached_body_states: Vec<CachedGravBodyState>,
    central_body_index: Option<usize>,
    timing_enabled: bool,
    timing_stats: GravityTimingStats,
}

impl GravityEffector {
    pub fn new() -> Self {
        Self {
            grav_bodies: Vec::new(),
            cached_body_states: Vec::new(),
            central_body_index: None,
            timing_enabled: false,
            timing_stats: GravityTimingStats::default(),
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &GravityTimingStats {
        &self.timing_stats
    }

    pub fn add_grav_body(&mut self, grav_body: GravBodyData) {
        self.grav_bodies.push(grav_body);
    }

    pub fn update_cache(
        &mut self,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        orientation_derivative_step_nanos: u64,
    ) {
        let total_started = self.timing_enabled.then(Instant::now);
        self.cached_body_states.clear();
        self.cached_body_states.reserve(self.grav_bodies.len());
        self.central_body_index = None;

        for (body_index, body) in self.grav_bodies.iter().enumerate() {
            if body.is_central_body {
                self.central_body_index = Some(body_index);
            }

            let phase_started = self.timing_enabled.then(Instant::now);
            let planet_state_msg = current_planet_state_msg_from_input_or_initial(body);
            let (position_inertial_m, velocity_inertial_mps) =
                if let Some(message) = planet_state_msg.as_ref() {
                    (message.position_inertial_m, message.velocity_inertial_mps)
                } else {
                    current_planet_state_from_input_or_initial(body, current_sim_nanos)
                };
            if let Some(started) = phase_started {
                self.timing_stats.update_cache_state_read_nanos += started.elapsed().as_nanos();
            }

            let orientation = match &body.gravity_model {
                GravityModel::SphericalHarmonics(model) => {
                    if let Some(message) = planet_state_msg.as_ref() {
                        if message.has_orientation {
                            Some(CachedOrientationState {
                                inertial_to_fixed: message.inertial_to_fixed,
                                inertial_to_fixed_dot: message.inertial_to_fixed_dot,
                            })
                        } else if let Some(orientation_model) = &model.orientation_model {
                            let phase_started = self.timing_enabled.then(Instant::now);
                            let inertial_to_fixed =
                                orientation_model.rotation_matrix(current_epoch);
                            if let Some(started) = phase_started {
                                self.timing_stats.update_cache_orientation_current_nanos +=
                                    started.elapsed().as_nanos();
                            }

                            let inertial_to_fixed_dot = if orientation_derivative_step_nanos == 0 {
                                Matrix3::zeros()
                            } else {
                                let derivative_step = Duration::from_total_nanoseconds(
                                    orientation_derivative_step_nanos as i128,
                                );
                                let previous_epoch = current_epoch - derivative_step;
                                let phase_started = self.timing_enabled.then(Instant::now);
                                let previous_inertial_to_fixed =
                                    orientation_model.rotation_matrix(previous_epoch);
                                if let Some(started) = phase_started {
                                    self.timing_stats.update_cache_orientation_previous_nanos +=
                                        started.elapsed().as_nanos();
                                }
                                (inertial_to_fixed - previous_inertial_to_fixed)
                                    / (orientation_derivative_step_nanos as f64 * 1.0e-9)
                            };

                            Some(CachedOrientationState {
                                inertial_to_fixed,
                                inertial_to_fixed_dot,
                            })
                        } else {
                            None
                        }
                    } else if let Some(orientation_model) = &model.orientation_model {
                        let phase_started = self.timing_enabled.then(Instant::now);
                        let inertial_to_fixed = orientation_model.rotation_matrix(current_epoch);
                        if let Some(started) = phase_started {
                            self.timing_stats.update_cache_orientation_current_nanos +=
                                started.elapsed().as_nanos();
                        }

                        let inertial_to_fixed_dot = if orientation_derivative_step_nanos == 0 {
                            Matrix3::zeros()
                        } else {
                            let derivative_step = Duration::from_total_nanoseconds(
                                orientation_derivative_step_nanos as i128,
                            );
                            let previous_epoch = current_epoch - derivative_step;
                            let phase_started = self.timing_enabled.then(Instant::now);
                            let previous_inertial_to_fixed =
                                orientation_model.rotation_matrix(previous_epoch);
                            if let Some(started) = phase_started {
                                self.timing_stats.update_cache_orientation_previous_nanos +=
                                    started.elapsed().as_nanos();
                            }
                            (inertial_to_fixed - previous_inertial_to_fixed)
                                / (orientation_derivative_step_nanos as f64 * 1.0e-9)
                        };

                        Some(CachedOrientationState {
                            inertial_to_fixed,
                            inertial_to_fixed_dot,
                        })
                    } else {
                        None
                    }
                }
                GravityModel::PointMass(_) => None,
            };

            self.cached_body_states.push(CachedGravBodyState {
                position_inertial_m,
                velocity_inertial_mps,
                cached_at_sim_nanos: current_sim_nanos,
                orientation,
            });
        }

        if let Some(started) = total_started {
            self.timing_stats.update_cache_calls += 1;
            self.timing_stats.update_cache_total_nanos += started.elapsed().as_nanos();
        }
    }

    pub fn central_body(&self) -> Option<&GravBodyData> {
        self.grav_bodies.iter().find(|body| body.is_central_body)
    }

    pub fn planet_body_input_mut(
        &mut self,
        planet_name: &str,
    ) -> Option<&mut Input<PlanetStateMsg>> {
        self.grav_bodies
            .iter_mut()
            .find(|body| body.planet_name == planet_name)
            .map(|body| &mut body.planet_body_in_msg)
    }

    pub fn compute_gravity_field(
        &mut self,
        relative_position_m: Vector3<f64>,
        current_epoch: Epoch,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        let total_started = self.timing_enabled.then(Instant::now);
        let central_body_index = self.central_body_index.or_else(|| {
            self.grav_bodies
                .iter()
                .position(|body| body.is_central_body)
        });

        let phase_started = self.timing_enabled.then(Instant::now);
        let inertial_position_m = if let Some(central_body_index) = central_body_index {
            relative_position_m
                + self.euler_stepped_body_position(central_body_index, current_sim_nanos)
        } else {
            relative_position_m
        };

        let central_body_position_m = central_body_index
            .map(|body_index| self.euler_stepped_body_position(body_index, current_sim_nanos))
            .unwrap_or_else(Vector3::zeros);
        if let Some(started) = phase_started {
            self.timing_stats.compute_gravity_position_step_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        let mut total_acceleration = Vector3::zeros();
        for (body_index, body) in self.grav_bodies.iter().enumerate() {
            let body_position_m = self.euler_stepped_body_position(body_index, current_sim_nanos);
            let spacecraft_wrt_body_m = inertial_position_m - body_position_m;
            total_acceleration += self.compute_cached_gravity_inertial(
                body_index,
                body,
                spacecraft_wrt_body_m,
                current_epoch,
                current_sim_nanos,
            );

            if central_body_index.is_some() && !body.is_central_body {
                total_acceleration += self.compute_cached_gravity_inertial(
                    body_index,
                    body,
                    body_position_m - central_body_position_m,
                    current_epoch,
                    current_sim_nanos,
                );
            }
        }
        if let Some(started) = phase_started {
            self.timing_stats.compute_gravity_accel_eval_nanos += started.elapsed().as_nanos();
        }
        if let Some(started) = total_started {
            self.timing_stats.compute_gravity_field_calls += 1;
            self.timing_stats.compute_gravity_field_total_nanos += started.elapsed().as_nanos();
        }
        total_acceleration
    }

    pub fn inertial_position_and_velocity(
        &self,
        relative_position_m: Vector3<f64>,
        relative_velocity_mps: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let central_body_index = self.central_body_index.or_else(|| {
            self.grav_bodies
                .iter()
                .position(|body| body.is_central_body)
        });

        if let Some(central_body_index) = central_body_index {
            let central_position_m =
                self.euler_stepped_body_position(central_body_index, current_sim_nanos);
            let central_velocity_mps =
                self.euler_stepped_body_velocity(central_body_index, current_sim_nanos);

            (
                relative_position_m + central_position_m,
                relative_velocity_mps + central_velocity_mps,
            )
        } else {
            (relative_position_m, relative_velocity_mps)
        }
    }

    fn euler_stepped_body_position(
        &self,
        body_index: usize,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        if let Some(cached_state) = self.cached_body_states.get(body_index) {
            let dt_seconds =
                signed_time_delta_seconds(current_sim_nanos, cached_state.cached_at_sim_nanos);
            cached_state.position_inertial_m + cached_state.velocity_inertial_mps * dt_seconds
        } else {
            let body = &self.grav_bodies[body_index];
            current_planet_state_from_input_or_initial(body, current_sim_nanos).0
        }
    }

    fn euler_stepped_body_velocity(
        &self,
        body_index: usize,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        if let Some(cached_state) = self.cached_body_states.get(body_index) {
            let _ = current_sim_nanos;
            cached_state.velocity_inertial_mps
        } else {
            let body = &self.grav_bodies[body_index];
            current_planet_state_from_input_or_initial(body, current_sim_nanos).1
        }
    }

    fn compute_cached_gravity_inertial(
        &self,
        body_index: usize,
        body: &GravBodyData,
        position_inertial_m: Vector3<f64>,
        current_epoch: Epoch,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        let Some(cached_state) = self.cached_body_states.get(body_index) else {
            return body.compute_gravity_inertial(
                position_inertial_m,
                current_epoch,
                current_sim_nanos,
            );
        };

        match (&body.gravity_model, &cached_state.orientation) {
            (GravityModel::PointMass(model), _) => model.compute_field(position_inertial_m),
            (GravityModel::SphericalHarmonics(model), Some(orientation)) => {
                let dt_seconds =
                    signed_time_delta_seconds(current_sim_nanos, cached_state.cached_at_sim_nanos);
                let inertial_to_fixed =
                    orientation.inertial_to_fixed + orientation.inertial_to_fixed_dot * dt_seconds;
                let position_body_fixed_m = inertial_to_fixed * position_inertial_m;
                let acceleration_body_fixed_mps2 =
                    model.compute_field(position_body_fixed_m, model.max_deg, true);
                inertial_to_fixed.transpose() * acceleration_body_fixed_mps2
            }
            (GravityModel::SphericalHarmonics(model), None) => {
                let theta = model.body_rotation_rate_radps * current_sim_nanos as f64 * 1.0e-9;
                let position_body_fixed_m = rotate_about_z(position_inertial_m, -theta);
                let acceleration_body_fixed_mps2 =
                    model.compute_field(position_body_fixed_m, model.max_deg, true);
                rotate_about_z(acceleration_body_fixed_mps2, theta)
            }
        }
    }
}

fn get_k(degree: usize) -> f64 {
    if degree == 0 { 1.0 } else { 2.0 }
}

fn initialize_pines_parameters(
    max_deg: usize,
) -> (
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
) {
    let mut a_bar_seed: Vec<Vec<f64>> = Vec::with_capacity(max_deg + 2);
    let mut n1: Vec<Vec<f64>> = Vec::with_capacity(max_deg + 2);
    let mut n2: Vec<Vec<f64>> = Vec::with_capacity(max_deg + 2);

    for i in 0..=max_deg + 1 {
        let mut a_row = vec![0.0; i + 1];
        a_row[i] = if i == 0 {
            1.0
        } else {
            (((2 * i + 1) as f64 * get_k(i)) / (2 * i) as f64 / get_k(i - 1)).sqrt()
                * a_bar_seed[i - 1][i - 1]
        };

        let mut n1_row = vec![0.0; i + 1];
        let mut n2_row = vec![0.0; i + 1];
        for m in 0..=i {
            if i >= m + 2 {
                n1_row[m] =
                    (((2 * i + 1) * (2 * i - 1)) as f64 / ((i - m) * (i + m)) as f64).sqrt();
                n2_row[m] = (((i + m - 1) * (2 * i + 1) * (i - m - 1)) as f64
                    / ((i + m) * (i - m) * (2 * i - 3)) as f64)
                    .sqrt();
            }
        }

        a_bar_seed.push(a_row);
        n1.push(n1_row);
        n2.push(n2_row);
    }

    let mut n_quot1: Vec<Vec<f64>> = Vec::with_capacity(max_deg + 1);
    let mut n_quot2: Vec<Vec<f64>> = Vec::with_capacity(max_deg + 1);
    for l in 0..=max_deg {
        let mut nq1_row = vec![0.0; l + 1];
        let mut nq2_row = vec![0.0; l + 1];
        for m in 0..=l {
            if m < l {
                nq1_row[m] =
                    (((l - m) as f64 * get_k(m) * (l + m + 1) as f64) / get_k(m + 1)).sqrt();
            }
            nq2_row[m] = (((l + m + 2) * (l + m + 1) * (2 * l + 1)) as f64 * get_k(m)
                / ((2 * l + 3) as f64 * get_k(m + 1)))
            .sqrt();
        }
        n_quot1.push(nq1_row);
        n_quot2.push(nq2_row);
    }

    (a_bar_seed, n1, n2, n_quot1, n_quot2)
}

fn rotate_about_z(vector: Vector3<f64>, angle_rad: f64) -> Vector3<f64> {
    let cos_theta = angle_rad.cos();
    let sin_theta = angle_rad.sin();
    Vector3::new(
        cos_theta * vector.x - sin_theta * vector.y,
        sin_theta * vector.x + cos_theta * vector.y,
        vector.z,
    )
}

#[derive(Clone)]
struct AniseOrientationModel {
    almanac: Almanac,
    inertial_frame: Frame,
    fixed_frame: Frame,
}

impl AniseOrientationModel {
    fn new(
        first_kernel_path: impl AsRef<Path>,
        additional_kernel_paths: &[PathBuf],
        inertial_frame: Frame,
        fixed_frame: Frame,
    ) -> Self {
        let first_kernel_path = resolve_repo_relative_path(first_kernel_path.as_ref());
        let mut almanac = Almanac::new(&first_kernel_path.to_string_lossy())
            .expect("failed to load ANISE kernel");
        for kernel_path in additional_kernel_paths {
            let resolved_path = resolve_repo_relative_path(kernel_path);
            almanac = almanac
                .load(&resolved_path.to_string_lossy())
                .expect("failed to load additional ANISE kernel");
        }

        Self {
            almanac,
            inertial_frame,
            fixed_frame,
        }
    }

    fn rotation_matrix(&self, epoch: Epoch) -> Matrix3<f64> {
        let rotation = self
            .almanac
            .rotate(self.inertial_frame, self.fixed_frame, epoch)
            .expect("failed to compute ANISE body-fixed rotation");
        matrix3_from_anise(&rotation.rot_mat)
    }
}

impl fmt::Debug for AniseOrientationModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AniseOrientationModel")
            .field("inertial_frame", &self.inertial_frame)
            .field("fixed_frame", &self.fixed_frame)
            .finish()
    }
}

fn current_planet_state_from_input_or_initial(
    body: &GravBodyData,
    current_sim_nanos: u64,
) -> (Vector3<f64>, Vector3<f64>) {
    if body.planet_body_in_msg.is_connected() {
        let state = body.planet_body_in_msg.read();
        return (state.position_inertial_m, state.velocity_inertial_mps);
    }

    let position_m =
        body.initial_position_m + body.initial_velocity_mps * (current_sim_nanos as f64 * 1.0e-9);
    let velocity_mps = body.initial_velocity_mps;

    (position_m, velocity_mps)
}

fn current_planet_state_msg_from_input_or_initial(body: &GravBodyData) -> Option<PlanetStateMsg> {
    if body.planet_body_in_msg.is_connected() {
        Some(body.planet_body_in_msg.read())
    } else {
        None
    }
}

fn signed_time_delta_seconds(current_sim_nanos: u64, baseline_sim_nanos: u64) -> f64 {
    (current_sim_nanos as i128 - baseline_sim_nanos as i128) as f64 * 1.0e-9
}

fn matrix3_from_anise(rot_mat: &anise::math::Matrix3) -> Matrix3<f64> {
    Matrix3::new(
        rot_mat[(0, 0)],
        rot_mat[(0, 1)],
        rot_mat[(0, 2)],
        rot_mat[(1, 0)],
        rot_mat[(1, 1)],
        rot_mat[(1, 2)],
        rot_mat[(2, 0)],
        rot_mat[(2, 1)],
        rot_mat[(2, 2)],
    )
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
