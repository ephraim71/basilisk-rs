use std::path::Path;
use std::time::Instant;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{Input, Output, PlanetOrientation, PlanetStateMsg};

use super::{
    GravityError, GravityFrame, GravityModel, PointMassGravityModel, PolyhedralGravityModel,
    SphericalHarmonicsGravityModel,
};

#[derive(Debug)]
pub struct GravBodyData {
    name: String,
    is_central_body: bool,
    model: Box<dyn GravityModel>,
    fallback_state: PlanetStateMsg,
    planet_body_in: Input<PlanetStateMsg>,
    cached_state: Option<CachedGravBodyState>,
}

#[derive(Clone, Debug)]
struct CachedGravBodyState {
    state: PlanetStateMsg,
    cached_at_sim_nanos: u64,
}

#[derive(Clone, Debug)]
struct GravBodyStateAtTime {
    position_inertial_m: Vector3<f64>,
    velocity_inertial_mps: Vector3<f64>,
    inertial_to_fixed: Option<Matrix3<f64>>,
}

#[derive(Clone, Debug, Default)]
pub struct GravityTimingStats {
    pub update_cache_calls: u64,
    pub update_cache_total_nanos: u128,
    pub update_cache_state_read_nanos: u128,
    pub compute_gravity_field_calls: u64,
    pub compute_gravity_field_total_nanos: u128,
    pub compute_gravity_position_step_nanos: u128,
    pub compute_gravity_accel_eval_nanos: u128,
}

impl GravBodyData {
    pub fn new(
        name: impl Into<String>,
        model: impl GravityModel + 'static,
    ) -> Result<Self, GravityError> {
        let name = name.into();
        if name.trim().is_empty() {
            return Err(GravityError::EmptyBodyName);
        }
        Ok(Self {
            name,
            is_central_body: false,
            model: Box::new(model),
            fallback_state: PlanetStateMsg::default(),
            planet_body_in: Input::default(),
            cached_state: None,
        })
    }

    pub fn point_mass(
        name: impl Into<String>,
        mu_m3ps2: f64,
        is_central_body: bool,
        initial_position_m: Vector3<f64>,
        initial_velocity_mps: Vector3<f64>,
    ) -> Result<Self, GravityError> {
        Self::new(name, PointMassGravityModel::new(mu_m3ps2)?).map(|body| {
            body.central_if(is_central_body)
                .with_static_ephemeris(PlanetStateMsg {
                    position_inertial_m: initial_position_m,
                    velocity_inertial_mps: initial_velocity_mps,
                    orientation: None,
                })
        })
    }

    pub fn spherical_harmonics_from_file(
        name: impl Into<String>,
        path: impl AsRef<Path>,
        max_degree: usize,
        is_central_body: bool,
        initial_position_m: Vector3<f64>,
        initial_velocity_mps: Vector3<f64>,
    ) -> Result<Self, GravityError> {
        Self::new(
            name,
            SphericalHarmonicsGravityModel::from_file(path, max_degree)?,
        )
        .map(|body| {
            body.central_if(is_central_body)
                .with_static_ephemeris(PlanetStateMsg {
                    position_inertial_m: initial_position_m,
                    velocity_inertial_mps: initial_velocity_mps,
                    orientation: None,
                })
        })
    }

    pub fn polyhedral(
        name: impl Into<String>,
        model: PolyhedralGravityModel,
        is_central_body: bool,
        initial_position_m: Vector3<f64>,
        initial_velocity_mps: Vector3<f64>,
    ) -> Result<Self, GravityError> {
        Self::new(name, model).map(|body| {
            body.central_if(is_central_body)
                .with_static_ephemeris(PlanetStateMsg {
                    position_inertial_m: initial_position_m,
                    velocity_inertial_mps: initial_velocity_mps,
                    orientation: None,
                })
        })
    }

    pub fn central(mut self) -> Self {
        self.is_central_body = true;
        self
    }

    pub fn central_if(mut self, is_central_body: bool) -> Self {
        self.is_central_body = is_central_body;
        self
    }

    pub fn with_static_ephemeris(mut self, state: PlanetStateMsg) -> Self {
        self.fallback_state = state;
        self
    }

    pub fn with_static_orientation(mut self, orientation: PlanetOrientation) -> Self {
        self.fallback_state.orientation = Some(orientation);
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn is_central_body(&self) -> bool {
        self.is_central_body
    }

    pub fn gravitational_parameter_m3ps2(&self) -> f64 {
        self.model.gravitational_parameter_m3ps2()
    }

    pub fn planet_body_input_mut(&mut self) -> &mut Input<PlanetStateMsg> {
        &mut self.planet_body_in
    }

    fn state_at(&self, current_sim_nanos: u64) -> GravBodyStateAtTime {
        let cached = self.cached_state.as_ref();
        let state = cached
            .map(|cached| &cached.state)
            .unwrap_or(&self.fallback_state);
        let cached_at = cached.map(|cached| cached.cached_at_sim_nanos).unwrap_or(0);
        let dt_seconds = signed_time_delta_seconds(current_sim_nanos, cached_at);
        let inertial_to_fixed = state.orientation.as_ref().map(|orientation| {
            orientation.inertial_to_fixed + orientation.inertial_to_fixed_dot * dt_seconds
        });
        GravBodyStateAtTime {
            position_inertial_m: state.position_inertial_m
                + state.velocity_inertial_mps * dt_seconds,
            velocity_inertial_mps: state.velocity_inertial_mps,
            inertial_to_fixed,
        }
    }

    fn evaluate_acceleration_inertial(
        &mut self,
        position_wrt_body_inertial_m: Vector3<f64>,
        inertial_to_fixed: Option<Matrix3<f64>>,
    ) -> Result<Vector3<f64>, GravityError> {
        match self.model.frame() {
            GravityFrame::InertialInvariant => {
                self.model.acceleration_mps2(position_wrt_body_inertial_m)
            }
            GravityFrame::BodyFixed => {
                let inertial_to_fixed = inertial_to_fixed
                    .ok_or_else(|| GravityError::MissingOrientation(self.name.clone()))?;
                let position_fixed_m = inertial_to_fixed * position_wrt_body_inertial_m;
                let acceleration_fixed_mps2 = self.model.acceleration_mps2(position_fixed_m)?;
                Ok(inertial_to_fixed.transpose() * acceleration_fixed_mps2)
            }
        }
    }

    fn evaluate_specific_potential(
        &mut self,
        position_wrt_body_inertial_m: Vector3<f64>,
        inertial_to_fixed: Option<Matrix3<f64>>,
    ) -> Result<f64, GravityError> {
        let model_position_m = match self.model.potential_frame() {
            GravityFrame::InertialInvariant => position_wrt_body_inertial_m,
            GravityFrame::BodyFixed => {
                inertial_to_fixed
                    .ok_or_else(|| GravityError::MissingOrientation(self.name.clone()))?
                    * position_wrt_body_inertial_m
            }
        };
        self.model.specific_potential_jpkg(model_position_m)
    }
}

#[derive(Debug)]
pub struct GravityEffector {
    bodies: Vec<GravBodyData>,
    central_body_index: Option<usize>,
    pub central_body_out: Output<PlanetStateMsg>,
    timing_enabled: bool,
    timing_stats: GravityTimingStats,
}

impl Default for GravityEffector {
    fn default() -> Self {
        Self::new()
    }
}

impl GravityEffector {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            central_body_index: None,
            central_body_out: Output::default(),
            timing_enabled: false,
            timing_stats: GravityTimingStats::default(),
        }
    }

    pub fn add_grav_body(&mut self, body: GravBodyData) -> Result<(), GravityError> {
        if self
            .bodies
            .iter()
            .any(|existing| existing.name == body.name)
        {
            return Err(GravityError::DuplicateBodyName(body.name));
        }
        if body.is_central_body {
            if let Some(index) = self.central_body_index {
                return Err(GravityError::MultipleCentralBodies {
                    existing: self.bodies[index].name.clone(),
                    attempted: body.name,
                });
            }
            self.central_body_index = Some(self.bodies.len());
        }
        self.bodies.push(body);
        Ok(())
    }

    pub fn bodies(&self) -> &[GravBodyData] {
        &self.bodies
    }

    pub fn central_body(&self) -> Option<&GravBodyData> {
        self.central_body_index.map(|index| &self.bodies[index])
    }

    pub fn planet_body_input_mut(
        &mut self,
        name: &str,
    ) -> Result<&mut Input<PlanetStateMsg>, GravityError> {
        self.bodies
            .iter_mut()
            .find(|body| body.name == name)
            .map(GravBodyData::planet_body_input_mut)
            .ok_or_else(|| GravityError::BodyNotFound(name.to_string()))
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &GravityTimingStats {
        &self.timing_stats
    }

    pub fn has_complete_cache(&self) -> bool {
        self.bodies.iter().all(|body| body.cached_state.is_some())
    }

    pub fn update_cache(&mut self, current_sim_nanos: u64) -> Result<(), GravityError> {
        let total_started = self.timing_enabled.then(Instant::now);
        for body in &mut self.bodies {
            let phase_started = self.timing_enabled.then(Instant::now);
            let state = if body.planet_body_in.is_connected() {
                body.planet_body_in.read()
            } else {
                body.fallback_state.clone()
            };
            if let Some(started) = phase_started {
                self.timing_stats.update_cache_state_read_nanos += started.elapsed().as_nanos();
            }
            if body.model.frame() == GravityFrame::BodyFixed && state.orientation.is_none() {
                return Err(GravityError::MissingOrientation(body.name.clone()));
            }
            body.cached_state = Some(CachedGravBodyState {
                state,
                cached_at_sim_nanos: current_sim_nanos,
            });
        }

        if let Some(index) = self.central_body_index
            && let Some(cached) = &self.bodies[index].cached_state
        {
            self.central_body_out.write(cached.state.clone());
        }
        if let Some(started) = total_started {
            self.timing_stats.update_cache_calls += 1;
            self.timing_stats.update_cache_total_nanos += started.elapsed().as_nanos();
        }
        Ok(())
    }

    pub fn compute_gravity_field(
        &mut self,
        relative_position_m: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> Result<Vector3<f64>, GravityError> {
        let total_started = self.timing_enabled.then(Instant::now);
        let phase_started = self.timing_enabled.then(Instant::now);
        let central_position_m = self
            .central_body_index
            .map(|index| {
                self.bodies[index]
                    .state_at(current_sim_nanos)
                    .position_inertial_m
            })
            .unwrap_or_else(Vector3::zeros);
        let inertial_position_m = relative_position_m + central_position_m;
        if let Some(started) = phase_started {
            self.timing_stats.compute_gravity_position_step_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        let mut total_acceleration = Vector3::zeros();
        for body in &mut self.bodies {
            let state = body.state_at(current_sim_nanos);
            total_acceleration += body.evaluate_acceleration_inertial(
                inertial_position_m - state.position_inertial_m,
                state.inertial_to_fixed,
            )?;
            if self.central_body_index.is_some() && !body.is_central_body {
                total_acceleration += body.evaluate_acceleration_inertial(
                    state.position_inertial_m - central_position_m,
                    state.inertial_to_fixed,
                )?;
            }
        }
        if let Some(started) = phase_started {
            self.timing_stats.compute_gravity_accel_eval_nanos += started.elapsed().as_nanos();
        }
        if let Some(started) = total_started {
            self.timing_stats.compute_gravity_field_calls += 1;
            self.timing_stats.compute_gravity_field_total_nanos += started.elapsed().as_nanos();
        }
        Ok(total_acceleration)
    }

    pub fn specific_potential_jpkg(
        &mut self,
        relative_position_m: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> Result<f64, GravityError> {
        let central_position_m = self
            .central_body_index
            .map(|index| {
                self.bodies[index]
                    .state_at(current_sim_nanos)
                    .position_inertial_m
            })
            .unwrap_or_else(Vector3::zeros);
        let inertial_position_m = relative_position_m + central_position_m;
        let mut potential = 0.0;
        for body in &mut self.bodies {
            let state = body.state_at(current_sim_nanos);
            potential += body.evaluate_specific_potential(
                inertial_position_m - state.position_inertial_m,
                state.inertial_to_fixed,
            )?;
            if self.central_body_index.is_some() && !body.is_central_body {
                // Match the intended Basilisk relative-frame energy contribution.
                potential += body.evaluate_specific_potential(
                    state.position_inertial_m - central_position_m,
                    state.inertial_to_fixed,
                )?;
            }
        }
        Ok(potential)
    }

    pub fn inertial_position_and_velocity(
        &self,
        relative_position_m: Vector3<f64>,
        relative_velocity_mps: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        if let Some(index) = self.central_body_index {
            let central = self.bodies[index].state_at(current_sim_nanos);
            (
                relative_position_m + central.position_inertial_m,
                relative_velocity_mps + central.velocity_inertial_mps,
            )
        } else {
            (relative_position_m, relative_velocity_mps)
        }
    }
}

fn signed_time_delta_seconds(current_sim_nanos: u64, baseline_sim_nanos: u64) -> f64 {
    (current_sim_nanos as i128 - baseline_sim_nanos as i128) as f64 * 1.0e-9
}
