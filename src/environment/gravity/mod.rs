use std::fmt;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::Epoch;
use nalgebra::Vector3;

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

        let mut a_bar = self.a_bar_seed.clone();
        for l in 1..=degree + 1 {
            a_bar[l][l - 1] = (((2 * l) as f64 * get_k(l - 1)) / get_k(l)).sqrt() * a_bar[l][l] * u;
        }

        let mut r_e = vec![0.0; order + 2];
        let mut i_m = vec![0.0; order + 2];
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
        let mut rho_l = vec![0.0; degree + 2];
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
                    let rotation = orientation_model
                        .almanac
                        .rotate(
                            orientation_model.inertial_frame,
                            orientation_model.fixed_frame,
                            current_epoch,
                        )
                        .expect("failed to compute ANISE body-fixed rotation");
                    let position_body_fixed_m =
                        apply_anise_rotation(&rotation.rot_mat, position_inertial_m);
                    let acceleration_body_fixed_mps2 =
                        model.compute_field(position_body_fixed_m, model.max_deg, true);
                    apply_anise_rotation_transpose(&rotation.rot_mat, acceleration_body_fixed_mps2)
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
}

impl GravityEffector {
    pub fn new() -> Self {
        Self {
            grav_bodies: Vec::new(),
        }
    }

    pub fn add_grav_body(&mut self, grav_body: GravBodyData) {
        self.grav_bodies.push(grav_body);
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
        &self,
        relative_position_m: Vector3<f64>,
        current_epoch: Epoch,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        let inertial_position_m = if let Some(central_body) = self.central_body() {
            relative_position_m + self.current_planet_state(central_body, current_sim_nanos).0
        } else {
            relative_position_m
        };

        let central_body_position_m = self
            .central_body()
            .map(|body| self.current_planet_state(body, current_sim_nanos).0)
            .unwrap_or_else(Vector3::zeros);

        self.grav_bodies.iter().fold(Vector3::zeros(), |sum, body| {
            let body_position_m = self.current_planet_state(body, current_sim_nanos).0;
            let spacecraft_wrt_body_m = inertial_position_m - body_position_m;
            let mut acceleration = sum
                + body.compute_gravity_inertial(
                    spacecraft_wrt_body_m,
                    current_epoch,
                    current_sim_nanos,
                );

            if self.central_body().is_some() && !body.is_central_body {
                acceleration += body.compute_gravity_inertial(
                    body_position_m - central_body_position_m,
                    current_epoch,
                    current_sim_nanos,
                );
            }

            acceleration
        })
    }

    pub fn inertial_position_and_velocity(
        &self,
        relative_position_m: Vector3<f64>,
        relative_velocity_mps: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        if let Some(central_body) = self.central_body() {
            let (central_position_m, central_velocity_mps) =
                self.current_planet_state(central_body, current_sim_nanos);

            (
                relative_position_m + central_position_m,
                relative_velocity_mps + central_velocity_mps,
            )
        } else {
            (relative_position_m, relative_velocity_mps)
        }
    }

    fn current_planet_state(
        &self,
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
}

impl fmt::Debug for AniseOrientationModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AniseOrientationModel")
            .field("inertial_frame", &self.inertial_frame)
            .field("fixed_frame", &self.fixed_frame)
            .finish()
    }
}

fn apply_anise_rotation(rot_mat: &anise::math::Matrix3, vector: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        rot_mat[(0, 0)] * vector.x + rot_mat[(0, 1)] * vector.y + rot_mat[(0, 2)] * vector.z,
        rot_mat[(1, 0)] * vector.x + rot_mat[(1, 1)] * vector.y + rot_mat[(1, 2)] * vector.z,
        rot_mat[(2, 0)] * vector.x + rot_mat[(2, 1)] * vector.y + rot_mat[(2, 2)] * vector.z,
    )
}

fn apply_anise_rotation_transpose(
    rot_mat: &anise::math::Matrix3,
    vector: Vector3<f64>,
) -> Vector3<f64> {
    Vector3::new(
        rot_mat[(0, 0)] * vector.x + rot_mat[(1, 0)] * vector.y + rot_mat[(2, 0)] * vector.z,
        rot_mat[(0, 1)] * vector.x + rot_mat[(1, 1)] * vector.y + rot_mat[(2, 1)] * vector.z,
        rot_mat[(0, 2)] * vector.x + rot_mat[(1, 2)] * vector.y + rot_mat[(2, 2)] * vector.z,
    )
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
