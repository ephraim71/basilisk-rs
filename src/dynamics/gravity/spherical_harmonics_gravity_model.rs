use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use nalgebra::Vector3;

use super::gravity_model::{checked_radius, validate_positive};
use super::{GravityError, GravityFrame, GravityModel};

#[derive(Clone, Debug)]
pub struct SphericalHarmonicsGravityModel {
    reference_radius_m: f64,
    mu_m3ps2: f64,
    max_degree: usize,
    c_bar: Vec<Vec<f64>>,
    s_bar: Vec<Vec<f64>>,
    n1: Vec<Vec<f64>>,
    n2: Vec<Vec<f64>>,
    n_quot1: Vec<Vec<f64>>,
    n_quot2: Vec<Vec<f64>>,
    scratch: SphericalHarmonicsScratch,
}

#[derive(Clone, Debug)]
struct SphericalHarmonicsScratch {
    a_bar: Vec<Vec<f64>>,
    r_e: Vec<f64>,
    i_m: Vec<f64>,
    rho_l: Vec<f64>,
}

impl SphericalHarmonicsGravityModel {
    pub fn from_file(path: impl AsRef<Path>, max_degree: usize) -> Result<Self, GravityError> {
        let path = resolve_repo_relative_path(path.as_ref());
        let file = File::open(&path).map_err(|error| invalid_file(&path, error.to_string()))?;
        let mut lines = BufReader::new(file).lines();
        let header = lines
            .next()
            .ok_or_else(|| invalid_file(&path, "missing header"))?
            .map_err(|error| invalid_file(&path, error.to_string()))?;
        let fields: Vec<_> = header.split(',').map(str::trim).collect();
        if fields.len() < 8 {
            return Err(invalid_file(
                &path,
                "header must contain at least eight comma-separated fields",
            ));
        }

        let reference_radius_m = parse_f64(&path, fields[0], "reference radius")?;
        let mu_m3ps2 = parse_f64(&path, fields[1], "gravitational parameter")?;
        let _rotation_rate_radps = parse_f64(&path, fields[2], "rotation rate")?;
        let file_max_degree = parse_usize(&path, fields[3], "maximum degree")?;
        let file_max_order = parse_usize(&path, fields[4], "maximum order")?;
        let normalized = parse_usize(&path, fields[5], "normalization flag")? == 1;
        let reference_longitude = parse_f64(&path, fields[6], "reference longitude")?;
        let reference_latitude = parse_f64(&path, fields[7], "reference latitude")?;

        validate_positive("reference_radius_m", reference_radius_m)?;
        validate_positive("mu_m3ps2", mu_m3ps2)?;
        let available = file_max_degree.min(file_max_order);
        if max_degree > available {
            return Err(GravityError::InvalidDegree {
                requested: max_degree,
                available,
            });
        }
        if !normalized {
            return Err(invalid_file(
                &path,
                "only normalized spherical-harmonic coefficients are supported",
            ));
        }
        if reference_longitude != 0.0 || reference_latitude != 0.0 {
            return Err(invalid_file(
                &path,
                "reference longitude and latitude must both be zero",
            ));
        }

        let mut c_bar = triangular_zeros(max_degree + 1);
        let mut s_bar = triangular_zeros(max_degree + 1);
        for line in lines {
            let line = line.map_err(|error| invalid_file(&path, error.to_string()))?;
            if line.trim().is_empty() {
                continue;
            }
            let fields: Vec<_> = line.split(',').map(str::trim).collect();
            if fields.len() < 4 {
                return Err(invalid_file(
                    &path,
                    format!("coefficient row has fewer than four fields: {line}"),
                ));
            }
            let degree = parse_usize(&path, fields[0], "coefficient degree")?;
            let order = parse_usize(&path, fields[1], "coefficient order")?;
            if order > degree {
                return Err(invalid_file(
                    &path,
                    format!("coefficient order {order} exceeds degree {degree}"),
                ));
            }
            if degree > max_degree {
                continue;
            }
            c_bar[degree][order] = parse_f64(&path, fields[2], "C coefficient")?;
            s_bar[degree][order] = parse_f64(&path, fields[3], "S coefficient")?;
        }

        let (a_bar, n1, n2, n_quot1, n_quot2) = initialize_pines_parameters(max_degree);
        Ok(Self {
            reference_radius_m,
            mu_m3ps2,
            max_degree,
            c_bar,
            s_bar,
            n1,
            n2,
            n_quot1,
            n_quot2,
            scratch: SphericalHarmonicsScratch {
                a_bar,
                r_e: vec![0.0; max_degree + 2],
                i_m: vec![0.0; max_degree + 2],
                rho_l: vec![0.0; max_degree + 2],
            },
        })
    }

    pub fn max_degree(&self) -> usize {
        self.max_degree
    }

    pub fn reference_radius_m(&self) -> f64 {
        self.reference_radius_m
    }

    pub fn mu_m3ps2(&self) -> f64 {
        self.mu_m3ps2
    }

    pub fn acceleration_at_degree_mps2(
        &mut self,
        position_body_fixed_m: Vector3<f64>,
        degree: usize,
        include_zero_degree: bool,
    ) -> Result<Vector3<f64>, GravityError> {
        if degree > self.max_degree {
            return Err(GravityError::InvalidDegree {
                requested: degree,
                available: self.max_degree,
            });
        }

        let radius = checked_radius(position_body_fixed_m)?;
        let s = position_body_fixed_m.x / radius;
        let t = position_body_fixed_m.y / radius;
        let u = position_body_fixed_m.z / radius;
        let order = degree;

        self.scratch.r_e[..=order + 1].fill(0.0);
        self.scratch.i_m[..=order + 1].fill(0.0);
        self.scratch.rho_l[..=degree + 1].fill(0.0);
        let SphericalHarmonicsScratch {
            a_bar,
            r_e,
            i_m,
            rho_l,
        } = &mut self.scratch;

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

        let rho = self.reference_radius_m / radius;
        rho_l[0] = self.mu_m3ps2 / radius;
        rho_l[1] = rho_l[0] * rho;
        let mut a1 = 0.0;
        let mut a2 = 0.0;
        let mut a3 = 0.0;
        let mut a4 = if include_zero_degree {
            -rho_l[1] / self.reference_radius_m
        } else {
            0.0
        };

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
            a1 += rho_l[l + 1] / self.reference_radius_m * sum_a1;
            a2 += rho_l[l + 1] / self.reference_radius_m * sum_a2;
            a3 += rho_l[l + 1] / self.reference_radius_m * sum_a3;
            a4 -= rho_l[l + 1] / self.reference_radius_m * sum_a4;
        }

        Ok(Vector3::new(a1 + s * a4, a2 + t * a4, a3 + u * a4))
    }
}

impl GravityModel for SphericalHarmonicsGravityModel {
    fn frame(&self) -> GravityFrame {
        GravityFrame::BodyFixed
    }

    fn gravitational_parameter_m3ps2(&self) -> f64 {
        self.mu_m3ps2
    }

    fn potential_frame(&self) -> GravityFrame {
        GravityFrame::InertialInvariant
    }

    fn acceleration_mps2(
        &mut self,
        position_m: Vector3<f64>,
    ) -> Result<Vector3<f64>, GravityError> {
        self.acceleration_at_degree_mps2(position_m, self.max_degree, true)
    }

    fn specific_potential_jpkg(&mut self, position_m: Vector3<f64>) -> Result<f64, GravityError> {
        // Official Basilisk currently uses point-mass potential for this model.
        Ok(-self.mu_m3ps2 / checked_radius(position_m)?)
    }
}

fn triangular_zeros(rows: usize) -> Vec<Vec<f64>> {
    (0..rows).map(|degree| vec![0.0; degree + 1]).collect()
}

fn get_k(degree: usize) -> f64 {
    if degree == 0 { 1.0 } else { 2.0 }
}

type PinesParameters = (
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
    Vec<Vec<f64>>,
);

fn initialize_pines_parameters(max_degree: usize) -> PinesParameters {
    let mut a_bar_seed: Vec<Vec<f64>> = Vec::with_capacity(max_degree + 2);
    let mut n1: Vec<Vec<f64>> = Vec::with_capacity(max_degree + 2);
    let mut n2: Vec<Vec<f64>> = Vec::with_capacity(max_degree + 2);

    for i in 0..=max_degree + 1 {
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

    let mut n_quot1 = Vec::with_capacity(max_degree + 1);
    let mut n_quot2 = Vec::with_capacity(max_degree + 1);
    for l in 0..=max_degree {
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

fn parse_f64(path: &Path, value: &str, field: &str) -> Result<f64, GravityError> {
    let parsed = value
        .parse::<f64>()
        .map_err(|error| invalid_file(path, format!("failed to parse {field}: {error}")))?;
    if !parsed.is_finite() {
        return Err(invalid_file(path, format!("{field} must be finite")));
    }
    Ok(parsed)
}

fn parse_usize(path: &Path, value: &str, field: &str) -> Result<usize, GravityError> {
    value
        .parse::<usize>()
        .map_err(|error| invalid_file(path, format!("failed to parse {field}: {error}")))
}

fn invalid_file(path: &Path, reason: impl Into<String>) -> GravityError {
    GravityError::InvalidGravityFile {
        path: path.to_path_buf(),
        reason: reason.into(),
    }
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
