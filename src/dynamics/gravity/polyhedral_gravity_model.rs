use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Vector3};

use super::gravity_model::{checked_radius, validate_positive};
use super::{GravityError, GravityFrame, GravityModel};

#[derive(Clone, Debug)]
pub struct PolyhedralGravityModel {
    mu_m3ps2: f64,
    vertices_m: Vec<Vector3<f64>>,
    faces: Vec<FaceGeometry>,
    volume_m3: f64,
}

#[derive(Clone, Debug)]
struct FaceGeometry {
    indices: [usize; 3],
    normal: Vector3<f64>,
}

type PolyhedronMesh = (Vec<Vector3<f64>>, Vec<[usize; 3]>);

impl PolyhedralGravityModel {
    pub fn new(
        mu_m3ps2: f64,
        vertices_m: Vec<Vector3<f64>>,
        faces: Vec<[usize; 3]>,
    ) -> Result<Self, GravityError> {
        let mu_m3ps2 = validate_positive("mu_m3ps2", mu_m3ps2)?;
        if vertices_m.len() < 4 {
            return Err(GravityError::InvalidPolyhedron(
                "at least four vertices are required".to_string(),
            ));
        }
        if faces.len() < 4 {
            return Err(GravityError::InvalidPolyhedron(
                "at least four triangular faces are required".to_string(),
            ));
        }
        if vertices_m
            .iter()
            .any(|vertex| !vertex.iter().all(|component| component.is_finite()))
        {
            return Err(GravityError::InvalidPolyhedron(
                "all vertex coordinates must be finite".to_string(),
            ));
        }

        let mut edge_counts = HashMap::<(usize, usize), usize>::new();
        let mut face_geometry = Vec::with_capacity(faces.len());
        let mut volume_m3 = 0.0;
        for (face_index, indices) in faces.into_iter().enumerate() {
            if indices.iter().any(|index| *index >= vertices_m.len()) {
                return Err(GravityError::InvalidPolyhedron(format!(
                    "face {face_index} references a vertex outside 0..{}",
                    vertices_m.len()
                )));
            }
            if indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0] {
                return Err(GravityError::InvalidPolyhedron(format!(
                    "face {face_index} repeats a vertex"
                )));
            }

            let [i, j, k] = indices;
            let normal_unnormalized =
                (vertices_m[j] - vertices_m[i]).cross(&(vertices_m[k] - vertices_m[j]));
            let normal_norm = normal_unnormalized.norm();
            if !normal_norm.is_finite() || normal_norm == 0.0 {
                return Err(GravityError::InvalidPolyhedron(format!(
                    "face {face_index} is degenerate"
                )));
            }
            let normal = normal_unnormalized / normal_norm;
            volume_m3 += vertices_m[i]
                .cross(&vertices_m[j])
                .dot(&vertices_m[k])
                .abs()
                / 6.0;
            for (a, b) in [(i, j), (j, k), (k, i)] {
                *edge_counts.entry((a.min(b), a.max(b))).or_default() += 1;
            }
            face_geometry.push(FaceGeometry { indices, normal });
        }

        if edge_counts.values().any(|count| *count != 2) {
            return Err(GravityError::InvalidPolyhedron(
                "mesh must be closed: every edge must belong to exactly two faces".to_string(),
            ));
        }
        if !volume_m3.is_finite() || volume_m3 <= 0.0 {
            return Err(GravityError::InvalidPolyhedron(
                "mesh volume must be finite and positive".to_string(),
            ));
        }

        Ok(Self {
            mu_m3ps2,
            vertices_m,
            faces: face_geometry,
            volume_m3,
        })
    }

    /// Loads the `.txt`, `.obj`, or `.tab` mesh formats, whose vertex
    /// coordinates are expressed in km.
    pub fn from_mesh_file(path: impl AsRef<Path>, mu_m3ps2: f64) -> Result<Self, GravityError> {
        let path = resolve_repo_relative_path(path.as_ref());
        let contents =
            fs::read_to_string(&path).map_err(|error| invalid_file(&path, error.to_string()))?;
        let extension = path.extension().and_then(|value| value.to_str());
        let (vertices_m, faces) = match extension {
            Some("txt") => parse_txt(&path, &contents)?,
            Some("obj") => parse_obj(&path, &contents, false)?,
            Some("tab") => parse_tab(&path, &contents)?,
            _ => {
                return Err(invalid_file(
                    &path,
                    "polyhedron extension must be .txt, .obj, or .tab",
                ));
            }
        };
        Self::new(mu_m3ps2, vertices_m, faces)
    }

    pub fn mu_m3ps2(&self) -> f64 {
        self.mu_m3ps2
    }

    pub fn volume_m3(&self) -> f64 {
        self.volume_m3
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices_m.len()
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }
}

impl GravityModel for PolyhedralGravityModel {
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
        checked_radius(position_m)?;
        let mut edge_contribution = Vector3::zeros();
        let mut face_contribution = Vector3::zeros();

        for face in &self.faces {
            let [i, j, k] = face.indices;
            let ri = self.vertices_m[i] - position_m;
            let rj = self.vertices_m[j] - position_m;
            let rk = self.vertices_m[k] - position_m;
            let normal = face.normal;

            for (first_index, r1, r2) in
                [(i.min(j), ri, rj), (j.min(k), rj, rk), (i.min(k), rk, ri)]
            {
                let edge = r2 - r1;
                let edge_length = edge.norm();
                let edge_normal_unnormalized = edge.cross(&normal);
                let edge_normal_norm = edge_normal_unnormalized.norm();
                let a = r1.norm();
                let b = r2.norm();
                let denominator = a + b - edge_length;
                if edge_normal_norm == 0.0 || denominator <= 0.0 {
                    return Err(GravityError::SingularPosition);
                }
                let edge_normal = edge_normal_unnormalized / edge_normal_norm;
                let edge_factor = ((a + b + edge_length) / denominator).ln();
                let edge_dyad: Matrix3<f64> = normal * edge_normal.transpose();
                let reference = self.vertices_m[first_index] - position_m;
                edge_contribution += edge_dyad * reference * edge_factor;
            }

            let solid_angle_numerator = ri.dot(&rj.cross(&rk));
            let solid_angle_denominator = ri.norm() * rj.norm() * rk.norm()
                + ri.norm() * rj.dot(&rk)
                + rj.norm() * rk.dot(&ri)
                + rk.norm() * ri.dot(&rj);
            let solid_angle = 2.0 * solid_angle_numerator.atan2(solid_angle_denominator);
            face_contribution += normal * normal.dot(&ri) * solid_angle;
        }

        Ok((self.mu_m3ps2 / self.volume_m3) * (-edge_contribution + face_contribution))
    }

    fn specific_potential_jpkg(&mut self, position_m: Vector3<f64>) -> Result<f64, GravityError> {
        // A point-mass potential is used for this model.
        Ok(-self.mu_m3ps2 / checked_radius(position_m)?)
    }
}

fn parse_txt(path: &Path, contents: &str) -> Result<PolyhedronMesh, GravityError> {
    let mut lines = contents.lines().filter(|line| !line.trim().is_empty());
    let header = lines
        .next()
        .ok_or_else(|| invalid_file(path, "missing header"))?;
    let counts = parse_usizes(path, header, 2, "vertex/face counts")?;
    let (vertex_count, face_count) = (counts[0], counts[1]);
    let mut vertices = Vec::with_capacity(vertex_count);
    let mut faces = Vec::with_capacity(face_count);
    for index in 0..vertex_count {
        let line = lines
            .next()
            .ok_or_else(|| invalid_file(path, format!("missing vertex {index}")))?;
        vertices.push(parse_vertex_km(path, line, false)?);
    }
    for index in 0..face_count {
        let line = lines
            .next()
            .ok_or_else(|| invalid_file(path, format!("missing face {index}")))?;
        faces.push(parse_face(path, line, true, false)?);
    }
    Ok((vertices, faces))
}

fn parse_obj(
    path: &Path,
    contents: &str,
    zero_based_faces: bool,
) -> Result<PolyhedronMesh, GravityError> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for line in contents.lines() {
        let trimmed = line.trim();
        if let Some(vertex) = trimmed.strip_prefix("v ") {
            vertices.push(parse_vertex_km(path, vertex, false)?);
        } else if let Some(face) = trimmed.strip_prefix("f ") {
            faces.push(parse_face(path, face, !zero_based_faces, true)?);
        }
    }
    Ok((vertices, faces))
}

fn parse_tab(path: &Path, contents: &str) -> Result<PolyhedronMesh, GravityError> {
    let lines: Vec<_> = contents
        .lines()
        .filter(|line| !line.trim().is_empty())
        .collect();
    if let Some(header) = lines.first()
        && let Ok(counts) = parse_usizes(path, header, 2, "vertex/face counts")
    {
        let (vertex_count, face_count) = (counts[0], counts[1]);
        let mut vertices = Vec::with_capacity(vertex_count);
        let mut faces = Vec::with_capacity(face_count);
        for line in lines.iter().skip(1).take(vertex_count) {
            vertices.push(parse_vertex_km(path, line, true)?);
        }
        for line in lines.iter().skip(1 + vertex_count).take(face_count) {
            faces.push(parse_face(path, line, true, false)?);
        }
        if vertices.len() != vertex_count || faces.len() != face_count {
            return Err(invalid_file(
                path,
                "tab file ended before all vertices/faces",
            ));
        }
        return Ok((vertices, faces));
    }
    parse_obj(path, contents, true)
}

fn parse_vertex_km(
    path: &Path,
    line: &str,
    skip_first: bool,
) -> Result<Vector3<f64>, GravityError> {
    let values: Vec<_> = line.split_whitespace().collect();
    let offset = usize::from(skip_first);
    if values.len() < offset + 3 {
        return Err(invalid_file(path, format!("invalid vertex row: {line}")));
    }
    let mut xyz = [0.0; 3];
    for component in 0..3 {
        xyz[component] = values[offset + component]
            .parse::<f64>()
            .map_err(|error| invalid_file(path, format!("invalid vertex row '{line}': {error}")))?
            * 1.0e3;
    }
    Ok(Vector3::new(xyz[0], xyz[1], xyz[2]))
}

fn parse_face(
    path: &Path,
    line: &str,
    one_based: bool,
    obj_syntax: bool,
) -> Result<[usize; 3], GravityError> {
    let values: Vec<_> = line.split_whitespace().collect();
    if values.len() < 3 {
        return Err(invalid_file(path, format!("invalid face row: {line}")));
    }
    let offset = usize::from(values.len() >= 4 && !obj_syntax);
    let mut face = [0; 3];
    for index in 0..3 {
        let token = if obj_syntax {
            values[offset + index].split('/').next().unwrap_or_default()
        } else {
            values[offset + index]
        };
        let parsed = token
            .parse::<usize>()
            .map_err(|error| invalid_file(path, format!("invalid face row '{line}': {error}")))?;
        face[index] = if one_based {
            parsed
                .checked_sub(1)
                .ok_or_else(|| invalid_file(path, "face indices must start at one"))?
        } else {
            parsed
        };
    }
    Ok(face)
}

fn parse_usizes(
    path: &Path,
    line: &str,
    count: usize,
    description: &str,
) -> Result<Vec<usize>, GravityError> {
    let values: Result<Vec<_>, _> = line
        .split_whitespace()
        .take(count)
        .map(str::parse::<usize>)
        .collect();
    let values =
        values.map_err(|error| invalid_file(path, format!("invalid {description}: {error}")))?;
    if values.len() != count {
        return Err(invalid_file(path, format!("invalid {description}")));
    }
    Ok(values)
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
