//! The surface landmarks of upstream `scenarioSmallBodyLandmarks`: the
//! polyhedral shape they are drawn from and the draw itself.
//!
//! | upstream | here |
//! |---|---|
//! | `gravityEffector.loadPolyFromFileToList(path)` (PDS3 `.tab`) | [`PolyhedronShape::from_pds3_tab`] |
//! | the script's `landmark_distribution(...)` | [`landmark_distribution`] |
//! | `np.random.seed(0)`, `np.random.choice(n, k, replace=False)` | [`NumpyRandomState`] |
//!
//! The landmark faces are drawn with numpy's legacy global generator, so the
//! same 100 faces come out only from the same random stream:
//! [`NumpyRandomState`] reproduces numpy's legacy `RandomState` (MT19937
//! seeded with `init_genrand`, and the Fisher-Yates shuffle with
//! `random_interval`'s masked rejection that `permutation`, and therefore
//! `choice` without replacement, uses).

#![allow(dead_code)]

use std::path::{Path, PathBuf};

use nalgebra::Vector3;

/// Upstream's Eros shape, `supportData/LocalGravData/eros007790.tab`
/// (`just fetch-small-body-data`).
pub const EROS_SHAPE: &str = "assets/gravity/eros007790.tab";

/// A triangulated shape model.
#[derive(Clone, Debug)]
pub struct PolyhedronShape {
    /// Vertex positions \[m\], in file order.
    pub vertices_m: Vec<Vector3<f64>>,
    /// Zero-based vertex indices of each face, in file order.
    pub faces: Vec<[usize; 3]>,
}

impl PolyhedronShape {
    /// Upstream `loadPolyFromFileToList` for a PDS3 `.tab` file: `v x y z`
    /// rows in km (returned in m) and `f i j k` rows of zero-based vertex
    /// indices. Upstream shifts the face indices to one-based and the
    /// scenario shifts them back; here they stay zero-based. A relative
    /// `path` is taken from the crate root.
    ///
    /// # Panics
    ///
    /// If the file cannot be read or a row does not parse.
    pub fn from_pds3_tab(path: impl AsRef<Path>) -> Self {
        let path = crate_path(path.as_ref());
        let contents = std::fs::read_to_string(&path).unwrap_or_else(|error| {
            panic!(
                "cannot read {} ({error}); run `just fetch-small-body-data`",
                path.display()
            )
        });
        let mut vertices_m = Vec::new();
        let mut faces = Vec::new();
        for line in contents.lines() {
            let fields: Vec<&str> = line.split_whitespace().collect();
            match fields.first() {
                Some(&"v") => {
                    let km = |index: usize| -> f64 {
                        fields[index]
                            .parse::<f64>()
                            .unwrap_or_else(|_| panic!("bad vertex row {line:?}"))
                    };
                    vertices_m.push(Vector3::new(km(1), km(2), km(3)) * 1e3);
                }
                Some(&"f") => {
                    let index = |column: usize| -> usize {
                        fields[column]
                            .parse::<usize>()
                            .unwrap_or_else(|_| panic!("bad face row {line:?}"))
                    };
                    faces.push([index(1), index(2), index(3)]);
                }
                _ => {}
            }
        }
        Self { vertices_m, faces }
    }
}

fn crate_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}

/// Upstream `landmark_distribution(vert_list, face_list, n_vert, n_face,
/// n_lmk)`: the centers and unit normals (`(p1 - p0) × (p2 - p0)`) of
/// `count` faces drawn without replacement after `np.random.seed(0)`, in
/// increasing face order.
pub fn landmark_distribution(
    shape: &PolyhedronShape,
    count: usize,
) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
    let mut random = NumpyRandomState::seed(0);
    let mut indices = random.choice_without_replacement(shape.faces.len(), count);
    indices.sort_unstable();
    indices
        .into_iter()
        .map(|face| {
            let [p0, p1, p2] = shape.faces[face].map(|vertex| shape.vertices_m[vertex]);
            let center = (p0 + p1 + p2) / 3.0;
            let normal = (p1 - p0).cross(&(p2 - p0)).normalize();
            (center, normal)
        })
        .unzip()
}

/// numpy's legacy `RandomState`, the generator behind `np.random.seed` and
/// the `np.random.*` functions: MT19937 with `init_genrand` seeding.
pub struct NumpyRandomState {
    key: [u32; 624],
    pos: usize,
}

impl NumpyRandomState {
    /// `np.random.seed(seed)` for an integer seed (`mt19937_seed`).
    pub fn seed(seed: u32) -> Self {
        let mut key = [0_u32; 624];
        key[0] = seed;
        for index in 1..624 {
            key[index] = 1_812_433_253_u32
                .wrapping_mul(key[index - 1] ^ (key[index - 1] >> 30))
                .wrapping_add(index as u32);
        }
        Self { key, pos: 624 }
    }

    /// `mt19937_next32`.
    pub fn next_u32(&mut self) -> u32 {
        const UPPER_MASK: u32 = 0x8000_0000;
        const LOWER_MASK: u32 = 0x7fff_ffff;
        const MATRIX_A: u32 = 0x9908_b0df;
        if self.pos == 624 {
            for index in 0..624 {
                let y = (self.key[index] & UPPER_MASK) | (self.key[(index + 1) % 624] & LOWER_MASK);
                let mag = if y & 1 == 1 { MATRIX_A } else { 0 };
                self.key[index] = self.key[(index + 397) % 624] ^ (y >> 1) ^ mag;
            }
            self.pos = 0;
        }
        let mut y = self.key[self.pos];
        self.pos += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^ (y >> 18)
    }

    /// `legacy_double` (`np.random.rand()`): 53 random bits in `[0, 1)`.
    pub fn next_double(&mut self) -> f64 {
        let a = f64::from(self.next_u32() >> 5);
        let b = f64::from(self.next_u32() >> 6);
        (a * 67_108_864.0 + b) / 9_007_199_254_740_992.0
    }

    /// `random_interval(max)`: uniform in `[0, max]` by masked rejection
    /// (`max` below 2^32, so 32-bit draws).
    pub fn random_interval(&mut self, max: u32) -> u32 {
        if max == 0 {
            return 0;
        }
        let mut mask = max;
        for shift in [1, 2, 4, 8, 16] {
            mask |= mask >> shift;
        }
        loop {
            let value = self.next_u32() & mask;
            if value <= max {
                return value;
            }
        }
    }

    /// `np.random.permutation(n)`: `arange(n)` shuffled in place, from the
    /// last element down.
    pub fn permutation(&mut self, n: usize) -> Vec<usize> {
        let mut values: Vec<usize> = (0..n).collect();
        for index in (1..n).rev() {
            let other = self.random_interval(index as u32) as usize;
            values.swap(index, other);
        }
        values
    }

    /// `np.random.choice(n, count, replace=False)` without weights: the first
    /// `count` entries of `permutation(n)`.
    pub fn choice_without_replacement(&mut self, n: usize, count: usize) -> Vec<usize> {
        assert!(
            count <= n,
            "cannot take {count} samples from {n} without replacement"
        );
        let mut values = self.permutation(n);
        values.truncate(count);
        values
    }
}

#[cfg(test)]
mod tests {
    use super::NumpyRandomState;

    /// `np.random.seed(0); np.random.rand()`.
    #[test]
    fn the_stream_matches_numpy_seed_zero() {
        let mut random = NumpyRandomState::seed(0);
        assert_eq!(random.next_double(), 0.548_813_503_927_324_8);
    }

    /// `np.random.seed(0); np.random.permutation(10)`.
    #[test]
    fn the_permutation_matches_numpy_seed_zero() {
        let mut random = NumpyRandomState::seed(0);
        assert_eq!(random.permutation(10), [2, 8, 4, 9, 1, 6, 7, 3, 0, 5]);
    }

    /// The scenario's draw, `np.random.seed(0)`, `np.random.choice(7790,
    /// 100, replace=False)`, sorted (numpy 2.5.5): the first and last faces.
    #[test]
    fn the_landmark_faces_match_numpy() {
        let mut random = NumpyRandomState::seed(0);
        let mut faces = random.choice_without_replacement(7790, 100);
        faces.sort_unstable();
        assert_eq!(
            faces[..10],
            [72, 200, 234, 444, 462, 495, 629, 980, 1003, 1029]
        );
        assert_eq!(faces[95..], [7499, 7542, 7622, 7744, 7775]);
    }
}
