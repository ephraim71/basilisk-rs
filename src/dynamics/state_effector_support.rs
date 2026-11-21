//! Helpers shared by the jointed state effectors (spinning bodies, translating
//! bodies, prescribed motion).

use std::cell::RefCell;

use nalgebra::Vector3;

use crate::spacecraft::BackSubMatrices;

/// Adds one effector's contributions to the spacecraft's.
pub(crate) fn add_back_sub(total: &mut BackSubMatrices, terms: &BackSubMatrices) {
    total.matrix_a += terms.matrix_a;
    total.matrix_b += terms.matrix_b;
    total.matrix_c += terms.matrix_c;
    total.matrix_d += terms.matrix_d;
    total.vec_trans += terms.vec_trans;
    total.vec_rot += terms.vec_rot;
}

pub(crate) fn all_finite(values: &[f64]) -> bool {
    values.iter().all(|value| value.is_finite())
}

/// A joint axis scaled to unit length the way upstream normalizes a user
/// axis: divided by its largest component magnitude first, so a large finite
/// axis cannot overflow the squared norm.
pub(crate) fn normalized_axis(axis: Vector3<f64>) -> Vector3<f64> {
    (axis / axis.amax()).normalize()
}

/// The kinematics of the last state an effector was asked about.
///
/// The spacecraft asks for an effector's mass properties, contributions,
/// derivatives, energy and momentum several times at each state it
/// evaluates, so the effectors keep the geometry of the last one. The entry
/// is keyed on everything the geometry depends on: the state (with locked
/// rates zeroed) or other numbers in `key`, and the parameters `params`.
#[derive(Debug)]
pub(crate) struct KinematicsCache<P, K> {
    entry: RefCell<Option<CacheEntry<P, K>>>,
}

#[derive(Debug)]
struct CacheEntry<P, K> {
    key: Vec<f64>,
    params: P,
    kinematics: K,
}

impl<P, K> Default for KinematicsCache<P, K> {
    fn default() -> Self {
        Self {
            entry: RefCell::new(None),
        }
    }
}

impl<P: Clone + PartialEq, K: Clone> KinematicsCache<P, K> {
    /// The cached kinematics if `key` and `params` match the entry's,
    /// otherwise `compute()`, which then replaces the entry.
    pub(crate) fn get_or_compute(&self, key: &[f64], params: &P, compute: impl FnOnce() -> K) -> K {
        if let Some(entry) = self.entry.borrow().as_ref()
            && entry.key == key
            && entry.params == *params
        {
            return entry.kinematics.clone();
        }
        let kinematics = compute();
        self.entry.replace(Some(CacheEntry {
            key: key.to_vec(),
            params: params.clone(),
            kinematics: kinematics.clone(),
        }));
        kinematics
    }
}

#[cfg(test)]
mod tests {
    use super::KinematicsCache;
    use std::cell::Cell;

    #[test]
    fn cache_recomputes_when_the_key_or_the_parameters_change() {
        let cache = KinematicsCache::<f64, f64>::default();
        let calls = Cell::new(0);
        let compute = |value: f64| {
            calls.set(calls.get() + 1);
            value
        };
        assert_eq!(cache.get_or_compute(&[1.0], &2.0, || compute(3.0)), 3.0);
        assert_eq!(cache.get_or_compute(&[1.0], &2.0, || compute(4.0)), 3.0);
        assert_eq!(calls.get(), 1);
        assert_eq!(cache.get_or_compute(&[1.5], &2.0, || compute(5.0)), 5.0);
        assert_eq!(cache.get_or_compute(&[1.5], &2.5, || compute(6.0)), 6.0);
        assert_eq!(
            cache.get_or_compute(&[f64::NAN], &2.5, || compute(7.0)),
            7.0
        );
        assert_eq!(
            cache.get_or_compute(&[f64::NAN], &2.5, || compute(8.0)),
            8.0
        );
        assert_eq!(calls.get(), 5);
    }
}
