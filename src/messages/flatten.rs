//! Telemetry flattening shared by the message types.
//!
//! A message names its fields once, in column order, through
//! [`impl_telemetry!`]; how each field turns into columns is decided by its
//! type through [`TelemetryValue`], following the path conventions the
//! hand-written messages already use:
//!
//! | field type | columns |
//! |---|---|
//! | `f64`, integers | `name` |
//! | `bool` | `name`, as `1.0` / `0.0` |
//! | `Vector2` / `Vector3` | `name.x`, `name.y` (, `name.z`) |
//! | other column vectors (`SVector<f64, N>`) | `name.0` … `name.{N-1}` |
//! | matrices (`Matrix3`, `SMatrix`) | `name.m11`, `name.m12` … row-major and 1-based; `name.m{r}_{c}` once a dimension exceeds 9 |
//! | `UnitQuaternion` | `name.x`, `name.y`, `name.z`, `name.w` |
//! | `[T; N]` | the element's columns under `name.0` … `name.{N-1}` |
//! | nested message | its own columns under `name.` |
//! | unit-only enum | `name`, as its upstream integer code |

use nalgebra::{SMatrix, UnitQuaternion};

use crate::telemetry::TelemetryField;

/// A value that contributes one or more telemetry columns under a path.
pub(crate) trait TelemetryValue {
    /// Appends this value's columns to `fields`, each path starting with `path`.
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>);
}

/// `name` qualified by its parent's path; a top-level field has no parent.
pub(crate) fn child_path(parent: &str, name: &str) -> String {
    if parent.is_empty() {
        name.to_owned()
    } else {
        format!("{parent}.{name}")
    }
}

/// Appends one column.
pub(crate) fn push_scalar(fields: &mut Vec<TelemetryField>, path: &str, value: f64) {
    fields.push(TelemetryField {
        path: path.to_owned(),
        value,
    });
}

impl TelemetryValue for f64 {
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
        push_scalar(fields, path, *self);
    }
}

impl TelemetryValue for bool {
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
        push_scalar(fields, path, if *self { 1.0 } else { 0.0 });
    }
}

macro_rules! integer_telemetry {
    ($($integer:ty),+) => {
        $(
            impl TelemetryValue for $integer {
                /// Telemetry columns are `f64`; counters and nanosecond time
                /// tags stay exact up to 2^53.
                fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
                    push_scalar(fields, path, *self as f64);
                }
            }
        )+
    };
}

integer_telemetry!(i32, i64, u32, u64, usize);

impl<const R: usize, const C: usize> TelemetryValue for SMatrix<f64, R, C> {
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
        const AXES: [&str; 3] = ["x", "y", "z"];

        if C == 1 && (R == 2 || R == 3) {
            for (axis, value) in AXES.iter().zip(self.iter()) {
                push_scalar(fields, &format!("{path}.{axis}"), *value);
            }
        } else if C == 1 {
            for (index, value) in self.iter().enumerate() {
                push_scalar(fields, &format!("{path}.{index}"), *value);
            }
        } else {
            // `m11` stays unambiguous only while both indices are one digit.
            let separator = if R > 9 || C > 9 { "_" } else { "" };
            for (row, values) in self.row_iter().enumerate() {
                for (column, value) in values.iter().enumerate() {
                    push_scalar(
                        fields,
                        &format!("{path}.m{}{separator}{}", row + 1, column + 1),
                        *value,
                    );
                }
            }
        }
    }
}

impl TelemetryValue for UnitQuaternion<f64> {
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
        let q = self.quaternion();
        for (component, value) in [("x", q.i), ("y", q.j), ("z", q.k), ("w", q.w)] {
            push_scalar(fields, &format!("{path}.{component}"), value);
        }
    }
}

impl<T: TelemetryValue, const N: usize> TelemetryValue for [T; N] {
    fn flatten_into(&self, path: &str, fields: &mut Vec<TelemetryField>) {
        for (index, element) in self.iter().enumerate() {
            element.flatten_into(&format!("{path}.{index}"), fields);
        }
    }
}

/// Implements telemetry for a message from its field list.
///
/// * `impl_telemetry!(Message { field_a, field_b })` implements
///   [`TelemetryValue`] (so the message can nest inside another one) and
///   [`crate::telemetry::TelemetryMessage`], with columns in the listed order.
/// * `impl_telemetry!(enum Flag as u8)` implements [`TelemetryValue`] for a
///   unit-only enum as its integer code.
macro_rules! impl_telemetry {
    (enum $flag:ty as $code:ty) => {
        impl $crate::messages::flatten::TelemetryValue for $flag {
            fn flatten_into(
                &self,
                path: &str,
                fields: &mut Vec<$crate::telemetry::TelemetryField>,
            ) {
                $crate::messages::flatten::push_scalar(fields, path, f64::from(*self as $code));
            }
        }
    };
    ($message:ty { $($field:ident),+ $(,)? }) => {
        impl $crate::messages::flatten::TelemetryValue for $message {
            fn flatten_into(
                &self,
                path: &str,
                fields: &mut Vec<$crate::telemetry::TelemetryField>,
            ) {
                $(
                    $crate::messages::flatten::TelemetryValue::flatten_into(
                        &self.$field,
                        &$crate::messages::flatten::child_path(path, stringify!($field)),
                        fields,
                    );
                )+
            }
        }

        impl $crate::telemetry::TelemetryMessage for $message {
            fn flatten(&self) -> Vec<$crate::telemetry::TelemetryField> {
                let mut fields = Vec::new();
                $crate::messages::flatten::TelemetryValue::flatten_into(self, "", &mut fields);
                fields
            }
        }
    };
}

pub(crate) use impl_telemetry;
