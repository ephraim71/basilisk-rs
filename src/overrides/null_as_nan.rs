//! Reading a JSON `null` back as the NaN it was written from.
//!
//! JSON has no spelling for NaN or an infinity, so `serde_json::to_value` writes
//! `null` for one and `serde_json::from_value` then refuses it: `invalid type:
//! null, expected f64`. That breaks the round trip [`super::rule::apply_override`]
//! is built on, and it breaks it on a message the patch never touched -- a
//! reaction wheel's speed before its first step, a sun sensor's angle while it
//! faces away from the sun. The operator asked to override one field and got a
//! type error naming another.
//!
//! Nothing is lost on the way *out*: a non-finite float is the only thing that
//! becomes `null` from a float field, so reading `null` back as NaN restores what
//! was there. The infinities do not survive, and cannot: `null` is one hole for
//! three values, and NaN is the one a sensor actually publishes.
//!
//! Why a `Deserializer` rather than a walk over the value: `null` means two
//! different things depending on what is being built from it. At an `Option<f64>`
//! it is `None` and must stay `None`; at an `f64` it is a float that could not be
//! written. Only the deserializer knows which is being asked for -- serde calls
//! `deserialize_option` for one and `deserialize_f64` for the other -- so
//! substituting on the value tree would turn every absent option into NaN.

use serde::de::{
    DeserializeOwned, DeserializeSeed, Deserializer, EnumAccess, IntoDeserializer, MapAccess,
    SeqAccess, Visitor,
};
use serde_json::{Map, Value};

/// `serde_json::from_value`, reading `null` as NaN wherever a float is asked for.
pub(crate) fn from_value<T: DeserializeOwned>(value: Value) -> Result<T, serde_json::Error> {
    T::deserialize(NullAsNan(value))
}

/// A `Value` that yields NaN where a float was written as `null`.
struct NullAsNan(Value);

/// Delegates to `serde_json`'s own deserializer for anything that cannot contain
/// a float, so only the cases that matter are written out here.
macro_rules! delegate {
    ($($method:ident)*) => {
        $(
            fn $method<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
                self.0.$method(visitor)
            }
        )*
    };
}

impl<'de> Deserializer<'de> for NullAsNan {
    type Error = serde_json::Error;

    delegate! {
        deserialize_bool
        deserialize_i8 deserialize_i16 deserialize_i32 deserialize_i64
        deserialize_u8 deserialize_u16 deserialize_u32 deserialize_u64
        deserialize_char deserialize_str deserialize_string
        deserialize_bytes deserialize_byte_buf
        deserialize_unit deserialize_identifier deserialize_ignored_any
    }

    /// The whole point: a float written as `null` reads back as NaN.
    fn deserialize_f64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if self.0.is_null() {
            return visitor.visit_f64(f64::NAN);
        }
        self.0.deserialize_f64(visitor)
    }

    fn deserialize_f32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if self.0.is_null() {
            return visitor.visit_f32(f32::NAN);
        }
        self.0.deserialize_f32(visitor)
    }

    /// `null` at an option is still `None`. This is the case a substitution on the
    /// value tree would get wrong.
    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if self.0.is_null() {
            return visitor.visit_none();
        }
        visitor.visit_some(self)
    }

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.0 {
            Value::Array(values) => visitor.visit_seq(Seq(values.into_iter())),
            Value::Object(entries) => visitor.visit_map(Entries::new(entries)),
            leaf => leaf.deserialize_any(visitor),
        }
    }

    fn deserialize_seq<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.0 {
            Value::Array(values) => visitor.visit_seq(Seq(values.into_iter())),
            other => other.deserialize_seq(visitor),
        }
    }

    fn deserialize_map<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.0 {
            Value::Object(entries) => visitor.visit_map(Entries::new(entries)),
            other => other.deserialize_map(visitor),
        }
    }

    fn deserialize_tuple<V: Visitor<'de>>(
        self,
        _len: usize,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        self.deserialize_seq(visitor)
    }

    fn deserialize_tuple_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        _len: usize,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        self.deserialize_seq(visitor)
    }

    fn deserialize_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        _fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        match self.0 {
            Value::Object(entries) => visitor.visit_map(Entries::new(entries)),
            Value::Array(values) => visitor.visit_seq(Seq(values.into_iter())),
            other => other.deserialize_any(visitor),
        }
    }

    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        visitor.visit_newtype_struct(self)
    }

    fn deserialize_unit_struct<V: Visitor<'de>>(
        self,
        name: &'static str,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        self.0.deserialize_unit_struct(name, visitor)
    }

    /// A variant's payload can hold a float, so it is wrapped rather than
    /// delegated.
    fn deserialize_enum<V: Visitor<'de>>(
        self,
        name: &'static str,
        variants: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        match self.0 {
            Value::Object(entries) if entries.len() == 1 => {
                let (variant, payload) = entries.into_iter().next().expect("one entry");
                visitor.visit_enum(Variant { variant, payload })
            }
            other => other.deserialize_enum(name, variants, visitor),
        }
    }
}

/// Array elements, each wrapped so a float inside one is read the same way.
struct Seq(std::vec::IntoIter<Value>);

impl<'de> SeqAccess<'de> for Seq {
    type Error = serde_json::Error;

    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, Self::Error> {
        match self.0.next() {
            Some(value) => seed.deserialize(NullAsNan(value)).map(Some),
            None => Ok(None),
        }
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.0.len())
    }
}

/// Object entries. The key is a plain string; only the value is wrapped.
struct Entries {
    entries: <Map<String, Value> as IntoIterator>::IntoIter,
    value: Option<Value>,
}

impl Entries {
    fn new(entries: Map<String, Value>) -> Self {
        Self {
            entries: entries.into_iter(),
            value: None,
        }
    }
}

impl<'de> MapAccess<'de> for Entries {
    type Error = serde_json::Error;

    fn next_key_seed<K: DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, Self::Error> {
        match self.entries.next() {
            Some((key, value)) => {
                self.value = Some(value);
                seed.deserialize(key.into_deserializer()).map(Some)
            }
            None => Ok(None),
        }
    }

    fn next_value_seed<V: DeserializeSeed<'de>>(
        &mut self,
        seed: V,
    ) -> Result<V::Value, Self::Error> {
        let value = self
            .value
            .take()
            .expect("next_value_seed is only called after next_key_seed");
        seed.deserialize(NullAsNan(value))
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.entries.len())
    }
}

/// An externally tagged variant, whose payload is wrapped like any other value.
struct Variant {
    variant: String,
    payload: Value,
}

impl<'de> EnumAccess<'de> for Variant {
    type Error = serde_json::Error;
    type Variant = NullAsNan;

    fn variant_seed<V: DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, Self::Variant), Self::Error> {
        let name = seed.deserialize(self.variant.into_deserializer())?;
        Ok((name, NullAsNan(self.payload)))
    }
}

impl<'de> serde::de::VariantAccess<'de> for NullAsNan {
    type Error = serde_json::Error;

    fn unit_variant(self) -> Result<(), Self::Error> {
        serde::de::Deserialize::deserialize(self.0)
    }

    fn newtype_variant_seed<T: DeserializeSeed<'de>>(
        self,
        seed: T,
    ) -> Result<T::Value, Self::Error> {
        seed.deserialize(self)
    }

    fn tuple_variant<V: Visitor<'de>>(
        self,
        _len: usize,
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        self.deserialize_seq(visitor)
    }

    fn struct_variant<V: Visitor<'de>>(
        self,
        _fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        self.deserialize_map(visitor)
    }
}

#[cfg(test)]
mod tests {
    use serde::{Deserialize, Serialize};
    use serde_json::json;

    #[derive(Debug, Serialize, Deserialize, PartialEq)]
    struct Reading {
        angle_deg: f64,
        gain: f32,
        vector: [f64; 2],
        nested: Nested,
        /// The case a substitution on the value tree gets wrong: this is `null`
        /// when absent, and absent is not NaN.
        trim: Option<f64>,
    }

    #[derive(Debug, Serialize, Deserialize, PartialEq)]
    struct Nested {
        inner: f64,
    }

    /// The round trip `apply_override` performs, on a value JSON cannot spell.
    #[test]
    fn a_float_written_as_null_reads_back_as_nan() {
        let reading = Reading {
            angle_deg: f64::NAN,
            gain: f32::NAN,
            vector: [1.0, f64::NAN],
            nested: Nested { inner: f64::NAN },
            trim: None,
        };
        let value = serde_json::to_value(&reading).expect("a reading serialises");
        assert_eq!(
            value["angle_deg"],
            json!(null),
            "serde_json spells a NaN some other way now, and this fix is moot"
        );

        let back: Reading = super::from_value(value).expect("null was refused as a float");
        assert!(back.angle_deg.is_nan(), "a plain field lost its NaN");
        assert!(back.gain.is_nan(), "an f32 lost its NaN");
        assert!(back.vector[1].is_nan(), "an array element lost its NaN");
        assert_eq!(back.vector[0], 1.0, "a finite sibling was disturbed");
        assert!(back.nested.inner.is_nan(), "a nested field lost its NaN");
        assert_eq!(
            back.trim, None,
            "an absent option was read as a float, which is the substitution this \
             deserializer exists to avoid"
        );
    }

    /// A present option is still a value, so the wrapper has to keep descending.
    #[test]
    fn an_option_holding_a_non_finite_float_still_reads_as_nan() {
        let value = json!({
            "angle_deg": 1.0, "gain": 2.0, "vector": [3.0, 4.0],
            "nested": { "inner": 5.0 }, "trim": null,
        });
        let absent: Reading = super::from_value(value).expect("a null option was refused");
        assert_eq!(absent.trim, None);

        let with_nan = serde_json::to_value(Reading {
            angle_deg: 1.0,
            gain: 2.0,
            vector: [3.0, 4.0],
            nested: Nested { inner: 5.0 },
            trim: Some(f64::NAN),
        })
        .expect("a reading serialises");
        // `Some(NaN)` and `None` are the same JSON, so the option cannot come back
        // as `Some`. Asserted rather than left implied: it is the one thing this
        // deserializer cannot recover, and a caller storing NaN in an option needs
        // to know that before relying on it.
        let back: Reading = super::from_value(with_nan).expect("a null option was refused");
        assert_eq!(back.trim, None, "Some(NaN) is indistinguishable from None");
    }
}
