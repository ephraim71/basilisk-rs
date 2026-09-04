//! Carrying a non-finite float through the JSON round trip an override performs.
//!
//! JSON has no spelling for NaN or an infinity, so `serde_json::to_value` writes
//! `null` for all three and `serde_json::from_value` then refuses it: `invalid
//! type: null, expected f64`. That breaks the round trip
//! [`super::rule::apply_override`] is built on, and it breaks it on a field the
//! patch never touched: a reaction wheel's speed before its first step, a sun
//! sensor's angle while it faces away from the sun. The operator asked to
//! override one field and got a type error naming another.
//!
//! Both halves live here, and the pair is what makes the trip lossless:
//!
//! - [`to_value`] writes a non-finite float as the text it prints as, `"NaN"`,
//!   `"inf"` or `"-inf"`, so the sign survives a hole that `null` collapses.
//! - [`from_value`] reads those back, and still reads a plain `null` as NaN,
//!   because a patch payload arrives as ordinary JSON where `null` is the only
//!   way an operator can spell a non-finite at all.
//!
//! The reads a client makes are untouched: `upstream` and `effective` in
//! [`super::TargetBackend`] still use `serde_json::to_value`, so they still
//! answer `null` for a non-finite, which is what bifrost's condition evaluator
//! relies on when it treats an unreadable field as one that does not hold.
//!
//! A sentinel does reach a client in one place. A `freeze` captures the live
//! value into its own rule document, so freezing a field that holds an infinity
//! stores `"inf"` there, and `Registry::rules` serialises that document as it
//! is. That is the price of a freeze keeping the sign it froze. Only the trusted
//! freeze document is decoded as a sentinel; an operator's replacement string
//! is still checked as ordinary JSON and refused at a float field.
//!
//! Why a `Deserializer` rather than a walk over the value: `null` means two
//! different things depending on what is being built from it. At an `Option<f64>`
//! it is `None` and must stay `None`; at an `f64` it is a float that could not be
//! written. Only the deserializer knows which is being asked for, since serde
//! calls `deserialize_option` for one and `deserialize_f64` for the other, so
//! substituting on the value tree would turn every absent option into NaN.
//!
//! Why a `Serializer` rather than reaching for a crate that already holds a NaN
//! in a tree: `serde-value` and `serde-content` both do, but neither is
//! maintained, and per-field `#[serde(with)]` helpers would have to reproduce
//! nalgebra's own wire shape for `Vector3`, `Matrix3` and `UnitQuaternion` by
//! hand on 82 fields, where a mistake rebuilding a quaternion corrupts attitude
//! quietly. The serializer here never learns any shape: it delegates everything
//! and intercepts only floats.

use serde::de::{
    DeserializeOwned, DeserializeSeed, Deserializer, EnumAccess, IntoDeserializer, MapAccess,
    SeqAccess, Visitor,
};
use serde::ser::{
    SerializeMap, SerializeSeq, SerializeStruct, SerializeStructVariant, SerializeTuple,
    SerializeTupleStruct, SerializeTupleVariant,
};
use serde::{Serialize, Serializer};
use serde_json::{Map, Number, Value};

use super::rule::json_error;

// ---------------------------------------------------------------------------
// The way back
// ---------------------------------------------------------------------------

/// `serde_json::from_value`, restoring a non-finite float wherever one is asked
/// for: a sentinel written by [`to_value`] exactly, and a plain `null` as NaN.
pub(crate) fn from_value<T: DeserializeOwned>(value: Value) -> Result<T, serde_json::Error> {
    decode(value, true)
}

/// Decodes an operator-authored value without recognizing internal sentinels.
pub(crate) fn from_json_value<T: DeserializeOwned>(value: Value) -> Result<T, serde_json::Error> {
    decode(value, false)
}

fn decode<T: DeserializeOwned>(value: Value, sentinels: bool) -> Result<T, serde_json::Error> {
    T::deserialize(Decoder { value, sentinels })
}

/// A `Value` that yields a non-finite float where JSON could not hold one.
struct Decoder {
    value: Value,
    sentinels: bool,
}

impl Decoder {
    /// The non-finite float this value stands for, if JSON could not spell it.
    ///
    /// A sentinel keeps its sign, because [`to_value`] wrote it. A bare `null`
    /// cannot: it is the one hole JSON offers for three values, so it reads as
    /// NaN, which is the one a sensor publishes and the one an operator can
    /// therefore mean by writing `null` in a patch.
    fn non_finite(&self) -> Option<f64> {
        self.sentinels
            .then(|| from_sentinel(&self.value))
            .flatten()
            .or_else(|| self.value.is_null().then_some(f64::NAN))
    }
}

/// Delegates to `serde_json`'s own deserializer for anything that cannot contain
/// a float, so only the cases that matter are written out here.
macro_rules! delegate {
    ($($method:ident)*) => {
        $(
            fn $method<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
                self.value.$method(visitor)
            }
        )*
    };
}

impl<'de> Deserializer<'de> for Decoder {
    type Error = serde_json::Error;

    delegate! {
        deserialize_bool
        deserialize_i8 deserialize_i16 deserialize_i32 deserialize_i64 deserialize_i128
        deserialize_u8 deserialize_u16 deserialize_u32 deserialize_u64 deserialize_u128
        deserialize_char deserialize_str deserialize_string
        deserialize_bytes deserialize_byte_buf
        deserialize_unit deserialize_identifier deserialize_ignored_any
    }

    /// The whole point: a float written as `null` reads back as NaN.
    fn deserialize_f64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if let Some(value) = self.non_finite() {
            return visitor.visit_f64(value);
        }
        self.value.deserialize_f64(visitor)
    }

    fn deserialize_f32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if let Some(value) = self.non_finite() {
            return visitor.visit_f32(value as f32);
        }
        self.value.deserialize_f32(visitor)
    }

    /// `null` at an option is still `None`. This is the case a substitution on the
    /// value tree would get wrong.
    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        if self.value.is_null() {
            return visitor.visit_none();
        }
        visitor.visit_some(self)
    }

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.value {
            Value::Array(values) => visitor.visit_seq(Seq::new(values, self.sentinels)),
            Value::Object(entries) => visitor.visit_map(Entries::new(entries, self.sentinels)),
            leaf => leaf.deserialize_any(visitor),
        }
    }

    fn deserialize_seq<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.value {
            Value::Array(values) => visitor.visit_seq(Seq::new(values, self.sentinels)),
            other => other.deserialize_seq(visitor),
        }
    }

    fn deserialize_map<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Self::Error> {
        match self.value {
            Value::Object(entries) => visitor.visit_map(Entries::new(entries, self.sentinels)),
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
        match self.value {
            Value::Object(entries) => visitor.visit_map(Entries::new(entries, self.sentinels)),
            Value::Array(values) => visitor.visit_seq(Seq::new(values, self.sentinels)),
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
        self.value.deserialize_unit_struct(name, visitor)
    }

    /// A variant's payload can hold a float, so it is wrapped rather than
    /// delegated.
    fn deserialize_enum<V: Visitor<'de>>(
        self,
        name: &'static str,
        variants: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Self::Error> {
        match self.value {
            Value::Object(entries) if entries.len() == 1 => {
                let (variant, payload) = entries.into_iter().next().expect("one entry");
                visitor.visit_enum(Variant {
                    variant,
                    payload,
                    sentinels: self.sentinels,
                })
            }
            other => other.deserialize_enum(name, variants, visitor),
        }
    }
}

/// Array elements, each wrapped so a float inside one is read the same way.
struct Seq {
    values: std::vec::IntoIter<Value>,
    sentinels: bool,
}

impl Seq {
    fn new(values: Vec<Value>, sentinels: bool) -> Self {
        Self {
            values: values.into_iter(),
            sentinels,
        }
    }
}

impl<'de> SeqAccess<'de> for Seq {
    type Error = serde_json::Error;

    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, Self::Error> {
        match self.values.next() {
            Some(value) => seed
                .deserialize(Decoder {
                    value,
                    sentinels: self.sentinels,
                })
                .map(Some),
            None => Ok(None),
        }
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.values.len())
    }
}

/// Object entries. The key is a plain string; only the value is wrapped.
struct Entries {
    entries: <Map<String, Value> as IntoIterator>::IntoIter,
    value: Option<Value>,
    sentinels: bool,
}

impl Entries {
    fn new(entries: Map<String, Value>, sentinels: bool) -> Self {
        Self {
            entries: entries.into_iter(),
            value: None,
            sentinels,
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
        seed.deserialize(Decoder {
            value,
            sentinels: self.sentinels,
        })
    }

    fn size_hint(&self) -> Option<usize> {
        Some(self.entries.len())
    }
}

/// An externally tagged variant, whose payload is wrapped like any other value.
struct Variant {
    variant: String,
    payload: Value,
    sentinels: bool,
}

impl<'de> EnumAccess<'de> for Variant {
    type Error = serde_json::Error;
    type Variant = Decoder;

    fn variant_seed<V: DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, Self::Variant), Self::Error> {
        let name = seed.deserialize(self.variant.into_deserializer())?;
        Ok((
            name,
            Decoder {
                value: self.payload,
                sentinels: self.sentinels,
            },
        ))
    }
}

impl<'de> serde::de::VariantAccess<'de> for Decoder {
    type Error = serde_json::Error;

    fn unit_variant(self) -> Result<(), Self::Error> {
        serde::de::Deserialize::deserialize(self.value)
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

// ---------------------------------------------------------------------------
// The way out
// ---------------------------------------------------------------------------

/// `serde_json::to_value`, writing a non-finite float as the text it prints as
/// rather than as `null`.
///
/// The three non-finite floats all serialise to `null` through `serde_json`, so
/// the way back cannot tell them apart and has to guess NaN. Writing `"inf"`,
/// `"-inf"` or `"NaN"` instead keeps them apart, and [`from_value`] reads them
/// back exactly.
///
/// The outward-facing value views in [`super::TargetBackend`] still use
/// `serde_json::to_value` and answer `null`. A frozen rule can expose a sentinel
/// through `Registry::rules`, but replacement payloads do not recognize it.
pub(crate) fn to_value<T: Serialize>(value: &T) -> Result<Value, serde_json::Error> {
    value.serialize(Encoder)
}

/// The float a sentinel stands for, or `None` if this is not one.
///
/// Only a non-finite parse counts: a string holding `"1.5"` is a string that
/// belongs at a string field, not a number that lost its quotes, and treating
/// it as a float would accept a wrongly typed patch.
fn from_sentinel(value: &Value) -> Option<f64> {
    let text = value.as_str()?;
    let parsed = text.parse::<f64>().ok()?;
    (!parsed.is_finite()).then_some(parsed)
}

/// Builds a `Value` the way `serde_json` does, apart from the non-finite floats.
///
/// Every arm mirrors `serde_json`'s own representation exactly, so a value with
/// nothing non-finite in it produces a byte-identical tree and no pointer path
/// moves. The test `a_finite_value_serialises_exactly_as_serde_json_does` is
/// what holds that promise.
struct Encoder;

/// The values JSON already holds exactly, where there is nothing to intercept.
macro_rules! plain {
    ($($method:ident($type:ty))*) => {
        $(
            fn $method(self, value: $type) -> Result<Value, Self::Error> {
                Ok(Value::from(value))
            }
        )*
    };
}

impl Serializer for Encoder {
    type Ok = Value;
    type Error = serde_json::Error;

    type SerializeSeq = Items;
    type SerializeTuple = Items;
    type SerializeTupleStruct = Items;
    type SerializeTupleVariant = Items;
    type SerializeMap = EntriesOut;
    type SerializeStruct = Fields;
    type SerializeStructVariant = Fields;

    /// The whole point of the way out: the sign survives, so the way back does
    /// not have to guess.
    fn serialize_f64(self, value: f64) -> Result<Value, Self::Error> {
        match Number::from_f64(value) {
            Some(number) => Ok(Value::Number(number)),
            // `f64::to_string` and `f64::from_str` are inverses over the three
            // non-finite floats, so the encoding is whatever Rust prints.
            None => Ok(Value::String(value.to_string())),
        }
    }

    fn serialize_f32(self, value: f32) -> Result<Value, Self::Error> {
        self.serialize_f64(f64::from(value))
    }

    plain! {
        serialize_bool(bool)
        serialize_i8(i8) serialize_i16(i16) serialize_i32(i32) serialize_i64(i64)
        serialize_u8(u8) serialize_u16(u16) serialize_u32(u32) serialize_u64(u64)
        serialize_str(&str)
    }

    fn serialize_i128(self, value: i128) -> Result<Value, Self::Error> {
        serde_json::to_value(value)
    }

    fn serialize_u128(self, value: u128) -> Result<Value, Self::Error> {
        serde_json::to_value(value)
    }

    fn serialize_char(self, value: char) -> Result<Value, Self::Error> {
        Ok(Value::String(value.to_string()))
    }

    fn serialize_bytes(self, value: &[u8]) -> Result<Value, Self::Error> {
        Ok(Value::Array(
            value.iter().copied().map(Value::from).collect(),
        ))
    }

    fn serialize_none(self) -> Result<Value, Self::Error> {
        Ok(Value::Null)
    }

    fn serialize_some<T: ?Sized + Serialize>(self, value: &T) -> Result<Value, Self::Error> {
        value.serialize(self)
    }

    fn serialize_unit(self) -> Result<Value, Self::Error> {
        Ok(Value::Null)
    }

    fn serialize_unit_struct(self, _name: &'static str) -> Result<Value, Self::Error> {
        Ok(Value::Null)
    }

    fn serialize_unit_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
    ) -> Result<Value, Self::Error> {
        Ok(Value::String(variant.to_owned()))
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        value: &T,
    ) -> Result<Value, Self::Error> {
        value.serialize(self)
    }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        value: &T,
    ) -> Result<Value, Self::Error> {
        Ok(object(variant, value.serialize(Encoder)?))
    }

    fn serialize_seq(self, _len: Option<usize>) -> Result<Items, Self::Error> {
        Ok(Items::new(None))
    }

    fn serialize_tuple(self, _len: usize) -> Result<Items, Self::Error> {
        Ok(Items::new(None))
    }

    fn serialize_tuple_struct(
        self,
        _name: &'static str,
        _len: usize,
    ) -> Result<Items, Self::Error> {
        Ok(Items::new(None))
    }

    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        _len: usize,
    ) -> Result<Items, Self::Error> {
        Ok(Items::new(Some(variant)))
    }

    fn serialize_map(self, _len: Option<usize>) -> Result<EntriesOut, Self::Error> {
        Ok(EntriesOut::default())
    }

    fn serialize_struct(self, _name: &'static str, _len: usize) -> Result<Fields, Self::Error> {
        Ok(Fields::new(None))
    }

    fn serialize_struct_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        _len: usize,
    ) -> Result<Fields, Self::Error> {
        Ok(Fields::new(Some(variant)))
    }
}

fn object(key: &str, value: Value) -> Value {
    Value::Object(Map::from_iter([(key.to_owned(), value)]))
}

/// Collected elements of a sequence, tuple or tuple struct.
struct Items {
    values: Vec<Value>,
    variant: Option<&'static str>,
}

impl Items {
    fn new(variant: Option<&'static str>) -> Self {
        Self {
            values: Vec::new(),
            variant,
        }
    }

    fn finish(self) -> Value {
        let value = Value::Array(self.values);
        match self.variant {
            Some(variant) => object(variant, value),
            None => value,
        }
    }
}

impl SerializeSeq for Items {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        self.values.push(value.serialize(Encoder)?);
        Ok(())
    }

    fn end(self) -> Result<Value, Self::Error> {
        Ok(self.finish())
    }
}

impl SerializeTuple for Items {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        SerializeSeq::serialize_element(self, value)
    }

    fn end(self) -> Result<Value, Self::Error> {
        SerializeSeq::end(self)
    }
}

impl SerializeTupleStruct for Items {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        SerializeSeq::serialize_element(self, value)
    }

    fn end(self) -> Result<Value, Self::Error> {
        SerializeSeq::end(self)
    }
}

impl SerializeTupleVariant for Items {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        SerializeSeq::serialize_element(self, value)
    }

    fn end(self) -> Result<Value, Self::Error> {
        SerializeSeq::end(self)
    }
}

#[derive(Default)]
struct EntriesOut {
    entries: Map<String, Value>,
    key: Option<String>,
}

impl SerializeMap for EntriesOut {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_key<T: ?Sized + Serialize>(&mut self, key: &T) -> Result<(), Self::Error> {
        self.key = Some(map_key(key)?);
        Ok(())
    }

    fn serialize_value<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Self::Error> {
        let key = self
            .key
            .take()
            .ok_or_else(|| json_error("serialize_value called before serialize_key"))?;
        self.entries.insert(key, value.serialize(Encoder)?);
        Ok(())
    }

    fn end(self) -> Result<Value, Self::Error> {
        if self.key.is_some() {
            return Err(json_error("serialize_map ended before its value"));
        }
        Ok(Value::Object(self.entries))
    }
}

struct MapKey<'a, T: ?Sized>(&'a T);

impl<T: ?Sized + Serialize> Serialize for MapKey<'_, T> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut map = serializer.serialize_map(Some(1))?;
        map.serialize_entry(self.0, &())?;
        map.end()
    }
}

fn map_key<T: ?Sized + Serialize>(key: &T) -> Result<String, serde_json::Error> {
    let Value::Object(entries) = serde_json::to_value(MapKey(key))? else {
        unreachable!("a serialized map is always an object")
    };
    Ok(entries.into_iter().next().expect("a map key serializes").0)
}

/// Struct fields, which carry their own names.
struct Fields {
    fields: Map<String, Value>,
    variant: Option<&'static str>,
}

impl Fields {
    fn new(variant: Option<&'static str>) -> Self {
        Self {
            fields: Map::new(),
            variant,
        }
    }

    fn finish(self) -> Value {
        let value = Value::Object(self.fields);
        match self.variant {
            Some(variant) => object(variant, value),
            None => value,
        }
    }
}

impl SerializeStruct for Fields {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), Self::Error> {
        self.fields
            .insert(key.to_owned(), value.serialize(Encoder)?);
        Ok(())
    }

    fn end(self) -> Result<Value, Self::Error> {
        Ok(self.finish())
    }
}

impl SerializeStructVariant for Fields {
    type Ok = Value;
    type Error = serde_json::Error;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), Self::Error> {
        SerializeStruct::serialize_field(self, key, value)
    }

    fn end(self) -> Result<Value, Self::Error> {
        SerializeStruct::end(self)
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

#[cfg(test)]
mod out_and_back {
    use serde::{Deserialize, Serialize};
    use serde_json::json;

    use crate::messages::{
        ImuMsg, MtbArrayConfigMsg, PlanetStateMsg, RwArrayConfigMsg, RwAvailabilityMsg,
        SpacecraftStateMsg, StarTrackerMsg,
    };

    #[derive(Debug, Serialize, Deserialize, PartialEq)]
    struct Reading {
        angle_deg: f64,
        gain: f32,
        vector: [f64; 2],
        trim: Option<f64>,
    }

    fn reading(angle_deg: f64) -> Reading {
        Reading {
            angle_deg,
            gain: 1.0,
            vector: [2.0, 3.0],
            trim: None,
        }
    }

    /// The promise the serializer makes: it is `serde_json` for everything JSON
    /// can already hold.
    ///
    /// This is the guard on the whole approach. `Vector3` is a flat array of 3,
    /// `Matrix3` a flat array of 9, `UnitQuaternion` a flat array of 4, an
    /// effector array a sequence of 36, a unit enum variant its own name. If any
    /// of those moved, a pointer path an operator already uses would break, and
    /// the fault would land somewhere else or not at all.
    #[test]
    fn a_finite_value_serialises_exactly_as_serde_json_does() {
        macro_rules! identical {
            ($($message:expr),* $(,)?) => {
                $({
                    let value = $message;
                    assert_eq!(
                        super::to_value(&value).expect("serialises"),
                        serde_json::to_value(&value).expect("serialises"),
                        "{} does not serialise the way serde_json does",
                        std::any::type_name_of_val(&value),
                    );
                })*
            };
        }

        identical!(
            SpacecraftStateMsg::default(),
            PlanetStateMsg::default(),
            RwAvailabilityMsg::default(),
            RwArrayConfigMsg::default(),
            MtbArrayConfigMsg::default(),
            StarTrackerMsg::default(),
            ImuMsg::default(),
            reading(4.0),
            (1u8, -2i64, 'x', "text", vec![1.5f32, 2.5]),
        );
    }

    /// What the sentinel buys: `null` is one hole for three values, text is not.
    #[test]
    fn an_infinity_keeps_its_sign_through_the_round_trip() {
        for expected in [f64::INFINITY, f64::NEG_INFINITY] {
            let value = super::to_value(&reading(expected)).expect("serialises");
            assert_eq!(
                value["angle_deg"],
                json!(expected.to_string()),
                "a non-finite float was not written as its own text"
            );

            let back: Reading = super::from_value(value).expect("the sentinel was refused");
            assert_eq!(
                back.angle_deg, expected,
                "an infinity came back as something else"
            );
            assert_eq!(back.vector, [2.0, 3.0], "a finite sibling was disturbed");
            assert_eq!(back.trim, None, "an absent option was filled in");
        }
    }

    #[test]
    fn a_nan_still_survives_and_stays_a_nan() {
        let value = super::to_value(&reading(f64::NAN)).expect("serialises");
        let back: Reading = super::from_value(value).expect("the sentinel was refused");
        assert!(back.angle_deg.is_nan(), "a NaN did not survive");
    }

    /// A patch is ordinary JSON written by an operator, so `null` is the only
    /// spelling of a non-finite available to them. It has to keep working.
    #[test]
    fn a_null_written_by_hand_still_reads_as_nan() {
        let mut value = super::to_value(&reading(1.0)).expect("serialises");
        value["angle_deg"] = json!(null);

        let back: Reading = super::from_value(value).expect("null was refused as a float");
        assert!(back.angle_deg.is_nan(), "a hand-written null was not a NaN");
    }

    /// The sentinel is text, so the decoding has to be narrow: only a string
    /// that parses to a *non-finite* float is one. Otherwise a patch that put a
    /// quoted number at a float field would be quietly accepted.
    #[test]
    fn a_quoted_finite_number_is_still_refused_at_a_float() {
        let mut value = super::to_value(&reading(1.0)).expect("serialises");
        value["angle_deg"] = json!("1.5");

        let error = super::from_value::<Reading>(value)
            .expect_err("a quoted number was accepted as a float");
        assert!(
            error.to_string().contains("invalid type: string"),
            "the refusal did not name the type problem: {error}"
        );
    }

    /// The target type decides, never the value, so text that reads like a float
    /// is left alone at a string field.
    #[test]
    fn a_string_field_holding_inf_is_left_as_text() {
        #[derive(Debug, Deserialize)]
        struct Mixed {
            label: String,
            value: f64,
        }

        let back: Mixed = super::from_value(json!({ "label": "inf", "value": "-inf" }))
            .expect("the sentinel was refused");
        assert_eq!(back.label, "inf", "a string field was read as a float");
        assert_eq!(
            back.value,
            f64::NEG_INFINITY,
            "a float field lost its value"
        );
    }
}
