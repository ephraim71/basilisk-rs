//! Build script. It generates the Vizard protobuf types for the `vizard`
//! feature and records where the MuJoCo library is for the `mujoco` feature;
//! without those features it does nothing, so a default build needs neither
//! `protoc`, libzmq nor MuJoCo.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(feature = "vizard")]
    vizard::generate_protobuf_types();
    #[cfg(feature = "mujoco")]
    mujoco::link_runtime_path();
}

#[cfg(feature = "mujoco")]
mod mujoco {
    /// MuJoCo is a shared library outside the system's search path, so the
    /// crate's tests, examples and binaries record its directory (which
    /// mujoco-rs reports as `DEP_MUJOCO_LIB_DIR`) as a runtime search path.
    pub fn link_runtime_path() {
        println!("cargo:rerun-if-env-changed=DEP_MUJOCO_LIB_DIR");
        if let Ok(lib_dir) = std::env::var("DEP_MUJOCO_LIB_DIR") {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
        }
    }
}

#[cfg(feature = "vizard")]
mod vizard {
    /// Vizard's wire format, upstream `utilities/vizProtobuffer/vizMessage.proto`.
    const PROTO: &str = "src/vizard/vizMessage.proto";

    /// Fields whose proto comments start with a commented-out field. prost
    /// turns those comments into doc comments, where the indented line reads
    /// as a Rust code block that `cargo test` would try to compile.
    const FIELDS_WITHOUT_DOCS: [&str; 2] = [
        ".vizProtobufferMessage.VizMessage.StandardCameraSettings.setMode",
        ".vizProtobufferMessage.VizMessage.VizSettingsPb.instrumentSettings",
    ];

    /// Writes `viz_protobuffer_message.rs` (the proto package, snake-cased) to
    /// `OUT_DIR`, where `src/vizard.rs` includes it.
    ///
    /// prost-build runs `protoc`, found through the `PROTOC` environment
    /// variable or on `PATH`.
    pub fn generate_protobuf_types() {
        println!("cargo:rerun-if-changed={PROTO}");
        println!("cargo:rerun-if-env-changed=PROTOC");
        let generated = prost_build::Config::new()
            .disable_comments(FIELDS_WITHOUT_DOCS)
            .compile_protos(&[PROTO], &["src/vizard"]);
        if let Err(error) = generated {
            panic!(
                "the `vizard` feature generates its protobuf types with protoc \
                 (set PROTOC or put protoc on PATH): {error}"
            );
        }
    }
}
