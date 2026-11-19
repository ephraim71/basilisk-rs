//! Protobuffer mapping of [`VizInterface`](super::VizInterface): the scene
//! description and the input messages, as Vizard's `vizMessage.proto` types.
//!
//! This is where the units change to the ones Vizard reads: angles to
//! degrees (a coarse sun sensor's boresight-to-edge field of view to an
//! edge-to-edge one), a celestial body's gravitational parameter and radius
//! to km^3/s^2 and km, and a camera's focal length to mm. Out-of-range
//! settings are passed through with the warnings upstream logs.

use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    CameraConfigMsg, PlanetStateMsg, SpacecraftStateMsg, VizEventReply, VizUserInputMsg,
};
use crate::utilities::astro_constants::R2D;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::add_mrp;
use crate::vizard::proto::{self, viz_message as pb};
use crate::vizard::viz_structures::{
    GravBodyInfo, LiveVizSettings, Location, QuadMap, Rgba, VizEventDialog, VizSetting,
    VizSettings, VizSpacecraft,
};

/// Smallest camera position \[m\] sent for a standard camera; a shorter one
/// is left out so Vizard places the camera itself (upstream's `1e-5`).
const STD_CAMERA_POSITION_EPSILON_M: f64 = 1.0e-5;

fn vector(v: &Vector3<f64>) -> Vec<f64> {
    v.iter().copied().collect()
}

fn rgba(color: &Rgba) -> Vec<i32> {
    color.iter().map(|channel| i32::from(*channel)).collect()
}

/// Empty for `None`: upstream sends no entries for an unset color vector.
fn optional_rgba(color: &Option<Rgba>) -> Vec<i32> {
    color.as_ref().map(rgba).unwrap_or_default()
}

fn rgba_list(colors: &[Rgba]) -> Vec<i32> {
    colors.iter().flat_map(rgba).collect()
}

fn setting(value: VizSetting) -> i32 {
    value as i32
}

/// Upstream `std::lround(angle * R2D)`.
fn whole_degrees(angle_rad: f64) -> i32 {
    (angle_rad * R2D).round() as i32
}

/// The DCM's nine entries, row by row, as upstream walks `J20002Pfix[i][j]`.
fn row_major(dcm: &Matrix3<f64>) -> Vec<f64> {
    (0..3)
        .flat_map(|row| (0..3).map(move |column| dcm[(row, column)]))
        .collect()
}

/// Upstream's range warnings for the settings whose type still admits an
/// out-of-range value. The -1/0/1 switches upstream also checks are
/// [`VizSetting`]s, which cannot be out of range.
fn warn_on_out_of_range(settings: &VizSettings) {
    if settings.ambient > 8.0 || (settings.ambient < 0.0 && settings.ambient != -1.0) {
        log::warn!(
            "vizInterface: The Vizard ambient light value must be within [0,8].  A value of {} was received.",
            settings.ambient
        );
    }
    if settings.orbit_lines_on.abs() > 2 {
        log::warn!(
            "vizInterface: The Vizard orbitLinesOn flag must be either -1, 0, 1 or 2.  A value of {} was received.",
            settings.orbit_lines_on
        );
    }
    if settings.true_trajectory_lines_on.abs() > 5 {
        log::warn!(
            "vizInterface: The Vizard trueTrajectoryLinesOn flag must be either -1, 0, 1 2, 3, 4 or 5.  A value of {} was received.",
            settings.true_trajectory_lines_on
        );
    }
    if settings.custom_gui_reference_height_px < 300.0
        && settings.custom_gui_reference_height_px.abs() > 1.0
    {
        log::warn!(
            "vizInterface: The Vizard customGUIReferenceHeight flag must be either -1 or > 300.  A value of {} was received.",
            settings.custom_gui_reference_height_px
        );
    }
}

/// Upstream `WriteProtobuffer`'s `VizSettingsPb` block.
pub(super) fn settings(settings: &VizSettings) -> pb::VizSettingsPb {
    warn_on_out_of_range(settings);
    pb::VizSettingsPb {
        ambient: settings.ambient,
        orbit_lines_on: settings.orbit_lines_on,
        true_trajectory_lines_on: settings.true_trajectory_lines_on,
        spacecraft_c_son: setting(settings.spacecraft_cs_on),
        planet_c_son: setting(settings.planet_cs_on),
        skybox: settings.sky_box.clone(),
        point_lines: settings
            .point_line_list
            .iter()
            .map(|line| pb::PointLine {
                from_body_name: line.from_body_name.clone(),
                to_body_name: line.to_body_name.clone(),
                line_color: rgba(&line.line_color),
            })
            .collect(),
        keep_out_in_cones: settings
            .cone_list
            .iter()
            .map(|cone| pb::KeepOutInCone {
                is_keep_in: cone.is_keep_in,
                position: vector(&cone.position_b_m),
                normal_vector: vector(&cone.normal_vector_b),
                incidence_angle: cone.incidence_angle_rad * R2D,
                cone_height: cone.cone_height_m,
                from_body_name: cone.from_body_name.clone(),
                to_body_name: cone.to_body_name.clone(),
                cone_color: rgba(&cone.cone_color),
                cone_name: cone.cone_name.clone(),
            })
            .collect(),
        view_camera_boresight_hud: setting(settings.view_camera_boresight_hud),
        view_camera_frustum_hud: setting(settings.view_camera_frustum_hud),
        view_camera_view_hud: setting(settings.view_camera_view_hud),
        show_cs_labels: setting(settings.show_cs_labels),
        show_celestial_body_labels: setting(settings.show_celestial_body_labels),
        show_spacecraft_labels: setting(settings.show_spacecraft_labels),
        show_camera_labels: setting(settings.show_camera_labels),
        custom_gui_reference_height: settings.custom_gui_reference_height_px,
        default_spacecraft_sprite: settings.default_spacecraft_sprite.clone(),
        show_spacecraft_as_sprites: setting(settings.show_spacecraft_as_sprites),
        show_celestial_bodies_as_sprites: setting(settings.show_celestial_bodies_as_sprites),
        show24hr_clock: setting(settings.show_24hr_clock),
        show_data_rate_display: setting(settings.show_data_rate_display),
        keyboard_angular_rate: settings.keyboard_angular_rate_radps * R2D,
        keyboard_zoom_rate: settings.keyboard_zoom_rate,
        default_thruster_color: optional_rgba(&settings.default_thruster_color),
        default_thruster_plume_life_scalar: settings.default_thruster_plume_life_scalar,
        orbit_line_segments: settings.orbit_line_segments,
        relative_orbit_range: settings.relative_orbit_range_deg,
        show_hill_frame: setting(settings.show_hill_frame),
        show_velocity_frame: setting(settings.show_velocity_frame),
        relative_orbit_frame: settings.relative_orbit_frame,
        main_camera_target: settings.main_camera_target.clone(),
        spacecraft_shadow_brightness: settings.spacecraft_shadow_brightness,
        spacecraft_size_multiplier: settings.spacecraft_size_multiplier,
        spacecraft_helio_view_size_multiplier: settings.spacecraft_helio_view_size_multiplier,
        force_start_at_spacecraft_local_view: setting(
            settings.force_start_at_spacecraft_local_view,
        ),
        use_simple_location_markers: setting(settings.use_simple_location_markers),
        show_location_comm_lines: setting(settings.show_location_comm_lines),
        show_location_cones: setting(settings.show_location_cones),
        show_location_labels: setting(settings.show_location_labels),
        atmospheres_off: setting(settings.atmospheres_off),
        sc_view_to_planet_view_boundary_multiplier: settings
            .sc_view_to_planet_view_boundary_multiplier,
        planet_view_to_helio_view_boundary_multiplier: settings
            .planet_view_to_helio_view_boundary_multiplier,
        sun_intensity: settings.sun_intensity,
        attenuate_sun_light_with_distance: setting(settings.attenuate_sun_light_with_distance),
        show_light_labels: setting(settings.show_light_labels),
        celestial_body_helio_view_size_multiplier: settings
            .celestial_body_helio_view_size_multiplier,
        show_mission_time: setting(settings.show_mission_time),
        keyboard_live_input: settings.keyboard_live_input.clone(),
        message_buffer_size: settings.message_buffer_size_bytes,
        true_path_relative_body: settings.true_path_relative_body.clone(),
        true_path_rotating_frame: settings.true_path_rotating_frame.clone(),
        true_path_fixed_frame: settings.true_path_fixed_frame.clone(),
        show_quad_map_labels: setting(settings.show_quad_map_labels),
        spacecraft_orbit_line_width: settings.spacecraft_orbit_line_width,
        celestial_body_orbit_line_width: settings.celestial_body_orbit_line_width,
        lines_and_frames_line_width: settings.lines_and_frames_line_width,
        use_line_renderers_for_target_lines_and_frames: setting(
            settings.use_line_renderers_for_target_lines_and_frames,
        ),
        osculating_orbit_line_range: settings
            .osculating_orbit_line_range_rad
            .iter()
            .flatten()
            .map(|angle_rad| whole_degrees(*angle_rad))
            .collect(),
        osculating_ground_track_range: settings
            .osculating_ground_track_range_rad
            .iter()
            .flatten()
            .map(|angle_rad| whole_degrees(*angle_rad))
            .collect(),
        show_osculating_ground_track_lines: setting(settings.show_osculating_ground_track_lines),
        show_true_path_ground_track_lines: setting(settings.show_true_path_ground_track_lines),
        actuator_settings: settings
            .actuator_gui_settings_list
            .iter()
            .map(|actuator| pb::ActuatorSettings {
                spacecraft_name: actuator.spacecraft_name.clone(),
                view_thruster_panel: setting(actuator.view_thruster_panel),
                view_thruster_hud: setting(actuator.view_thruster_hud),
                view_rw_panel: setting(actuator.view_rw_panel),
                view_rwhud: setting(actuator.view_rw_hud),
                show_thruster_labels: setting(actuator.show_thruster_labels),
                show_rw_labels: setting(actuator.show_rw_labels),
            })
            .collect(),
        instrument_settings: settings
            .instrument_gui_settings_list
            .iter()
            .map(|instrument| pb::InstrumentSettings {
                spacecraft_name: instrument.spacecraft_name.clone(),
                view_css_panel: setting(instrument.view_css_panel),
                view_css_coverage: setting(instrument.view_css_coverage),
                view_css_boresight: setting(instrument.view_css_boresight),
                show_css_labels: setting(instrument.show_css_labels),
                show_generic_sensor_labels: setting(instrument.show_generic_sensor_labels),
                show_transceiver_labels: setting(instrument.show_transceiver_labels),
                show_transceiver_frustum: setting(instrument.show_transceiver_frustum),
                show_generic_storage_panel: setting(instrument.show_generic_storage_panel),
                show_multi_shape_labels: setting(instrument.show_multi_shape_labels),
            })
            .collect(),
        custom_models: settings
            .custom_model_list
            .iter()
            .map(|model| pb::CustomModel {
                model_path: model.model_path.clone(),
                sim_bodies_to_modify: model.sim_bodies_to_modify.clone(),
                offset: vector(&model.offset_m),
                rotation: vector(&(model.rotation_rad * R2D)),
                scale: vector(&model.scale),
                custom_texture_path: model.custom_texture_path.clone(),
                normal_map_path: model.normal_map_path.clone(),
                shader: model.shader,
                color: optional_rgba(&model.color),
            })
            .collect(),
        standard_camera_settings: settings
            .std_camera_list
            .iter()
            .map(|camera| pb::StandardCameraSettings {
                spacecraft_name: camera.spacecraft_name.clone(),
                set_mode: camera.set_mode as i32,
                show_hud_elements_in_image: setting(camera.show_hud_elements_in_image),
                field_of_view: camera
                    .field_of_view_rad
                    .filter(|fov_rad| *fov_rad >= 0.0)
                    .map_or(-1.0, |fov_rad| fov_rad * R2D),
                body_target: camera.body_target.clone(),
                set_view: camera.set_view as i32,
                pointing_vector: vector(&camera.pointing_vector_b),
                position: if camera.position_b_m.norm() > STD_CAMERA_POSITION_EPSILON_M {
                    vector(&camera.position_b_m)
                } else {
                    Vec::new()
                },
                display_name: camera.display_name.clone(),
            })
            .collect(),
    }
}

/// Upstream `WriteProtobuffer`'s `LiveVizSettingsPb` block.
pub(super) fn live_settings(live: &LiveVizSettings) -> pb::LiveVizSettingsPb {
    pb::LiveVizSettingsPb {
        target_lines: live
            .target_line_list
            .iter()
            .map(|line| pb::PointLine {
                from_body_name: line.from_body_name.clone(),
                to_body_name: line.to_body_name.clone(),
                line_color: rgba(&line.line_color),
            })
            .collect(),
        relative_orbit_chief: live.relative_orbit_chief.clone(),
        terminate_vizard: live.terminate_vizard,
        playback_paused: live.playback_paused,
        playback_in_real_time: live.playback_in_real_time,
        playback_multiplier: live.playback_multiplier,
    }
}

pub(super) fn event_dialog(dialog: &VizEventDialog) -> proto::VizEventDialog {
    proto::VizEventDialog {
        event_handler_id: dialog.event_handler_id.clone(),
        display_string: dialog.display_string.clone(),
        user_options: dialog.user_options.clone(),
        force_event_dialog_maximize: false,
        duration_of_display: dialog.duration_of_display_s,
        use_sim_elapsed_time_for_duration: dialog.use_sim_elapsed_time_for_duration,
        use_confirmation_panel: setting(dialog.use_confirmation_panel),
        hide_on_selection: dialog.hide_on_selection,
        dialog_format: dialog.dialog_format.clone(),
    }
}

/// The time stamp of frame `frame_number` at `sim_nanos`; Vizard takes the
/// elapsed time in nanoseconds, as a `double`.
pub(super) fn time_stamp(frame_number: i64, sim_nanos: u64) -> pb::TimeStamp {
    pb::TimeStamp {
        frame_number,
        sim_time_elapsed: sim_nanos as f64,
    }
}

/// Upstream's `EpochMsgPayload` fields: the UTC calendar date of `epoch`.
pub(super) fn epoch(epoch: Epoch) -> pb::EpochDateTime {
    let (year, month, day, hours, minutes, seconds, nanos) = epoch.to_gregorian_utc();
    pb::EpochDateTime {
        year,
        month: i32::from(month),
        day: i32::from(day),
        hours: i32::from(hours),
        minutes: i32::from(minutes),
        seconds: f64::from(seconds) + f64::from(nanos) * 1.0e-9,
    }
}

pub(super) fn location(location: &Location) -> pb::Location {
    pb::Location {
        station_name: location.station_name.clone(),
        parent_body_name: location.parent_body_name.clone(),
        r_gp_p: vector(&location.r_gp_p_m),
        g_hat_p: vector(&location.g_hat_p),
        field_of_view: location.field_of_view_rad * R2D,
        // Upstream always sends its four entries; unset, they are its `{-1}`.
        color: location
            .color
            .as_ref()
            .map_or_else(|| vec![-1, 0, 0, 0], rgba),
        range: location.range_m,
        marker_scale: location.marker_scale,
        is_hidden: location.is_hidden,
        label: location.label.clone(),
    }
}

pub(super) fn quad_map(quad_map: &QuadMap) -> pb::QuadMap {
    pb::QuadMap {
        id: quad_map.id,
        parent_body_name: quad_map.parent_body_name.clone(),
        vertices: quad_map.vertices_m.iter().flat_map(vector).collect(),
        color: optional_rgba(&quad_map.color),
        is_hidden: quad_map.is_hidden,
        label: quad_map.label.clone(),
    }
}

/// Upstream's `Spacecraft` block for one spacecraft whose state input is
/// connected, with `state` read from it. Devices whose input is unconnected
/// are left out; the others are read here.
pub(super) fn spacecraft(sc: &VizSpacecraft, state: &SpacecraftStateMsg) -> pb::Spacecraft {
    let reaction_wheels = sc
        .reaction_wheels
        .iter()
        .filter(|wheel| wheel.rw_state_in_msg.is_connected())
        .map(|wheel| {
            let rw_state = wheel.rw_state_in_msg.read();
            let rw_config = wheel.rw_config_in_msg.read();
            pb::ReactionWheel {
                position: vector(&rw_config.position_m),
                // The unit spin axis the wheel effector works with.
                spin_axis_vector: vector(&normalize_or_zero(rw_config.spin_axis_body)),
                wheel_speed: rw_state.omega_radps,
                wheel_torque: rw_state.applied_motor_torque_nm,
                max_speed: rw_config.max_speed_radps,
                max_torque: rw_config.max_torque_nm,
            }
        })
        .collect();

    let thrusters = sc
        .thrusters
        .iter()
        .filter(|thruster| thruster.thr_in_msg.is_connected())
        .map(|thruster| {
            let output = thruster.thr_in_msg.read();
            pb::Thruster {
                position: vector(&(output.thruster_location_m - thruster.thr_info.thr_offset_m)),
                thrust_vector: vector(&output.thruster_direction),
                current_thrust: output.thrust_force_n,
                max_thrust: output.max_thrust_n,
                thruster_tag: thruster.thr_info.thr_tag.clone(),
                color: optional_rgba(&thruster.thr_info.color),
            }
        })
        .collect();

    let css = sc
        .coarse_sun_sensors
        .iter()
        .filter(|sensor| sensor.css_signal_in_msg.is_connected())
        .map(|sensor| pb::CoarseSunSensor {
            position: vector(&sensor.r_b_m),
            // Vizard draws the edge-to-edge field of view.
            field_of_view: sensor.fov_rad * 2.0 * R2D,
            normal_vector: vector(&sensor.n_hat_b),
            current_msmt: sensor.css_signal_in_msg.read().sensed_value,
            max_msmt: sensor.max_signal,
            css_group_id: sensor.css_group_id,
            min_msmt: sensor.min_signal,
        })
        .collect();

    pb::Spacecraft {
        spacecraft_name: sc.spacecraft_name.clone(),
        parent_spacecraft_name: sc.parent_spacecraft_name.clone(),
        position: vector(&state.position_m),
        velocity: vector(&state.velocity_mps),
        rotation: vector(&state.sigma_bn),
        spacecraft_sprite: sc.spacecraft_sprite.clone(),
        reaction_wheels,
        thrusters,
        css,
        generic_sensors: sc
            .generic_sensors
            .iter()
            .map(|sensor| pb::GenericSensor {
                position: vector(&sensor.r_sb_b_m),
                field_of_view: sensor
                    .field_of_view_rad
                    .iter()
                    .map(|fov_rad| fov_rad * R2D)
                    .collect(),
                normal_vector: vector(&sensor.normal_vector),
                is_hidden: sensor.is_hidden,
                // Upstream `(int)genericSensorCmd`.
                activity_status: sensor.generic_sensor_cmd as i32,
                size: sensor.size_m,
                color: rgba_list(&sensor.color),
                label: sensor.label.clone(),
            })
            .collect(),
        ellipsoids: sc
            .ellipsoids
            .iter()
            .map(|ellipsoid| pb::Ellipsoid {
                is_on: setting(ellipsoid.is_on),
                use_body_frame: setting(ellipsoid.use_body_frame),
                position: vector(&ellipsoid.position_m),
                semi_major_axes: vector(&ellipsoid.semi_major_axes_m),
                color: optional_rgba(&ellipsoid.color),
                show_grid_lines: setting(ellipsoid.show_grid_lines),
            })
            .collect(),
        transceivers: sc
            .transceivers
            .iter()
            .map(|transceiver| pb::Transceiver {
                position: vector(&transceiver.r_sb_b_m),
                field_of_view: transceiver.field_of_view_rad * R2D,
                normal_vector: vector(&transceiver.normal_vector),
                is_hidden: transceiver.is_hidden,
                transmit_status: transceiver.transceiver_state,
                color: optional_rgba(&transceiver.color),
                label: transceiver.label.clone(),
                animation_speed: transceiver.animation_speed,
            })
            .collect(),
        storage_devices: sc
            .generic_storage
            .iter()
            .map(|storage| pb::GenericStorage {
                label: storage.label.clone(),
                current_value: storage.current_value,
                max_value: storage.max_value,
                units: storage.units.clone(),
                color: rgba_list(&storage.color),
                thresholds: storage.thresholds.clone(),
            })
            .collect(),
        lights: sc
            .lights
            .iter()
            .map(|light| pb::Light {
                label: light.label.clone(),
                position: vector(&light.position_m),
                field_of_view: light.field_of_view_rad * R2D,
                normal_vector: vector(&light.normal_vector),
                light_on: setting(light.light_on),
                range: light.range_m,
                intensity: light.intensity,
                show_light_marker: setting(light.show_light_marker),
                marker_diameter: light.marker_diameter_m,
                color: optional_rgba(&light.color),
                gamma_saturation: light.gamma_saturation,
                show_lens_flare: setting(light.show_lens_flare),
                lens_flare_brightness: light.lens_flare_brightness,
                lens_flare_fade_speed: light.lens_flare_fade_speed,
            })
            .collect(),
        model_dictionary_key: sc.model_dictionary_key.clone(),
        logo_texture: sc.logo_texture.clone(),
        osc_orbit_line_color: optional_rgba(&sc.osc_orbit_line_color),
        true_trajectory_line_color: optional_rgba(&sc.true_trajectory_line_color),
        ground_track_line_color: optional_rgba(&sc.ground_track_line_color),
        ground_track_body_name: sc.ground_track_body_name.clone(),
        multi_shapes: sc
            .msm_info
            .msm_list
            .iter()
            .map(|shape| pb::MultiShape {
                is_on: setting(shape.is_on),
                position: vector(&shape.position_m),
                radius: shape.radius_m,
                current_value: shape.current_value_c,
                max_value: shape.max_value_c,
                positive_color: optional_rgba(&shape.positive_color),
                negative_color: optional_rgba(&shape.negative_color),
                neutral_opacity: shape.neutral_opacity,
                shape: shape.shape.clone(),
                dimensions: vector(&shape.dimensions_m),
                rotation: vector(&shape.rotation),
            })
            .collect(),
    }
}

/// Upstream's `CameraConfig` block.
///
/// Vizard's camera looks along its -x axis with z up the sensor, not along
/// +z with y down as the camera frame does, so upstream composes the MRP
/// `(1/3, 1/3, -1/3)` onto `sigma_CB` to give Vizard its camera attitude.
pub(super) fn camera(config: &CameraConfigMsg) -> pb::CameraConfig {
    let sigma_cu_c = Vector3::new(1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0);
    let unity_camera_mrp = add_mrp(config.sigma_cb, sigma_cu_c);
    pb::CameraConfig {
        camera_id: config.camera_id,
        parent_name: config.parent_name.clone(),
        field_of_view: config.field_of_view_rad * R2D,
        resolution: config
            .resolution_px
            .iter()
            .map(|px| i64::from(*px))
            .collect(),
        update_camera_parameters: config.update_camera_parameters,
        camera_pos_b: vector(&config.camera_pos_b_m),
        camera_dir_b: vector(&unity_camera_mrp),
        render_rate: config.render_rate_nanos,
        skybox: config.sky_box.clone(),
        post_processing_on: i64::from(setting(config.post_processing_on)),
        pp_focus_distance: config.pp_focus_distance_m,
        pp_aperture: config.pp_aperture,
        pp_focal_length: config.pp_focal_length_m * 1000.0,
        pp_max_blur_size: i64::from(config.pp_max_blur_size),
        render_mode: config.render_mode as i64,
        depth_map_clipping_planes: if config.depth_map_clipping_planes_m[0] > 0.0 {
            config.depth_map_clipping_planes_m.to_vec()
        } else {
            Vec::new()
        },
        show_hud_elements_in_image: setting(config.show_hud_elements_in_image),
    }
}

/// Upstream's `CelestialBody` block. Without an orientation the body is
/// identity-oriented, as upstream's default SPICE payload.
pub(super) fn celestial_body(info: &GravBodyInfo, planet: &PlanetStateMsg) -> pb::CelestialBody {
    let dcm_pn = planet
        .orientation
        .as_ref()
        .map_or_else(Matrix3::identity, |orientation| {
            orientation.inertial_to_fixed
        });
    pb::CelestialBody {
        body_name: info.body_name.clone(),
        position: vector(&planet.position_inertial_m),
        velocity: vector(&planet.velocity_inertial_mps),
        rotation: row_major(&dcm_pn),
        mu: info.mu_m3ps2 / 1.0e9,
        radius_eq: info.rad_equator_m / 1000.0,
        radius_ratio: info.radius_ratio,
        model_dictionary_key: info.model_dictionary_key.clone(),
    }
}

/// Upstream `receiveUserInput`'s `VizUserInputMsgPayload` from a `VizInput`.
pub(super) fn user_input(input: &proto::VizInput) -> VizUserInputMsg {
    VizUserInputMsg {
        // Upstream `static_cast<int>`.
        frame_number: input.frame_number as i32,
        keyboard_input: input
            .key_inputs
            .as_ref()
            .map(|keys| keys.keys.clone())
            .unwrap_or_default(),
        viz_event_replies: input
            .replies
            .iter()
            .map(|reply| VizEventReply {
                event_handler_id: reply.event_handler_id.clone(),
                reply: reply.reply.clone(),
                event_handler_destroyed: reply.event_handler_destroyed,
            })
            .collect(),
    }
}
