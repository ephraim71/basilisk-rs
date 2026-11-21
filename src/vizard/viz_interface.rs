//! Vizard interface: port of upstream Basilisk
//! `simulation/vizard/vizInterface`.
//!
//! Each update packs the simulation state into one Vizard protobuffer
//! message ([`proto::VizMessage`]): the spacecraft and their devices, the
//! celestial bodies, the instrument cameras, and the scene settings. The
//! message then goes to any combination of
//!
//! * a **file** (`save_file`) for Vizard to play back: each message
//!   preceded by its length as a varint, the stream [`read_viz_messages`]
//!   reads back;
//! * a **live** Vizard (`live_stream`), in lockstep over a two-way ZeroMQ
//!   socket, which also returns the user's keyboard input and dialog replies
//!   on [`VizInterface::user_input_msg`];
//! * **broadcast** Vizards (`broadcast_stream`), over a ZeroMQ publisher
//!   socket;
//! * a **headless** Vizard (`no_display`), which only renders instrument
//!   camera images, published on each camera's
//!   [`VizCamera::opnav_image_out_msg`].
//!
//! Nothing is sent at time zero; the first message is frame 1, at the first
//! update after it. The epoch and the settings ([`VizInterface::settings`])
//! go in the first message after each reset. While broadcasting, the
//! settings go out again every `broadcast_settings_send_delay_s` of
//! wall-clock time, and, as upstream, with every frame when there is neither
//! a file nor a live Vizard to mark them sent. Locations, quad maps and
//! event dialogs go out once, in the next message, and are then cleared.
//!
//! # Ports
//!
//! Upstream's vectors of input messages are one port per item, each
//! registered with an `add_*` method: [`VizInterface::add_spacecraft`],
//! then [`VizSpacecraft::add_reaction_wheel`], [`VizSpacecraft::add_thruster`]
//! and [`VizSpacecraft::add_coarse_sun_sensor`] on it;
//! [`VizInterface::add_celestial_body`]; [`VizInterface::add_camera`].
//!
//! Upstream skips a message that is unlinked or not yet written. Ports carry
//! no header, so the stand-in is "connected ⇒ written at the current update"
//! (as in `css_inputs`): every connected input is fresh at every update, and
//! a spacecraft or device is left out of the message exactly when its input
//! is unconnected.
//!
//! # Differences from upstream
//!
//! * The epoch Vizard is given is the simulation's start epoch, from the
//!   scheduler. Upstream reads an optional `epochInMsg` and otherwise sends
//!   2019-01-01 00:00:00.
//! * A reaction wheel is read from two ports, the state and configuration
//!   the wheel effector publishes, where upstream reads one `RWConfigLogMsg`.
//!   A coarse sun sensor is its [`SunSensorMsg`](crate::messages::SunSensorMsg)
//!   signal plus geometry copied from its configuration, where upstream
//!   reads one `CSSConfigLogMsg`.
//! * Upstream's reset fails for a celestial body name of 64 characters or
//!   more, the size of the default SPICE payload's name buffer. The name
//!   here is only the [`GravBodyInfo`]'s `String`, so any length is sent.
//! * Upstream can write a frame's length prefix without the frame (a
//!   headless camera frame) or the frame without its prefix (the frame that
//!   terminates Vizard), leaving the file unreadable from that point. Here a
//!   frame is written whole, at the point upstream writes the prefix, and
//!   the terminating frame is not saved.
//! * A camera with a zero render rate never requests an image; upstream
//!   divides by it.
//! * Upstream's checks that `thrInfo` matches `thrInMsgs` in length have no
//!   counterpart: each thruster port carries its group.
//! * Upstream reports failures with `bskError`, which stops a Python
//!   simulation. Here they are logged and the update carries on.
//!
//! # Vizard handshake
//!
//! The broadcast socket is bound; the two-way socket connects to Vizard. On
//! reset the two-way socket sends `PING`, waits for the answer and sends
//! `PING` again. Each frame then collects the answer to the last request,
//! sends `SIM_UPDATE`, two empty frames and the serialized message, and
//! expects `OK`. In live mode it then sends `REQUEST_INPUT` and receives a
//! status string (`VIZARD_INPUT`, or `ERROR`) and a [`proto::VizInput`],
//! whose dialog replies the broadcast socket passes on as `SYNC_SETTINGS`.
//! For each camera
//! due an image it sends `REQUEST_IMAGE_<cameraID>` and receives the image
//! length (big-endian `int32`) and the image. A headless frame ends with
//! `PING`. The broadcast socket publishes `SIM_UPDATE` and the message.

mod link;
mod protobuf;

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use hifitime::Duration;
use prost::Message;

use self::link::{Broadcaster, VizardLink};
use super::proto;
use super::viz_structures::{
    GravBodyInfo, LiveVizSettings, Location, QuadMap, VizCelestialBody, VizEventDialog, VizSetting,
    VizSettings, VizSpacecraft,
};
use crate::messages::{
    CameraConfigMsg, CameraImageMsg, Input, Output, PlanetStateMsg, VizUserInputMsg,
};
use crate::{Module, SimulationContext};

/// Upstream `firstPass` limit: Vizard needs ten images before it places the
/// planets, so a headless Vizard is sent the first eleven frames whether or
/// not a camera is due.
const HEADLESS_WARM_UP_FRAMES: u32 = 11;

/// Upstream `imageType` of a Vizard image: four channels, RGBA.
const VIZARD_IMAGE_CHANNELS: i8 = 4;

/// User-settable parameters of [`VizInterface`]. `Default` is upstream's
/// constructor state.
#[derive(Clone, Debug, PartialEq)]
pub struct VizInterfaceConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `saveFile` with `protoFilename`: file to save the messages
    /// to, or `None` not to save them.
    pub save_file: Option<PathBuf>,
    /// Upstream `liveStream`: stream to a two-way Vizard in lockstep. A
    /// reset waits for that Vizard to answer.
    pub live_stream: bool,
    /// Upstream `broadcastStream`: publish for listening Vizards.
    pub broadcast_stream: bool,
    /// Upstream `noDisplay`: drive a headless two-way Vizard for camera
    /// images. A reset waits for that Vizard to answer. Cannot be combined
    /// with live or broadcast streaming.
    pub no_display: bool,
    /// Upstream `reqComProtocol`: protocol of the two-way socket.
    pub req_com_protocol: String,
    /// Upstream `reqComAddress`: address of the two-way Vizard.
    pub req_com_address: String,
    /// Upstream `reqPortNumber`: port of the two-way Vizard.
    pub req_port_number: String,
    /// Upstream `pubComProtocol`: protocol of the broadcast socket.
    pub pub_com_protocol: String,
    /// Upstream `pubComAddress`: address to broadcast at.
    pub pub_com_address: String,
    /// Upstream `pubPortNumber`: port to broadcast at; `*` picks a free one
    /// (see [`VizInterface::broadcast_endpoint`]).
    pub pub_port_number: String,
    /// Upstream `broadcastSettingsSendDelay` \[s\]: wall-clock time between
    /// repeats of the settings on the broadcast socket.
    pub broadcast_settings_send_delay_s: f64,
}

impl Default for VizInterfaceConfig {
    fn default() -> Self {
        Self {
            name: "vizInterface".to_string(),
            save_file: None,
            live_stream: false,
            broadcast_stream: false,
            no_display: false,
            req_com_protocol: "tcp".to_string(),
            req_com_address: "0.0.0.0".to_string(),
            req_port_number: "5556".to_string(),
            pub_com_protocol: "tcp".to_string(),
            pub_com_address: "0.0.0.0".to_string(),
            pub_port_number: "5570".to_string(),
            broadcast_settings_send_delay_s: 2.0,
        }
    }
}

impl VizInterfaceConfig {
    /// Checks the configuration. Upstream leaves the mode check to
    /// `vizSupport.enableUnityVisualization`.
    pub fn validate(&self) -> Result<(), String> {
        if self.no_display && (self.live_stream || self.broadcast_stream) {
            return Err(
                "no_display mode cannot be used with live_stream or broadcast_stream".to_string(),
            );
        }
        if self.broadcast_settings_send_delay_s.is_nan()
            || self.broadcast_settings_send_delay_s < 0.0
        {
            return Err(format!(
                "broadcast_settings_send_delay_s must be non-negative, got {}",
                self.broadcast_settings_send_delay_s
            ));
        }
        Ok(())
    }

    fn req_address(&self) -> String {
        format!(
            "{}://{}:{}",
            self.req_com_protocol, self.req_com_address, self.req_port_number
        )
    }

    fn pub_address(&self) -> String {
        format!(
            "{}://{}:{}",
            self.pub_com_protocol, self.pub_com_address, self.pub_port_number
        )
    }
}

/// An instrument camera: an entry of upstream `cameraConfInMsgs` with its
/// entry of `opnavImageOutMsgs`, added by [`VizInterface::add_camera`].
#[derive(Clone, Debug, Default)]
pub struct VizCamera {
    /// Upstream `cameraConfInMsgs[i]`: the camera is sent to Vizard only
    /// while this is connected.
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `opnavImageOutMsgs[i]`: the image Vizard rendered, in live
    /// or headless mode.
    pub opnav_image_out_msg: Output<CameraImageMsg>,
}

/// Packs the simulation into Vizard messages: upstream `VizInterface`.
pub struct VizInterface {
    /// User-settable parameters.
    pub config: VizInterfaceConfig,
    /// Upstream `settings`: sent in the first message after a reset.
    pub settings: VizSettings,
    /// Upstream `liveSettings`: sent in every message.
    pub live_settings: LiveVizSettings,
    /// Upstream `scData`; see [`Self::add_spacecraft`].
    pub spacecraft: Vec<VizSpacecraft>,
    /// Upstream `gravBodyInformation` with `spiceInMsgs`; see
    /// [`Self::add_celestial_body`].
    pub celestial_bodies: Vec<VizCelestialBody>,
    /// Upstream `cameraConfInMsgs` with `opnavImageOutMsgs`; see
    /// [`Self::add_camera`].
    pub cameras: Vec<VizCamera>,
    /// Upstream `locations`: sent with the next message, then cleared.
    pub locations: Vec<Location>,
    /// Upstream `quadMaps`: sent with the next message, then cleared.
    pub quad_maps: Vec<QuadMap>,
    /// Upstream `vizEventDialogs`: sent with the next message, then cleared.
    pub viz_event_dialogs: Vec<VizEventDialog>,
    /// Upstream `userInputMsg`: what the live Vizard's user did since the
    /// previous frame.
    pub user_input_msg: Output<VizUserInputMsg>,
    /// Upstream `FrameNumber`: -1 after a reset, then one more each update.
    frame_number: i64,
    /// Upstream `settings.dataFresh`: the settings are due to be sent.
    settings_fresh: bool,
    /// Upstream `epochMsgStatus.dataFresh`: the epoch is due to be sent.
    epoch_fresh: bool,
    /// Upstream `firstPass`: frames sent to a two-way Vizard, counted up to
    /// [`HEADLESS_WARM_UP_FRAMES`]. Like upstream, a reset keeps the count.
    first_pass: u32,
    /// Upstream `lastSettingsSendTime` \[s\]: wall-clock time the settings
    /// were last sent.
    last_settings_send_time_s: i64,
    /// Upstream `outputStream`.
    output_stream: Option<BufWriter<File>>,
    /// Upstream `publisher_socket`.
    broadcaster: Option<Broadcaster>,
    /// Upstream `requester_socket`.
    vizard: Option<VizardLink>,
    // The streaming modes in force, set from the configuration at reset.
    // Upstream clears its public flags once it has told Vizard to terminate;
    // these copies are cleared instead, leaving the configuration as set.
    /// Upstream `liveStream`, as in force.
    live_stream: bool,
    /// Upstream `broadcastStream`, as in force.
    broadcast_stream: bool,
    /// Upstream `noDisplay`, as in force.
    no_display: bool,
}

impl VizInterface {
    /// Builds the module with no spacecraft, bodies or cameras.
    ///
    /// # Panics
    ///
    /// If `config` fails [`VizInterfaceConfig::validate`].
    pub fn new(config: VizInterfaceConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid VizInterfaceConfig: {message}");
        }
        Self {
            live_stream: config.live_stream,
            broadcast_stream: config.broadcast_stream,
            no_display: config.no_display,
            config,
            settings: VizSettings::default(),
            live_settings: LiveVizSettings::default(),
            spacecraft: Vec::new(),
            celestial_bodies: Vec::new(),
            cameras: Vec::new(),
            locations: Vec::new(),
            quad_maps: Vec::new(),
            viz_event_dialogs: Vec::new(),
            user_input_msg: Output::default(),
            frame_number: -1,
            settings_fresh: true,
            epoch_fresh: true,
            first_pass: 0,
            last_settings_send_time_s: wall_clock_s(),
            output_stream: None,
            broadcaster: None,
            vizard: None,
        }
    }

    /// Adds a spacecraft (an entry of upstream `scData`) and returns it, to
    /// connect its state input and add its devices.
    pub fn add_spacecraft(&mut self, spacecraft_name: impl Into<String>) -> &mut VizSpacecraft {
        self.spacecraft.push(VizSpacecraft::new(spacecraft_name));
        self.spacecraft.last_mut().expect("just pushed")
    }

    /// Adds a celestial body (an entry of upstream `gravBodyInformation` and
    /// `spiceInMsgs`) and returns its optional ephemeris input.
    pub fn add_celestial_body(
        &mut self,
        grav_body_info: GravBodyInfo,
    ) -> &mut Input<PlanetStateMsg> {
        self.celestial_bodies.push(VizCelestialBody {
            grav_body_info,
            planet_state_in_msg: Input::default(),
        });
        &mut self
            .celestial_bodies
            .last_mut()
            .expect("just pushed")
            .planet_state_in_msg
    }

    /// Upstream `addCamMsgToModule`: adds an instrument camera and returns
    /// its configuration input and image output.
    pub fn add_camera(&mut self) -> &mut VizCamera {
        self.cameras.push(VizCamera::default());
        self.cameras.last_mut().expect("just pushed")
    }

    /// Upstream `FrameNumber`: the number of the last frame built, -1 before
    /// the first update after a reset.
    pub fn frame_number(&self) -> i64 {
        self.frame_number
    }

    /// The address the broadcast socket is bound at, with a `*` port
    /// resolved; `None` when not broadcasting.
    pub fn broadcast_endpoint(&self) -> Option<&str> {
        self.broadcaster.as_ref().map(Broadcaster::endpoint)
    }

    /// Upstream `Reset`.
    fn reset_state(&mut self) {
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
        self.live_stream = self.config.live_stream;
        self.broadcast_stream = self.config.broadcast_stream;
        self.no_display = self.config.no_display;

        self.broadcaster = None;
        if self.broadcast_stream {
            let address = self.config.pub_address();
            match Broadcaster::bind(&address) {
                Ok(broadcaster) => {
                    log::info!("Broadcasting at {}", broadcaster.endpoint());
                    self.broadcaster = Some(broadcaster);
                }
                Err(error) => log::error!(
                    "Broadcast socket did not connect correctly. ZMQ error code: {}",
                    error.to_raw()
                ),
            }
        }

        self.vizard = None;
        if self.live_stream || self.no_display {
            let address = self.config.req_address();
            log::info!("Waiting for Vizard at {address}");
            match VizardLink::connect(&address) {
                Ok(link) => {
                    log::info!("Basilisk-Vizard connection made");
                    self.vizard = Some(link);
                }
                Err(error) => log::error!(
                    "2-way socket did not connect correctly. ZMQ error code: {}",
                    error.to_raw()
                ),
            }
        }

        self.check_connections();

        self.frame_number = -1;
        // Replacing the stream flushes and closes the previous file.
        self.output_stream = None;
        if let Some(path) = &self.config.save_file {
            match File::create(path) {
                Ok(file) => {
                    log::info!("VizInterface: Writing data to {}", path.display());
                    self.output_stream = Some(BufWriter::new(file));
                }
                Err(error) => log::error!(
                    "VizInterface: Unable to open file {} for writing: {error}",
                    path.display()
                ),
            }
        }

        self.settings_fresh = true;
        self.epoch_fresh = true;
    }

    /// Upstream `Reset`'s link checks.
    fn check_connections(&self) {
        for spacecraft in &self.spacecraft {
            if !spacecraft.sc_state_in_msg.is_connected() {
                log::error!("vizInterface: spacecraft msg not linked.");
            }
            for (index, sensor) in spacecraft.coarse_sun_sensors.iter().enumerate() {
                if !sensor.css_signal_in_msg.is_connected() {
                    log::error!("vizInterface: CSS({index}) msg not linked.");
                }
            }
            for (index, wheel) in spacecraft.reaction_wheels.iter().enumerate() {
                if !wheel.rw_state_in_msg.is_connected() || !wheel.rw_config_in_msg.is_connected() {
                    log::error!("vizInterface: RW({index}) msg not linked.");
                }
            }
            for (index, thruster) in spacecraft.thrusters.iter().enumerate() {
                if !thruster.thr_in_msg.is_connected() {
                    log::error!("vizInterface: TH({index}) msg requested but not found.");
                }
            }
        }
    }

    /// The parts of upstream `ReadBSKMessages` that change the scene
    /// description: values set from optional command and state inputs.
    fn read_inputs(&mut self) {
        for spacecraft in &mut self.spacecraft {
            if spacecraft.true_trajectory_line_color_in_msg.is_connected() {
                spacecraft.true_trajectory_line_color = Some(
                    spacecraft
                        .true_trajectory_line_color_in_msg
                        .read()
                        .color_rgba,
                );
            }
            for sensor in &mut spacecraft.generic_sensors {
                if sensor.generic_sensor_cmd_in_msg.is_connected() {
                    sensor.generic_sensor_cmd = sensor.generic_sensor_cmd_in_msg.read().device_cmd;
                }
            }
            for light in &mut spacecraft.lights {
                if light.on_off_cmd_in_msg.is_connected() {
                    // Upstream `(int)deviceCmd`, then clamped to -1, 0 or 1.
                    light.light_on =
                        match (light.on_off_cmd_in_msg.read().device_cmd as i32).clamp(-1, 1) {
                            -1 => VizSetting::Disabled,
                            0 => VizSetting::UseVizDefault,
                            _ => VizSetting::Enabled,
                        };
                }
            }
            for transceiver in &mut spacecraft.transceivers {
                if transceiver.transceiver_state_in_msgs.is_empty() {
                    continue;
                }
                transceiver.transceiver_state = 0;
                for input in &transceiver.transceiver_state_in_msgs {
                    if !input.is_connected() {
                        continue;
                    }
                    let baud_rate_bps = input.read().baud_rate_bps;
                    if baud_rate_bps < 0.0 {
                        transceiver.transceiver_state |= 1;
                    } else if baud_rate_bps > 0.0 {
                        transceiver.transceiver_state |= 2;
                    }
                }
            }
            for storage in &mut spacecraft.generic_storage {
                if storage.battery_state_in_msg.is_connected() {
                    let battery = storage.battery_state_in_msg.read();
                    storage.current_value = battery.storage_level_j;
                    storage.max_value = battery.storage_capacity_j;
                }
                if storage.data_storage_state_in_msg.is_connected() {
                    let data = storage.data_storage_state_in_msg.read();
                    storage.current_value = data.storage_level_bits;
                    storage.max_value = data.storage_capacity_bits;
                }
                if storage.fuel_tank_state_in_msg.is_connected() {
                    let tank = storage.fuel_tank_state_in_msg.read();
                    storage.current_value = tank.fuel_mass_kg;
                    storage.max_value = tank.max_fuel_mass_kg;
                }
            }
            let msm = &mut spacecraft.msm_info;
            if msm.msm_charge_in_msg.is_connected() {
                let charges_c = msm.msm_charge_in_msg.read().charges_c;
                if charges_c.len() == msm.msm_list.len() {
                    for (shape, charge_c) in msm.msm_list.iter_mut().zip(charges_c) {
                        shape.current_value_c = charge_c;
                    }
                } else {
                    log::error!(
                        "vizInterface: the number of charged in MSM message and the number of msm vizInterface spheres must be the same."
                    );
                }
            }
        }
    }

    /// The message of the current frame: upstream `WriteProtobuffer` up to
    /// serialization. Sending it clears the one-shot lists and the epoch
    /// flag.
    fn build_message(&mut self, context: &SimulationContext, now_s: i64) -> proto::VizMessage {
        let mut message = proto::VizMessage::default();

        if self.broadcast_stream
            && (now_s - self.last_settings_send_time_s) as f64
                >= self.config.broadcast_settings_send_delay_s
        {
            self.settings_fresh = true;
        }
        if self.settings_fresh {
            message.settings = Some(protobuf::settings(&self.settings));
            self.last_settings_send_time_s = now_s;
        }
        message.live_settings = Some(protobuf::live_settings(&self.live_settings));
        message.viz_event_dialogs = self
            .viz_event_dialogs
            .drain(..)
            .map(|dialog| protobuf::event_dialog(&dialog))
            .collect();
        message.current_time = Some(protobuf::time_stamp(
            self.frame_number,
            context.current_sim_nanos,
        ));
        if self.epoch_fresh {
            let start_epoch = context.current_epoch
                - Duration::from_total_nanoseconds(i128::from(context.current_sim_nanos));
            message.epoch = Some(protobuf::epoch(start_epoch));
            self.epoch_fresh = false;
        }
        message.locations = self
            .locations
            .drain(..)
            .map(|location| protobuf::location(&location))
            .collect();
        message.quad_maps = self
            .quad_maps
            .drain(..)
            .map(|quad_map| protobuf::quad_map(&quad_map))
            .collect();
        message.spacecraft = self
            .spacecraft
            .iter()
            .filter(|spacecraft| spacecraft.sc_state_in_msg.is_connected())
            .map(|spacecraft| protobuf::spacecraft(spacecraft, &spacecraft.sc_state_in_msg.read()))
            .collect();
        message.cameras = self
            .cameras
            .iter()
            .filter(|camera| camera.camera_config_in_msg.is_connected())
            .map(|camera| protobuf::camera(&camera.camera_config_in_msg.read()))
            .collect();
        message.celestial_bodies = self
            .celestial_bodies
            .iter()
            .map(|body| {
                protobuf::celestial_body(&body.grav_body_info, &body.planet_state_in_msg.read())
            })
            .collect();
        message
    }

    /// The configurations of the cameras, in `cameras` order; a default
    /// (zero render rate, off) one for an unconnected camera.
    fn camera_configs(&self) -> Vec<CameraConfigMsg> {
        self.cameras
            .iter()
            .map(|camera| camera.camera_config_in_msg.read())
            .collect()
    }

    /// Upstream `WriteProtobuffer`: builds the frame and sends and saves it.
    fn write_protobuffer(&mut self, context: &SimulationContext) {
        let now_s = wall_clock_s();
        let mut message = self.build_message(context, now_s);
        let sim_nanos = context.current_sim_nanos;

        if self.broadcast_stream
            && let Some(broadcaster) = &self.broadcaster
            && let Err(error) = broadcaster.send("SIM_UPDATE", &message.encode_to_vec())
        {
            log::error!("Broadcast protobuffer did not send to socket: {error}");
        }

        // Settings that went out on the broadcast socket are not repeated to
        // the two-way Vizard or the file.
        if (self.live_stream && self.first_pass != 0 && self.last_settings_send_time_s == now_s)
            || self.output_stream.is_some()
        {
            if !self.settings_fresh {
                message.settings = None;
            }
            self.settings_fresh = false;
            if !self.live_settings.terminate_vizard {
                self.save_frame(&message);
            }
        }

        let cameras = self.camera_configs();
        let headless_frame = self.no_display
            && cameras.iter().any(|camera| {
                image_due(camera, sim_nanos) || self.first_pass < HEADLESS_WARM_UP_FRAMES
            });
        if !(self.live_stream || headless_frame) {
            return;
        }
        if self.first_pass < HEADLESS_WARM_UP_FRAMES {
            self.first_pass += 1;
        }
        let Some(mut vizard) = self.vizard.take() else {
            return;
        };
        self.exchange_frame(&mut vizard, &message, &cameras, headless_frame, sim_nanos);
        self.vizard = Some(vizard);
    }

    /// Upstream `WriteProtobuffer`'s two-way exchange of one frame.
    fn exchange_frame(
        &mut self,
        vizard: &mut VizardLink,
        message: &proto::VizMessage,
        cameras: &[CameraConfigMsg],
        headless_frame: bool,
        sim_nanos: u64,
    ) {
        if let Err(error) = vizard.discard_pending_reply() {
            log::error!("Vizard 2-way: no answer to the previous request: {error}");
        }
        if let Err(error) = vizard.send_sim_update(&message.encode_to_vec()) {
            log::error!("Vizard 2-way: SIM_UPDATE did not send: {error}");
        }

        if self.live_settings.terminate_vizard {
            self.broadcast_stream = false;
            self.live_stream = false;
            self.no_display = false;
            return;
        }

        match vizard.receive() {
            Ok(frames) if frames.first().is_some_and(|status| !status.is_empty()) => {
                if frames[0] != b"OK" {
                    log::error!("Vizard 2-way [0]: Error processing SIM_UPDATE.");
                }
            }
            _ => log::error!("Vizard: Did not return a status (OK) message during SIM_UPDATE."),
        }

        if self.live_stream {
            self.receive_user_input(vizard);
        }
        for (index, camera) in cameras.iter().enumerate() {
            if image_due(camera, sim_nanos) {
                self.request_image(vizard, index, camera, sim_nanos);
            }
        }
        if headless_frame && let Err(error) = vizard.send("PING") {
            log::error!("Vizard 2-way: keep-alive PING did not send: {error}");
        }
    }

    /// Upstream's file write: the frame's length as a varint, then the frame.
    fn save_frame(&mut self, message: &proto::VizMessage) {
        let Some(stream) = &mut self.output_stream else {
            return;
        };
        let written = stream
            .write_all(&message.encode_length_delimited_to_vec())
            .and_then(|()| stream.flush());
        if let Err(error) = written {
            log::error!("VizInterface: could not write the Vizard file: {error}");
        }
    }

    /// Upstream `receiveUserInput`: asks the live Vizard for the user's input
    /// since the previous frame and publishes it.
    fn receive_user_input(&mut self, vizard: &mut VizardLink) {
        let reply = vizard.send("REQUEST_INPUT").and_then(|()| vizard.receive());
        let frames = match reply {
            Ok(frames) => frames,
            Err(error) => {
                log::error!("Vizard 2-way [1]: Communication error. No data on socket: {error}");
                return;
            }
        };
        if frames.first().is_some_and(|status| status == b"ERROR") {
            log::error!("Vizard 2-way [2]: Invalid request string.");
        }
        let Some(payload) = frames.get(1) else {
            log::error!("Vizard 2-way [2]: Did not return a user input message.");
            return;
        };
        let input = proto::VizInput::decode(payload.as_slice()).unwrap_or_else(|error| {
            log::error!("Vizard 2-way: could not parse the user input message: {error}");
            proto::VizInput::default()
        });

        // Rebroadcast the dialog replies so listening Vizards stay in step.
        let mut sync_settings = input.broadcast_sync_settings.clone().unwrap_or_default();
        sync_settings
            .dialog_events
            .extend(input.replies.iter().cloned());
        if sync_settings.encoded_len() > 0
            && self.broadcast_stream
            && let Some(broadcaster) = &self.broadcaster
            && let Err(error) = broadcaster.send("SYNC_SETTINGS", &sync_settings.encode_to_vec())
        {
            log::error!("Broadcast protobuffer did not send to socket: {error}");
        }

        self.user_input_msg.write(protobuf::user_input(&input));
    }

    /// Upstream `requestImage`: asks Vizard for camera `index`'s image and
    /// publishes it.
    fn request_image(
        &mut self,
        vizard: &mut VizardLink,
        index: usize,
        camera: &CameraConfigMsg,
        sim_nanos: u64,
    ) {
        let request = format!("REQUEST_IMAGE_{}", camera.camera_id);
        let image = match vizard.send(&request).and_then(|()| vizard.receive()) {
            Ok(frames) => decode_image(&frames).unwrap_or_else(|problem| {
                log::error!("Vizard image response {problem}.");
                Vec::new()
            }),
            Err(error) => {
                log::error!("Vizard image response was not received: {error}");
                Vec::new()
            }
        };
        self.cameras[index]
            .opnav_image_out_msg
            .write(CameraImageMsg {
                time_tag_nanos: sim_nanos,
                valid: !image.is_empty(),
                camera_id: camera.camera_id,
                image,
                image_type: VIZARD_IMAGE_CHANNELS,
            });
    }
}

impl Module for VizInterface {
    fn init(&mut self) {
        self.reset_state();
        self.user_input_msg.write(VizUserInputMsg::default());
        for camera in &self.cameras {
            camera.opnav_image_out_msg.write(CameraImageMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`.
    fn update(&mut self, context: &SimulationContext) {
        self.frame_number += 1;
        self.read_inputs();
        if context.current_sim_nanos > 0 {
            self.write_protobuffer(context);
        }
    }
}

/// Wall-clock time \[s\] since the Unix epoch, upstream's `time(0)`.
fn wall_clock_s() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs() as i64)
}

/// Whether `camera` takes an image at `sim_nanos`: on, and at a multiple of
/// its render rate. A zero render rate never is.
fn image_due(camera: &CameraConfigMsg, sim_nanos: u64) -> bool {
    camera.is_on
        && sim_nanos
            .checked_rem(camera.render_rate_nanos)
            .is_some_and(|remainder| remainder == 0)
}

/// The image of a Vizard image reply: its length frame (a big-endian
/// `int32`; upstream byte-swaps a native-endian read) must match the image
/// frame.
fn decode_image(frames: &[Vec<u8>]) -> Result<Vec<u8>, &'static str> {
    let [length, image, ..] = frames else {
        return Err("was not received");
    };
    let length: [u8; 4] = length
        .get(..4)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or("length field is invalid")?;
    let length = u32::from_be_bytes(length);
    if length > i32::MAX as u32 || length as usize != image.len() {
        return Err("buffer length is invalid");
    }
    Ok(image.clone())
}

/// Reads a file [`VizInterface`] saved: its messages in order.
pub fn read_viz_file(path: impl AsRef<Path>) -> io::Result<Vec<proto::VizMessage>> {
    read_viz_messages(File::open(path)?)
}

/// Reads length-delimited Vizard messages, the format [`VizInterface`] saves,
/// until the end of `reader`.
pub fn read_viz_messages(mut reader: impl Read) -> io::Result<Vec<proto::VizMessage>> {
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    let mut remaining = bytes.as_slice();
    let mut messages = Vec::new();
    while !remaining.is_empty() {
        let message = proto::VizMessage::decode_length_delimited(&mut remaining)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        messages.push(message);
    }
    Ok(messages)
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `_UnitTest/test_vizInterface.py`, plus checks of the
    //! message mapping and of the two-way protocol against an in-process
    //! stand-in for Vizard (a ZeroMQ reply socket on the loopback interface).
    //!
    //! `test_vizInterface_broadcast_without_subscriber` also bounds the
    //! process's memory growth, which caught a protobuffer leaked every
    //! frame. That part is not ported: each frame's message is dropped here
    //! when the update returns, and the RSS of a test process shared with
    //! concurrently running tests would not measure this module.

    use std::path::{Path, PathBuf};
    use std::thread::{self, JoinHandle};
    use std::time::{SystemTime, UNIX_EPOCH};

    use hifitime::Epoch;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};
    use prost::Message;

    use super::{VizInterface, VizInterfaceConfig, protobuf, read_viz_file};
    use crate::dynamics::gravity::GravBodyData;
    use crate::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffectorConfig;
    use crate::messages::{
        CameraConfigMsg, ChargeMsmMsg, ColorMsg, DataNodeUsageMsg, DataStorageStatusMsg,
        DeviceCommandMsg, FuelTankMsg, Output, PlanetOrientation, PlanetStateMsg,
        PowerStorageStatusMsg, ReactionWheelStateMsg, SpacecraftStateMsg, SunSensorMsg,
        ThrusterOutputMsg,
    };
    use crate::sensors::coarse_sun_sensor::CoarseSunSensorConfig;
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{D2R, R2D};
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use crate::utilities::rigid_body_kinematics::add_mrp;
    use crate::vizard::proto;
    use crate::vizard::viz_structures::{
        GenericSensor, GenericStorage, GravBodyInfo, Light, Location, MultiShape, QuadMap,
        StdCameraSettings, ThrClusterMap, Transceiver, VizCoarseSunSensor, VizEventDialog,
        VizSetting,
    };
    use crate::{Module, SimulationContext};

    const SECOND_NANOS: u64 = 1_000_000_000;

    /// Idle time \[ms\] after which the stand-in Vizard gives up waiting for
    /// the next request.
    const FAKE_VIZARD_IDLE_MS: i32 = 2000;

    fn unique_path(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock after Unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "basilisk_rs_viz_{label}_{}_{nonce}.bin",
            std::process::id()
        ))
    }

    fn start_epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn context_at(sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: sim_nanos,
            current_epoch: start_epoch()
                + hifitime::Duration::from_total_nanoseconds(i128::from(sim_nanos)),
        }
    }

    fn saving_to(path: &Path) -> VizInterface {
        VizInterface::new(VizInterfaceConfig {
            save_file: Some(path.to_path_buf()),
            ..VizInterfaceConfig::default()
        })
    }

    /// Runs `init` and then updates at `0, step, ..., frames * step`.
    fn run_frames(viz: &mut VizInterface, frames: u64, step_nanos: u64) {
        viz.init();
        for frame in 0..=frames {
            viz.update(&context_at(frame * step_nanos));
        }
    }

    fn earth() -> GravBodyInfo {
        GravBodyInfo {
            body_name: "earth".to_string(),
            mu_m3ps2: 3.986_004_415e14,
            rad_equator_m: 6_378_136.6,
            radius_ratio: 1.0,
            model_dictionary_key: String::new(),
        }
    }

    fn assert_close(actual: &[f64], expected: &[f64], accuracy: f64, what: &str) {
        assert_eq!(actual.len(), expected.len(), "{what}: length");
        for (a, e) in actual.iter().zip(expected) {
            assert!(
                (a - e).abs() <= accuracy,
                "{what}: {actual:?} vs {expected:?}"
            );
        }
    }

    /// Upstream `test_vizInterface`: a spacecraft in GEO, saved to a file for
    /// ten 1 s frames, and the file read back.
    #[test]
    fn saved_file_round_trips_states_bodies_and_settings() {
        let accuracy = 1e-8;
        let frames = 10;
        let path = unique_path("round_trip");

        let mu = earth().mu_m3ps2;
        let elements = ClassicElements {
            semi_major_axis: 42_000.0 * 1000.0,
            eccentricity: 0.00001,
            inclination_rad: 0.0 * D2R,
            raan_rad: 0.0 * D2R,
            arg_periapsis_rad: 0.0 * D2R,
            true_anomaly_rad: 0.0 * D2R,
            ..ClassicElements::default()
        };
        let (r_n, v_n) = elements_to_state(mu, &elements);
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: SECOND_NANOS,
            initial_position_m: r_n,
            initial_velocity_mps: v_n,
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::new(0.1, 0.2, 0.3),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                    .expect("valid Earth"),
            )
            .expect("unique central body");

        let mut recorder = MessageRecorder::<SpacecraftStateMsg>::default();
        let mut viz = saving_to(&path);
        viz.settings.orbit_lines_on = 1;
        viz.settings.spacecraft_cs_on = VizSetting::Enabled;
        viz.settings.keyboard_live_input = "abcd".to_string();
        viz.add_celestial_body(earth());
        {
            let mut sim = Simulation::new(start_epoch(), false);
            sim.connect(&spacecraft.state_out, &mut recorder.input_msg);
            sim.connect(
                &spacecraft.state_out,
                &mut viz.add_spacecraft("testSat").sc_state_in_msg,
            );
            sim.add_module("spacecraft", &mut spacecraft, SECOND_NANOS, 10);
            sim.add_module("recorder", &mut recorder, SECOND_NANOS, 0);
            sim.add_module("viz", &mut viz, SECOND_NANOS, 0);
            sim.run_for(frames * SECOND_NANOS);
        }
        drop(viz);

        let messages = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        assert_eq!(messages.len(), frames as usize, "File is missing messages");

        for (i, message) in messages.iter().enumerate() {
            // Spacecraft states: message i is the state recorded at t = i + 1.
            assert_eq!(message.spacecraft.len(), 1, "Number of spacecraft mismatch");
            let truth = &recorder.samples[i + 1];
            let sc = &message.spacecraft[0];
            assert_eq!(sc.spacecraft_name, "testSat");
            assert_close(
                &sc.position,
                truth.position_m.as_slice(),
                accuracy,
                "Position",
            );
            assert_close(
                &sc.velocity,
                truth.velocity_mps.as_slice(),
                accuracy,
                "Velocity",
            );
            assert_close(
                &sc.rotation,
                truth.sigma_bn.as_slice(),
                accuracy,
                "Rotation",
            );

            // Celestial bodies: the unconnected Earth sits at the origin.
            assert_eq!(
                message.celestial_bodies.len(),
                1,
                "Celestial bodies mismatch"
            );
            assert_eq!(message.celestial_bodies[0].position, vec![0.0; 3]);
            assert_eq!(message.celestial_bodies[0].velocity, vec![0.0; 3]);

            // Time stamps, and settings in the first message only.
            let time = message.current_time.expect("time stamp");
            assert_eq!(time.frame_number, i as i64 + 1, "Frame number is incorrect");
            assert_eq!(
                time.sim_time_elapsed,
                (SECOND_NANOS * (i as u64 + 1)) as f64,
                "Sim time elapsed is incorrect"
            );
            if i == 0 {
                let settings = message
                    .settings
                    .as_ref()
                    .expect("Should have settings message at first timestep");
                assert!(
                    message.epoch.is_some(),
                    "Should have epoch message at first timestep"
                );
                assert_eq!(settings.orbit_lines_on, 1, "Orbit lines not on");
                assert_eq!(settings.spacecraft_c_son, 1, "Spacecraft CS not on");
                assert_eq!(
                    settings.keyboard_live_input, "abcd",
                    "Incorrect key listeners"
                );
            } else {
                assert!(
                    message.settings.is_none(),
                    "Should only have settings message at first timestep"
                );
                assert!(message.epoch.is_none());
            }
        }
    }

    /// Upstream `test_vizInterface_long_gravity_body_name_reset`. Upstream's
    /// reset fails because the default SPICE payload's name buffer holds 63
    /// characters; the name here is a `String`, so the reset succeeds and
    /// the full name reaches Vizard.
    #[test]
    fn long_celestial_body_name_resets_and_is_sent_whole() {
        let path = unique_path("long_name");
        let long_body_name = format!("body_{}", "x".repeat(128));
        assert_eq!(long_body_name.len(), 133);

        let mut viz = saving_to(&path);
        viz.add_celestial_body(GravBodyInfo {
            body_name: long_body_name.clone(),
            ..GravBodyInfo::default()
        });
        run_frames(&mut viz, 1, SECOND_NANOS);
        drop(viz);

        let messages = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].celestial_bodies[0].body_name, long_body_name);
    }

    /// Upstream `test_vizInterface_closes_output_files`: a reset releases
    /// the previous file, complete, and dropping the module releases the
    /// current one.
    #[test]
    fn reset_and_drop_release_complete_output_files() {
        let first_output = unique_path("first");
        let second_output = unique_path("second");

        let mut viz = saving_to(&first_output);
        run_frames(&mut viz, 1, SECOND_NANOS);
        viz.config.save_file = Some(second_output.clone());
        viz.reset(&context_at(0));
        assert_eq!(read_viz_file(&first_output).expect("first file").len(), 1);
        std::fs::remove_file(&first_output).expect("first file released");

        viz.update(&context_at(0));
        viz.update(&context_at(SECOND_NANOS));
        drop(viz);
        let second = read_viz_file(&second_output).expect("second file");
        std::fs::remove_file(&second_output).expect("second file released");
        // A reset starts the frame count and the settings over.
        assert_eq!(second.len(), 1);
        assert_eq!(second[0].current_time.expect("time").frame_number, 1);
        assert!(second[0].settings.is_some() && second[0].epoch.is_some());
    }

    /// Upstream `test_vizInterface_broadcast_without_subscriber`: publishing
    /// with no subscriber keeps running; the frame count is checked (see the
    /// module comment for the memory bound).
    #[test]
    fn broadcast_without_subscriber_keeps_counting_frames() {
        let task_rate_nanos = 10_000_000;
        let total_frames = 2200;
        let state = Output::new(SpacecraftStateMsg::default());
        let mut viz = VizInterface::new(VizInterfaceConfig {
            broadcast_stream: true,
            pub_com_address: "127.0.0.1".to_string(),
            pub_port_number: "*".to_string(),
            ..VizInterfaceConfig::default()
        });
        state.connect_to(&mut viz.add_spacecraft("broadcastSat").sc_state_in_msg);
        {
            let mut sim = Simulation::new(start_epoch(), false);
            sim.add_module("viz", &mut viz, task_rate_nanos, 0);
            sim.run_for(200 * task_rate_nanos);
            sim.run_for((total_frames - 200) * task_rate_nanos);
        }
        assert!(viz.broadcast_endpoint().is_some());
        assert_eq!(viz.frame_number(), total_frames as i64);
    }

    /// A subscriber receives `SIM_UPDATE` and the frame; broadcasting alone
    /// never marks the settings sent, so, as upstream, every frame carries
    /// them.
    #[test]
    fn broadcast_frames_reach_a_subscriber() {
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(7.0e6, 0.0, 0.0),
            ..SpacecraftStateMsg::default()
        });
        let mut viz = VizInterface::new(VizInterfaceConfig {
            broadcast_stream: true,
            pub_com_address: "127.0.0.1".to_string(),
            pub_port_number: "*".to_string(),
            ..VizInterfaceConfig::default()
        });
        state.connect_to(&mut viz.add_spacecraft("broadcastSat").sc_state_in_msg);
        viz.init();
        let endpoint = viz.broadcast_endpoint().expect("bound").to_string();

        let subscriber = zmq::Context::new().socket(zmq::SUB).expect("socket");
        subscriber.set_subscribe(b"").expect("subscribe");
        subscriber.set_rcvtimeo(100).expect("timeout");
        subscriber.connect(&endpoint).expect("connect");

        // A subscriber misses what is published before it has joined.
        let mut received = Vec::new();
        for frame in 0..200 {
            viz.update(&context_at(frame * SECOND_NANOS));
            if let Ok(frames) = subscriber.recv_multipart(0) {
                received.push(frames);
                break;
            }
        }
        let frames = received.pop().expect("a broadcast frame");
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], b"SIM_UPDATE");
        let message = proto::VizMessage::decode(frames[1].as_slice()).expect("frame");
        assert_eq!(message.spacecraft[0].spacecraft_name, "broadcastSat");
        assert_eq!(message.spacecraft[0].position, vec![7.0e6, 0.0, 0.0]);
        assert!(message.settings.is_some());
    }

    /// What the stand-in Vizard received.
    #[derive(Debug, Default)]
    struct VizardLog {
        pings: usize,
        sim_updates: Vec<proto::VizMessage>,
        input_requests: usize,
        image_requests: Vec<String>,
    }

    /// A stand-in for a two-way Vizard on a free loopback port: it answers
    /// every request as Vizard does, replying `input` to `REQUEST_INPUT` and
    /// `image` to every image request, until it is told to terminate or
    /// idles. Returns the port and the log.
    fn spawn_fake_vizard(
        input: proto::VizInput,
        image: Vec<u8>,
    ) -> (String, JoinHandle<VizardLog>) {
        let socket = zmq::Context::new().socket(zmq::REP).expect("socket");
        socket.set_rcvtimeo(FAKE_VIZARD_IDLE_MS).expect("timeout");
        socket.set_linger(0).expect("linger");
        socket.bind("tcp://127.0.0.1:*").expect("bind");
        let endpoint = socket
            .get_last_endpoint()
            .expect("endpoint")
            .expect("utf-8");
        let port = endpoint.rsplit(':').next().expect("port").to_string();
        let handle = thread::spawn(move || {
            let mut log = VizardLog::default();
            while let Ok(frames) = socket.recv_multipart(0) {
                let request = String::from_utf8_lossy(&frames[0]).into_owned();
                match request.as_str() {
                    "PING" => {
                        log.pings += 1;
                        socket.send("PING", 0).expect("reply");
                    }
                    "SIM_UPDATE" => {
                        assert_eq!(frames.len(), 4);
                        assert!(frames[1].is_empty() && frames[2].is_empty());
                        let message =
                            proto::VizMessage::decode(frames[3].as_slice()).expect("frame");
                        let terminate = message
                            .live_settings
                            .as_ref()
                            .is_some_and(|live| live.terminate_vizard);
                        log.sim_updates.push(message);
                        if terminate {
                            break;
                        }
                        socket.send("OK", 0).expect("reply");
                    }
                    "REQUEST_INPUT" => {
                        log.input_requests += 1;
                        socket
                            .send_multipart([b"VIZARD_INPUT".to_vec(), input.encode_to_vec()], 0)
                            .expect("reply");
                    }
                    other if other.starts_with("REQUEST_IMAGE_") => {
                        log.image_requests.push(other.to_string());
                        let length = u32::try_from(image.len()).expect("small image");
                        socket
                            .send_multipart([length.to_be_bytes().to_vec(), image.clone()], 0)
                            .expect("reply");
                    }
                    other => panic!("unexpected request {other}"),
                }
            }
            log
        });
        (port, handle)
    }

    fn two_way(port: &str, live_stream: bool, no_display: bool) -> VizInterfaceConfig {
        VizInterfaceConfig {
            live_stream,
            no_display,
            req_com_address: "127.0.0.1".to_string(),
            req_port_number: port.to_string(),
            ..VizInterfaceConfig::default()
        }
    }

    fn camera_config(camera_id: i64, render_rate_nanos: u64) -> CameraConfigMsg {
        CameraConfigMsg {
            camera_id,
            is_on: true,
            render_rate_nanos,
            resolution_px: [512, 512],
            field_of_view_rad: 40.0 * D2R,
            ..CameraConfigMsg::default()
        }
    }

    /// Live streaming: the handshake, one `SIM_UPDATE` per frame, the user
    /// input published, images requested when due, and termination.
    #[test]
    fn live_stream_runs_vizard_in_lockstep() {
        let input = proto::VizInput {
            frame_number: 42,
            key_inputs: Some(proto::viz_input::KeyboardInput {
                keys: "ab".to_string(),
            }),
            replies: vec![proto::VizEventReply {
                event_handler_id: "dialog".to_string(),
                reply: "yes".to_string(),
                event_handler_destroyed: true,
            }],
            broadcast_sync_settings: None,
        };
        let image = vec![1, 2, 3, 4, 5];
        let (port, vizard) = spawn_fake_vizard(input, image.clone());

        let state = Output::new(SpacecraftStateMsg::default());
        let camera = Output::new(camera_config(7, 2 * SECOND_NANOS));
        let mut viz = VizInterface::new(two_way(&port, true, false));
        state.connect_to(&mut viz.add_spacecraft("liveSat").sc_state_in_msg);
        camera.connect_to(&mut viz.add_camera().camera_config_in_msg);
        viz.settings.keyboard_live_input = "ab".to_string();

        run_frames(&mut viz, 3, SECOND_NANOS);
        let user_input = viz.user_input_msg.read();
        assert_eq!(user_input.frame_number, 42);
        assert_eq!(user_input.keyboard_input, "ab");
        assert_eq!(user_input.viz_event_replies.len(), 1);
        assert_eq!(user_input.viz_event_replies[0].reply, "yes");
        assert!(user_input.viz_event_replies[0].event_handler_destroyed);
        let camera_image = viz.cameras[0].opnav_image_out_msg.read();
        assert!(camera_image.valid);
        assert_eq!(camera_image.image, image);
        assert_eq!(camera_image.camera_id, 7);
        assert_eq!(camera_image.time_tag_nanos, 2 * SECOND_NANOS);
        assert_eq!(camera_image.image_type, 4);

        viz.live_settings.terminate_vizard = true;
        viz.update(&context_at(4 * SECOND_NANOS));
        // Once terminated, the two-way socket is left alone.
        viz.update(&context_at(5 * SECOND_NANOS));
        let log = vizard.join().expect("stand-in Vizard");

        assert_eq!(log.pings, 2, "the reset handshake");
        let frame_numbers: Vec<i64> = log
            .sim_updates
            .iter()
            .map(|message| message.current_time.expect("time").frame_number)
            .collect();
        assert_eq!(frame_numbers, vec![1, 2, 3, 4]);
        // As upstream, the first live frame is sent before it is counted, so
        // the settings go out with the first two frames.
        let with_settings: Vec<bool> = log
            .sim_updates
            .iter()
            .map(|message| message.settings.is_some())
            .collect();
        assert_eq!(with_settings, vec![true, true, false, false]);
        assert_eq!(log.input_requests, 3);
        assert_eq!(log.image_requests, vec!["REQUEST_IMAGE_7".to_string()]);
        assert_eq!(log.sim_updates[0].cameras.len(), 1);
    }

    /// Headless mode: the first eleven frames go to Vizard whatever the
    /// cameras, then only frames with an image due, each closed by a `PING`;
    /// every frame is saved.
    #[test]
    fn headless_mode_exchanges_warm_up_and_camera_frames() {
        let path = unique_path("headless");
        let (port, vizard) = spawn_fake_vizard(proto::VizInput::default(), vec![9; 16]);

        let camera = Output::new(camera_config(3, 5 * SECOND_NANOS));
        let mut viz = VizInterface::new(VizInterfaceConfig {
            save_file: Some(path.clone()),
            ..two_way(&port, false, true)
        });
        camera.connect_to(&mut viz.add_camera().camera_config_in_msg);
        run_frames(&mut viz, 16, SECOND_NANOS);
        let image = viz.cameras[0].opnav_image_out_msg.read();
        drop(viz);
        let log = vizard.join().expect("stand-in Vizard");

        let frame_numbers: Vec<i64> = log
            .sim_updates
            .iter()
            .map(|message| message.current_time.expect("time").frame_number)
            .collect();
        assert_eq!(frame_numbers, vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15]);
        assert_eq!(log.input_requests, 0);
        assert_eq!(log.image_requests.len(), 3, "images at 5, 10 and 15 s");
        assert_eq!(log.pings, 2 + frame_numbers.len());
        assert!(image.valid);
        assert_eq!(image.time_tag_nanos, 15 * SECOND_NANOS);
        assert_eq!(image.image, vec![9; 16]);

        let saved = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        assert_eq!(saved.len(), 16);
    }

    /// The frame that terminates Vizard is not saved.
    #[test]
    fn terminating_frame_is_not_saved() {
        let path = unique_path("terminate");
        let mut viz = saving_to(&path);
        run_frames(&mut viz, 2, SECOND_NANOS);
        viz.live_settings.terminate_vizard = true;
        viz.update(&context_at(3 * SECOND_NANOS));
        drop(viz);
        let saved = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        assert_eq!(saved.len(), 2);
    }

    /// The epoch is the simulation's start epoch, sent once per reset.
    #[test]
    fn epoch_is_the_simulation_start_epoch() {
        let path = unique_path("epoch");
        let mut viz = saving_to(&path);
        viz.init();
        viz.update(&context_at(0));
        let mut late = context_at(2_500_000_000);
        late.current_epoch = Epoch::from_gregorian_utc(2024, 6, 30, 23, 59, 59, 0)
            + hifitime::Duration::from_total_nanoseconds(2_500_000_000);
        viz.update(&late);
        viz.update(&context_at(3 * SECOND_NANOS));
        drop(viz);
        let saved = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        let epoch = saved[0].epoch.expect("epoch in the first frame");
        assert_eq!(
            (
                epoch.year,
                epoch.month,
                epoch.day,
                epoch.hours,
                epoch.minutes
            ),
            (2024, 6, 30, 23, 59)
        );
        assert!((epoch.seconds - 59.0).abs() < 1e-9);
        assert!(saved[1].epoch.is_none());
    }

    /// Locations, quad maps and dialogs go out once, with the next frame.
    #[test]
    fn one_shot_items_are_sent_once() {
        let path = unique_path("one_shot");
        let mut viz = saving_to(&path);
        viz.locations.push(Location {
            station_name: "Boulder".to_string(),
            parent_body_name: "earth".to_string(),
            r_gp_p_m: Vector3::new(1.0, 2.0, 3.0),
            g_hat_p: Vector3::z(),
            field_of_view_rad: 10.0 * D2R,
            ..Location::default()
        });
        viz.quad_maps.push(QuadMap {
            id: 4,
            parent_body_name: "earth".to_string(),
            vertices_m: vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
            color: Some([10, 20, 30, 40]),
            ..QuadMap::default()
        });
        viz.viz_event_dialogs.push(VizEventDialog {
            event_handler_id: "choice".to_string(),
            user_options: vec!["a".to_string(), "b".to_string()],
            ..VizEventDialog::default()
        });
        run_frames(&mut viz, 2, SECOND_NANOS);
        drop(viz);
        let saved = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");

        let location = &saved[0].locations[0];
        assert_eq!(location.station_name, "Boulder");
        assert_close(&[location.field_of_view], &[10.0], 1e-12, "location fov");
        assert_eq!(location.color, vec![-1, 0, 0, 0], "upstream's unset color");
        assert_eq!(location.r_gp_p, vec![1.0, 2.0, 3.0]);
        assert_eq!(
            saved[0].quad_maps[0].vertices,
            vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        );
        assert_eq!(saved[0].quad_maps[0].color, vec![10, 20, 30, 40]);
        let dialog = &saved[0].viz_event_dialogs[0];
        assert_eq!(dialog.user_options, vec!["a", "b"]);
        assert_eq!(dialog.duration_of_display, -1.0);
        assert_eq!(dialog.dialog_format, "none");
        assert!(saved[1].locations.is_empty());
        assert!(saved[1].quad_maps.is_empty());
        assert!(saved[1].viz_event_dialogs.is_empty());
    }

    /// The devices of a spacecraft, from their ports and configuration.
    #[test]
    fn spacecraft_devices_map_to_vizard_units() {
        let mut viz = VizInterface::new(VizInterfaceConfig::default());
        let sc = viz.add_spacecraft("deviceSat");
        sc.parent_spacecraft_name = "mother".to_string();

        let wheel_state = Output::new(ReactionWheelStateMsg {
            omega_radps: 100.0,
            applied_motor_torque_nm: 0.2,
            ..ReactionWheelStateMsg::default()
        });
        let wheel_config = Output::new(ReactionWheelStateEffectorConfig {
            position_m: Vector3::new(0.1, 0.0, 0.0),
            spin_axis_body: Vector3::new(0.0, 2.0, 0.0),
            max_speed_radps: 600.0,
            max_torque_nm: 0.5,
            ..ReactionWheelStateEffectorConfig::default()
        });
        let wheel = sc.add_reaction_wheel();
        wheel_state.connect_to(&mut wheel.rw_state_in_msg);
        wheel_config.connect_to(&mut wheel.rw_config_in_msg);
        // A wheel without its state input is left out.
        sc.add_reaction_wheel();

        let thruster = Output::new(ThrusterOutputMsg {
            max_thrust_n: 1.0,
            thrust_force_n: 0.5,
            thruster_location_m: Vector3::new(1.0, 2.0, 3.0),
            thruster_direction: Vector3::new(0.0, 0.0, -1.0),
            ..ThrusterOutputMsg::default()
        });
        thruster.connect_to(sc.add_thruster(ThrClusterMap {
            thr_tag: "acs".to_string(),
            color: Some([255, 0, 0, 255]),
            thr_offset_m: Vector3::new(0.5, 0.5, 0.5),
        }));

        let css_config = CoarseSunSensorConfig {
            name: "css".to_string(),
            position_m: Vector3::new(0.0, 0.0, 1.0),
            body_to_sensor_quaternion: UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                std::f64::consts::FRAC_PI_2,
            ),
            fov_half_angle_rad: 60.0 * D2R,
            scale_factor: 1.0,
            kelly_factor: 0.0,
            k_power: 2.0,
            bias: 0.0,
            noise_std: 0.0,
            noise_prop: 0.0,
            walk_bounds: -1.0,
            min_output: 0.0,
            max_output: 2.0,
        };
        let css_signal = Output::new(SunSensorMsg {
            sensed_value: 0.7,
            ..SunSensorMsg::default()
        });
        css_signal.connect_to(
            sc.add_coarse_sun_sensor(VizCoarseSunSensor::from_sensor_config(&css_config)),
        );

        let sensor_cmd = Output::new(DeviceCommandMsg { device_cmd: 2 });
        let mut generic_sensor = GenericSensor {
            field_of_view_rad: vec![10.0 * D2R, 20.0 * D2R],
            color: vec![[1, 2, 3, 4], [5, 6, 7, 8]],
            ..GenericSensor::default()
        };
        sensor_cmd.connect_to(&mut generic_sensor.generic_sensor_cmd_in_msg);
        sc.generic_sensors.push(generic_sensor);

        let light_cmd = Output::new(DeviceCommandMsg { device_cmd: 5 });
        let mut light = Light {
            field_of_view_rad: 30.0 * D2R,
            ..Light::default()
        };
        light_cmd.connect_to(&mut light.on_off_cmd_in_msg);
        sc.lights.push(light);

        let battery = Output::new(PowerStorageStatusMsg {
            storage_level_j: 3.0,
            storage_capacity_j: 10.0,
            ..PowerStorageStatusMsg::default()
        });
        let tank = Output::new(FuelTankMsg {
            fuel_mass_kg: 40.0,
            max_fuel_mass_kg: 50.0,
            ..FuelTankMsg::default()
        });
        let mut battery_bar = GenericStorage::default();
        battery.connect_to(&mut battery_bar.battery_state_in_msg);
        let mut tank_bar = GenericStorage::default();
        tank.connect_to(&mut tank_bar.fuel_tank_state_in_msg);
        sc.generic_storage.extend([battery_bar, tank_bar]);

        let charges = Output::new(ChargeMsmMsg {
            charges_c: vec![1e-6, -2e-6],
        });
        sc.msm_info.msm_list = vec![MultiShape::default(), MultiShape::default()];
        charges.connect_to(&mut sc.msm_info.msm_charge_in_msg);

        let color = Output::new(ColorMsg {
            color_rgba: [9, 8, 7, 6],
        });
        sc.true_trajectory_line_color = Some([1, 1, 1, 1]);
        color.connect_to(&mut sc.true_trajectory_line_color_in_msg);

        viz.read_inputs();
        let state = SpacecraftStateMsg::default();
        let message = protobuf::spacecraft(&viz.spacecraft[0], &state);

        assert_eq!(message.parent_spacecraft_name, "mother");
        assert_eq!(message.reaction_wheels.len(), 1);
        let rw = &message.reaction_wheels[0];
        assert_eq!(rw.position, vec![0.1, 0.0, 0.0]);
        assert_eq!(rw.spin_axis_vector, vec![0.0, 1.0, 0.0]);
        assert_eq!(
            (rw.wheel_speed, rw.wheel_torque, rw.max_speed, rw.max_torque),
            (100.0, 0.2, 600.0, 0.5)
        );

        let thr = &message.thrusters[0];
        assert_eq!(thr.position, vec![0.5, 1.5, 2.5], "offset subtracted");
        assert_eq!(thr.thrust_vector, vec![0.0, 0.0, -1.0]);
        assert_eq!((thr.current_thrust, thr.max_thrust), (0.5, 1.0));
        assert_eq!(thr.thruster_tag, "acs");
        assert_eq!(thr.color, vec![255, 0, 0, 255]);

        let css = &message.css[0];
        assert_close(&[css.field_of_view], &[120.0], 1e-9, "edge-to-edge css fov");
        assert_close(&css.normal_vector, &[0.0, 1.0, 0.0], 1e-12, "css boresight");
        assert_eq!(css.position, vec![0.0, 0.0, 1.0]);
        assert_eq!(
            (css.current_msmt, css.min_msmt, css.max_msmt),
            (0.7, 0.0, 2.0)
        );

        let gs = &message.generic_sensors[0];
        assert_close(
            &gs.field_of_view,
            &[10.0, 20.0],
            1e-12,
            "generic sensor fov",
        );
        assert_eq!(gs.color, vec![1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(gs.activity_status, 2);

        let light = &message.lights[0];
        assert_eq!(light.light_on, 1, "a command above 1 clamps to on");
        assert_close(&[light.field_of_view], &[30.0], 1e-12, "light fov");
        assert_eq!((light.intensity, light.marker_diameter), (-1.0, -1.0));
        assert!(light.color.is_empty());

        let bars = &message.storage_devices;
        assert_eq!((bars[0].current_value, bars[0].max_value), (3.0, 10.0));
        assert_eq!((bars[1].current_value, bars[1].max_value), (40.0, 50.0));

        let shapes = &message.multi_shapes;
        assert_eq!(
            (shapes[0].current_value, shapes[1].current_value),
            (1e-6, -2e-6)
        );
        assert_eq!(shapes[0].neutral_opacity, -1);
        assert_eq!(message.true_trajectory_line_color, vec![9, 8, 7, 6]);
        assert!(message.osc_orbit_line_color.is_empty());
    }

    /// A transceiver's state combines its data nodes: sending for a
    /// negative baud rate, receiving for a positive one; a storage bar
    /// follows a data storage unit.
    #[test]
    fn transceiver_and_data_storage_follow_their_inputs() {
        let mut viz = VizInterface::new(VizInterfaceConfig::default());
        let sending = Output::new(DataNodeUsageMsg {
            baud_rate_bps: -100.0,
            ..DataNodeUsageMsg::default()
        });
        let receiving = Output::new(DataNodeUsageMsg {
            baud_rate_bps: 50.0,
            ..DataNodeUsageMsg::default()
        });
        let storage_unit = Output::new(DataStorageStatusMsg {
            storage_level_bits: 8.0,
            storage_capacity_bits: 64.0,
            ..DataStorageStatusMsg::default()
        });
        let sc = viz.add_spacecraft("sat");
        let mut both = Transceiver {
            transceiver_state: 7,
            ..Transceiver::default()
        };
        sending.connect_to(both.add_transceiver_state_input());
        receiving.connect_to(both.add_transceiver_state_input());
        both.add_transceiver_state_input();
        let mut idle = Transceiver::default();
        idle.add_transceiver_state_input();
        let manual = Transceiver {
            transceiver_state: 2,
            ..Transceiver::default()
        };
        sc.transceivers.extend([both, idle, manual]);
        let mut bar = GenericStorage::default();
        storage_unit.connect_to(&mut bar.data_storage_state_in_msg);
        sc.generic_storage.push(bar);

        viz.read_inputs();
        let message = protobuf::spacecraft(&viz.spacecraft[0], &SpacecraftStateMsg::default());
        let states: Vec<i32> = message
            .transceivers
            .iter()
            .map(|transceiver| transceiver.transmit_status)
            .collect();
        assert_eq!(states, vec![3, 0, 2]);
        let bar = &message.storage_devices[0];
        assert_eq!((bar.current_value, bar.max_value), (8.0, 64.0));
    }

    /// Light commands cast to `int` and clamp to -1, 0 or 1, as upstream.
    #[test]
    fn light_commands_clamp_to_a_switch() {
        for (device_cmd, expected) in [
            (0, VizSetting::UseVizDefault),
            (1, VizSetting::Enabled),
            (7, VizSetting::Enabled),
            (u64::from(u32::MAX), VizSetting::Disabled),
        ] {
            let mut viz = VizInterface::new(VizInterfaceConfig::default());
            let command = Output::new(DeviceCommandMsg { device_cmd });
            let sc = viz.add_spacecraft("sat");
            sc.lights.push(Light::default());
            command.connect_to(&mut sc.lights[0].on_off_cmd_in_msg);
            viz.read_inputs();
            assert_eq!(
                viz.spacecraft[0].lights[0].light_on, expected,
                "{device_cmd}"
            );
        }
    }

    /// A charge message whose length differs from the shape list changes no
    /// shape.
    #[test]
    fn mismatched_msm_charges_are_ignored() {
        let mut viz = VizInterface::new(VizInterfaceConfig::default());
        let charges = Output::new(ChargeMsmMsg {
            charges_c: vec![1.0, 2.0, 3.0],
        });
        let sc = viz.add_spacecraft("sat");
        sc.msm_info.msm_list = vec![MultiShape::default(); 2];
        charges.connect_to(&mut sc.msm_info.msm_charge_in_msg);
        viz.read_inputs();
        assert!(
            viz.spacecraft[0]
                .msm_info
                .msm_list
                .iter()
                .all(|shape| shape.current_value_c == 0.0)
        );
    }

    /// Unconnected spacecraft, devices and cameras are left out.
    #[test]
    fn unconnected_inputs_are_left_out() {
        let path = unique_path("unconnected");
        let mut viz = saving_to(&path);
        let sc = viz.add_spacecraft("ghost");
        sc.add_thruster(ThrClusterMap::default());
        viz.add_camera();
        run_frames(&mut viz, 1, SECOND_NANOS);
        drop(viz);
        let saved = read_viz_file(&path).expect("readable Vizard file");
        std::fs::remove_file(&path).expect("removable file");
        assert!(saved[0].spacecraft.is_empty());
        assert!(saved[0].cameras.is_empty());
    }

    /// Camera attitude, units and the depth-map planes.
    #[test]
    fn camera_maps_to_vizard_frame_and_units() {
        let config = CameraConfigMsg {
            camera_id: 2,
            parent_name: "sat".to_string(),
            field_of_view_rad: 0.5,
            resolution_px: [640, 480],
            render_rate_nanos: 123,
            camera_pos_b_m: Vector3::new(0.1, 0.2, 0.3),
            sigma_cb: Vector3::new(0.1, -0.2, 0.3),
            pp_focal_length_m: 0.05,
            depth_map_clipping_planes_m: [0.5, 50.0],
            post_processing_on: VizSetting::Disabled,
            ..CameraConfigMsg::default()
        };
        let camera = protobuf::camera(&config);
        let expected_mrp = add_mrp(config.sigma_cb, Vector3::new(1.0, 1.0, -1.0) / 3.0);
        assert_close(
            &camera.camera_dir_b,
            expected_mrp.as_slice(),
            1e-15,
            "camera MRP",
        );
        assert_close(&[camera.field_of_view], &[0.5 * R2D], 1e-12, "camera fov");
        assert_close(&[camera.pp_focal_length], &[50.0], 1e-12, "focal length mm");
        assert_eq!(camera.resolution, vec![640, 480]);
        assert_eq!(camera.camera_pos_b, vec![0.1, 0.2, 0.3]);
        assert_eq!(camera.render_rate, 123);
        assert_eq!(camera.depth_map_clipping_planes, vec![0.5, 50.0]);
        assert_eq!(camera.post_processing_on, -1);

        let no_planes = protobuf::camera(&CameraConfigMsg::default());
        assert!(no_planes.depth_map_clipping_planes.is_empty());
    }

    /// Celestial bodies in km and km^3/s^2, rotation row by row.
    #[test]
    fn celestial_body_maps_units_and_orientation() {
        let dcm = Matrix3::new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        let planet = PlanetStateMsg {
            position_inertial_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_inertial_mps: Vector3::new(4.0, 5.0, 6.0),
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: dcm,
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
        };
        let body = protobuf::celestial_body(&earth(), &planet);
        assert_close(&[body.mu], &[398_600.441_5], 1e-9, "mu in km^3/s^2");
        assert_close(&[body.radius_eq], &[6378.1366], 1e-9, "radius in km");
        assert_eq!(
            body.rotation,
            vec![0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        );
        let default_body = protobuf::celestial_body(&earth(), &PlanetStateMsg::default());
        assert_eq!(
            default_body.rotation,
            vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    /// Settings conversions: degrees, unset colors and standard cameras.
    #[test]
    fn settings_map_to_vizard_units() {
        let mut settings = crate::vizard::viz_structures::VizSettings {
            keyboard_angular_rate_radps: 2.0 * D2R,
            default_thruster_color: Some([1, 2, 3, 4]),
            osculating_orbit_line_range_rad: Some([-10.4 * D2R, 20.6 * D2R]),
            ..Default::default()
        };
        settings.std_camera_list = vec![
            StdCameraSettings {
                field_of_view_rad: Some(30.0 * D2R),
                position_b_m: Vector3::new(0.0, 0.0, 2.0),
                ..StdCameraSettings::default()
            },
            StdCameraSettings::default(),
        ];
        let pb = protobuf::settings(&settings);
        assert_close(&[pb.keyboard_angular_rate], &[2.0], 1e-12, "key rate deg/s");
        assert_eq!(pb.default_thruster_color, vec![1, 2, 3, 4]);
        assert_eq!(pb.osculating_orbit_line_range, vec![-10, 21]);
        assert!(pb.osculating_ground_track_range.is_empty());
        assert_eq!(pb.ambient, -1.0);
        assert_eq!(pb.force_start_at_spacecraft_local_view, -1);
        assert_close(
            &[pb.standard_camera_settings[0].field_of_view],
            &[30.0],
            1e-12,
            "fov",
        );
        assert_eq!(pb.standard_camera_settings[0].position, vec![0.0, 0.0, 2.0]);
        assert_eq!(pb.standard_camera_settings[0].set_mode, 1);
        assert_eq!(pb.standard_camera_settings[1].field_of_view, -1.0);
        assert!(pb.standard_camera_settings[1].position.is_empty());
        assert_eq!(
            pb.standard_camera_settings[1].pointing_vector,
            vec![1.0, 0.0, 0.0]
        );

        settings.default_thruster_color = None;
        assert!(
            protobuf::settings(&settings)
                .default_thruster_color
                .is_empty()
        );
    }

    #[test]
    #[should_panic(expected = "no_display")]
    fn headless_mode_cannot_also_stream() {
        VizInterface::new(VizInterfaceConfig {
            no_display: true,
            broadcast_stream: true,
            ..VizInterfaceConfig::default()
        });
    }

    #[test]
    fn image_due_needs_a_render_rate_and_the_camera_on() {
        let camera = camera_config(1, 2 * SECOND_NANOS);
        assert!(super::image_due(&camera, 4 * SECOND_NANOS));
        assert!(!super::image_due(&camera, 3 * SECOND_NANOS));
        assert!(!super::image_due(
            &CameraConfigMsg {
                is_on: false,
                ..camera.clone()
            },
            4 * SECOND_NANOS
        ));
        assert!(!super::image_due(&camera_config(1, 0), 4 * SECOND_NANOS));
    }

    #[test]
    fn image_reply_length_must_match_the_image() {
        let image = vec![7_u8; 3];
        assert_eq!(
            super::decode_image(&[3_u32.to_be_bytes().to_vec(), image.clone()]),
            Ok(image.clone())
        );
        assert!(super::decode_image(&[4_u32.to_be_bytes().to_vec(), image.clone()]).is_err());
        assert!(super::decode_image(&[vec![0, 3], image]).is_err());
        assert!(super::decode_image(&[3_u32.to_be_bytes().to_vec()]).is_err());
    }
}
