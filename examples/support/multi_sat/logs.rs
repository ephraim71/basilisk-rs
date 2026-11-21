//! The recorders upstream's MultiSat scenarios add to each spacecraft's
//! dynamics task (`log_outputs`), and their CSV file.
//!
//! [`SpacecraftLogs`] holds one in-memory history per recorded message, as
//! upstream's `msg.recorder(samplingTime)`; [`Logged`] selects the groups a
//! scenario records. Scheduled in the spacecraft's dynamics process after
//! its modules, the histories see the dynamics of the current tick and the
//! flight software of the previous one, as upstream's recorders in the
//! dynamics task. A recorder runs at its sampling time, but no more often
//! than its task (upstream records once per task call).
//!
//! [`SpacecraftLogs::write_csv`] writes
//! `examples/output/scenarios/<stem>.csv` with `time_ns` and, per group:
//!
//! | group | columns |
//! |---|---|
//! | always | `r_BN_N_m_<i>`, `v_BN_N_m_per_s_<i>` (navigation), `sigma_BN_<i>`, `omega_BN_B_rad_per_s_<i>`, `wheelSpeeds_rad_per_s_<w>`, `rwTorque_Nm_<w>` (`u_current`) |
//! | [`Logged::guidance`] | `sigma_BR_<i>`, `omega_BR_B_rad_per_s_<i>`, `motorTorque_Nm_<w>` |
//! | [`Logged::reference`] | `sigma_RN_<i>`, `omega_RN_N_rad_per_s_<i>` |
//! | [`Logged::power`] | `rwPower_W_<w>`, `panelPower_W`, `sinkPower_W`, `storageLevel_Ws`, `netPower_W` |
//! | [`Logged::propulsion`] | `fuelMass_kg`, `thrustForce_B_N_<t>_<i>`, `thrustFactor_<t>` |

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, FuelTankMsg,
    NavigationAttitudeMsg, NavigationTranslationMsg, PowerNodeUsageMsg, PowerStorageStatusMsg,
    ReactionWheelStateMsg, RwSpeedMsg, ThrusterOutputMsg,
};
use basilisk_rs::simulation::Simulation;
use nalgebra::Vector3;

use super::MultiSatSim;
use super::message_log::MessageLog;
use super::table_csv::write_table;

/// The groups of messages a scenario records.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Logged {
    /// `attGuidMsg` and `rwMotorTorqueOutMsg` (needs flight software).
    pub guidance: bool,
    /// `attRefMsg` (needs flight software).
    pub reference: bool,
    /// The power nodes and the battery.
    pub power: bool,
    /// The fuel tank and the thrusters.
    pub propulsion: bool,
}

impl Logged {
    /// Navigation and wheels only (`scenario_BasicOrbitMultiSat`).
    pub const ORBIT: Self = Self {
        guidance: false,
        reference: false,
        power: false,
        propulsion: false,
    };
    /// Plus the guidance (`scenario_AttGuidMultiSat`).
    pub const ATTITUDE: Self = Self {
        guidance: true,
        ..Self::ORBIT
    };
    /// Everything (`scenario_StationKeepingMultiSat`).
    pub const ALL: Self = Self {
        guidance: true,
        reference: true,
        power: true,
        propulsion: true,
    };
}

/// The recorded histories of one spacecraft.
pub struct SpacecraftLogs {
    pub index: usize,
    pub period_nanos: u64,
    /// Upstream `snTransLog`.
    pub trans: MessageLog<NavigationTranslationMsg>,
    /// Upstream `snAttLog`.
    pub att: MessageLog<NavigationAttitudeMsg>,
    /// Upstream `rwSpeedLog`.
    pub rw_speed: MessageLog<RwSpeedMsg>,
    /// Upstream `rwLogs`.
    pub rw_states: Vec<MessageLog<ReactionWheelStateMsg>>,
    /// Upstream `attErrorLog`.
    pub att_guid: Option<MessageLog<AttitudeGuidanceMsg>>,
    /// Upstream `rwMotorLog`.
    pub rw_motor: Option<MessageLog<ArrayMotorTorqueMsg>>,
    /// Upstream `attRefLog`.
    pub att_ref: Option<MessageLog<AttitudeReferenceMsg>>,
    /// Upstream `rwPowerLogs`, `spLog`, `psLog` and `pmLog`.
    pub rw_power: Vec<MessageLog<PowerNodeUsageMsg>>,
    pub panel: Option<MessageLog<PowerNodeUsageMsg>>,
    pub sink: Option<MessageLog<PowerNodeUsageMsg>>,
    pub battery: Option<MessageLog<PowerStorageStatusMsg>>,
    /// Upstream `fuelLog` and `thrLogs`.
    pub fuel: Option<MessageLog<FuelTankMsg>>,
    pub thrusters: Vec<MessageLog<ThrusterOutputMsg>>,
}

fn log_of<T: Clone + Default>(output: &basilisk_rs::messages::Output<T>) -> MessageLog<T> {
    let mut log = MessageLog::default();
    output.connect_to(&mut log.input);
    log
}

impl SpacecraftLogs {
    /// Upstream `log_outputs` for spacecraft `index`, sampled every
    /// `sampling_nanos` (at least once per dynamics step).
    pub fn new(sim: &MultiSatSim, index: usize, sampling_nanos: u64, logged: Logged) -> Self {
        let dynamics = &sim.dynamics[index];
        let fsw = (logged.guidance || logged.reference).then(|| {
            sim.fsw
                .get(index)
                .expect("guidance and reference logs need flight software")
        });
        Self {
            index,
            period_nanos: sampling_nanos.max(dynamics.task_period_nanos),
            trans: log_of(&dynamics.simple_nav.translation_out_msg),
            att: log_of(&dynamics.simple_nav.attitude_out_msg),
            rw_speed: log_of(&dynamics.rw_speed_out_msg),
            rw_states: dynamics.rw_out_msgs.iter().map(log_of).collect(),
            att_guid: logged
                .guidance
                .then(|| log_of(&fsw.expect("checked").gateway.att_guid)),
            rw_motor: logged
                .guidance
                .then(|| log_of(&fsw.expect("checked").gateway.rw_motor_torque)),
            att_ref: logged
                .reference
                .then(|| log_of(&fsw.expect("checked").gateway.att_ref)),
            rw_power: if logged.power {
                dynamics
                    .rw_power
                    .iter()
                    .map(|node| log_of(&node.node_power_out_msg))
                    .collect()
            } else {
                Vec::new()
            },
            panel: logged
                .power
                .then(|| log_of(&dynamics.solar_panel.node_power_out_msg)),
            sink: logged
                .power
                .then(|| log_of(&dynamics.power_sink.node_power_out_msg)),
            battery: logged
                .power
                .then(|| log_of(&dynamics.battery.bat_power_out_msg)),
            fuel: logged
                .propulsion
                .then(|| log_of(&dynamics.fuel_tank_out_msg)),
            thrusters: if logged.propulsion {
                dynamics.thruster_out_msgs.iter().map(log_of).collect()
            } else {
                Vec::new()
            },
        }
    }

    /// Adds the recorders to the spacecraft's dynamics process, after its
    /// modules (upstream's recorder priority -1).
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.period_nanos;
        simulation.add_module("snTransLog", &mut self.trans, period, -1);
        simulation.add_module("snAttLog", &mut self.att, period, -1);
        if let Some(log) = &mut self.att_ref {
            simulation.add_module("attRefLog", log, period, -1);
        }
        if let Some(log) = &mut self.att_guid {
            simulation.add_module("attErrorLog", log, period, -1);
        }
        if let Some(log) = &mut self.rw_motor {
            simulation.add_module("rwMotorLog", log, period, -1);
        }
        simulation.add_module("rwSpeedLog", &mut self.rw_speed, period, -1);
        for log in &mut self.rw_states {
            simulation.add_module("rwLog", log, period, -1);
        }
        for log in &mut self.rw_power {
            simulation.add_module("rwPowerLog", log, period, -1);
        }
        for log in [&mut self.panel, &mut self.sink].into_iter().flatten() {
            simulation.add_module("powerLog", log, period, -1);
        }
        if let Some(log) = &mut self.battery {
            simulation.add_module("pmLog", log, period, -1);
        }
        if let Some(log) = &mut self.fuel {
            simulation.add_module("fuelLog", log, period, -1);
        }
        for log in &mut self.thrusters {
            simulation.add_module("thrLog", log, period, -1);
        }
    }

    /// The navigation positions \[m\] (upstream `snTransLog.r_BN_N`).
    pub fn positions(&self) -> Vec<Vector3<f64>> {
        self.trans
            .values
            .iter()
            .map(|nav| nav.position_bn_n_m)
            .collect()
    }

    /// The navigation velocities \[m/s\].
    pub fn velocities(&self) -> Vec<Vector3<f64>> {
        self.trans
            .values
            .iter()
            .map(|nav| nav.velocity_bn_n_mps)
            .collect()
    }

    /// Writes the histories as `examples/output/scenarios/<stem>.csv`.
    pub fn write_csv(&self, stem: &str) -> PathBuf {
        let wheels = self.rw_states.len();
        let mut headers = vec!["time_ns".to_string()];
        let push3 = |headers: &mut Vec<String>, prefix: &str| {
            headers.extend((0..3).map(|axis| format!("{prefix}_{axis}")));
        };
        push3(&mut headers, "r_BN_N_m");
        push3(&mut headers, "v_BN_N_m_per_s");
        push3(&mut headers, "sigma_BN");
        push3(&mut headers, "omega_BN_B_rad_per_s");
        headers.extend((0..wheels).map(|wheel| format!("wheelSpeeds_rad_per_s_{wheel}")));
        headers.extend((0..wheels).map(|wheel| format!("rwTorque_Nm_{wheel}")));
        if self.att_guid.is_some() {
            push3(&mut headers, "sigma_BR");
            push3(&mut headers, "omega_BR_B_rad_per_s");
            headers.extend((0..wheels).map(|wheel| format!("motorTorque_Nm_{wheel}")));
        }
        if self.att_ref.is_some() {
            push3(&mut headers, "sigma_RN");
            push3(&mut headers, "omega_RN_N_rad_per_s");
        }
        if self.battery.is_some() {
            headers.extend((0..wheels).map(|wheel| format!("rwPower_W_{wheel}")));
            headers.extend(
                [
                    "panelPower_W",
                    "sinkPower_W",
                    "storageLevel_Ws",
                    "netPower_W",
                ]
                .map(String::from),
            );
        }
        if self.fuel.is_some() {
            headers.push("fuelMass_kg".to_string());
            for thruster in 0..self.thrusters.len() {
                push3(&mut headers, &format!("thrustForce_B_N_{thruster}"));
                headers.push(format!("thrustFactor_{thruster}"));
            }
        }

        let rows: Vec<Vec<f64>> = (0..self.trans.values.len())
            .map(|sample| {
                let mut row = vec![self.trans.times_nanos[sample] as f64];
                let trans = &self.trans.values[sample];
                let att = &self.att.values[sample];
                row.extend(trans.position_bn_n_m.iter());
                row.extend(trans.velocity_bn_n_mps.iter());
                row.extend(att.sigma_bn.iter());
                row.extend(att.omega_bn_b_radps.iter());
                row.extend(&self.rw_speed.values[sample].wheel_speeds_radps[..wheels]);
                row.extend(
                    self.rw_states
                        .iter()
                        .map(|log| log.values[sample].applied_motor_torque_nm),
                );
                if let (Some(guid), Some(motor)) = (&self.att_guid, &self.rw_motor) {
                    row.extend(guid.values[sample].sigma_br.iter());
                    row.extend(guid.values[sample].omega_br_b_radps.iter());
                    row.extend(&motor.values[sample].motor_torque_nm[..wheels]);
                }
                if let Some(reference) = &self.att_ref {
                    row.extend(reference.values[sample].sigma_rn.iter());
                    row.extend(reference.values[sample].omega_rn_n_radps.iter());
                }
                if let (Some(panel), Some(sink), Some(battery)) =
                    (&self.panel, &self.sink, &self.battery)
                {
                    row.extend(
                        self.rw_power
                            .iter()
                            .map(|log| log.values[sample].net_power_w),
                    );
                    row.push(panel.values[sample].net_power_w);
                    row.push(sink.values[sample].net_power_w);
                    row.push(battery.values[sample].storage_level_j);
                    row.push(battery.values[sample].current_net_power_w);
                }
                if let Some(fuel) = &self.fuel {
                    row.push(fuel.values[sample].fuel_mass_kg);
                    for thruster in &self.thrusters {
                        row.extend(thruster.values[sample].thrust_force_b_n.iter());
                        row.push(thruster.values[sample].thrust_factor);
                    }
                }
                row
            })
            .collect();
        write_table(stem, &headers, &rows)
    }
}

/// Writes a navigation translation history (the barycenter's) as
/// `examples/output/scenarios/<stem>.csv` with `time_ns`, `r_BN_N_m_<i>` and
/// `v_BN_N_m_per_s_<i>`.
pub fn write_translation_csv(stem: &str, log: &MessageLog<NavigationTranslationMsg>) -> PathBuf {
    let mut headers = vec!["time_ns".to_string()];
    for prefix in ["r_BN_N_m", "v_BN_N_m_per_s"] {
        headers.extend((0..3).map(|axis| format!("{prefix}_{axis}")));
    }
    let rows: Vec<Vec<f64>> = log
        .times_nanos
        .iter()
        .zip(&log.values)
        .map(|(time, nav)| {
            let mut row = vec![*time as f64];
            row.extend(nav.position_bn_n_m.iter());
            row.extend(nav.velocity_bn_n_mps.iter());
            row
        })
        .collect();
    write_table(stem, &headers, &rows)
}
