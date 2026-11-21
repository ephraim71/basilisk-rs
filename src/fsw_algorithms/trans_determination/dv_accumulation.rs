//! Accumulated delta-v from accelerometer packets: port of upstream Basilisk
//! `dvAccumulation`.
//!
//! Every update sorts the buffered accelerometer packets by measurement time
//! and integrates the acceleration of each packet newer than the last one
//! processed, `Δv += (t_k - t_prev) a_k`, in body components. The output
//! navigation message carries the total and the time tag of the newest packet
//! used; its position and velocity stay zero.
//!
//! Reset zeroes the total and takes the newest packet then in the buffer as
//! already processed, so stale packets are never integrated. On the first
//! update after a reset, the first newer packet only starts the integration:
//! its own acceleration is not integrated, so that the first time step is not
//! measured from an arbitrary earlier time.

use nalgebra::Vector3;

use crate::messages::{
    AccelerometerDataMsg, AccelerometerPacket, Input, NavigationTranslationMsg, Output,
};
use crate::time::NANO2SEC;
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`DvAccumulation`]; upstream has none.
#[derive(Clone, Debug)]
pub struct DvAccumulationConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for DvAccumulationConfig {
    fn default() -> Self {
        Self {
            name: "dvAccumulation".to_string(),
        }
    }
}

/// Delta-v accumulator: upstream `dvAccumulation`.
#[derive(Clone, Debug)]
pub struct DvAccumulation {
    /// Module configuration.
    pub config: DvAccumulationConfig,
    /// Upstream `accPktInMsg`: buffered accelerometer packets.
    pub acc_pkt_in_msg: Input<AccelerometerDataMsg>,
    /// Upstream `dvAcumOutMsg`: accumulated delta-v and its time tag.
    pub dv_acum_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `dvInitialized`: whether a packet newer than the reset buffer
    /// has started the integration.
    dv_initialized: bool,
    /// Upstream `previousTime` \[ns\]: measurement time of the newest packet
    /// processed.
    previous_time_nanos: u64,
    /// Upstream `vehAccumDV_B` \[m/s\]: accumulated delta-v, body components.
    veh_accum_dv_b_mps: Vector3<f64>,
}

impl DvAccumulation {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: DvAccumulationConfig) -> Self {
        Self {
            config,
            acc_pkt_in_msg: Input::default(),
            dv_acum_out_msg: Output::default(),
            dv_initialized: false,
            previous_time_nanos: 0,
            veh_accum_dv_b_mps: Vector3::zeros(),
        }
    }

    /// Upstream `Reset_dvAccumulation`.
    fn reset_accumulator(&mut self) {
        if !self.acc_pkt_in_msg.is_connected() {
            log::error!("dvAccumulation.acc_pkt_in_msg is not connected");
        }
        let mut input = self.acc_pkt_in_msg.read();
        quick_sort(&mut input.accel_packets);

        self.veh_accum_dv_b_mps = Vector3::zeros();
        self.dv_initialized = false;
        // The newest valid time tag in the buffer; zero marks an empty slot.
        self.previous_time_nanos = input
            .accel_packets
            .iter()
            .rev()
            .map(|packet| packet.meas_time_nanos)
            .find(|&time| time > 0)
            .unwrap_or(0);
    }

    /// Upstream `Update_dvAccumulation`: integrates the new packets.
    fn accumulate(&mut self) -> NavigationTranslationMsg {
        let mut input = self.acc_pkt_in_msg.read();
        quick_sort(&mut input.accel_packets);

        if !self.dv_initialized
            && let Some(first_new) = input
                .accel_packets
                .iter()
                .find(|packet| packet.meas_time_nanos > self.previous_time_nanos)
        {
            self.previous_time_nanos = first_new.meas_time_nanos;
            self.dv_initialized = true;
        }

        for packet in &input.accel_packets {
            if packet.meas_time_nanos > self.previous_time_nanos {
                let dt_s = diff_nanos_to_seconds(packet.meas_time_nanos, self.previous_time_nanos);
                self.veh_accum_dv_b_mps += packet.accel_b_mps2 * dt_s;
                self.previous_time_nanos = packet.meas_time_nanos;
            }
        }

        NavigationTranslationMsg {
            time_tag_s: self.previous_time_nanos as f64 * NANO2SEC,
            accumulated_dv_b_mps: self.veh_accum_dv_b_mps,
            ..Default::default()
        }
    }
}

/// Sorts packets by ascending measurement time: port of upstream
/// `dvAccumulation_QuickSort` over the whole slice.
///
/// This is upstream's iterative quicksort with a Lomuto partition about the
/// last element. It is not stable, and the order it leaves packets with equal
/// time tags in decides which of them [`DvAccumulation`] integrates, so it is
/// ported as is rather than replaced by `sort_by_key`. Upstream's fixed-size
/// index stack, and the error it prints for ranges longer than
/// `MAX_ACC_BUF_PKT`, are not needed here.
pub fn quick_sort(packets: &mut [AccelerometerPacket]) {
    if packets.len() < 2 {
        return;
    }
    let mut stack = vec![(0, packets.len() - 1)];
    while let Some((start, end)) = stack.pop() {
        let partition_index = partition(packets, start, end);
        if partition_index > start + 1 {
            stack.push((start, partition_index - 1));
        }
        if partition_index + 1 < end {
            stack.push((partition_index + 1, end));
        }
    }
}

/// Upstream `dvAccumulation_partition`: moves every packet no later than
/// `packets[end]` before it and returns the pivot's final index.
fn partition(packets: &mut [AccelerometerPacket], start: usize, end: usize) -> usize {
    let pivot = packets[end].meas_time_nanos;
    let mut partition_index = start;
    for i in start..end {
        if packets[i].meas_time_nanos <= pivot {
            packets.swap(i, partition_index);
            partition_index += 1;
        }
    }
    packets.swap(partition_index, end);
    partition_index
}

impl Module for DvAccumulation {
    fn init(&mut self) {
        self.dv_acum_out_msg
            .write(NavigationTranslationMsg::default());
        self.reset_accumulator();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_accumulator();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let output = self.accumulate();
        self.dv_acum_out_msg.write(output);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    use super::{DvAccumulation, DvAccumulationConfig, quick_sort};
    use crate::messages::{
        AccelerometerDataMsg, AccelerometerPacket, MAX_ACC_BUF_PKT, NavigationTranslationMsg,
        Output,
    };
    use crate::{Module, SimulationContext};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// numpy's legacy `RandomState` (MT19937 with `init_genrand` seeding and
    /// the cached polar Box-Muller `gauss`), which `numpy.random.seed` and
    /// `numpy.random.normal` use. The upstream truth values depend on its
    /// exact stream.
    struct NumpyRandomState {
        key: [u32; 624],
        pos: usize,
        cached_gauss: Option<f64>,
    }

    impl NumpyRandomState {
        fn seed(seed: u32) -> Self {
            let mut key = [0_u32; 624];
            key[0] = seed;
            for i in 1..624 {
                key[i] = 1_812_433_253_u32
                    .wrapping_mul(key[i - 1] ^ (key[i - 1] >> 30))
                    .wrapping_add(i as u32);
            }
            Self {
                key,
                pos: 624,
                cached_gauss: None,
            }
        }

        fn next_u32(&mut self) -> u32 {
            const UPPER_MASK: u32 = 0x8000_0000;
            const LOWER_MASK: u32 = 0x7fff_ffff;
            const MATRIX_A: u32 = 0x9908_b0df;
            if self.pos == 624 {
                for i in 0..624 {
                    let y = (self.key[i] & UPPER_MASK) | (self.key[(i + 1) % 624] & LOWER_MASK);
                    let mag = if y & 1 == 1 { MATRIX_A } else { 0 };
                    self.key[i] = self.key[(i + 397) % 624] ^ (y >> 1) ^ mag;
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

        /// `legacy_double`: 53 random bits in `[0, 1)`.
        fn next_double(&mut self) -> f64 {
            let a = f64::from(self.next_u32() >> 5);
            let b = f64::from(self.next_u32() >> 6);
            (a * 67_108_864.0 + b) / 9_007_199_254_740_992.0
        }

        /// `legacy_gauss`.
        fn gauss(&mut self) -> f64 {
            if let Some(gauss) = self.cached_gauss.take() {
                return gauss;
            }
            loop {
                let x1 = 2.0 * self.next_double() - 1.0;
                let x2 = 2.0 * self.next_double() - 1.0;
                let r2 = x1 * x1 + x2 * x2;
                if r2 < 1.0 && r2 != 0.0 {
                    let f = (-2.0 * r2.ln() / r2).sqrt();
                    self.cached_gauss = Some(f * x1);
                    return f * x2;
                }
            }
        }

        /// `numpy.random.normal(loc, scale)`.
        fn normal(&mut self, loc: f64, scale: f64) -> f64 {
            loc + scale * self.gauss()
        }
    }

    /// Upstream `generateAccData`: 120 packets with time tags drawn from
    /// N(5e7, 1e7) ns and accelerations from N(0.1, 0.2) m/s^2.
    fn generate_acc_data(rng: &mut NumpyRandomState) -> AccelerometerDataMsg {
        let mut data = AccelerometerDataMsg::default();
        for packet in &mut data.accel_packets {
            // Python's abs(int(x)) truncates toward zero.
            packet.meas_time_nanos = rng.normal(5e7, 1e7).trunc().abs() as u64;
            packet.accel_b_mps2 = Vector3::from_fn(|_, _| rng.normal(0.1, 0.2));
        }
        data
    }

    fn packets_with_times(times: &[u64]) -> AccelerometerDataMsg {
        let mut data = AccelerometerDataMsg::default();
        for (packet, &time) in data.accel_packets.iter_mut().zip(times) {
            packet.meas_time_nanos = time;
        }
        data
    }

    fn sorted_times(data: &AccelerometerDataMsg) -> Vec<u64> {
        data.accel_packets
            .iter()
            .map(|p| p.meas_time_nanos)
            .collect()
    }

    /// Upstream `dvAccumulationTestFunction`, quicksort part: random and fully
    /// inverted time tags come out in ascending order. Upstream draws the
    /// random tags from numpy's unseeded generator; any random draw serves.
    #[test]
    fn matches_upstream_quick_sort_check() {
        let mut rng = StdRng::seed_from_u64(7);
        let random_times: Vec<u64> = (0..MAX_ACC_BUF_PKT)
            .map(|_| rng.gen_range(0..1_000_000))
            .collect();
        let inverted_times: Vec<u64> = (0..MAX_ACC_BUF_PKT)
            .map(|i| (MAX_ACC_BUF_PKT - i) as u64)
            .collect();
        for times in [random_times, inverted_times] {
            let mut data = packets_with_times(&times);
            quick_sort(&mut data.accel_packets);
            let mut expected = times.clone();
            expected.sort_unstable();
            assert_eq!(sorted_times(&data), expected);
        }
    }

    /// Upstream `dvAccumulationTestFunction`, module part: two random packet
    /// buffers from numpy seed 12345, the first written after the reset and
    /// run for 0..1 s, the second run for 1.5..2 s.
    #[test]
    fn matches_upstream_dv_accumulation_unit_test() {
        let input = Output::new(AccelerometerDataMsg::default());
        let mut module = DvAccumulation::new(DvAccumulationConfig::default());
        input.connect_to(&mut module.acc_pkt_in_msg);

        let mut rng = NumpyRandomState::seed(12345);
        module.init();
        input.write(generate_acc_data(&mut rng));
        let mut samples: Vec<NavigationTranslationMsg> = Vec::new();
        for tick in 0..=2 {
            module.update(&context(tick * TASK_NANOS));
            samples.push(module.dv_acum_out_msg.read());
        }
        input.write(generate_acc_data(&mut rng));
        for tick in 3..=4 {
            module.update(&context(tick * TASK_NANOS));
            samples.push(module.dv_acum_out_msg.read());
        }

        let true_dv_vector = [
            [4.82820079e-03, 7.81971465e-03, 2.29605663e-03],
            [4.82820079e-03, 7.81971465e-03, 2.29605663e-03],
            [4.82820079e-03, 7.81971465e-03, 2.29605663e-03],
            [6.44596343e-03, 9.00203561e-03, 2.60580728e-03],
            [6.44596343e-03, 9.00203561e-03, 2.60580728e-03],
        ];
        let true_time = [
            7.2123026e+07,
            7.2123026e+07,
            7.2123026e+07,
            7.6667436e+07,
            7.6667436e+07,
        ]
        .map(|nanos| nanos * 1.0e-9);
        let accuracy = 1e-6;
        for ((sample, true_dv), true_time) in samples.iter().zip(&true_dv_vector).zip(true_time) {
            let error = (sample.accumulated_dv_b_mps - Vector3::from(*true_dv)).amax();
            assert!(
                error < accuracy,
                "{} != {true_dv:?}",
                sample.accumulated_dv_b_mps
            );
            assert!((sample.time_tag_s - true_time).abs() < accuracy);
            assert_eq!(sample.position_bn_n_m, Vector3::zeros());
            assert_eq!(sample.velocity_bn_n_mps, Vector3::zeros());
        }
    }

    fn packet(time: u64, accel: [f64; 3]) -> AccelerometerPacket {
        AccelerometerPacket {
            meas_time_nanos: time,
            accel_b_mps2: Vector3::from(accel),
            ..Default::default()
        }
    }

    fn buffer(packets: &[AccelerometerPacket]) -> AccelerometerDataMsg {
        let mut data = AccelerometerDataMsg::default();
        data.accel_packets[..packets.len()].copy_from_slice(packets);
        data
    }

    // Not upstream: hand-derived integration. Packets buffered at reset are
    // never integrated; afterwards each newer packet adds dt * a, with dt
    // measured from the previous packet.
    #[test]
    fn integrates_only_packets_newer_than_the_reset_buffer() {
        let input = Output::new(buffer(&[packet(2_000_000_000, [9.0, 9.0, 9.0])]));
        let mut module = DvAccumulation::new(DvAccumulationConfig::default());
        input.connect_to(&mut module.acc_pkt_in_msg);
        module.init();

        input.write(buffer(&[
            packet(2_500_000_000, [1.0, 0.0, 0.0]),
            packet(1_000_000_000, [5.0, 5.0, 5.0]),
            packet(3_500_000_000, [0.0, 2.0, 0.0]),
            packet(2_000_000_000, [9.0, 9.0, 9.0]),
        ]));
        module.update(&context(0));
        // The 2.0 s packet was buffered at reset. The first newer packet
        // (2.5 s) only starts the integration, so just the 3.5 s packet is
        // integrated, over 1 s.
        let output = module.dv_acum_out_msg.read();
        assert_eq!(output.accumulated_dv_b_mps, Vector3::new(0.0, 2.0, 0.0));
        assert_eq!(output.time_tag_s, 3.5);

        input.write(buffer(&[packet(4_000_000_000, [0.0, 0.0, -4.0])]));
        module.update(&context(TASK_NANOS));
        let output = module.dv_acum_out_msg.read();
        assert_eq!(output.accumulated_dv_b_mps, Vector3::new(0.0, 2.0, -2.0));
        assert_eq!(output.time_tag_s, 4.0);

        // Reset zeroes the total and skips everything already buffered.
        module.reset(&context(TASK_NANOS));
        module.update(&context(2 * TASK_NANOS));
        let output = module.dv_acum_out_msg.read();
        assert_eq!(output.accumulated_dv_b_mps, Vector3::zeros());
        assert_eq!(output.time_tag_s, 4.0);
    }

    // Not upstream: of packets with equal time tags only the first in the
    // sorted order is integrated, and upstream's quicksort decides that order.
    #[test]
    fn integrates_one_of_several_packets_with_the_same_time_tag() {
        let input = Output::new(AccelerometerDataMsg::default());
        let mut module = DvAccumulation::new(DvAccumulationConfig::default());
        input.connect_to(&mut module.acc_pkt_in_msg);
        module.init();
        let packets = [
            packet(1_000_000_000, [0.0, 0.0, 0.0]),
            packet(2_000_000_000, [1.0, 0.0, 0.0]),
            packet(2_000_000_000, [0.0, 1.0, 0.0]),
        ];
        let mut sorted = buffer(&packets);
        quick_sort(&mut sorted.accel_packets);
        let first_at_two_seconds = sorted
            .accel_packets
            .iter()
            .find(|p| p.meas_time_nanos == 2_000_000_000)
            .map(|p| p.accel_b_mps2)
            .expect("two packets carry that tag");

        input.write(buffer(&packets));
        module.update(&context(0));
        assert_eq!(
            module.dv_acum_out_msg.read().accumulated_dv_b_mps,
            first_at_two_seconds
        );
    }

    #[test]
    fn quick_sort_handles_short_slices() {
        let mut empty: [AccelerometerPacket; 0] = [];
        quick_sort(&mut empty);
        let mut single = [packet(5, [0.0; 3])];
        quick_sort(&mut single);
        assert_eq!(single[0].meas_time_nanos, 5);
    }
}
