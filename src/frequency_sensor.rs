// meh??
// #![feature(stdsimd)]
// // #![cfg(target_feature = "simd128")]
// #![cfg(target_arch = "wasm32")]

/// Features contain the output of the frequency sensor module.
/// `amplitudes` is the lowpass-filtered magnitude of each bucket over the time of [length] frames.
/// `scales` are calculated based on a running variance of the amplitude in an attempt to
/// keep scale[i] * amplitude[n][i] mostly in the range of (-1, 1).
/// `diff` is the lowpass-filtered magnitude of the difference of each new frame minus the prior.
/// `energy` is the accumulation of diff over time.
#[derive(Clone, Debug)]
pub struct Features {
    amplitudes: Vec<Vec<f32>>,
    scales: Vec<f32>,
    diff: Vec<f32>,
    energy: Vec<f32>,

    size: usize,
    length: usize,
    index: usize,
}

impl Features {
    fn new(size: usize, length: usize) -> Self {
        Self {
            size,
            length,
            amplitudes: (0..length).map(|_| vec![0f32; size]).collect(),
            scales: vec![0f32; size],
            diff: vec![0f32; size],
            energy: vec![0f32; size],
            index: 0,
        }
    }

    fn increment_index(&mut self) {
        self.index += 1;
        self.index %= self.length;
    }

    fn current_index(&self, i: usize) -> usize {
        let mut i = self.index as i32 - (i % self.length) as i32;
        if i < 0 {
            i += self.length as i32;
        }
        i as usize
    }

    pub fn get_amplitudes(&self, i: usize) -> &Vec<f32> {
        &self.amplitudes[self.current_index(i)]
    }

    fn get_amplitudes_mut(&mut self, i: usize) -> &mut Vec<f32> {
        let i = self.current_index(i);
        &mut self.amplitudes[i]
    }

    pub fn get_scales(&self) -> &Vec<f32> {
        &self.scales
    }

    pub fn get_diff(&self) -> &Vec<f32> {
        &self.diff
    }

    pub fn get_energy(&self) -> &Vec<f32> {
        &self.energy
    }
}

use super::params::{FilterParams, FrequencySensorParams, GainControllerParams};

/// Filter implements a bank of N single pole IIR filters that process a frame
/// in parallel.
struct Filter {
    values: Vec<f32>,
    params: FilterParams,
}

impl Filter {
    fn new(size: usize, params: FilterParams) -> Filter {
        Filter {
            values: vec![0f32; size],
            params,
        }
    }

    // fn process_simd(&self, input: &Vec<f32>) {
    //     unsafe {
    //         let a: v128 = std::mem::transmute([self.params.a; 4]);
    //         let b: v128 = std::mem::transmute([self.params.b; 4]);
    //     }

    //     let mut i = 0;
    //     let len = input.len() as i32 - 4;
    //     while i < len {
    //         unsafe {
    //             let v_in: v128 = std::mem::transmute(input[i..i + 4]);
    //             let v_val: v128 = std::mem::transmute(self.values[i..i + 4]);
    //             let v = f32x4_add(f32x4_mul(a, v_in), f32x4_mul(b, v_val));
    //             self.values[i..i + 4] = std::mem::transmute(v);
    //         }
    //         i += 4;
    //     }
    // }

    fn process(&mut self, input: &Vec<f32>) {
        for i in 0..input.len() {
            self.values[i] = self.params.a * input[i] + self.params.b * self.values[i];
        }
    }
}

/// BiasedFilter uses separate coefficients depending on whether the input is greater or
/// less than the current value.
struct BiasedFilter {
    values: Vec<f32>,
    pos_params: FilterParams,
    neg_params: FilterParams,
}

impl BiasedFilter {
    fn new(size: usize, pos_params: FilterParams, neg_params: FilterParams) -> BiasedFilter {
        BiasedFilter {
            values: vec![0f32; size],
            pos_params,
            neg_params,
        }
    }

    fn process(&mut self, input: &Vec<f32>) {
        for i in 0..input.len() {
            let params = if input[i] < self.values[i] {
                &self.pos_params
            } else {
                &self.neg_params
            };
            self.values[i] = params.a * input[i] + params.b * self.values[i];
        }
    }
}

/// GainController is a PD controller which adjusts gain with a target value of 1.
struct GainController {
    filter: Filter,
    values: Vec<f32>,
    err: Vec<f32>,
    params: GainControllerParams,
}

impl GainController {
    fn new(size: usize, params: GainControllerParams) -> GainController {
        GainController {
            filter: Filter::new(size, params.filter_params),
            values: vec![1f32; size],
            err: vec![0f32; size],
            params,
        }
    }

    fn log_error(x: f32) -> f32 {
        let x = 0.000000001f32 + x;
        let l = x.abs().log2();
        -x.signum() * l * l * l.signum()
    }

    fn process(&mut self, input: &mut Vec<f32>) {
        for i in 0..input.len() {
            input[i] *= self.values[i] * self.params.pre_gain;
        }

        self.filter.process(input);

        for i in 0..input.len() {
            let e = GainController::log_error(self.filter.values[i]);
            // integrate error
            self.err[i] = 0.99 * self.err[i] + 0.01 * e;

            let u = self.params.kp * e + self.params.ki * self.err[i];
            self.values[i] = match self.values[i] + u {
                x if x > 1e8 => 1e8,
                x if x < 1e-6 => 1e-6,
                x => x,
            };
        }
    }
}

/// FrequencySensor maintains a `Features` vector that tracks incoming frames.
pub struct FrequencySensor {
    params: FrequencySensorParams,
    features: Features,

    gain_controller: GainController,
    amp_filter: Filter,
    amp_feedback: Filter,
    diff_filter: Filter,
    diff_feedback: Filter,
    scale_filter: BiasedFilter,

    size: usize,

    scale_buffer: Vec<f32>,
    diff_buffer: Vec<f32>,
}

impl FrequencySensor {
    pub fn new(size: usize, length: usize, params: FrequencySensorParams) -> FrequencySensor {
        FrequencySensor {
            params,
            size,
            features: Features::new(size, length),
            gain_controller: GainController::new(size, params.gain_control),
            amp_filter: Filter::new(size, params.amp_filter),
            amp_feedback: Filter::new(size, params.amp_feedback),
            diff_filter: Filter::new(size, params.diff_filter),
            diff_feedback: Filter::new(size, params.diff_feedback),
            scale_filter: BiasedFilter::new(size, params.pos_scale_filter, params.neg_scale_filter),
            scale_buffer: vec![0f32; size],
            diff_buffer: vec![0f32; size],
        }
    }

    /// get_features returns the current features vector
    pub fn get_features(&self) -> &Features {
        &self.features
    }

    /// set_params updates the sensors params
    pub fn set_params(&mut self, fp: FrequencySensorParams) {
        self.params = fp;
    }

    /// process updates the features vector
    pub fn process(&mut self, input: &mut Vec<f32>) {
        self.apply_preemphasis(input);
        self.apply_gain_control(input);
        self.apply_filters(input);
        self.apply_effects();
        self.apply_sync();
        self.apply_value_scaling();
    }

    pub fn print_debug(&self) {
        let feat = self.get_features();
        println!("{{");

        println!("\t\"amplitudes\":    {},", VecF32(feat.get_amplitudes(0)));
        println!("\t\"energy\":        {},", VecF32(feat.get_energy()));
        println!("\t\"diff\":          {},", VecF32(feat.get_diff()));
        println!("\t\"scales\":        {},", VecF32(feat.get_scales()));

        println!(
            "\t\"gain\":          {},",
            VecF32(&self.gain_controller.values)
        );
        println!(
            "\t\"gain_err\":      {},",
            VecF32(&self.gain_controller.err)
        );

        println!("\t\"amp_filter\":    {},", VecF32(&self.amp_filter.values));
        println!(
            "\t\"amp_feedback\":  {},",
            VecF32(&self.amp_feedback.values)
        );
        println!("\t\"diff_filter\":   {},", VecF32(&self.diff_filter.values));
        println!(
            "\t\"diff_feedback\": {}",
            VecF32(&self.diff_feedback.values)
        );

        println!("}}");
    }

    fn apply_preemphasis(&mut self, input: &mut Vec<f32>) {
        let incr = (self.params.preemphasis - 1.) / self.size as f32;
        for i in 0..self.size {
            input[i] *= 1. + i as f32 * incr;
        }
    }

    fn apply_gain_control(&mut self, input: &mut Vec<f32>) {
        self.gain_controller.process(input);
    }

    fn apply_filters(&mut self, input: &Vec<f32>) {
        self.diff_buffer.copy_from_slice(input);

        self.amp_filter.process(input);
        self.amp_feedback.process(input);

        for i in 0..self.size {
            self.diff_buffer[i] = self.amp_filter.values[i] - self.diff_buffer[i];
        }

        self.diff_filter.process(&self.diff_buffer);
        self.diff_feedback.process(&self.diff_buffer);
    }

    fn apply_effects(&mut self) {
        let dg = self.params.diff_gain;
        let ag = self.params.amp_scale;
        let ao = self.params.amp_offset;

        self.features.increment_index();
        {
            let amp = self.features.get_amplitudes_mut(0);
            for i in 0..self.size {
                amp[i] = ao + ag * (self.amp_filter.values[i] + self.amp_feedback.values[i]);
            }
        }
        for i in 0..self.size {
            let diff = dg * (self.diff_filter.values[i] + self.diff_feedback.values[i]);
            self.features.diff[i] = diff;
            self.features.energy[i] = self.features.energy[i] + diff - self.params.drag;
        }
    }

    fn apply_sync(&mut self) {
        use std::f32::consts::PI;
        let twopi = 2. * PI;

        let energy = &mut self.features.energy;
        let size_f = self.size as f32;
        let mean = energy.iter().sum::<f32>() / size_f;

        let sync = self.params.sync;
        for i in 0..self.size {
            if i > 0 {
                energy[i] += sync * FrequencySensor::signed_square_diff(energy[i - 1], energy[i]);
            }

            if i < (self.size - 1) {
                energy[i] += sync * FrequencySensor::signed_square_diff(energy[i + 1], energy[i]);
            }

            energy[i] += (sync / size_f) * FrequencySensor::signed_square_diff(mean, energy[i]);
        }

        let mean = energy.iter().sum::<f32>() / size_f;

        if mean < -twopi {
            // wait until all elements go past the mark so theres no sign flips
            if energy.iter().any(|&x| x > -twopi) {
                return;
            }
            for i in 0..self.size {
                energy[i] = twopi + energy[i];
            }
        }
        if mean > twopi {
            if energy.iter().any(|&x| x < twopi) {
                return;
            }
            for i in 0..self.size {
                energy[i] -= twopi;
            }
        }
    }

    fn apply_value_scaling(&mut self) {
        let amp = self.features.get_amplitudes(0);

        for i in 0..self.size {
            self.scale_buffer[i] = (self.features.scales[i] * (amp[i] - 1.)).abs();
        }

        self.scale_filter.process(&self.scale_buffer);

        for i in 0..self.size {
            let mut vsh = self.scale_filter.values[i];
            if vsh < 0.001 {
                vsh = 0.001;
            }
            let vs = 1. / vsh;
            self.scale_filter.values[i] = vsh;
            self.features.scales[i] = vs;
        }
    }

    fn signed_square_diff(a: f32, b: f32) -> f32 {
        let diff = a - b;
        diff.signum() * diff * diff
    }
}

use std::fmt::{Display, Error, Formatter};

struct VecF32<'a>(&'a Vec<f32>);

impl<'a> VecF32<'a> {
    fn fmt_num(num: f32) -> String {
        format!(
            "{:>6}.{:06}",
            num as i32,
            ((num.abs() % 1.0) * 1000000.) as i32
        )
    }
}

impl<'a> Display for VecF32<'a> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        let mut comma_separated = String::new();

        for &num in &self.0[0..self.0.len() - 1] {
            comma_separated.push_str(&VecF32::fmt_num(num));
            comma_separated.push_str(", ");
        }
        let num = self.0[self.0.len() - 1];
        comma_separated.push_str(&VecF32::fmt_num(num));

        write!(f, "[ {} ]", comma_separated)
    }
}
