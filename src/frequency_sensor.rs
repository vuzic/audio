// meh??
// #![feature(stdsimd)]
// // #![cfg(target_feature = "simd128")]
// #![cfg(target_arch = "wasm32")]
use core::fmt::Write;

use serde::{Deserialize, Serialize};

use crate::filter::*;
use crate::gain_control::{GainController, Params as GainControllerParams};

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct FrequencySensorParams {
    pub preemphasis: f64,
    pub diff_gain: f64,
    pub amp_scale: f64,
    pub amp_offset: f64,
    pub sync: f64,
    pub drag: f64,
    pub amp_filter: FilterParams,
    pub amp_feedback: FilterParams,
    pub diff_filter: FilterParams,
    pub diff_feedback: FilterParams,
    pub pos_scale_filter: FilterParams,
    pub neg_scale_filter: FilterParams,

    pub gain_control: GainControllerParams,
}

impl Default for FrequencySensorParams {
    fn default() -> Self {
        Self {
            amp_filter: FilterParams::new(8., 1.),
            amp_feedback: FilterParams::new(200., -1.),
            diff_filter: FilterParams::new(16., 1.),
            diff_feedback: FilterParams::new(100., -0.05),
            gain_control: GainControllerParams {
                pre_gain: 1.0,
                ki: 0.1,
                kp: 0.1,
                kd: 0.1,
                filter_params: FilterParams::new(1720., 1.),
            },
            amp_offset: 0.,
            preemphasis: 2.,
            sync: 0.001,
            amp_scale: 1.,
            diff_gain: 1.,
            drag: 0.001,
            pos_scale_filter: FilterParams::new(100., 1.),
            neg_scale_filter: FilterParams::new(1000., 1.),
        }
    }
}

/// Features contain the output of the frequency sensor module.
/// `amplitudes` is the lowpass-filtered magnitude of each bucket over the time of [length] frames.
/// `scales` are calculated based on a running variance of the amplitude in an attempt to
/// keep scale[i] * amplitude[n][i] mostly in the range of (-1, 1).
/// `diff` is the lowpass-filtered magnitude of the difference of each new frame minus the prior.
/// `energy` is the accumulation of diff over time.
#[derive(Clone, Debug, Default)]
pub struct Features {
    amplitudes: Vec<Vec<f64>>,
    scales: Vec<f64>,
    diff: Vec<f64>,
    energy: Vec<f64>,

    size: usize,
    length: usize,
    index: usize,

    frame_count: usize,
}

impl Features {
    pub fn new(size: usize, length: usize) -> Self {
        Self {
            size,
            length,
            amplitudes: (0..length).map(|_| vec![0f64; size]).collect(),
            scales: vec![0f64; size],
            diff: vec![0f64; size],
            energy: vec![0f64; size],
            index: 0,
            frame_count: 0,
        }
    }

    fn increment_index(&mut self) {
        self.frame_count += 1;
        self.index = self.frame_count % self.length;
    }

    fn current_index(&self, i: usize) -> usize {
        let mut i = self.index as i32 - (i % self.length) as i32;
        if i < 0 {
            i += self.length as i32;
        }
        i as usize
    }

    pub fn get_amplitudes(&self, i: usize) -> &Vec<f64> {
        &self.amplitudes[self.current_index(i)]
    }

    fn get_amplitudes_mut(&mut self, i: usize) -> &mut Vec<f64> {
        let i = self.current_index(i);
        &mut self.amplitudes[i]
    }

    pub fn get_scales(&self) -> &Vec<f64> {
        &self.scales
    }

    pub fn get_diff(&self) -> &Vec<f64> {
        &self.diff
    }

    pub fn get_energy(&self) -> &Vec<f64> {
        &self.energy
    }

    pub fn get_frame_count(&self) -> usize {
        self.frame_count
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

    scale_buffer: Vec<f64>,
    diff_buffer: Vec<f64>,
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
            scale_buffer: vec![0f64; size],
            diff_buffer: vec![0f64; size],
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
    pub fn process(&mut self, input: &mut Vec<f64>) {
        self.features.frame_count += 1;
        self.apply_preemphasis(input);
        self.apply_gain_control(input);
        self.apply_filters(input);
        self.apply_effects();
        self.apply_sync();
        self.apply_value_scaling();
    }

    pub fn write_debug<W>(&self, w: &mut W) -> core::fmt::Result
    where
        W: Write,
    {
        let feat = self.get_features();
        // writeln!(w, "{{")?;

        writeln!(w, "\t\"frame_count\":   {},", feat.frame_count)?;

        self.gain_controller.get_state().write_debug(w)?;

        writeln!(
            w,
            "\t\"amplitudes\":    {},",
            VecFmt(feat.get_amplitudes(0))
        )?;
        writeln!(w, "\t\"energy\":        {},", VecFmt(feat.get_energy()))?;
        writeln!(w, "\t\"diff\":          {},", VecFmt(feat.get_diff()))?;
        writeln!(w, "\t\"scales\":        {},", VecFmt(feat.get_scales()))?;

        writeln!(
            w,
            "\t\"amp_filter\":    {},",
            VecFmt(self.amp_filter.get_values())
        )?;
        writeln!(
            w,
            "\t\"amp_feedback\":  {},",
            VecFmt(self.amp_feedback.get_values())
        )?;
        writeln!(
            w,
            "\t\"diff_filter\":   {},",
            VecFmt(self.diff_filter.get_values())
        )?;
        writeln!(
            w,
            "\t\"diff_feedback\": {}",
            VecFmt(self.diff_feedback.get_values())
        )

        // writeln!(w, "}}")
    }

    fn apply_preemphasis(&mut self, input: &mut Vec<f64>) {
        let incr = (self.params.preemphasis - 1.) / self.size as f64;
        for i in 0..self.size {
            input[i] *= 1. + i as f64 * incr;
        }
    }

    fn apply_gain_control(&mut self, input: &mut Vec<f64>) {
        self.gain_controller.process(input);
    }

    fn apply_filters(&mut self, input: &Vec<f64>) {
        self.diff_buffer.copy_from_slice(input);

        self.amp_filter.process(input);
        self.amp_feedback.process(input);

        let amp_filter = self.amp_filter.get_values();
        for i in 0..self.size {
            self.diff_buffer[i] = amp_filter[i] - self.diff_buffer[i];
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
            let amp_filter = self.amp_filter.get_values();
            let amp_feedback = self.amp_feedback.get_values();
            for i in 0..self.size {
                amp[i] = ao + ag * (amp_filter[i] + amp_feedback[i]);
            }
        }
        let diff_filter = self.diff_filter.get_values();
        let diff_feedback = self.diff_feedback.get_values();
        for i in 0..self.size {
            let diff = dg * (diff_filter[i] + diff_feedback[i]);
            self.features.diff[i] = diff;
            self.features.energy[i] = self.features.energy[i] + diff - self.params.drag;
        }
    }

    fn apply_sync(&mut self) {
        let energy = &mut self.features.energy;
        let size_f = self.size as f64;
        let mean = energy.iter().sum::<f64>() / size_f;

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
    }

    fn apply_value_scaling(&mut self) {
        let amp = self.features.get_amplitudes(0);

        for i in 0..self.size {
            self.scale_buffer[i] = (self.features.scales[i] * (amp[i] - 1.)).abs();
        }

        self.scale_filter.process(&self.scale_buffer);
        let scale_filter = self.scale_filter.get_values_mut();

        for i in 0..self.size {
            let mut vsh = scale_filter[i];
            if vsh < 0.001 {
                vsh = 0.001;
            }
            let vs = 1. / vsh;
            scale_filter[i] = vsh;
            self.features.scales[i] = vs;
        }
    }

    fn signed_square_diff(a: f64, b: f64) -> f64 {
        let diff = a - b;
        diff.signum() * diff * diff
    }
}

use std::fmt::{Display, Error, Formatter};

struct VecFmt<'a>(&'a Vec<f64>);

impl<'a> VecFmt<'a> {
    fn fmt_num(num: f64) -> String {
        format!(
            "{:>6}.{:06}",
            num as i32,
            ((num.abs() % 1.0) * 1000000.) as i32
        )
    }
}

impl<'a> Display for VecFmt<'a> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        let mut comma_separated = String::new();

        for &num in &self.0[0..self.0.len() - 1] {
            comma_separated.push_str(&VecFmt::fmt_num(num));
            comma_separated.push_str(", ");
        }
        let num = self.0[self.0.len() - 1];
        comma_separated.push_str(&VecFmt::fmt_num(num));

        write!(f, "[ {} ]", comma_separated)
    }
}
