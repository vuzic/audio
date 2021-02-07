use core::fmt::Write;

use serde::{Deserialize, Serialize};

use crate::filter::{Filter, FilterParams};
use crate::util::VecFmt;

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct Params {
    pub filter_params: FilterParams,
    pub kp: f64,
    pub kd: f64,
    pub ki: f64,
    pub pre_gain: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            kd: 0.1,
            kp: 0.1,
            ki: 0.1,
            pre_gain: 1.0,
            filter_params: FilterParams::new(100., 1.),
        }
    }
}

/// GainController is a PID controller which adjusts gain with a target value of 1.
pub struct GainController {
    filter: Filter,
    values: Vec<f64>,
    err: Vec<f64>,
}

impl GainController {
    pub fn new(size: usize) -> GainController {
        GainController {
            filter: Filter::new(size),
            values: vec![1f64; size],
            err: vec![0f64; size],
        }
    }

    /*
    fn log_error(x: f64) -> f64 {
        let x = 0.000000001f64 + x;
        let l = x.abs().log2();
        -x.signum() * l // * l * l.signum()
    }
    */

    fn error(x: f64) -> f64 {
        let x = x.max(0.0000001);
        (if x < 1. { 1. / x - 1. } else { 1. - x }).clamp(-32., 32.)
    }

    pub fn process(&mut self, input: &mut Vec<f64>, params: &Params) {
        for i in 0..input.len() {
            input[i] *= self.values[i] * params.pre_gain;
        }

        self.filter.process(input, &params.filter_params);
        let filter_values = self.filter.get_values();

        for i in 0..input.len() {
            let e = GainController::error(filter_values[i]);
            // "integrate" error
            self.err[i] = 0.99 * self.err[i] + 0.01 * e;

            let u = params.kp * e + params.ki * self.err[i] + params.kd * (self.err[i] - e);
            self.values[i] = match self.values[i] + u {
                x if x > 1e6 => 1e6,
                x if x < 1e-6 => 1e-6,
                x => x,
            };
        }
    }

    pub fn get_values(&self) -> &Vec<f64> {
        &self.values
    }

    pub fn get_state(&self) -> State {
        State {
            gain: self.values.to_owned(),
            filter_values: self.filter.get_values().to_owned(),
            err: self.err.to_owned(),
        }
    }
}

#[derive(Serialize, Debug, Default, Clone)]
pub struct State {
    pub gain: Vec<f64>,
    pub filter_values: Vec<f64>,
    pub err: Vec<f64>,
}

impl State {
    pub fn write_debug<W>(&self, w: &mut W) -> core::fmt::Result
    where
        W: Write,
    {
        writeln!(w, "\t\"gain\":          {},", VecFmt(&self.gain))?;
        writeln!(w, "\t\"gain_filter\":   {},", VecFmt(&self.filter_values))?;
        writeln!(w, "\t\"gain_err\":      {},", VecFmt(&self.err))
    }
}

pub struct BoostController {
    gc: GainController,
}

impl BoostController {
    pub fn new() -> Self {
        Self {
            gc: GainController::new(1),
        }
    }

    pub fn process(&mut self, frame: &mut Vec<f64>, params: &Params) {
        let s: f64 = frame.iter().map(|x: &f64| x * x).sum();
        let rms = (s / frame.len() as f64).sqrt();
        let mut p = vec![rms];
        self.gc.process(&mut p, params);
        let scale = self.gc.get_values()[0];
        for i in 0..frame.len() {
            frame[i] *= scale;
        }
    }

    pub fn get_state(&self) -> BoostState {
        let s = self.gc.get_state();
        BoostState {
            gain: s.gain[0],
            filter_value: s.filter_values[0],
            err: s.err[0],
        }
    }
}

#[derive(Debug, Serialize, Default, Clone)]
pub struct BoostState {
    pub gain: f64,
    pub filter_value: f64,
    pub err: f64,
}

impl BoostState {
    pub fn write_debug<W>(&self, w: &mut W) -> core::fmt::Result
    where
        W: Write,
    {
        writeln!(w, "\t\"boost\":          {},", self.gain)?;
        writeln!(w, "\t\"boost_filter\":   {},", self.filter_value)?;
        writeln!(w, "\t\"boost_err\":      {},", self.err)
    }
}
