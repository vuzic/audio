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

/// GainController is a PID controller which adjusts gain with a target value of 1.
pub struct GainController {
    filter: Filter,
    // neg_filter: Filter,
    values: Vec<f64>,
    err: Vec<f64>,
    params: Params,
}

impl GainController {
    pub fn new(size: usize, params: Params) -> GainController {
        GainController {
            filter: Filter::new(size, params.filter_params),
            values: vec![1f64; size],
            err: vec![0f64; size],
            params,
        }
    }

    fn log_error(x: f64) -> f64 {
        let x = 0.000000001f64 + x;
        let l = x.abs().log2();
        -x.signum() * l //* l * l.signum()
    }

    pub fn process(&mut self, input: &mut Vec<f64>) {
        for i in 0..input.len() {
            input[i] *= self.values[i] * self.params.pre_gain;
        }

        self.filter.process(input);
        let filter_values = self.filter.get_values();

        for i in 0..input.len() {
            let e = GainController::log_error(filter_values[i]);
            // integrate error
            self.err[i] = 0.99 * self.err[i] + 0.01 * e;

            let u = self.params.kp * e
                + self.params.ki * self.err[i]
                + self.params.kd * (self.err[i] - e);
            self.values[i] = match self.values[i] + u {
                x if x > 1e8 => 1e8,
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

#[derive(Serialize, Deserialize)]
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