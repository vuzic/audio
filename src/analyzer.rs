use serde::{Deserialize, Serialize};

use super::bucketer::Bucketer;
use super::frequency_sensor::{
    Features, FrequencySensor, FrequencySensorParams, State as FrequencySensorState,
};
use super::sfft::SlidingFFT;
use crate::gain_control::{BoostController, BoostState, Params as GainControllerParams};

pub struct Analyzer {
    boost: BoostController,
    sfft: SlidingFFT,
    bucketer: Bucketer,
    frequency_sensor: FrequencySensor,

    block_size: usize,
    sample_count: usize,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct AnalyzerParams {
    pub boost: GainControllerParams,
    pub fs: FrequencySensorParams,
}

#[derive(Debug, Serialize, Default, Clone)]
pub struct AnalyzerState {
    pub boost: BoostState,
    pub fs: FrequencySensorState,
}

impl Default for AnalyzerParams {
    fn default() -> Self {
        Self {
            boost: Default::default(),
            fs: Default::default(),
        }
    }
}

impl Analyzer {
    pub fn new(fft_size: usize, block_size: usize, size: usize, length: usize) -> Analyzer {
        let boost = BoostController::new();
        let sfft = SlidingFFT::new(fft_size);
        let bucketer = Bucketer::new(fft_size / 2, size, 32., 22000.);
        let frequency_sensor = FrequencySensor::new(size, length);
        Analyzer {
            boost,
            sfft,
            bucketer,
            frequency_sensor,
            block_size,
            sample_count: 0,
        }
    }

    pub fn process(&mut self, frame: &mut Vec<f64>, params: &AnalyzerParams) -> Option<Features> {
        self.sample_count += frame.len();
        self.boost.process(frame, &params.boost);
        self.sfft.push_input(frame);
        if self.sample_count >= self.block_size {
            self.sample_count = 0;
            let spectrum = self.sfft.process();
            let bins = self.bucketer.bucket(spectrum);
            self.frequency_sensor.process(bins, &params.fs);
            return Some(self.frequency_sensor.get_features().to_owned());
        }
        None
    }

    pub fn get_features(&self) -> &Features {
        &self.frequency_sensor.get_features()
    }

    pub fn write_debug<W>(&self, w: &mut W) -> core::fmt::Result
    where
        W: core::fmt::Write,
    {
        writeln!(w, "{{")?;
        self.boost.get_state().write_debug(w)?;
        self.frequency_sensor.write_debug(w)?;
        writeln!(w, "}}")
    }

    pub fn get_state(&self) -> AnalyzerState {
        AnalyzerState {
            boost: self.boost.get_state(),
            fs: self.frequency_sensor.get_state(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Analyzer;

    #[test]
    fn it_works() {
        let mut a = Analyzer::new(128, 128, 16, 2);

        use std::f64::consts::PI;
        let mut input: Vec<f64> = (0..128)
            .map(|x| (x as f64 * 2. * PI / 128.).cos())
            .collect();

        for _ in 0..128 {
            a.process(&mut input, &Default::default());
        }

        println!("{:?}", a.get_features());
    }
}
