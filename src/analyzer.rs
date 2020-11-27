use super::bucketer::Bucketer;
use super::frequency_sensor::{Features, FrequencySensor, FrequencySensorParams};
use super::sfft::SlidingFFT;
use crate::gain_control::{BoostController, Params as GainControllerParams};

pub struct Analyzer {
    boost: BoostController,
    sfft: SlidingFFT,
    bucketer: Bucketer,
    frequency_sensor: FrequencySensor,

    block_size: usize,
    sample_count: usize,
}

impl Analyzer {
    pub fn new(
        fft_size: usize,
        block_size: usize,
        size: usize,
        length: usize,
        boost_params: GainControllerParams,
        fs_params: FrequencySensorParams,
    ) -> Analyzer {
        let boost = BoostController::new(boost_params);
        let sfft = SlidingFFT::new(fft_size);
        let bucketer = Bucketer::new(fft_size / 2, size, 32., 22000.);
        let frequency_sensor = FrequencySensor::new(size, length, fs_params);
        Analyzer {
            boost,
            sfft,
            bucketer,
            frequency_sensor,
            block_size,
            sample_count: 0,
        }
    }

    pub fn process(&mut self, frame: &mut Vec<f64>) -> Option<Features> {
        self.sample_count += frame.len();
        self.boost.process(frame);
        self.sfft.push_input(frame);
        if self.sample_count >= self.block_size {
            self.sample_count = 0;
            let spectrum = self.sfft.process();
            let bins = self.bucketer.bucket(spectrum);
            self.frequency_sensor.process(bins);
            return Some(self.frequency_sensor.get_features().to_owned());
        }
        None
    }

    pub fn get_features(&self) -> &Features {
        &self.frequency_sensor.get_features()
    }

    pub fn set_params(&mut self, fp: FrequencySensorParams) {
        self.frequency_sensor.set_params(fp);
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
}

#[cfg(test)]
mod tests {
    use super::Analyzer;
    use super::FrequencySensorParams;
    use crate::filter::FilterParams;
    use crate::gain_control::Params as GainControllerParams;

    #[test]
    fn it_works() {
        let params = FrequencySensorParams {
            amp_filter: FilterParams::new(1., 1.),
            amp_feedback: FilterParams::new(100., -1.),
            diff_filter: FilterParams::new(1., 1.),
            diff_feedback: FilterParams::new(100., -0.5),
            gain_control: GainControllerParams::defaults(),
            amp_offset: 1.,
            preemphasis: 1.,
            sync: 0.001,
            amp_scale: 1.,
            diff_gain: 1.,
            drag: 0.001,
            pos_scale_filter: FilterParams::new(10., 1.),
            neg_scale_filter: FilterParams::new(10., 1.),
        };
        let boost_params = GainControllerParams::defaults();
        let mut a = Analyzer::new(128, 16, 2, boost_params, params);

        use std::f64::consts::PI;
        let input: Vec<f64> = (0..128)
            .map(|x| (x as f64 * 2. * PI / 128.).cos())
            .collect();

        for _ in 0..128 {
            a.process(&input);
        }

        println!("{:?}", a.get_features());
    }
}
