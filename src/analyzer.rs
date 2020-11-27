use super::bucketer::Bucketer;
use super::frequency_sensor::{Features, FrequencySensor, FrequencySensorParams};
use super::sfft::SlidingFFT;

pub struct Analyzer {
    sfft: SlidingFFT,
    bucketer: Bucketer,
    frequency_sensor: FrequencySensor,
}

impl Analyzer {
    pub fn new(
        fft_size: usize,
        size: usize,
        length: usize,
        fs_params: FrequencySensorParams,
    ) -> Analyzer {
        let sfft = SlidingFFT::new(fft_size);
        let bucketer = Bucketer::new(fft_size / 2, size, 32., 16000.);
        let frequency_sensor = FrequencySensor::new(size, length, fs_params);
        Analyzer {
            sfft,
            bucketer,
            frequency_sensor,
        }
    }

    pub fn process(&mut self, frame: &Vec<f64>) {
        self.sfft.push_input(frame);
        let spectrum = self.sfft.process();
        let bins = self.bucketer.bucket(spectrum);
        self.frequency_sensor.process(bins);
    }

    pub fn get_features(&self) -> &Features {
        &self.frequency_sensor.get_features()
    }

    pub fn set_params(&mut self, fp: FrequencySensorParams) {
        self.frequency_sensor.set_params(fp);
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
            gain_control: GainControllerParams {
                kd: 0.001,
                kp: 0.001,
                ki: 0.001,
                pre_gain: (1 << 16) as f64,
                filter_params: FilterParams::new(100., 1.),
            },
            amp_offset: 1.,
            preemphasis: 1.,
            sync: 0.001,
            amp_scale: 1.,
            diff_gain: 1.,
            drag: 0.001,
            pos_scale_filter: FilterParams::new(10., 1.),
            neg_scale_filter: FilterParams::new(10., 1.),
        };
        let mut a = Analyzer::new(128, 16, 2, params);

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
