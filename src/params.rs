use serde::{Deserialize, Deserializer, Serialize, Serializer};

#[derive(Copy, Clone, Debug)]
pub struct FilterParams {
    pub a: f32,
    pub b: f32,
    tau: f32,
    gain: f32,
}

impl FilterParams {
    pub fn new(tau: f32, gain: f32) -> FilterParams {
        let mut f = FilterParams {
            a: 0.,
            b: 0.,
            tau: 0.,
            gain: 0.,
        };
        f.set_coefficients(tau, gain);
        f
    }

    pub fn set_coefficients(&mut self, tau: f32, gain: f32) {
        self.tau = tau;
        self.gain = gain;
        if tau == 0. {
            self.a = gain;
            self.b = 0.;
            return;
        }
        let b = 0.5 * (2f32).powf((tau - 1.) / tau);
        let a = 1. - b;
        self.a = a * gain;
        self.b = b * gain;
    }

    pub fn get_coefficients(&self) -> Vec<f32> {
        vec![self.tau, self.gain]
    }
}

impl Serialize for FilterParams {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        #[derive(Serialize)]
        struct Params {
            tau: f32,
            gain: f32,
        }
        let p = Params {
            tau: self.tau,
            gain: self.gain,
        };
        p.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for FilterParams {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Params {
            tau: f32,
            gain: f32,
        };
        let p = Params::deserialize(deserializer)?;
        Ok(Self::new(p.tau, p.gain))
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct FrequencySensorParams {
    pub preemphasis: f32,
    pub diff_gain: f32,
    pub amp_scale: f32,
    pub amp_offset: f32,
    pub sync: f32,
    pub drag: f32,

    pub amp_filter: FilterParams,
    pub amp_feedback: FilterParams,
    pub diff_filter: FilterParams,
    pub diff_feedback: FilterParams,
    pub pos_scale_filter: FilterParams,
    pub neg_scale_filter: FilterParams,

    pub gain_control: GainControllerParams,
}

impl FrequencySensorParams {
    pub fn defaults() -> FrequencySensorParams {
        FrequencySensorParams {
            amp_filter: FilterParams::new(8., 1.),
            amp_feedback: FilterParams::new(200., -1.),
            diff_filter: FilterParams::new(16., 1.),
            diff_feedback: FilterParams::new(100., -0.05),
            gain_control: GainControllerParams {
                pre_gain: (1 << 16) as f32,
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
