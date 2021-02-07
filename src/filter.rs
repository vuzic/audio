use serde::{Deserialize, Deserializer, Serialize, Serializer};

#[derive(Copy, Clone, Debug)]
pub struct FilterParams {
    pub a: f64,
    pub b: f64,
    tau: f64,
    gain: f64,
}

impl FilterParams {
    pub fn new(tau: f64, gain: f64) -> FilterParams {
        let mut f = FilterParams {
            a: 0.,
            b: 0.,
            tau: 0.,
            gain: 0.,
        };
        f.set_coefficients(tau, gain);
        f
    }

    pub fn set_coefficients(&mut self, tau: f64, gain: f64) {
        self.tau = tau;
        self.gain = gain;
        if tau == 0. {
            self.a = gain;
            self.b = 0.;
            return;
        }
        let b = 0.5 * (2f64).powf((tau - 1.) / tau);
        let a = 1. - b;
        self.a = a * gain;
        self.b = b * gain;
    }

    pub fn get_coefficients(&self) -> Vec<f64> {
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
            tau: f64,
            gain: f64,
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
            tau: f64,
            gain: f64,
        }
        let p = Params::deserialize(deserializer)?;
        Ok(Self::new(p.tau, p.gain))
    }
}

/// Filter implements a bank of N single pole IIR filters that process a frame
/// in parallel.
pub struct Filter {
    values: Vec<f64>,
}

impl Filter {
    pub fn new(size: usize) -> Filter {
        Filter {
            values: vec![0f64; size],
        }
    }

    // fn process_simd(&self, input: &Vec<f64>) {
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
    //             let v = f64x4_add(f64x4_mul(a, v_in), f64x4_mul(b, v_val));
    //             self.values[i..i + 4] = std::mem::transmute(v);
    //         }
    //         i += 4;
    //     }
    // }

    pub fn process(&mut self, input: &Vec<f64>, params: &FilterParams) {
        for i in 0..input.len() {
            self.values[i] = params.a * input[i] + params.b * self.values[i];
        }
    }

    pub fn get_values(&self) -> &Vec<f64> {
        &self.values
    }
}

/// BiasedFilter uses separate coefficients depending on whether the input is greater or
/// less than the current value.
pub struct BiasedFilter {
    values: Vec<f64>,
}

impl BiasedFilter {
    pub fn new(size: usize) -> BiasedFilter {
        BiasedFilter {
            values: vec![0f64; size],
        }
    }

    pub fn process(&mut self, input: &Vec<f64>, params: (&FilterParams, &FilterParams)) {
        for i in 0..input.len() {
            let params = if input[i] < self.values[i] {
                params.0
            } else {
                params.1
            };
            self.values[i] = params.a * input[i] + params.b * self.values[i];
        }
    }

    pub fn get_values(&self) -> &Vec<f64> {
        &self.values
    }

    pub fn get_values_mut(&mut self) -> &mut Vec<f64> {
        &mut self.values
    }
}
