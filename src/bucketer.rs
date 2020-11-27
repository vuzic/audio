/// Bucketer takes an FFT frame of a given size and returns a given number of frequency bins
/// whose indices are caculated using a logrithmic scale. The zero'th element in the
/// spectrum is always its own bucket, so the bucketer always returns N+1 buckets.
pub struct Bucketer {
    pub indices: Vec<usize>,
    output: Vec<f32>,
}

fn to_log_scale(x: f32) -> f32 {
    (x + 1.).log2()
}

fn from_log_scale(x: f32) -> f32 {
    (2f32).powf(x) + 1.
}

impl Bucketer {
    pub fn new(input_size: usize, buckets: usize, f_min: f32, f_max: f32) -> Bucketer {
        let output = vec![0f32; buckets];
        let mut indices = vec![0; buckets - 1];

        let s_min = to_log_scale(f_min);
        let s_max = to_log_scale(f_max);

        let buckets_f = buckets as f32;
        let input_size_f = input_size as f32;
        let space = (s_max - s_min) / buckets_f;

        let mut last_idx = 0;
        let mut offset = 0f32;
        let delta = (s_max - s_min) / input_size_f;

        for i in 0..indices.len() {
            let adj = space - delta * offset / buckets_f;

            let v = from_log_scale((i + 1) as f32 * adj + s_min + offset * delta);
            let mut idx = (input_size_f * v / f_max).ceil() as usize;

            if idx <= last_idx {
                idx = last_idx + 1;
                offset += 1.;
            }
            if idx >= input_size {
                idx = input_size - 1;
            }

            indices[i] = idx;
            last_idx = idx;
        }

        Bucketer { indices, output }
    }

    /// bucket returns the input of the input split into `size` bins
    pub fn bucket(&mut self, input: &Vec<f32>) -> &mut Vec<f32> {
        for i in 0..self.output.len() {
            let start = if i == 0 { 0 } else { self.indices[i - 1] };
            let stop = if i == self.output.len() - 1 {
                input.len()
            } else {
                self.indices[i]
            };

            let sum: f32 = input[start..stop].iter().sum();
            self.output[i] = sum / (stop - start) as f32;
        }

        &mut self.output
    }
}

#[cfg(test)]
mod tests {
    use super::Bucketer;

    #[test]
    fn it_works() {
        let mut b = Bucketer::new(16, 16, 32., 16000.);
        let d = vec![1f32; 16];

        let out = b.bucket(&d);
        assert_eq!(out, &d);

        let mut b = Bucketer::new(16, 4, 32., 16000.);
        let out = b.bucket(&d);
        assert_eq!(out, &vec![1f32; 4]);

        let out = b.bucket(&(0u8..16).map(f32::from).collect::<Vec<f32>>());
        // dunno if this is "right" but whatever..
        assert_eq!(out, &vec![0f32, 1., 2.5, 9.5]);
    }
}
