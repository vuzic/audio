use std::f32::consts::PI;
use std::sync::Arc;

extern crate rustfft;
use rustfft::num_complex::Complex;
use rustfft::FFTplanner;
use rustfft::FFT;

use super::buffer::WindowBuffer;

/// SlidingFFT implements a sliding FFT with (1 - frame_size / fft_size) overlap.
/// It uses a blackman-harris windowing function.
pub struct SlidingFFT {
    buffer: WindowBuffer,
    window: Vec<f32>,

    fft_size: usize,
    norm: f32,

    fft: Arc<dyn FFT<f32>>,

    complex: Vec<Complex<f32>>,
    output: Vec<f32>,
}

fn blackman_harris(i: usize, n: usize) -> f32 {
    let a0 = 0.35875;
    let a1 = 0.48829;
    let a2 = 0.14128;
    let a3 = 0.01168;
    let f = (PI * i as f32) / (n as f32 - 1.);
    a0 - a1 * f.cos() + a2 * (2. * f).cos() - a3 * (3. * f).cos()
}

fn log_magnitude(x: Complex<f32>) -> f32 {
    (1. + x.re * x.re + x.im * x.im).ln() * 0.5
}

impl SlidingFFT {
    pub fn new(fft_size: usize) -> SlidingFFT {
        let mut planner = FFTplanner::new(false);
        let fft = planner.plan_fft(fft_size);
        let buffer = WindowBuffer::new(fft_size * 2);

        let window = (0..fft_size)
            .map(|i| blackman_harris(i, fft_size))
            .collect();

        let complex = vec![Complex::from(0f32); fft_size];
        let output = vec![0f32; fft_size / 2];

        SlidingFFT {
            buffer,
            window,
            fft_size,
            norm: 1. / (fft_size as f32),
            complex,
            output,
            fft,
        }
    }

    pub fn push_input(&mut self, frame: &Vec<f32>) -> () {
        self.buffer.push(frame);
    }

    /// process returns the log magnitude of the fft of the most recent fft_size data.
    pub fn process(&mut self) -> &Vec<f32> {
        let fft_frame = self.buffer.get(self.fft_size);

        let mut input: Vec<Complex<f32>> = fft_frame
            .iter()
            .enumerate()
            .map(|(i, x)| x * self.window[i])
            .map(Complex::from)
            .collect();

        self.fft.process(&mut input, &mut self.complex);

        for i in 0..self.fft_size / 2 {
            self.output[i] = log_magnitude(self.complex[i] * self.norm);
        }

        &self.output
    }

    pub fn output_size(&self) -> usize {
        self.output.len()
    }
}

#[cfg(test)]
mod tests {
    use super::SlidingFFT;
    use std::f32::consts::PI;

    #[test]
    fn it_works() {
        let mut sfft = SlidingFFT::new(16);
        let d = (0..16)
            .map(|i| (i as f32 * 4. * PI / 16.).cos() + 1.)
            .collect();
        sfft.push_input(&d);
        let out = sfft.process();
        assert_eq!(
            out,
            // this value is kind of just chosen assuming this is basically correct
            &vec![
                0.051656775,
                0.009550215,
                0.013055116,
                0.014881688,
                0.0052858936,
                0.003163205,
                0.0023868245,
                0.0020535085
            ]
        );
    }
}
