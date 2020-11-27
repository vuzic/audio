/// WindowBuffer implements a sliding circular buffer with a given capacity. Pushing to the buffer
/// increments the current index. Get returns the N most recently pushed elements.
pub struct WindowBuffer {
    buffer: Vec<f64>,
    index: usize,
    capacity: usize,
}

impl WindowBuffer {
    pub fn new(capacity: usize) -> WindowBuffer {
        WindowBuffer {
            buffer: vec![0f64; capacity],
            index: 0,
            capacity,
        }
    }

    pub fn push(&mut self, x: &Vec<f64>) {
        if x.len() > self.capacity {
            panic!("cannot push size greater than capacity");
        }

        let e = self.index + x.len();
        let (en, wrap) = if e > self.capacity {
            (self.capacity, true)
        } else {
            (e, false)
        };

        for i in self.index..en {
            self.buffer[i] = x[i - self.index];
        }
        if wrap {
            let os = self.capacity - self.index;
            for i in 0..x.len() - os {
                self.buffer[i] = x[i + os];
            }
        }

        self.index = (self.index + x.len()) % self.capacity;
    }

    pub fn get(&self, size: usize) -> Vec<f64> {
        if size > self.capacity {
            panic!("cannot get size greater than capacity");
        }

        let mut out = vec![0f64; size];

        let s = self.index as i32 - size as i32;
        let (st, en, wrap) = if s < 0 {
            (self.capacity as i32 + s, self.capacity as i32, true)
        } else {
            (s, self.index as i32, false)
        };

        for i in st..en {
            out[(i - st) as usize] = self.buffer[i as usize];
        }
        if wrap {
            let os = en - st;
            for i in 0..self.index {
                out[i + os as usize] = self.buffer[i];
            }
        }

        out
    }
}

#[cfg(test)]
mod tests {
    use super::WindowBuffer;

    #[test]
    fn it_works() {
        let mut b = WindowBuffer::new(4);

        let v = vec![0f64, 1., 2., 3.];
        b.push(&v);
        assert_eq!(b.get(4), v);

        b.push(&vec![69., 420.]);
        assert_eq!(b.get(4), vec![2., 3., 69., 420.]);
    }
}
