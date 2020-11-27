use std::fmt::{Display, Error, Formatter};

pub struct VecFmt<'a>(pub &'a Vec<f64>);

impl<'a> VecFmt<'a> {
    pub fn fmt_num(num: f64) -> String {
        format!(
            "{:>6}.{:06}",
            num as i32,
            ((num.abs() % 1.0) * 1000000.) as i32
        )
    }
}

impl<'a> Display for VecFmt<'a> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        let mut comma_separated = String::new();

        for &num in &self.0[0..self.0.len() - 1] {
            comma_separated.push_str(&VecFmt::fmt_num(num));
            comma_separated.push_str(", ");
        }
        let num = self.0[self.0.len() - 1];
        comma_separated.push_str(&VecFmt::fmt_num(num));

        write!(f, "[ {} ]", comma_separated)
    }
}
