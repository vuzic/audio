use std::boxed::Box;
use std::error::Error;
use std::fmt;

/// DeviceError is returned when there is an error with the audio device.
#[derive(Debug)]
pub struct DeviceError(pub String, pub Option<Box<dyn Error>>);

impl fmt::Display for DeviceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let source = if let Some(e) = &self.1 {
            format!("{}", e)
        } else {
            "".to_owned()
        };
        write!(f, "Audio Device Error: {}{}", self.0, source)
    }
}

// impl Error for DeviceError {
//     fn source(&self) -> Option<&(dyn Error + 'static)> {
//         self.1
//             .as_ref()
//             .map(|b| Box::leak(b) as &(dyn Error + 'static))
//     }
// }
