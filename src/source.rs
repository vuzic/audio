use anyhow::{anyhow, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};

pub use cpal::Stream;

/// Source is an audio source
pub struct Source {
    device: cpal::Device,
}

impl<'a> Source {
    pub fn new(select_device: Option<&str>) -> Result<Self> {
        let host = cpal::default_host();

        let device = if let Some(device_name) = select_device {
            let devices = Self::list_devices();
            let device_names: Vec<String> = Self::list_devices().into_iter().flat_map(|d| d.1.map(|d| d.name().unwrap())).collect();
            devices
                .into_iter()
                .map(|x| x.1)
                .flatten()
                .filter(|d| d.name().map(|name| name == device_name).unwrap_or(false))
                .next()
                .ok_or_else(|| anyhow!("no input device with name '{}' was found. devices: {:?}", device_name, device_names))
        } else {
            host.default_input_device()
                .ok_or_else(|| anyhow!("could not get default input"))
        }?;

        Ok(Self { device })
    }

    pub fn get_stream<T: 'static + cpal::Sample>(
        &self,
        channels: u16,
        sample_rate: u32,
        buffer_size: u32,
        handle_stream: Box<dyn Fn(&[T]) -> () + Send>,
    ) -> Result<Stream> {
        let config = cpal::StreamConfig {
            buffer_size: cpal::BufferSize::Fixed(buffer_size),
            channels,
            sample_rate: cpal::SampleRate(sample_rate),
        };
        // let config = self.device.default_input_config().map_err(|e| {
        //     DeviceError("could not get default config".to_owned(), Some(Box::new(e)))
        // })?;
        // println!("Config: {:?}", &config);

        let stream = self
            .device
            .build_input_stream(
                &config,
                move |data: &[T], _: &_| {
                    handle_stream(data);
                },
                move |err| {
                    eprintln!("Audio Stream Error: {}", err);
                },
            )
            .map_err(|e| {
                if let cpal::BuildStreamError::StreamConfigNotSupported = e {
                    let configs: Vec<cpal::SupportedStreamConfigRange> =
                        self.device.supported_input_configs().unwrap().collect();
                    println!("Supported Configs: {:#?}", &configs);
                }
                anyhow!("could not build stream: {}", e)
            })?;

        stream
            .play()
            .map_err(|e| anyhow!("failed to start stream: {}", e))?;

        Ok(stream)
    }

    pub fn list_devices() -> Vec<(cpal::HostId, cpal::InputDevices<cpal::Devices>)> {
        cpal::available_hosts()
            .iter()
            .map(|&host_id| {
                let host = cpal::host_from_id(host_id).expect("couldnt get host with id");
                (
                    host_id,
                    host.input_devices()
                        .expect("could not get audio input devices"),
                )
            })
            .collect()
    }

    pub fn print_devices(show_supported_configs: bool) -> Result<()> {
        let hosts = Self::list_devices();
        for (host, devices) in hosts {
            for dev in devices {
                println!(
                    "({:?}) Audio Device:\t{:#?}",
                    host,
                    dev.name()
                        .map_err(|e| anyhow!("error getting name: {}", e))?,
                );
                if show_supported_configs {
                    let configs = dev
                        .supported_input_configs()
                        .map_err(|e| anyhow!("error getting input configs: {}", e))?
                        .collect::<Vec<cpal::SupportedStreamConfigRange>>();
                    println!("\tSupported Configs:\t{:#?}", &configs);
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::Source;
    use std::sync::{Arc, Mutex};

    #[test]
    fn it_works() {
        Source::print_devices(true).expect("failed to print devices");

        let s = Source::new(Some("pulse")).expect("failed to get device");
        let buf = Vec::new();
        let out = Arc::new(Mutex::new(Some(buf)));

        let out_clone = out.clone();
        let handle_stream = move |data: &[f32]| {
            if let Ok(mut guard) = out_clone.try_lock() {
                if let Some(writer) = guard.as_mut() {
                    writer.append(&mut data.to_vec());
                }
            }
            // println!(
            //     "Audio Data: {:?} {:?}",
            //     std::time::SystemTime::now(),
            //     data.len()
            // );
        };
        let handle_stream = Box::new(handle_stream) as Box<dyn Fn(&[f32]) -> () + Send>;

        let stream = s
            .get_stream(1, 44100, 256, handle_stream)
            .expect("failed to get stream");

        std::thread::sleep(std::time::Duration::from_secs(1));
        drop(stream);

        let res = out.lock().unwrap().take().unwrap();
        // might have to change this, but im seeing the driver only support 44.1 sample chunks
        // which im guessing is an linux/alsa thing since the cpal docs say that they use their
        // own wrapper to implement a non-blocking interface.
        assert_eq!(res.len(), 44100);
    }
}
