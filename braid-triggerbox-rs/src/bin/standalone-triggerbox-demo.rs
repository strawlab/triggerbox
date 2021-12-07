#[macro_use]
extern crate log;

use braid_triggerbox::{make_trig_fps_cmd, name_display, to_name_type, Cmd};
use structopt::StructOpt;

#[cfg(target_os = "macos")]
const DEFAULT_DEVICE_PATH: &str = "/dev/tty.usbmodem1423";

#[cfg(target_os = "linux")]
const DEFAULT_DEVICE_PATH: &str = "/dev/ttyUSB0";

#[cfg(target_os = "windows")]
const DEFAULT_DEVICE_PATH: &str = r#"COM3"#;

#[derive(Debug, StructOpt)]
#[structopt(name = "standalone-triggerbox-demo")]
struct Opt {
    /// Filename of device
    #[structopt(long = "device", default_value = DEFAULT_DEVICE_PATH)]
    device: String,
    /// Framerate
    #[structopt(long = "fps", default_value = "100")]
    fps: f64,
    /// Analog output 1
    #[structopt(long = "aout1", default_value = "0.0")]
    aout1: f64,
    /// Analog output 2
    #[structopt(long = "aout2", default_value = "0.0")]
    aout2: f64,
    /// Assert device name. Raises an error if device's name is not equal.
    #[structopt(long = "assert-device-name")]
    assert_device_name: Option<String>,
    /// Set device name. Sets flash storage on the device to store this name.
    #[structopt(long = "set-device-name")]
    set_device_name: Option<String>,
    /// Maximum acceptable measurement error (in milliseconds)
    #[structopt(long = "max-time-error-msec", default_value = "6")]
    max_acceptable_measurement_error: u64,
}

fn main() -> anyhow::Result<()> {
    env_logger::init();
    info!("braid_triggerbox starting");
    let opt = Opt::from_args();

    let mut quit_early = false;

    let (tx, rx) = crossbeam_channel::unbounded();

    tx.send(Cmd::StopPulsesAndReset)?;
    let (rate_cmd, rate_actual) = make_trig_fps_cmd(opt.fps);
    println!("Requested {} fps, using {} fps", opt.fps, rate_actual);
    tx.send(rate_cmd)?;
    if let Some(set_device_name) = opt.set_device_name {
        let actual_name = to_name_type(&set_device_name)?;
        println!("Setting name to {}", name_display(&Some(actual_name)));
        tx.send(Cmd::SetDeviceName(actual_name))?;
        quit_early = true;
    }

    tx.send(Cmd::SetAOut((opt.aout1, opt.aout2)))?;

    let assert_device_name = opt
        .assert_device_name
        .as_ref()
        .map(AsRef::as_ref)
        .map(to_name_type)
        .transpose()?;

    tx.send(Cmd::StartPulses)?;

    let cb = Box::new(|tm| {
        println!("got new time model: {:?}", tm);
    });

    let query_dt = std::time::Duration::from_secs(1);
    let max_acceptable_measurement_error =
        std::time::Duration::from_millis(opt.max_acceptable_measurement_error);

    let (control, _handle) = braid_triggerbox::launch_background_thread(
        cb,
        opt.device,
        rx,
        None,
        query_dt,
        assert_device_name,
        max_acceptable_measurement_error,
    )?;

    println!("Connecting to trigger device ..");
    while !tx.is_empty() {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    println!(".. connected.");

    if quit_early {
        return Ok(());
    }

    while !control.is_done() {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
    Ok(())
}
