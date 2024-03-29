#[macro_use]
extern crate log;

use braid_triggerbox::{make_trig_fps_cmd, name_display, to_name_type, Cmd};

use clap::{Parser, ValueEnum};

#[cfg(target_os = "macos")]
const DEFAULT_DEVICE_PATH: &str = "/dev/tty.usbmodem1423";

#[cfg(target_os = "linux")]
const DEFAULT_DEVICE_PATH: &str = "/dev/ttyUSB0";

#[cfg(target_os = "windows")]
const DEFAULT_DEVICE_PATH: &str = r#"COM3"#;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Filename of device
    #[structopt(long = "device", default_value = DEFAULT_DEVICE_PATH)]
    device: String,

    /// RunMode
    #[arg(long, value_enum, default_value_t = RunMode::FreeRun)]
    run_mode: RunMode,

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

    /// Sleep duration to allow device to wake up (in seconds)
    ///
    /// Emperically, an Arduino Nano requires 7 seconds to wake up.
    #[arg(long, default_value_t = 7.0)]
    sleep: f32,
}

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
enum RunMode {
    FreeRun,
    Stop,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let opt = Cli::parse();
    info!("braid_triggerbox starting. Run mode: {:?}", opt.run_mode);

    let mut quit_early = false;

    let (tx, rx) = tokio::sync::mpsc::channel(10);

    tx.send(Cmd::StopPulsesAndReset).await?;
    match &opt.run_mode {
        RunMode::FreeRun => {
            let (rate_cmd, rate_actual) = make_trig_fps_cmd(opt.fps);
            println!("Requested {} fps, using {} fps", opt.fps, rate_actual);
            tx.send(rate_cmd).await?;
        }
        RunMode::Stop => {
            tx.send(Cmd::StopPulsesAndReset).await?;
        }
    }

    if let Some(set_device_name) = opt.set_device_name {
        let actual_name = to_name_type(&set_device_name)?;
        println!("Setting name to {}", name_display(&Some(actual_name)));
        tx.send(Cmd::SetDeviceName(actual_name)).await?;
        quit_early = true;
    }

    tx.send(Cmd::SetAOut((opt.aout1, opt.aout2))).await?;

    let assert_device_name = opt
        .assert_device_name
        .as_ref()
        .map(AsRef::as_ref)
        .map(to_name_type)
        .transpose()?;

    match &opt.run_mode {
        RunMode::FreeRun => {
            tx.send(Cmd::StartPulses).await?;
        }
        RunMode::Stop => {}
    }

    let cb = Box::new(|tm| {
        println!("got new time model: {tm:?}");
    });

    let query_dt = std::time::Duration::from_secs(1);
    let max_acceptable_measurement_error =
        std::time::Duration::from_millis(opt.max_acceptable_measurement_error);

    println!("Connecting to trigger device.");

    if quit_early {
        println!("Should quit early, but cannot");
    }

    let sleep_dur = std::time::Duration::from_secs_f32(opt.sleep);

    let opts = braid_triggerbox::TriggerboxOptions {
        device_path: opt.device,
        query_dt,
        assert_device_name,
        max_acceptable_measurement_error,
        sleep_dur,
    };

    braid_triggerbox::run_triggerbox(cb, rx, None, opts).await?;

    Ok(())
}
