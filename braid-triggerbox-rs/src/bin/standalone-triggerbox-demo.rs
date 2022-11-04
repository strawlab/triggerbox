#[macro_use]
extern crate log;

use braid_triggerbox::{make_trig_fps_cmd, name_display, to_name_type, Cmd};
use braid_triggerbox_comms::LedInfo;

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

    /// LED pulse duration (in microseconds)
    #[structopt(long = "led-usec")]
    led_usec: Option<u32>,

    /// LED interval (in n frames). 0 means disabled.
    #[structopt(long = "led-nth-frame")]
    led_nth_frame: Option<u8>,

    /// LED maximum duty (in fraction of total time). 1.0 means no limit.
    #[structopt(long = "led-max-duty")]
    led_max_duty: Option<f32>,
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

    let mut led_info = LedInfo::default();
    let mut send_led = false;

    if let Some(led_usec) = opt.led_usec {
        led_info.max_duration_usec = led_usec;
        send_led = true;
    }

    if let Some(led_nth_frame) = opt.led_nth_frame {
        led_info.nth_frame = led_nth_frame;
        send_led = true;
    }

    if let Some(led_max_duty) = opt.led_max_duty {
        led_info.max_overall_duty_cycle = led_max_duty;
        send_led = true;
    }

    if send_led {
        tx.send(Cmd::SetLedPulse(led_info)).await?;
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
        println!("got new time model: {:?}", tm);
    });

    let query_dt = std::time::Duration::from_secs(1);
    let max_acceptable_measurement_error =
        std::time::Duration::from_millis(opt.max_acceptable_measurement_error);

    println!("Connecting to trigger device.");

    if quit_early {
        println!("Should quit early, but cannot");
    }

    braid_triggerbox::run_triggerbox(
        cb,
        opt.device,
        rx,
        None,
        query_dt,
        assert_device_name,
        max_acceptable_measurement_error,
    )
    .await?;

    Ok(())
}
