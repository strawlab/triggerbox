#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use embedded_hal::digital::v2::OutputPin;

    use heapless::spsc::{Consumer, Producer, Queue};

    use defmt::{debug, info, trace, warn};
    use rp_pico::{
        hal::{
            self, clocks::init_clocks_and_plls, prelude::*, pwm::Slices, timer::Alarm, usb::UsbBus,
            watchdog::Watchdog, Sio,
        },
        XOSC_CRYSTAL_FREQ,
    };

    use embedded_hal::PwmPin;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use braid_triggerbox_comms::{
        EmulatedNanoPwmClock, PacketParser, SyncVal, UdevMsg, UsbEvent, BUF_MAX_SZ,
    };
    use crc::{Crc, CRC_8_MAXIM_DOW};

    pub const Q_SZ: usize = 4;

    pub const CRC_MAXIM: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

    const SCAN_TIME_US: u32 = 1_000_000;
    const SCAN_TIME: fugit::Duration<u32, 1, 1_000_000> =
        fugit::Duration::<u32, 1, 1_000_000>::from_ticks(SCAN_TIME_US);

    static mut EVENT_QUEUE: Queue<UsbEvent, Q_SZ> = Queue::new();
    static mut PARSER_BACKING: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();

    #[shared]
    struct Shared {
        serial: SerialPort<'static, UsbBus>,
        frame_number: Pulsenumber,
        pwm_slices: Slices,
        timer: hal::Timer,
    }
    #[local]
    struct Local {
        alarm: hal::timer::Alarm0,
        led: hal::gpio::Pin<
            hal::gpio::bank0::Gpio25,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullNone,
        >,
        usb_dev: UsbDevice<'static, UsbBus>,
        packet_parser: PacketParser<'static>,
        event_tx: Producer<'static, UsbEvent, Q_SZ>,
        event_rx: Consumer<'static, UsbEvent, Q_SZ>,
        pwm_cycle: u8,
        /// A cached copy of what our PWM clock is doing.
        clock_scale: EmulatedNanoPwmClock,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let core = ctx.core;

        let mut resets = ctx.device.RESETS;
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut timer = hal::Timer::new(ctx.device.TIMER, &mut resets, &clocks);

        let usb_bus = ctx.local.usb_bus;
        usb_bus.replace(UsbBusAllocator::new(UsbBus::new(
            ctx.device.USBCTRL_REGS,
            ctx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));
        let serial = SerialPort::new(usb_bus.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Straw Lab")
            .product("Triggerbox RP2040")
            // .serial_number("TEST")
            .device_class(2)
            .build();

        let sio = Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let mut led = pins.led.reconfigure();
        led.set_low().unwrap();

        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME);
        alarm.enable_interrupt();

        // The delay object lets us wait for specified amounts of time (in
        // milliseconds)
        let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        // Init PWMs
        let mut pwm_slices = hal::pwm::Slices::new(ctx.device.PWM, &mut resets);

        let clock_scale =
            EmulatedNanoPwmClock::new(50_000, false, clocks.system_clock.freq().to_Hz() as u64);

        {
            // Configure PWM0
            let pwm0 = &mut pwm_slices.pwm0;
            pwm0.default_config();

            pwm0.set_div_int(clock_scale.div_int); // To set integer part of clock divider
            pwm0.set_div_frac(0); // No fractional part of clock divider
            let top = clock_scale.to_top();
            pwm0.set_top(top);

            // Output channel A on PWM0 to the GP0 pin
            let channel0 = &mut pwm0.channel_a;
            channel0.output_to(pins.gpio0);

            channel0.set_duty(top / 100);
            pwm0.enable();

            pwm0.enable_interrupt(); // call pwm_irq
        };

        let packet_parser = unsafe { PacketParser::new(&PARSER_BACKING) };

        let (event_tx, event_rx) = unsafe { EVENT_QUEUE.split() };

        static_assertions::const_assert_eq!(hexchar(0x00), b'0');
        static_assertions::const_assert_eq!(hexchar(0x01), b'1');
        static_assertions::const_assert_eq!(hexchar(0x02), b'2');
        static_assertions::const_assert_eq!(hexchar(0x0A), b'A');
        static_assertions::const_assert_eq!(hexchar(0x0F), b'F');
        static_assertions::const_assert_eq!(hexchar(0x10), b'0');
        static_assertions::const_assert_eq!(hexchar(0x11), b'1');
        static_assertions::const_assert_eq!(hexchar(0x12), b'2');
        static_assertions::const_assert_eq!(hexchar(0x1A), b'A');
        static_assertions::const_assert_eq!(hexchar(0x1F), b'F');

        (
            Shared {
                serial,
                frame_number: 0,
                pwm_slices,
                timer,
            },
            Local {
                alarm,
                led,
                usb_dev,
                clock_scale,
                packet_parser,
                event_tx,
                event_rx,
                pwm_cycle: 0,
            },
            init::Monotonics(),
        )
    }

    #[idle(
        shared = [serial, frame_number, pwm_slices],
        local = [event_rx, clock_scale]
    )]
    fn idle(mut ctx: idle::Context) -> ! {
        info!("Started!");
        loop {
            match ctx.local.event_rx.dequeue() {
                Some(ev) => handle_event(&mut ctx, ev),
                None => rtic::export::wfi(), // wait for interrupt
            }
        }
    }

    #[task(
        binds=USBCTRL_IRQ,
        priority = 1,
        shared = [serial, frame_number, pwm_slices, timer],
        local = [usb_dev, event_tx, packet_parser]
    )]
    fn usb_irq(mut ctx: usb_irq::Context) {
        let mut buf = [0u8; 64];

        let usb_dev = ctx.local.usb_dev;
        let read_result = ctx.shared.serial.lock(|serial| {
            if !usb_dev.poll(&mut [serial]) {
                Ok(0)
            } else {
                serial.read(&mut buf)
            }
        });
        match read_result {
            Ok(count) if count > 0 => {
                let now_usec = ctx.shared.timer.lock(|timer| timer.get_counter());
                match ctx.local.packet_parser.got_buf(now_usec, &buf[..count]) {
                    Ok(ev) => ctx.local.event_tx.enqueue(ev).ok().unwrap(),
                    Err(braid_triggerbox_comms::Error::AwaitingMoreData) => {}
                    Err(e) => warn!("error parsing: {}", e),
                };
            }
            _ => {}
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        local = [alarm, led, tog: bool = true],
    )]
    fn timer_irq(ctx: timer_irq::Context) {
        if *ctx.local.tog {
            ctx.local.led.set_high().unwrap();
        } else {
            ctx.local.led.set_low().unwrap();
        }
        *ctx.local.tog = !*ctx.local.tog;

        ctx.local.alarm.clear_interrupt();
        let _ = ctx.local.alarm.schedule(SCAN_TIME);
    }

    #[task(
        binds = PWM_IRQ_WRAP,
        priority = 2,
        shared = [frame_number, pwm_slices],
        local = [pwm_cycle],
    )]
    fn pwm_irq(ctx: pwm_irq::Context) {
        let p = ctx.shared.pwm_slices;
        let f = ctx.shared.frame_number;

        (p, f).lock(|pwm_slices, frame_number| {
            let pwm0 = &mut pwm_slices.pwm0;
            pwm0.clear_interrupt();
            *frame_number = frame_number.saturating_add(1);
        });
    }

    fn handle_event(ctx: &mut idle::Context, event: UsbEvent) {
        debug!("handling event: {:?}", event);
        use rtic::{mutex_prelude::TupleExt02, Mutex};
        match event {
            UsbEvent::TimestampQuery(value) => {
                let timestamp_request = fill_sample(value, ctx);
                send_data(&timestamp_request, b'P', ctx);
            }
            UsbEvent::VersionRequest => {
                send_data(&fill_sample(14, ctx), b'V', ctx);
            }
            UsbEvent::Sync(val) => {
                match val {
                    SyncVal::Sync0 => {
                        // stop clock, reset pulsenumber
                        (&mut ctx.shared.pwm_slices, &mut ctx.shared.frame_number).lock(
                            |pwm_slices, frame_number| {
                                let pwm0 = &mut pwm_slices.pwm0;
                                pwm0.disable();
                                *frame_number = 0;
                                pwm0.set_counter(0);
                            },
                        );
                    }
                    SyncVal::Sync1 => {
                        // start clock
                        ctx.shared.pwm_slices.lock(|pwm_slices| {
                            let pwm0 = &mut pwm_slices.pwm0;
                            pwm0.enable();
                        });
                    }
                    SyncVal::Sync2 => {
                        // stop clock
                        ctx.shared.pwm_slices.lock(|pwm_slices| {
                            let pwm0 = &mut pwm_slices.pwm0;
                            pwm0.disable();
                        });
                    }
                }
            }
            UsbEvent::SetTop(val) => {
                info!(
                    "Received TOP={}, prescaler_key='{}'",
                    val.avr_icr1(),
                    core::str::from_utf8(&[val.prescaler_key()][..]).unwrap_or("??")
                );

                let is_mode2 = match val.prescaler_key() {
                    b'1' => false,
                    b'2' => true,
                    _ => {
                        panic!(
                            "Unsupported prescaler_key: '{}' {}",
                            core::str::from_utf8(&[val.prescaler_key()][..]).unwrap_or("??"),
                            val.prescaler_key(),
                        );
                    }
                };
                let new_clock_scale = EmulatedNanoPwmClock::new(
                    val.avr_icr1(),
                    is_mode2,
                    ctx.local.clock_scale.system_clock_freq_hz(),
                );
                let top = new_clock_scale.to_top();

                let duty0 = (top / 100).max(1);
                let led_duty = (duty0 * 2).min(top - 1);
                let div_int = new_clock_scale.div_int;
                *ctx.local.clock_scale = new_clock_scale;

                ctx.shared.pwm_slices.lock(|pwm_slices| {
                    let pwm0 = &mut pwm_slices.pwm0;

                    // Output channel A on PWM0 to the GP0 pin
                    let channel0 = &mut pwm0.channel_a;

                    // Output channel B on PWM0 to the GP1 pin
                    let channel1 = &mut pwm0.channel_b;

                    channel0.set_duty(duty0);
                    channel1.set_duty(led_duty);

                    pwm0.set_top(top);
                    pwm0.set_div_int(div_int);
                });
            }
            UsbEvent::SetAOut(val) => {
                // AOUT values
                info!("ignoring AOUT command {}, {}", val.aout0, val.aout1);

                let aout_confirm = fill_sample(val.aout_sequence, ctx);
                send_data(&aout_confirm, b'V', ctx);
            }
            UsbEvent::Udev(val) => {
                match val {
                    UdevMsg::Query => {
                        let mut out_buf: [u8; 10] = [0; 10];
                        let crc = CRC_MAXIM.checksum(&out_buf[0..8]);
                        // Emulate arduino "_serial.print(crc,HEX);" which will
                        // print a single character if the value is less that 0x10.
                        let send_len = if crc >= 0x10 {
                            out_buf[8] = hexchar(crc >> 4);
                            out_buf[9] = hexchar(crc);
                            10
                        } else {
                            out_buf[8] = hexchar(crc);
                            9
                        };
                        send_buf(ctx, &out_buf[..send_len]);
                    }
                    UdevMsg::Set(_) => {
                        todo!();
                    }
                }
            }
        }
    }

    type Pulsenumber = u32; /* 2**32 @100Hz = 497 days */

    #[repr(C)]
    struct TimedSample {
        /// value of arbitrary data
        value: u8,
        pulsenumber: Pulsenumber,
        ticks: u16,
    }

    impl TimedSample {
        fn to_buf(&self, buf: &mut [u8]) {
            assert_eq!(buf.len(), 7);
            buf[0] = self.value;
            buf[1..5].copy_from_slice(&self.pulsenumber.to_le_bytes());
            buf[5..7].copy_from_slice(&self.ticks.to_le_bytes());
        }
    }

    fn send_data(samp: &TimedSample, header: u8, ctx: &mut idle::Context) {
        let mut buf: [u8; 32] = [0; 32];
        buf[0] = header;

        let payload_len = 7;
        buf[1] = payload_len.try_into().unwrap();
        samp.to_buf(&mut buf[2..2 + payload_len]);

        let mut chksum: u8 = 0;
        for i in 0..payload_len {
            let char: u8 = buf[2 + i];
            chksum = chksum.wrapping_add(char);
        }
        buf[2 + payload_len] = chksum;

        send_buf(ctx, &buf[..3 + payload_len]);
    }

    fn fill_sample(value: u8, ctx: &mut idle::Context) -> TimedSample {
        use rtic::mutex_prelude::TupleExt02;
        let (pulsenumber, ticks_real) = (&mut ctx.shared.pwm_slices, &mut ctx.shared.frame_number)
            .lock(|pwm_slices, frame_number| {
                let pwm0 = &mut pwm_slices.pwm0;
                (*frame_number, pwm0.get_counter())
            });

        let ticks = ctx.local.clock_scale.scale_ticks(ticks_real);

        TimedSample {
            value,
            pulsenumber,
            ticks,
        }
    }

    fn send_buf(ctx: &mut idle::Context, out_buf: &[u8]) {
        trace!("out_buf: {:?}", out_buf);
        use rtic::Mutex;
        let nbytes = ctx
            .shared
            .serial
            .lock(|serial| serial.write(out_buf).unwrap());
        assert_eq!(nbytes, out_buf.len());
    }

    const fn hexchar(inchar: u8) -> u8 {
        let lower_4_bits = inchar & 0x0F;
        if lower_4_bits < 0x0A {
            lower_4_bits + b'0'
        } else {
            lower_4_bits + (b'A' - 0x0A)
        }
    }
}
