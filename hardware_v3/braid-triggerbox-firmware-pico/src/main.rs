// SPDX-License-Identifier: MIT OR Apache-2.0

//! Braid triggerbox firmware for the Raspberry Pi Pico.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use embedded_hal::digital::OutputPin;

    use heapless::spsc::{Consumer, Producer, Queue};

    use defmt::{debug, info, trace, warn};
    use rp_pico::{
        XOSC_CRYSTAL_FREQ,
        hal::{
            self, Sio, clocks::init_clocks_and_plls, prelude::*, pwm::Slices, timer::Alarm,
            usb::UsbBus, watchdog::Watchdog,
        },
    };

    use embedded_hal::pwm::SetDutyCycle;
    use rtic::{Mutex, mutex_prelude::TupleExt02};

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use braid_triggerbox_comms::{
        DEVICE_FIRMWARE_VERSION, EmulatedNanoPwmClock, PacketParser, Prescaler, SyncVal,
        TopAndPrescaler, UdevMsg, UsbEvent,
    };
    use crc::{CRC_8_MAXIM_DOW, Crc};

    /// Capacity of the queue of events from the host.
    const Q_SZ: usize = 4;

    const CRC_MAXIM: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

    const SCAN_TIME_US: u32 = 1_000_000;
    const SCAN_TIME: fugit::Duration<u32, 1, 1_000_000> =
        fugit::Duration::<u32, 1, 1_000_000>::from_ticks(SCAN_TIME_US);

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
        packet_parser: PacketParser,
        event_tx: Producer<'static, UsbEvent>,
        event_rx: Consumer<'static, UsbEvent>,
        pwm_cycle: u8,
        /// A cached copy of what our PWM clock is doing.
        clock_scale: EmulatedNanoPwmClock,
        /// Whether the last requested trigger rate could be produced. If not,
        /// the trigger pulses stay stopped until a rate which can be produced
        /// is requested.
        rate_ok: bool,
    }

    #[expect(
        clippy::expect_used,
        reason = "if the hardware cannot be initialized, there is nothing else to do"
    )]
    #[init(local = [
        usb_bus: Option<UsbBusAllocator<UsbBus>> = None,
        event_queue: Queue<UsbEvent, Q_SZ> = Queue::new(),
    ])]
    fn init(ctx: init::Context<'_>) -> (Shared, Local, init::Monotonics) {
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
        .expect("init clocks");

        let mut timer = hal::Timer::new(ctx.device.TIMER, &mut resets, &clocks);

        let usb_bus: &'static UsbBusAllocator<UsbBus> =
            ctx.local.usb_bus.insert(UsbBusAllocator::new(UsbBus::new(
                ctx.device.USBCTRL_REGS,
                ctx.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut resets,
            )));
        let serial = SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[
                StringDescriptors::default()
                    .manufacturer("Straw Lab")
                    .product("Triggerbox RP2040"), // .serial_number("TEST")
            ])
            .expect("USB string descriptors")
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
        let Ok(()) = led.set_low();

        let mut alarm = timer.alarm_0().expect("alarm 0");
        if alarm.schedule(SCAN_TIME).is_err() {
            warn!("could not schedule LED timer");
        }
        alarm.enable_interrupt();

        // Init PWMs
        let mut pwm_slices = Slices::new(ctx.device.PWM, &mut resets);

        let clock_scale =
            EmulatedNanoPwmClock::new(50_000, false, clocks.system_clock.freq().to_Hz())
                .expect("initial PWM clock");

        {
            // Configure PWM0
            let pwm0 = &mut pwm_slices.pwm0;
            pwm0.default_config();

            pwm0.set_div_int(clock_scale.div_int()); // To set integer part of clock divider
            pwm0.set_div_frac(0); // No fractional part of clock divider
            let top = clock_scale.to_top();
            pwm0.set_top(top);

            // Output channel A on PWM0 to the GP0 pin
            let channel0 = &mut pwm0.channel_a;
            channel0.output_to(pins.gpio0);

            let Ok(()) = channel0.set_duty_cycle(top / 100);
            pwm0.enable();

            pwm0.enable_interrupt(); // call pwm_irq
        };

        let packet_parser = PacketParser::new();

        let (event_tx, event_rx) = ctx.local.event_queue.split();

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
                rate_ok: true,
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
        local = [event_rx, clock_scale, rate_ok]
    )]
    fn idle(mut ctx: idle::Context<'_>) -> ! {
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
    fn usb_irq(mut ctx: usb_irq::Context<'_>) {
        let mut buf = [0u8; 64];

        let usb_dev = ctx.local.usb_dev;
        let read_result = ctx.shared.serial.lock(|serial| {
            if usb_dev.poll(&mut [serial]) {
                serial.read(&mut buf)
            } else {
                Ok(0)
            }
        });
        if let Ok(count) = read_result
            && count > 0
        {
            let now_usec = ctx.shared.timer.lock(|timer| timer.get_counter());
            let now_usec = braid_triggerbox_comms::Instant::from_ticks(now_usec.ticks());
            let data = buf.get(..count).unwrap_or_default();
            // TODO: do not parse packets in IRQ
            match ctx.local.packet_parser.got_buf(now_usec, data) {
                Ok(ev) => {
                    if let Err(ev) = ctx.local.event_tx.enqueue(ev) {
                        warn!("event queue full, dropping event {}", ev);
                    }
                }
                Err(braid_triggerbox_comms::Error::AwaitingMoreData) => {}
                Err(e) => warn!("error parsing: {}", e),
            }
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        local = [alarm, led, tog: bool = true],
    )]
    fn timer_irq(ctx: timer_irq::Context<'_>) {
        let Ok(()) = ctx.local.led.set_state((*ctx.local.tog).into());
        *ctx.local.tog = !*ctx.local.tog;

        ctx.local.alarm.clear_interrupt();
        if ctx.local.alarm.schedule(SCAN_TIME).is_err() {
            warn!("could not schedule LED timer");
        }
    }

    #[task(
        binds = PWM_IRQ_WRAP,
        priority = 2,
        shared = [frame_number, pwm_slices],
        local = [pwm_cycle],
    )]
    fn pwm_irq(ctx: pwm_irq::Context<'_>) {
        let p = ctx.shared.pwm_slices;
        let f = ctx.shared.frame_number;

        (p, f).lock(|pwm_slices, frame_number| {
            let pwm0 = &mut pwm_slices.pwm0;
            pwm0.clear_interrupt();
            *frame_number = frame_number.saturating_add(1);
        });
    }

    fn handle_event(ctx: &mut idle::Context<'_>, event: UsbEvent) {
        debug!("handling event: {:?}", event);
        match event {
            UsbEvent::TimestampQuery(value) => {
                let timestamp_request = fill_sample(value, ctx);
                send_data(&timestamp_request, b'P', ctx);
            }
            UsbEvent::VersionRequest => {
                send_data(&fill_sample(DEVICE_FIRMWARE_VERSION, ctx), b'V', ctx);
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
                        if !*ctx.local.rate_ok {
                            warn!("Not starting trigger pulses: rate cannot be produced");
                            return;
                        }
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
            UsbEvent::SetTop(val) => set_top(ctx, &val),
            UsbEvent::SetAOut(val) => {
                // AOUT values
                info!("ignoring AOUT command {}, {}", val.aout0, val.aout1);

                let aout_confirm = fill_sample(val.aout_sequence, ctx);
                send_data(&aout_confirm, b'V', ctx);
            }
            UsbEvent::Udev(val) => {
                match val {
                    UdevMsg::Query => {
                        // Device names are not supported, so report an empty
                        // name.
                        let name = [0u8; 8];
                        let crc = CRC_MAXIM.checksum(&name);
                        let [n0, n1, n2, n3, n4, n5, n6, n7] = name;
                        // Emulate arduino "_serial.print(crc,HEX);" which will
                        // print a single character if the value is less that 0x10.
                        if crc >= 0x10 {
                            let (hi, lo) = (hexchar(crc >> 4), hexchar(crc));
                            send_buf(ctx, &[n0, n1, n2, n3, n4, n5, n6, n7, hi, lo]);
                        } else {
                            send_buf(ctx, &[n0, n1, n2, n3, n4, n5, n6, n7, hexchar(crc)]);
                        }
                    }
                    UdevMsg::Set(_) => {
                        warn!("Ignoring request to set device name: not supported");
                    }
                }
            }
        }
    }

    fn set_top(ctx: &mut idle::Context<'_>, val: &TopAndPrescaler) {
        info!(
            "Received TOP={}, prescaler_key='{}'",
            val.avr_icr1(),
            core::str::from_utf8(&[val.prescaler_key()][..]).unwrap_or("??")
        );

        let is_mode2 = match val.prescaler() {
            Some(Prescaler::Scale8) => false,
            Some(Prescaler::Scale64) => true,
            None => {
                warn!(
                    "Ignoring unsupported prescaler_key: {}",
                    val.prescaler_key()
                );
                return;
            }
        };
        let new_clock_scale = match EmulatedNanoPwmClock::new(
            val.avr_icr1(),
            is_mode2,
            ctx.local.clock_scale.system_clock_freq_hz(),
        ) {
            Ok(new_clock_scale) => new_clock_scale,
            Err(e) => {
                // Rather than trigger at the wrong rate, stop triggering.
                warn!(
                    "Stopping trigger pulses: cannot produce TOP={}: {}",
                    val.avr_icr1(),
                    e
                );
                *ctx.local.rate_ok = false;
                ctx.shared.pwm_slices.lock(|pwm_slices| {
                    pwm_slices.pwm0.disable();
                });
                return;
            }
        };
        *ctx.local.rate_ok = true;
        let top = new_clock_scale.to_top();

        let duty0 = (top / 100).max(1);
        let led_duty = duty0.saturating_mul(2).min(top.saturating_sub(1));
        let div_int = new_clock_scale.div_int();
        *ctx.local.clock_scale = new_clock_scale;

        ctx.shared.pwm_slices.lock(|pwm_slices| {
            let pwm0 = &mut pwm_slices.pwm0;

            // Output channel A on PWM0 to the GP0 pin
            let channel0 = &mut pwm0.channel_a;

            // Output channel B on PWM0 to the GP1 pin
            let channel1 = &mut pwm0.channel_b;

            let Ok(()) = channel0.set_duty_cycle(duty0);
            let Ok(()) = channel1.set_duty_cycle(led_duty);

            pwm0.set_top(top);
            pwm0.set_div_int(div_int);
        });
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
        fn to_bytes(&self) -> [u8; 7] {
            let [p0, p1, p2, p3] = self.pulsenumber.to_le_bytes();
            let [t0, t1] = self.ticks.to_le_bytes();
            [self.value, p0, p1, p2, p3, t0, t1]
        }
    }

    fn send_data(samp: &TimedSample, header: u8, ctx: &mut idle::Context<'_>) {
        let payload = samp.to_bytes();
        let chksum = payload.iter().fold(0u8, |acc, x| acc.wrapping_add(*x));
        let [d0, d1, d2, d3, d4, d5, d6] = payload;
        // The payload length is 7.
        send_buf(ctx, &[header, 7, d0, d1, d2, d3, d4, d5, d6, chksum]);
    }

    fn fill_sample(value: u8, ctx: &mut idle::Context<'_>) -> TimedSample {
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

    fn send_buf(ctx: &mut idle::Context<'_>, out_buf: &[u8]) {
        trace!("out_buf: {:?}", out_buf);
        match ctx.shared.serial.lock(|serial| serial.write(out_buf)) {
            Ok(nbytes) if nbytes == out_buf.len() => {}
            Ok(nbytes) => warn!("only sent {} of {} bytes", nbytes, out_buf.len()),
            Err(e) => warn!("error sending data: {}", defmt::Debug2Format(&e)),
        }
    }

    /// The uppercase hexadecimal digit of the lower 4 bits of `inchar`.
    const fn hexchar(inchar: u8) -> u8 {
        let lower_4_bits = inchar & 0x0F;
        // These cannot overflow because `lower_4_bits` is at most 0x0F.
        if lower_4_bits < 0x0A {
            lower_4_bits.wrapping_add(b'0')
        } else {
            lower_4_bits.wrapping_add(b'A' - 0x0A)
        }
    }
}
