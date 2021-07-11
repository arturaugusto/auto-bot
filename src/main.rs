//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![feature(exclusive_range_pattern)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

pub use core::mem::MaybeUninit;
use nb::block;
use panic_halt as _;

//use embedded_hal::digital::v2::InputPin; // the `set_high/low`function

use core::fmt::Write;
use cortex_m_rt::entry;

// use mpu6050::*;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::{
    // adc,
    // i2c::BlockingI2c,
    pac,
    prelude::*,
    pwm::Channel,
    // rtc::Rtc,
    serial::{Config, Serial},
    time::U32Ext,
    timer::{Tim2NoRemap, Tim3NoRemap, Timer},
};

use embedded_hal::digital::v2::OutputPin;

use pac::interrupt;

pub static mut ECHO_START: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA4<Input<Floating>>> =
    MaybeUninit::uninit();

pub static mut ECHO_END: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB3<Input<Floating>>> =
    MaybeUninit::uninit();

pub static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB8<Input<Floating>>> =
    MaybeUninit::uninit();

pub static mut ENABLE_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioc::PC13<Output<PushPull>>> =
    MaybeUninit::uninit();

// LEFT

pub static mut LEFT_REV_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB12<Output<PushPull>>> =
    MaybeUninit::uninit();

pub static mut LEFT_FWD_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB13<Output<PushPull>>> =
    MaybeUninit::uninit();

pub static mut LEFT_REV_PWM: MaybeUninit<
    stm32f1xx_hal::pwm::PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C2>,
> = MaybeUninit::uninit();

pub static mut LEFT_FWD_PWM: MaybeUninit<
    stm32f1xx_hal::pwm::PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C1>,
> = MaybeUninit::uninit();

// RIGHT

pub static mut RIGHT_REV_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB14<Output<PushPull>>> =
    MaybeUninit::uninit();

pub static mut RIGHT_FWD_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB15<Output<PushPull>>> =
    MaybeUninit::uninit();

pub static mut RIGHT_REV_PWM: MaybeUninit<
    stm32f1xx_hal::pwm::PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C4>,
> = MaybeUninit::uninit();

pub static mut RIGHT_FWD_PWM: MaybeUninit<
    stm32f1xx_hal::pwm::PwmChannel<stm32f1xx_hal::pac::TIM2, stm32f1xx_hal::pwm::C3>,
> = MaybeUninit::uninit();

pub static mut TRIG_HC_PWM: MaybeUninit<
    stm32f1xx_hal::pwm::PwmChannel<stm32f1xx_hal::pac::TIM3, stm32f1xx_hal::pwm::C1>,
> = MaybeUninit::uninit();

// COMMUNICATION

pub static mut RX: MaybeUninit<stm32f1xx_hal::serial::Rx<stm32f1xx_hal::pac::USART1>> =
    MaybeUninit::uninit();

pub static mut TX: MaybeUninit<stm32f1xx_hal::serial::Tx<stm32f1xx_hal::pac::USART1>> =
    MaybeUninit::uninit();

pub static mut DELAY: MaybeUninit<stm32f1xx_hal::delay::Delay> = MaybeUninit::uninit();

// pub static mut R_SPEED: u16 = 0u16;
// pub static mut L_SPEED: u16 = 0u16;

// pub static mut RTC: MaybeUninit<Rtc> = MaybeUninit::uninit();

const CMD_BUF_LEN: usize = 10usize;
pub static mut CMD_BUF: [u8; CMD_BUF_LEN] = [0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8];
pub static mut CMD_COUNT: usize = 0;

pub static mut ECHO_WIDTH: usize = 0usize;

#[interrupt]
fn EXTI4() {
    // let rtc = unsafe { &mut *RTC.as_mut_ptr() };
    let delay = unsafe { &mut *DELAY.as_mut_ptr() };
    let int_pin_pa4 = unsafe { &mut *ECHO_START.as_mut_ptr() };
    // let txs = unsafe { &mut *TX.as_mut_ptr() };
    if int_pin_pa4.check_interrupt() {
        delay.delay_us(1u16);
        unsafe {
            ECHO_WIDTH += 1;
        }
        // loop {
        // }
        // writeln!(txs, "start: {}", rtc.current_time()).unwrap();
    }
}

#[interrupt]
fn EXTI3() {
    // let rtc = unsafe { &mut *RTC.as_mut_ptr() };
    let int_pin_pb3 = unsafe { &mut *ECHO_END.as_mut_ptr() };
    let int_pin_pa4 = unsafe { &mut *ECHO_START.as_mut_ptr() };
    // let trig_hc_pwm = unsafe { &mut *TRIG_HC_PWM.as_mut_ptr() };
    let txs = unsafe { &mut *TX.as_mut_ptr() };
    if int_pin_pb3.check_interrupt() {
        unsafe {
            // in cm
            writeln!(txs, "{}", ((ECHO_WIDTH as f32) / 58f32) / 0.2f32).unwrap();
            ECHO_WIDTH = 0;
        }
        int_pin_pa4.clear_interrupt_pending_bit();
        int_pin_pb3.clear_interrupt_pending_bit();
    }
}

#[interrupt]
fn EXTI9_5() {
    let int_pin_pb8 = unsafe { &mut *INT_PIN.as_mut_ptr() };

    let trig_hc_pwm = unsafe { &mut *TRIG_HC_PWM.as_mut_ptr() };

    let int_pin_pb3 = unsafe { &mut *ECHO_END.as_mut_ptr() };
    let int_pin_pa4 = unsafe { &mut *ECHO_START.as_mut_ptr() };

    let rxs = unsafe { &mut *RX.as_mut_ptr() };
    let txs = unsafe { &mut *TX.as_mut_ptr() };

    let left_rev_pin_pb12 = unsafe { &mut *LEFT_REV_PIN.as_mut_ptr() };
    let left_fwd_pin_pb13 = unsafe { &mut *LEFT_FWD_PIN.as_mut_ptr() };

    let right_rev_pin_pb14 = unsafe { &mut *RIGHT_REV_PIN.as_mut_ptr() };
    let right_fwd_pin_pb15 = unsafe { &mut *RIGHT_FWD_PIN.as_mut_ptr() };

    let left_rev_pwm_pa0 = unsafe { &mut *LEFT_REV_PWM.as_mut_ptr() };
    let left_fwd_pwm_pa1 = unsafe { &mut *LEFT_FWD_PWM.as_mut_ptr() };

    let right_rev_pwm_pa2 = unsafe { &mut *RIGHT_REV_PWM.as_mut_ptr() };
    let right_fwd_pwm_pa3 = unsafe { &mut *RIGHT_FWD_PWM.as_mut_ptr() };

    let enable_pin_pc13 = unsafe { &mut *ENABLE_PIN.as_mut_ptr() };

    let delay = unsafe { &mut *DELAY.as_mut_ptr() };

    let max = left_rev_pwm_pa0.get_max_duty();
    // writeln!(txs, "max: {}", max).unwrap(); // 8000 for bluepill

    if int_pin_pb8.check_interrupt() {
        int_pin_pb8.clear_interrupt_pending_bit();
        match block!(rxs.read()) {
            Ok(received) => {
                // delay.delay_ms(1_000_u16);
                // block!(txs.write(received)).ok();
                // writeln!(txs, "{}", value).unwrap();
                // writeln!(txs, "{}", received).unwrap();

                match received {
                    // new line
                    10 => {
                        enable_pin_pc13.set_low().unwrap();
                        delay.delay_ms(1_000_u16);
                        // get instruction numerical value
                        let mut value = 0f32;

                        for i in 1..(CMD_BUF_LEN - 1) {
                            unsafe {
                                // ignore non numeric char
                                if CMD_BUF[i] >= 48u8 && CMD_BUF[i] < 58u8 {
                                    value = value * 10f32;
                                    value = value + ((CMD_BUF[i] - 48u8) as f32);
                                }
                            }
                        }

                        if value as u16 > max {
                            value = max as f32
                        }

                        let mut cmd: u8 = 0;
                        unsafe {
                            cmd = CMD_BUF[0];
                        }

                        unsafe {
                            write!(txs, "{}...", CMD_BUF[0]).unwrap();
                        }
                        match cmd {
                            // a
                            97 => {
                                left_rev_pwm_pa0.set_duty(max);
                                left_fwd_pin_pb13.set_low().unwrap();
                                left_fwd_pwm_pa1.set_duty(max - value as u16);
                                left_rev_pin_pb12.set_high().unwrap();
                            }

                            // A
                            65 => {
                                left_fwd_pwm_pa1.set_duty(max);
                                left_rev_pin_pb12.set_low().unwrap();
                                left_rev_pwm_pa0.set_duty(max - value as u16);
                                left_fwd_pin_pb13.set_high().unwrap();
                            }

                            // b
                            98 => {
                                right_rev_pwm_pa2.set_duty(max);
                                right_fwd_pin_pb15.set_low().unwrap();
                                right_fwd_pwm_pa3.set_duty(max - value as u16);
                                right_rev_pin_pb14.set_high().unwrap();
                            }

                            // B
                            66 => {
                                right_fwd_pwm_pa3.set_duty(max);
                                right_rev_pin_pb14.set_low().unwrap();
                                right_rev_pwm_pa2.set_duty(max - value as u16);
                                right_fwd_pin_pb15.set_high().unwrap();
                            }

                            // c
                            99 => {
                                trig_hc_pwm.disable();
                                int_pin_pa4.clear_interrupt_pending_bit();
                                int_pin_pb3.clear_interrupt_pending_bit();

                                right_rev_pin_pb14.set_low().unwrap();
                                right_fwd_pin_pb15.set_low().unwrap();
                                right_rev_pwm_pa2.set_duty(max);
                                right_fwd_pwm_pa3.set_duty(max);
                            }

                            // d
                            100 => {
                                trig_hc_pwm.enable();
                            }

                            _ => {}
                        }
                        writeln!(txs, "done").unwrap();
                        enable_pin_pc13.set_high().unwrap();
                        // if value > 0f32 {
                        // }

                        // fill with null
                        unsafe {
                            for i in 0..(CMD_BUF_LEN - 1) {
                                CMD_BUF[i] = 0u8;
                            }
                            CMD_COUNT = 0;
                        }
                    }
                    _ => unsafe {
                        CMD_BUF[CMD_COUNT] = received;
                        CMD_COUNT = CMD_COUNT + 1;
                    },
                };
            }
            _ => {} // block!(txs.write(33)).ok(); // !
        }

        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin_pb8.clear_interrupt_pending_bit();
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // let mut pwr = p.PWR;
    // let mut backup_domain = rcc.bkp.constrain(p.BKP, &mut rcc.apb1, &mut pwr);
    // let mut rtc = unsafe { &mut *RTC.as_mut_ptr() };
    // *rtc = Rtc::rtc(p.RTC, &mut backup_domain);
    // rtc.set_time(0);
    // rtc.select_frequency(16384.hz());

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    let (_pa15, pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // serial

    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    let (tx, rx) = serial.split();
    let rxs = unsafe { &mut *RX.as_mut_ptr() };
    let txs = unsafe { &mut *TX.as_mut_ptr() };
    *rxs = rx;
    *txs = tx;

    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let pins = (c1, c2, c3, c4);

    let mut pwm = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1).pwm::<Tim2NoRemap, _, _, _>(
        pins,
        &mut afio.mapr,
        1.khz(),
    );

    // TIM3
    let trig_hc_c1 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    let trig_hc_c2 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let trig_hc_c3 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let trig_hc_c4 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);

    let trig_hc_pins = (trig_hc_c1, trig_hc_c2, trig_hc_c3, trig_hc_c4);

    let trig_hc_pwm = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).pwm::<Tim3NoRemap, _, _, _>(
        trig_hc_pins,
        &mut afio.mapr,
        30.hz(),
    );

    {
        let enable_pin_pc13 = unsafe { &mut *ENABLE_PIN.as_mut_ptr() };
        *enable_pin_pc13 = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        enable_pin_pc13.set_low().unwrap();

        let left_rev_pin_pb12 = unsafe { &mut *LEFT_REV_PIN.as_mut_ptr() };
        *left_rev_pin_pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

        let left_fwd_pin_pb13 = unsafe { &mut *LEFT_FWD_PIN.as_mut_ptr() };
        *left_fwd_pin_pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);

        let right_rev_pin_pb14 = unsafe { &mut *RIGHT_REV_PIN.as_mut_ptr() };
        *right_rev_pin_pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);

        let right_fwd_pin_pb15 = unsafe { &mut *RIGHT_FWD_PIN.as_mut_ptr() };
        *right_fwd_pin_pb15 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);

        let int_pin_pb8 = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *int_pin_pb8 = gpiob.pb8.into_floating_input(&mut gpiob.crh);
        int_pin_pb8.make_interrupt_source(&mut afio);
        int_pin_pb8.trigger_on_edge(&p.EXTI, Edge::RISING_FALLING);
        int_pin_pb8.enable_interrupt(&p.EXTI);

        let int_pin_pa4 = unsafe { &mut *ECHO_START.as_mut_ptr() };
        *int_pin_pa4 = gpioa.pa4.into_floating_input(&mut gpioa.crl);
        int_pin_pa4.make_interrupt_source(&mut afio);
        int_pin_pa4.trigger_on_edge(&p.EXTI, Edge::RISING);
        int_pin_pa4.enable_interrupt(&p.EXTI);

        let int_pin_pb3 = unsafe { &mut *ECHO_END.as_mut_ptr() };
        // *int_pin_pb3 = gpiob.pb3.into_floating_input(&mut gpiob.crl);
        *int_pin_pb3 = pb3.into_floating_input(&mut gpiob.crl);
        int_pin_pb3.make_interrupt_source(&mut afio);
        int_pin_pb3.trigger_on_edge(&p.EXTI, Edge::FALLING);
        int_pin_pb3.enable_interrupt(&p.EXTI);
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI4);
        pac::NVIC::unmask(pac::Interrupt::EXTI3);
        pac::NVIC::unmask(pac::Interrupt::EXTI9_5);
    }

    // Enable clock on each of the channels
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    pwm.enable(Channel::C4);

    let pwm_channels = pwm.split();

    // left rev
    let left_rev_pwm_pa0 = unsafe { &mut *LEFT_REV_PWM.as_mut_ptr() };
    *left_rev_pwm_pa0 = pwm_channels.1;

    // left fwd
    let left_fwd_pwm_pa1 = unsafe { &mut *LEFT_FWD_PWM.as_mut_ptr() };
    *left_fwd_pwm_pa1 = pwm_channels.0;

    // right rev
    let right_rev_pwm_pa2 = unsafe { &mut *RIGHT_REV_PWM.as_mut_ptr() };
    *right_rev_pwm_pa2 = pwm_channels.3;

    // right fwd
    let right_fwd_pwm_pa3 = unsafe { &mut *RIGHT_FWD_PWM.as_mut_ptr() };
    *right_fwd_pwm_pa3 = pwm_channels.2;

    // trig_hc_pwm.enable(Channel::C1);
    // trig_hc_pwm.enable(Channel::C2);
    // trig_hc_pwm.enable(Channel::C3);
    // trig_hc_pwm.enable(Channel::C4);

    let trig_hc_pwm_channels = trig_hc_pwm.split();
    let trig_hc_pwm_pa6 = unsafe { &mut *TRIG_HC_PWM.as_mut_ptr() };
    *trig_hc_pwm_pa6 = trig_hc_pwm_channels.0;

    trig_hc_pwm_pa6.set_duty(30);

    /*
    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    //set up I2C

    let i2c_mode = stm32f1xx_hal::i2c::Mode::Fast {
        frequency: 400_000.hz(),
        duty_cycle: stm32f1xx_hal::i2c::DutyCycle::Ratio2to1,
    };

    let start_timeout_us = 10_000;
    let start_retries = 10;
    let addr_timeout_us = 10_000;
    let data_timeout_us = 10_000;

    let i2c = BlockingI2c::i2c2(
        p.I2C2,
        (scl, sda),
        i2c_mode,
        clocks,
        &mut rcc.apb1,
        start_timeout_us,
        start_retries,
        addr_timeout_us,
        data_timeout_us,
    );

    let delay = unsafe { &mut *DELAY.as_mut_ptr() };
    let mut mpu = Mpu6050::new(i2c);

    // get roll and pitch estimate

    // let acc = mpu.get_acc_angles().unwrap()?;
    mpu.init(delay).unwrap();
    delay.delay_ms(1000u16);

    match mpu.get_gyro() {
        Ok(r) => write!(txs, "{:?}\n", r[2]),
        // Ok(r) => block!(txs.write(r.data[0] as u8)).unwrap(),
        _ => write!(txs, "error\n"),
    };
    */

    // let mut adc1 = adc::Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);

    loop {
        // let adc1_data: u16 = adc1.read(&mut ch1).unwrap();
    }
}
