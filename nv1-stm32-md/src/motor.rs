//! STM32-specific implementations of [`MotorGroup`] for the two timer flavours
//! the board uses: TIM1 (advanced, complementary outputs) and TIM8 (general,
//! simple outputs).
//!
//! The trait, the higher-level [`Motors`] aggregator, and all polarity-aware
//! arithmetic live in `nv1-md-core` so they can be unit-tested on the host.

use embassy_stm32::timer::{
    complementary_pwm::ComplementaryPwm, simple_pwm::SimplePwm, AdvancedInstance4Channel, Channel,
    GeneralInstance4Channel,
};
use nv1_md_core::motor::{duty_pair, MotorGroup};

struct Channels {
    ch_a: Channel,
    ch_b: Channel,
}

pub struct MotorGroupComplementary<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    pwm: ComplementaryPwm<'a, TIM>,
    motor1: Channels,
    motor2: Channels,
    multiply: f32,
}

impl<'a, TIM> MotorGroupComplementary<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    pub fn new(
        pwm: ComplementaryPwm<'a, TIM>,
        motor1_a: Channel,
        motor1_b: Channel,
        motor2_a: Channel,
        motor2_b: Channel,
        width: u16,
    ) -> Self {
        let multiply = pwm.get_max_duty() as f32 / width as f32;
        Self {
            pwm,
            motor1: Channels {
                ch_a: motor1_a,
                ch_b: motor1_b,
            },
            motor2: Channels {
                ch_a: motor2_a,
                ch_b: motor2_b,
            },
            multiply,
        }
    }

    fn max_duty(&self) -> u32 {
        self.pwm.get_max_duty()
    }

    fn apply(&mut self, channels: Channels, speed: i16) {
        let (forward, reverse) = duty_pair(speed, self.multiply, self.max_duty());
        self.pwm.set_duty(channels.ch_a, forward);
        self.pwm.set_duty(channels.ch_b, reverse);
    }
}

impl<'a, TIM> MotorGroup for MotorGroupComplementary<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    fn set_speed1(&mut self, speed: i16) {
        let (a, b) = (self.motor1.ch_a, self.motor1.ch_b);
        self.apply(Channels { ch_a: a, ch_b: b }, speed);
    }

    fn stop1(&mut self) {
        self.pwm.set_duty(self.motor1.ch_a, 0);
        self.pwm.set_duty(self.motor1.ch_b, 0);
    }

    fn set_speed2(&mut self, speed: i16) {
        let (a, b) = (self.motor2.ch_a, self.motor2.ch_b);
        self.apply(Channels { ch_a: a, ch_b: b }, speed);
    }

    fn stop2(&mut self) {
        self.pwm.set_duty(self.motor2.ch_a, 0);
        self.pwm.set_duty(self.motor2.ch_b, 0);
    }
}

pub struct MotorGroupSimple<'a, TIM>
where
    TIM: GeneralInstance4Channel,
{
    pwm: SimplePwm<'a, TIM>,
    motor1: Channels,
    motor2: Channels,
    multiply: f32,
    max_duty: u32,
}

impl<'a, TIM> MotorGroupSimple<'a, TIM>
where
    TIM: GeneralInstance4Channel,
{
    pub fn new(
        mut pwm: SimplePwm<'a, TIM>,
        motor1_a: Channel,
        motor1_b: Channel,
        motor2_a: Channel,
        motor2_b: Channel,
        width: u16,
    ) -> Self {
        let max_duty = pwm.channel(motor1_a).max_duty_cycle();
        let multiply = max_duty as f32 / width as f32;
        Self {
            pwm,
            motor1: Channels {
                ch_a: motor1_a,
                ch_b: motor1_b,
            },
            motor2: Channels {
                ch_a: motor2_a,
                ch_b: motor2_b,
            },
            multiply,
            max_duty,
        }
    }

    fn apply(&mut self, channels: &Channels, speed: i16) {
        let (forward, reverse) = duty_pair(speed, self.multiply, self.max_duty);
        self.pwm.channel(channels.ch_a).set_duty_cycle(forward);
        self.pwm.channel(channels.ch_b).set_duty_cycle(reverse);
    }
}

impl<'a, TIM> MotorGroup for MotorGroupSimple<'a, TIM>
where
    TIM: GeneralInstance4Channel,
{
    fn set_speed1(&mut self, speed: i16) {
        let chans = Channels {
            ch_a: self.motor1.ch_a,
            ch_b: self.motor1.ch_b,
        };
        self.apply(&chans, speed);
    }

    fn stop1(&mut self) {
        self.pwm.channel(self.motor1.ch_a).set_duty_cycle(0);
        self.pwm.channel(self.motor1.ch_b).set_duty_cycle(0);
    }

    fn set_speed2(&mut self, speed: i16) {
        let chans = Channels {
            ch_a: self.motor2.ch_a,
            ch_b: self.motor2.ch_b,
        };
        self.apply(&chans, speed);
    }

    fn stop2(&mut self) {
        self.pwm.channel(self.motor2.ch_a).set_duty_cycle(0);
        self.pwm.channel(self.motor2.ch_b).set_duty_cycle(0);
    }
}
