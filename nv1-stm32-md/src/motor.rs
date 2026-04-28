use embassy_stm32::timer::{
    AdvancedInstance4Channel, Channel, GeneralInstance4Channel,
    complementary_pwm::ComplementaryPwm,
    simple_pwm::SimplePwm,
};

pub trait MotorGroup {
    fn set_speed1(&mut self, speed: i16);
    fn stop1(&mut self);
    fn set_speed2(&mut self, speed: i16);
    fn stop2(&mut self);
}

pub struct Motor {
    ch_a: Channel,
    ch_b: Channel,
}

pub struct MotorGroupComplementary<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    pwm: ComplementaryPwm<'a, TIM>,
    motor1: Motor,
    motor2: Motor,
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
    ) -> MotorGroupComplementary<'a, TIM> {
        let multiply = pwm.get_max_duty() as f32 / width as f32;

        MotorGroupComplementary {
            pwm,
            motor1: Motor {
                ch_a: motor1_a,
                ch_b: motor1_b,
            },
            motor2: Motor {
                ch_a: motor2_a,
                ch_b: motor2_b,
            },
            multiply,
        }
    }
}

impl<'a, TIM> MotorGroup for MotorGroupComplementary<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    fn set_speed1(&mut self, speed: i16) {
        if speed > 0 {
            self.pwm
                .set_duty(self.motor1.ch_a, (speed as f32 * self.multiply) as u32);
            self.pwm.set_duty(self.motor1.ch_b, 0);
        } else {
            self.pwm.set_duty(self.motor1.ch_a, 0);
            self.pwm
                .set_duty(self.motor1.ch_b, (-speed as f32 * self.multiply) as u32);
        }
    }

    fn stop1(&mut self) {
        self.pwm.set_duty(self.motor1.ch_a, 0);
        self.pwm.set_duty(self.motor1.ch_b, 0);
    }

    fn set_speed2(&mut self, speed: i16) {
        if speed > 0 {
            self.pwm
                .set_duty(self.motor2.ch_a, (speed as f32 * self.multiply) as u32);
            self.pwm.set_duty(self.motor2.ch_b, 0);
        } else {
            self.pwm.set_duty(self.motor2.ch_a, 0);
            self.pwm
                .set_duty(self.motor2.ch_b, (-speed as f32 * self.multiply) as u32);
        }
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
    motor1: Motor,
    motor2: Motor,
    multiply: f32,
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
    ) -> MotorGroupSimple<'a, TIM> {
        let multiply = pwm.channel(motor1_a).max_duty_cycle() as f32 / width as f32;

        MotorGroupSimple {
            pwm,
            motor1: Motor {
                ch_a: motor1_a,
                ch_b: motor1_b,
            },
            motor2: Motor {
                ch_a: motor2_a,
                ch_b: motor2_b,
            },
            multiply,
        }
    }
}

impl<'a, TIM> MotorGroup for MotorGroupSimple<'a, TIM>
where
    TIM: GeneralInstance4Channel,
{
    fn set_speed1(&mut self, speed: i16) {
        if speed > 0 {
            self.pwm
                .channel(self.motor1.ch_a)
                .set_duty_cycle((speed as f32 * self.multiply) as u32);
            self.pwm.channel(self.motor1.ch_b).set_duty_cycle(0);
        } else {
            self.pwm.channel(self.motor1.ch_a).set_duty_cycle(0);
            self.pwm
                .channel(self.motor1.ch_b)
                .set_duty_cycle((-speed as f32 * self.multiply) as u32);
        }
    }

    fn stop1(&mut self) {
        self.pwm.channel(self.motor1.ch_a).set_duty_cycle(0);
        self.pwm.channel(self.motor1.ch_b).set_duty_cycle(0);
    }

    fn set_speed2(&mut self, speed: i16) {
        if speed > 0 {
            self.pwm
                .channel(self.motor2.ch_a)
                .set_duty_cycle((speed as f32 * self.multiply) as u32);
            self.pwm.channel(self.motor2.ch_b).set_duty_cycle(0);
        } else {
            self.pwm.channel(self.motor2.ch_a).set_duty_cycle(0);
            self.pwm
                .channel(self.motor2.ch_b)
                .set_duty_cycle((-speed as f32 * self.multiply) as u32);
        }
    }

    fn stop2(&mut self) {
        self.pwm.channel(self.motor2.ch_a).set_duty_cycle(0);
        self.pwm.channel(self.motor2.ch_b).set_duty_cycle(0);
    }
}

pub struct Motors<G1, G2>
where
    G1: MotorGroup,
    G2: MotorGroup,
{
    pub group1: G1,
    pub group2: G2,
}

impl<G1, G2> Motors<G1, G2>
where
    G1: MotorGroup,
    G2: MotorGroup,
{
    pub fn new(group1: G1, group2: G2) -> Motors<G1, G2> {
        Motors { group1, group2 }
    }

    pub fn set_speed1(&mut self, speed: i16) {
        self.group1.set_speed1(speed);
    }

    pub fn stop1(&mut self) {
        self.group1.stop1();
    }

    pub fn set_speed2(&mut self, speed: i16) {
        self.group1.set_speed2(speed);
    }

    pub fn stop2(&mut self) {
        self.group1.stop2();
    }

    pub fn set_speed3(&mut self, speed: i16) {
        self.group2.set_speed1(speed);
    }

    pub fn stop3(&mut self) {
        self.group2.stop1();
    }

    pub fn set_speed4(&mut self, speed: i16) {
        self.group2.set_speed2(speed);
    }

    pub fn stop4(&mut self) {
        self.group2.stop2();
    }
}
