use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Movement {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ToHub {
    pub vel: Movement,
    pub kick: bool,
    pub goal_opp: Option<f32>,
    pub goal_own: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ToJetson {
    pub sys: System,
    pub vel: Movement,
    pub sensor: Sensor,
    pub config: JetsonConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct System {
    pub pause: bool,
    pub shutdown: bool,
    pub reboot: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Sensor {
    pub ir: Ir,
    pub on_line: bool,
    pub have_ball: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Ir {
    pub x: f32,
    pub y: f32,
    pub strength: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum JetsonConfig {
    #[default]
    None,
    OpenCV(OpenCVConfig),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct OpenCVConfig {
    pub opp_color: HSV,
    pub own_color: HSV,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct HSV {
    pub h_min: u8,
    pub h_max: u8,
    pub s_min: u8,
    pub s_max: u8,
    pub v_min: u8,
    pub v_max: u8,
}
