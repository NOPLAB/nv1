use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ToMD {
    pub enable: bool,
    pub m1: f32,
    pub m2: f32,
    pub m3: f32,
    pub m4: f32,
}
