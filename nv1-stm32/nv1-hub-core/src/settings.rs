//! Hub-side runtime settings, extracted from `nv1-hub/src/settings.rs`.
//!
//! Settings are persisted to flash by the firmware (`nv1-hub/src/settings.rs`),
//! but the schema, defaults, and getters/mutators live here so they can be
//! tested on the host and serialised without pulling in `embassy-stm32`.

use nv1_msg::hub::HSV;
use serde::{Deserialize, Serialize};

use crate::constants::{
    DEFAULT_HAVE_BALL_THRESHOLD, DEFAULT_LINE_THRESHOLD, DEFAULT_OPENCV_GOAL_BLUE,
    DEFAULT_OPENCV_GOAL_YELLOW, DEFAULT_ROBOT_SPEED_MULTIPLIER,
};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum GoalColor {
    #[default]
    Blue,
    Yellow,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Settings {
    pub line_threshold: f32,
    pub have_ball_threshold: u16,
    opp_goal_color: GoalColor,
    pub opencv_goal_blue: HSV,
    pub opencv_goal_yellow: HSV,
    pub robot_speed_multiplier: f32,
}

impl Settings {
    pub const DEFAULT: Self = Settings {
        line_threshold: DEFAULT_LINE_THRESHOLD,
        have_ball_threshold: DEFAULT_HAVE_BALL_THRESHOLD,
        robot_speed_multiplier: DEFAULT_ROBOT_SPEED_MULTIPLIER,
        opp_goal_color: GoalColor::Blue,
        opencv_goal_blue: DEFAULT_OPENCV_GOAL_BLUE,
        opencv_goal_yellow: DEFAULT_OPENCV_GOAL_YELLOW,
    };

    pub fn get_opp_color(&self) -> HSV {
        match self.opp_goal_color {
            GoalColor::Blue => self.opencv_goal_blue,
            GoalColor::Yellow => self.opencv_goal_yellow,
        }
    }

    pub fn get_own_color(&self) -> HSV {
        match self.opp_goal_color {
            GoalColor::Blue => self.opencv_goal_yellow,
            GoalColor::Yellow => self.opencv_goal_blue,
        }
    }

    pub fn toggle_goal_color(&mut self) {
        self.opp_goal_color = match self.opp_goal_color {
            GoalColor::Blue => GoalColor::Yellow,
            GoalColor::Yellow => GoalColor::Blue,
        };
    }

    pub fn get_opp_goal_color(&self) -> GoalColor {
        self.opp_goal_color
    }

    /// Replaces NaN numeric fields with defaults.
    ///
    /// Returns `true` when a value was repaired so the firmware caller
    /// can decide whether to write the result back to flash.
    pub fn sanitize(&mut self) -> bool {
        let mut changed = false;
        if self.line_threshold.is_nan() {
            self.line_threshold = DEFAULT_LINE_THRESHOLD;
            changed = true;
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_uses_blue_as_opp_color() {
        let s = Settings::DEFAULT;
        assert_eq!(s.get_opp_goal_color(), GoalColor::Blue);
        assert_eq!(s.get_opp_color(), DEFAULT_OPENCV_GOAL_BLUE);
        assert_eq!(s.get_own_color(), DEFAULT_OPENCV_GOAL_YELLOW);
    }

    #[test]
    fn toggle_swaps_colors() {
        let mut s = Settings::DEFAULT;
        s.toggle_goal_color();
        assert_eq!(s.get_opp_goal_color(), GoalColor::Yellow);
        assert_eq!(s.get_opp_color(), DEFAULT_OPENCV_GOAL_YELLOW);
        assert_eq!(s.get_own_color(), DEFAULT_OPENCV_GOAL_BLUE);
        s.toggle_goal_color();
        assert_eq!(s.get_opp_goal_color(), GoalColor::Blue);
    }

    #[test]
    fn sanitize_repairs_nan_threshold() {
        let mut s = Settings::DEFAULT;
        s.line_threshold = f32::NAN;
        assert!(s.sanitize());
        assert_eq!(s.line_threshold, DEFAULT_LINE_THRESHOLD);
    }

    #[test]
    fn sanitize_returns_false_when_clean() {
        let mut s = Settings::DEFAULT;
        assert!(!s.sanitize());
    }

    #[test]
    fn postcard_roundtrip_preserves_fields() {
        let mut s = Settings::DEFAULT;
        s.line_threshold = 0.42;
        s.have_ball_threshold = 1234;
        s.robot_speed_multiplier = 2.5;
        s.toggle_goal_color();

        let mut buf = [0u8; 128];
        let bytes = postcard::to_slice(&s, &mut buf).unwrap();
        let decoded: Settings = postcard::from_bytes(bytes).unwrap();

        assert_eq!(decoded.line_threshold, 0.42);
        assert_eq!(decoded.have_ball_threshold, 1234);
        assert_eq!(decoded.robot_speed_multiplier, 2.5);
        assert_eq!(decoded.get_opp_goal_color(), GoalColor::Yellow);
    }
}
