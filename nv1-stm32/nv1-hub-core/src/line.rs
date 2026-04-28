//! Line-detection state machine, extracted from `nv1-hub/src/sensors.rs`.
//!
//! The hub feeds a (possibly missing) line-direction vector into
//! [`LineProcessor::process_line`] every control tick. The processor
//! returns the correction vector to apply to the body, or `None` when
//! the robot is on open field. Tracking is sticky: once a line is
//! detected, the processor keeps issuing corrections for a short timeout
//! window even if the sensors briefly drop out, and it transitions
//! through `OutOfLineOverCenter` when the robot crosses past the line.

use core::f32::consts::PI;

use crate::constants::LINE_OVER_CENTER_THRESHOLD;
use crate::types::Vector2;

const COUNTER_TIMEOUT_LIMIT: u32 = 100;

/// Latched state of the line-detection state machine.
///
/// The tuple variants carry `(first_angle, first_x, first_y, counter)`,
/// where `first_*` is the line vector observed when the line was first
/// seen and `counter` increments each tick the line is missing.
#[derive(Debug, Clone, Copy)]
pub enum AdcState {
    OnGround,
    OnLine(f32, f32, f32, u32),
    OutOfLineOverCenter(f32, f32, f32, u32),
}

pub struct LineProcessor {
    pub state: AdcState,
}

impl Default for LineProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl LineProcessor {
    pub fn new() -> Self {
        Self {
            state: AdcState::OnGround,
        }
    }

    pub fn process_line(
        &mut self,
        line_vector: Option<Vector2>,
        _threshold: f32,
    ) -> Option<Vector2> {
        // Reject NaN/inf inputs at the boundary so a single bad reading
        // never poisons the latched state (first_angle, first_x, first_y
        // would carry NaN forward through every subsequent tick).
        let line_vector = line_vector.filter(|v| v.x.is_finite() && v.y.is_finite());
        match self.state {
            AdcState::OnGround => self.handle_on_ground_state(line_vector),
            AdcState::OnLine(first_angle, first_x, first_y, counter) => {
                self.handle_on_line_state(line_vector, first_angle, first_x, first_y, counter)
            }
            AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter) => self
                .handle_out_of_line_over_center_state(
                    line_vector,
                    first_angle,
                    first_x,
                    first_y,
                    counter,
                ),
        }
    }

    fn handle_on_ground_state(&mut self, line_vector: Option<Vector2>) -> Option<Vector2> {
        if let Some(detected_vector) = line_vector {
            let detected_angle = detected_vector.angle();
            self.state = AdcState::OnLine(detected_angle, detected_vector.x, detected_vector.y, 0);
            Some(-detected_vector)
        } else {
            self.state = AdcState::OnGround;
            None
        }
    }

    fn handle_on_line_state(
        &mut self,
        line_vector: Option<Vector2>,
        first_angle: f32,
        first_x: f32,
        first_y: f32,
        counter: u32,
    ) -> Option<Vector2> {
        if let Some(current_vector) = line_vector {
            let current_angle = current_vector.angle();
            let angle_tolerance = LINE_OVER_CENTER_THRESHOLD / 2.0;
            let min_allowed_angle = first_angle - angle_tolerance;
            let max_allowed_angle = first_angle + angle_tolerance;

            if !is_angle_in_range(current_angle, min_allowed_angle, max_allowed_angle) {
                self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, 0);
                Some(-current_vector)
            } else {
                self.state = AdcState::OnLine(first_angle, first_x, first_y, 0);
                Some(-current_vector)
            }
        } else if counter > COUNTER_TIMEOUT_LIMIT {
            self.state = AdcState::OnGround;
            None
        } else {
            let last_known_vector = Vector2::new(first_x, first_y);
            self.state = AdcState::OnLine(first_angle, first_x, first_y, counter + 1);
            Some(-last_known_vector)
        }
    }

    fn handle_out_of_line_over_center_state(
        &mut self,
        line_vector: Option<Vector2>,
        first_angle: f32,
        first_x: f32,
        first_y: f32,
        counter: u32,
    ) -> Option<Vector2> {
        let original_vector = Vector2::new(first_x, first_y);

        if line_vector.is_some() {
            self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter + 1);
            Some(-original_vector)
        } else if counter > COUNTER_TIMEOUT_LIMIT {
            self.state = AdcState::OnGround;
            None
        } else {
            self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter + 1);
            Some(-original_vector)
        }
    }
}

/// Returns true if `angle` lies within the inclusive arc from `a` to `b`,
/// taking the shorter arc when the two are within π of each other.
pub fn is_angle_in_range(angle: f32, a: f32, b: f32) -> bool {
    let normalize = |x: f32| -> f32 {
        let mut x = x;
        while x > PI {
            x -= 2.0 * PI;
        }
        while x <= -PI {
            x += 2.0 * PI;
        }
        x
    };

    let a = normalize(a);
    let b = normalize(b);
    let angle = normalize(angle);

    if libm::fabsf(b - a) <= PI {
        a <= angle && angle <= b
    } else {
        !(b <= angle && angle <= a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    fn matches_on_ground(state: &AdcState) -> bool {
        matches!(state, AdcState::OnGround)
    }

    fn matches_on_line(state: &AdcState) -> bool {
        matches!(state, AdcState::OnLine(..))
    }

    fn matches_over_center(state: &AdcState) -> bool {
        matches!(state, AdcState::OutOfLineOverCenter(..))
    }

    #[test]
    fn starts_on_ground_with_no_correction() {
        let p = LineProcessor::new();
        assert!(matches_on_ground(&p.state));
    }

    #[test]
    fn ground_with_no_line_returns_none() {
        let mut p = LineProcessor::new();
        assert!(p.process_line(None, 0.1).is_none());
        assert!(matches_on_ground(&p.state));
    }

    #[test]
    fn ground_to_on_line_returns_negated_vector() {
        let mut p = LineProcessor::new();
        let v = Vector2::new(1.0, 0.0);
        let out = p.process_line(Some(v), 0.1).unwrap();
        assert!(approx(out.x, -1.0) && approx(out.y, 0.0));
        assert!(matches_on_line(&p.state));
    }

    #[test]
    fn on_line_holds_last_vector_through_short_dropout() {
        let mut p = LineProcessor::new();
        let v = Vector2::new(1.0, 0.0);
        p.process_line(Some(v), 0.1);
        for i in 0..50 {
            let out = p
                .process_line(None, 0.1)
                .unwrap_or_else(|| panic!("dropout step {i} returned None"));
            assert!(approx(out.x, -1.0) && approx(out.y, 0.0));
        }
        assert!(matches_on_line(&p.state));
    }

    #[test]
    fn on_line_drops_to_ground_after_timeout() {
        let mut p = LineProcessor::new();
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        // 101 missing readings: counter goes 1..=101, then exceeds limit.
        for _ in 0..101 {
            let _ = p.process_line(None, 0.1);
        }
        assert!(p.process_line(None, 0.1).is_none());
        assert!(matches_on_ground(&p.state));
    }

    #[test]
    fn on_line_resets_dropout_counter_when_line_returns() {
        let mut p = LineProcessor::new();
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        for _ in 0..50 {
            let _ = p.process_line(None, 0.1);
        }
        // Line returns: counter should reset, not roll over to ground.
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        for _ in 0..101 {
            assert!(p.process_line(None, 0.1).is_some());
        }
    }

    #[test]
    fn opposite_angle_transitions_to_over_center() {
        let mut p = LineProcessor::new();
        // First sighting at angle 0.
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        // Now angle ~180° (opposite side) — should flip to OverCenter.
        let opp = p.process_line(Some(Vector2::new(-1.0, 0.0)), 0.1).unwrap();
        assert!(approx(opp.x, 1.0));
        assert!(matches_over_center(&p.state));
    }

    #[test]
    fn over_center_holds_original_vector_when_line_present() {
        let mut p = LineProcessor::new();
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        p.process_line(Some(Vector2::new(-1.0, 0.0)), 0.1);
        // Subsequent readings should return the negated original (1,0)→-1,0
        let out = p.process_line(Some(Vector2::new(0.5, 0.5)), 0.1).unwrap();
        assert!(approx(out.x, -1.0) && approx(out.y, 0.0));
        assert!(matches_over_center(&p.state));
    }

    #[test]
    fn over_center_returns_to_ground_after_timeout() {
        let mut p = LineProcessor::new();
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        p.process_line(Some(Vector2::new(-1.0, 0.0)), 0.1);
        for _ in 0..102 {
            let _ = p.process_line(None, 0.1);
        }
        assert!(matches_on_ground(&p.state));
    }

    #[test]
    fn nan_input_treated_as_no_line_from_ground() {
        let mut p = LineProcessor::new();
        let nan_vec = Some(Vector2::new(f32::NAN, 0.0));
        assert!(p.process_line(nan_vec, 0.1).is_none());
        assert!(matches_on_ground(&p.state));
    }

    #[test]
    fn nan_input_does_not_corrupt_latched_state() {
        let mut p = LineProcessor::new();
        // Latch onto a real reading first.
        p.process_line(Some(Vector2::new(1.0, 0.0)), 0.1);
        // A subsequent NaN reading must be filtered to None (treated as
        // dropout) — it must not overwrite the first_x/first_y fields
        // with NaN, which would propagate forever.
        let _ = p.process_line(Some(Vector2::new(f32::NAN, f32::NAN)), 0.1);
        let out = p.process_line(None, 0.1).unwrap();
        assert!(out.x.is_finite() && out.y.is_finite());
        assert!((out.x - -1.0).abs() < 1e-5);
    }

    #[test]
    fn is_angle_in_range_simple_arc() {
        // Arc from -45° to +45°
        let a = -45.0_f32.to_radians();
        let b = 45.0_f32.to_radians();
        assert!(is_angle_in_range(0.0, a, b));
        assert!(!is_angle_in_range(90.0_f32.to_radians(), a, b));
        assert!(!is_angle_in_range(-90.0_f32.to_radians(), a, b));
    }

    #[test]
    fn is_angle_in_range_wraps_around_pi() {
        // Arc from 170° to -170° (the short arc passing through 180°/-180°)
        let a = 170.0_f32.to_radians();
        let b = -170.0_f32.to_radians();
        assert!(is_angle_in_range(180.0_f32.to_radians(), a, b));
        assert!(is_angle_in_range(-180.0_f32.to_radians(), a, b));
        assert!(!is_angle_in_range(0.0, a, b));
    }
}
