//! Pure sensor projection math, extracted from `nv1-hub/src/sensors.rs`.
//!
//! Two routines feed the hub control loop:
//!
//! - [`calculate_line_vec_with_threshold`]: thresholded line-sensor vector
//!   sum. Returns `None` when no sensor crosses the threshold so the caller
//!   can keep using the previous estimate.
//! - [`calculate_adc_vec`]: generic IR ball-sensor vector sum that also
//!   reports the peak channel for haveball detection.
//!
//! [`generate_adc_vec`] is the matching helper that pre-computes the
//! per-channel sin/cos lookup tables consumed by the two routines above.

use num_traits::{AsPrimitive, Num};

use crate::types::Vector2;

/// Populates `sin` and `cos` with `sin/cos(i * one_angle + offset) * mul`
/// for each channel index. The two slices must be the same length.
pub fn generate_adc_vec<T>(sin: &mut [T], cos: &mut [T], offset: f32, one_angle: f32, mul: f32)
where
    f32: AsPrimitive<T>,
    T: Num + Copy + 'static,
{
    debug_assert_eq!(sin.len(), cos.len());
    for i in 0..sin.len() {
        sin[i] = (libm::sinf(i as f32 * one_angle + offset) * mul).as_();
        cos[i] = (libm::cosf(i as f32 * one_angle + offset) * mul).as_();
    }
}

/// Vector-sum the channels weighted by the (cos, sin) lookup tables.
///
/// Returns the unit-length direction `(x, y)` and the peak channel value.
/// All three input slices must be the same length; otherwise iteration
/// stops at the shorter one.
pub fn calculate_adc_vec<T>(adc: &[T], adc_sin: &[T], adc_cos: &[T], _mul: T) -> (f32, f32, T)
where
    T: Num + Copy + 'static + AsPrimitive<f32> + PartialOrd,
    f32: AsPrimitive<T>,
{
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;
    let mut max_adc: T = T::zero();

    for i in 0..adc.len() {
        if adc[i] > max_adc {
            max_adc = adc[i];
        }
        sum_x += (adc_cos[i] * adc[i]).as_();
        sum_y += (adc_sin[i] * adc[i]).as_();
    }

    let n = adc.len() as f32;
    let mean_x = sum_x / n;
    let mean_y = sum_y / n;
    let norm = libm::sqrtf(mean_x * mean_x + mean_y * mean_y);

    (mean_x / norm, mean_y / norm, max_adc)
}

/// Build a unit-length line vector from sensors over `threshold`.
///
/// Returns `None` if no channel exceeds the threshold so the caller can
/// distinguish "robot is over the line" from "robot is on the field".
pub fn calculate_line_vec_with_threshold(
    adc: &[f32],
    adc_sin: &[f32],
    adc_cos: &[f32],
    threshold: f32,
) -> Option<Vector2> {
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;
    let mut over_threshold = false;

    for i in 0..adc.len() {
        if adc[i] > threshold {
            sum_x += adc_cos[i];
            sum_y += adc_sin[i];
            over_threshold = true;
        }
    }

    if !over_threshold {
        return None;
    }

    let norm = libm::sqrtf(sum_x * sum_x + sum_y * sum_y);
    // Diametrically opposed sensors can cancel to (near) zero. We must
    // not divide by it: a zero norm yields NaN, and a tiny norm produced
    // by f32 sin/cos rounding noise yields a meaningless direction.
    // 1e-3 is well below the unit contribution of any single real sensor
    // and well above floating-point residue from a 32-channel LUT.
    if norm < 1e-3 {
        return None;
    }
    Some(Vector2::new(sum_x / norm, sum_y / norm))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn generate_adc_vec_radial_distribution() {
        let mut sin = [0.0f32; 4];
        let mut cos = [0.0f32; 4];
        generate_adc_vec(&mut sin, &mut cos, 0.0, FRAC_PI_2, 1.0);
        // i=0 -> (1, 0), i=1 -> (0, 1), i=2 -> (-1, 0), i=3 -> (0, -1)
        assert!(approx(cos[0], 1.0) && approx(sin[0], 0.0));
        assert!(approx(cos[1], 0.0) && approx(sin[1], 1.0));
        assert!(approx(cos[2], -1.0) && approx(sin[2], 0.0));
        assert!(approx(cos[3], 0.0) && approx(sin[3], -1.0));
    }

    #[test]
    fn line_vec_returns_none_when_no_sensor_active() {
        let adc = [0.05f32; 4];
        let mut sin = [0.0f32; 4];
        let mut cos = [0.0f32; 4];
        generate_adc_vec(&mut sin, &mut cos, 0.0, FRAC_PI_2, 1.0);
        assert!(calculate_line_vec_with_threshold(&adc, &sin, &cos, 0.1).is_none());
    }

    #[test]
    fn line_vec_returns_none_when_symmetric_sensors_cancel() {
        // Diametrically opposed sensors fire equally → vector sum is zero.
        // Must return None so the controller doesn't get NaN.
        let mut sin = [0.0f32; 4];
        let mut cos = [0.0f32; 4];
        generate_adc_vec(&mut sin, &mut cos, 0.0, FRAC_PI_2, 1.0);
        let adc = [0.5, 0.0, 0.5, 0.0];
        assert!(calculate_line_vec_with_threshold(&adc, &sin, &cos, 0.1).is_none());
    }

    #[test]
    fn line_vec_unit_length_when_one_sensor_trips() {
        let mut sin = [0.0f32; 4];
        let mut cos = [0.0f32; 4];
        generate_adc_vec(&mut sin, &mut cos, 0.0, FRAC_PI_2, 1.0);
        let mut adc = [0.0f32; 4];
        adc[0] = 0.5;
        let v = calculate_line_vec_with_threshold(&adc, &sin, &cos, 0.1).unwrap();
        assert!(approx(v.magnitude(), 1.0));
        assert!(approx(v.x, 1.0));
        assert!(approx(v.y, 0.0));
    }

    #[test]
    fn line_vec_threshold_filters_low_signal() {
        let mut sin = [0.0f32; 4];
        let mut cos = [0.0f32; 4];
        generate_adc_vec(&mut sin, &mut cos, 0.0, FRAC_PI_2, 1.0);
        let adc = [0.05, 0.5, 0.05, 0.05];
        let v = calculate_line_vec_with_threshold(&adc, &sin, &cos, 0.1).unwrap();
        assert!(approx(v.x, 0.0));
        assert!(approx(v.y, 1.0));
    }

    #[test]
    fn adc_vec_reports_peak_channel() {
        let adc: [u16; 4] = [10, 50, 30, 20];
        let mut sin = [0u16; 4];
        let mut cos = [0u16; 4];
        // Use a u16-typed LUT to keep T monomorphic across all 3 inputs.
        for (i, (s, c)) in sin.iter_mut().zip(cos.iter_mut()).enumerate() {
            let f = libm::sinf(i as f32 * FRAC_PI_2) * 1000.0;
            *s = f as u16;
            *c = (libm::cosf(i as f32 * FRAC_PI_2) * 1000.0) as u16;
        }
        let (_x, _y, peak) = calculate_adc_vec(&adc, &sin, &cos, 1u16);
        assert_eq!(peak, 50);
    }
}
