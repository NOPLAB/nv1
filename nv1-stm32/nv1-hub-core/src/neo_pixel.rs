//! Pure-logic helpers for the WS2812B "NeoPixel" PWM-DMA driver in
//! `nv1-hub/src/neo_pixel.rs`. The hub-side type owns the timer and DMA
//! channel; everything that just shuffles bits or scales colors lives
//! here so it can be tested on the host.
//!
//! The wire format is 24 bits per LED in `G`, `R`, `B` order, MSB
//! first. Each bit is transmitted as a fixed-period PWM pulse whose
//! high time encodes the bit value (~260 ns for `0`, ~640 ns for `1`).
//! [`colors_to_duty_buffer`] flattens an array of [`rgb::RGB8`] into the
//! per-bit duty values that get DMA'd into the timer's CCR register.

use rgb::RGB8;

/// Bits transmitted per LED (8 each for green, red, blue).
pub const BITS_PER_LED: usize = 24;

/// Snapshot of UI/state input consumed by the NeoPixel task.
#[derive(Debug, Clone, Copy)]
pub struct NeoPixelData {
    pub jetson_connecting: bool,
    pub pause: bool,
    pub ball_dir: f32,
}

/// PWM duty cycle values for a WS2812B `1` and `0` bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DutyValues {
    pub one_duty: u16,
    pub zero_duty: u16,
}

/// Compute [`DutyValues`] for a given PWM clock frequency and timer ARR.
///
/// The "1" bit is encoded as a 50% duty cycle pulse, and the "0" bit as
/// a 260 ns pulse — the WS2812B's specified short-high pulse width.
pub fn compute_duties(pwm_hz: u32, max_duty: u16) -> DutyValues {
    let one_duty: u16 = max_duty / 2;
    let zero_duty: u16 = (max_duty as f32 * (260.0e-6 * (pwm_hz / 1000) as f32)) as u16;
    DutyValues {
        one_duty,
        zero_duty,
    }
}

/// Encode `colors` into `duty_buffer` in WS2812B GRB MSB-first order.
///
/// `duty_buffer` must be sized exactly `colors.len() * BITS_PER_LED` —
/// the function asserts this in debug builds and silently truncates
/// otherwise.
pub fn colors_to_duty_buffer(colors: &[RGB8], duties: DutyValues, duty_buffer: &mut [u16]) {
    debug_assert_eq!(duty_buffer.len(), colors.len() * BITS_PER_LED);

    let pulse_for = |bit_set: bool| -> u16 {
        if bit_set {
            duties.one_duty
        } else {
            duties.zero_duty
        }
    };

    for (n, color) in colors.iter().enumerate() {
        let base = n * BITS_PER_LED;
        let (mut g, mut r, mut b) = (color.g, color.r, color.b);
        for i in 0..8 {
            duty_buffer[base + i] = pulse_for(g & 0b1000_0000 != 0);
            duty_buffer[base + i + 8] = pulse_for(r & 0b1000_0000 != 0);
            duty_buffer[base + i + 16] = pulse_for(b & 0b1000_0000 != 0);
            g <<= 1;
            r <<= 1;
            b <<= 1;
        }
    }
}

/// Apply the current per-channel brightness scaling factor in place.
///
/// `brightness` is clamped to the range `0..=45`. The scaling factor
/// applied to each channel is `tan(90° - brightness)`, which maps:
/// brightness=45 → ×1 (identity), brightness=0 → ×∞ (saturates to 255).
/// The mapping is lifted from the original firmware verbatim.
pub fn apply_brightness(colors: &mut [RGB8], brightness: u8) {
    let brightness = brightness.min(45);
    let angle = (90.0 - brightness as f32) * core::f32::consts::PI / 180.0;
    let scale = libm::tanf(angle);
    for c in colors.iter_mut() {
        c.r = (c.r as f32 * scale) as u8;
        c.g = (c.g as f32 * scale) as u8;
        c.b = (c.b as f32 * scale) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duty_values_match_50_percent_for_one_bit() {
        let d = compute_duties(800_000, 100);
        assert_eq!(d.one_duty, 50);
    }

    #[test]
    fn duty_values_zero_bit_smaller_than_one_bit() {
        // A "0" pulse must always be shorter than a "1" pulse.
        let d = compute_duties(800_000, 100);
        assert!(d.zero_duty < d.one_duty);
    }

    #[test]
    fn duty_buffer_zero_color_is_all_zero_duty() {
        let colors = [RGB8::new(0, 0, 0)];
        let duties = DutyValues {
            one_duty: 50,
            zero_duty: 13,
        };
        let mut buf = [0u16; BITS_PER_LED];
        colors_to_duty_buffer(&colors, duties, &mut buf);
        assert!(buf.iter().all(|&d| d == duties.zero_duty));
    }

    #[test]
    fn duty_buffer_full_color_is_all_one_duty() {
        let colors = [RGB8::new(0xFF, 0xFF, 0xFF)];
        let duties = DutyValues {
            one_duty: 50,
            zero_duty: 13,
        };
        let mut buf = [0u16; BITS_PER_LED];
        colors_to_duty_buffer(&colors, duties, &mut buf);
        assert!(buf.iter().all(|&d| d == duties.one_duty));
    }

    #[test]
    fn duty_buffer_grb_order_msb_first() {
        // Distinct bit patterns in each channel verify both ordering
        // and bit positioning.
        let colors = [RGB8::new(0b1000_0000, 0b0100_0000, 0b0010_0000)];
        // R = 0b1000_0000 (bit 7 set, others 0)
        // G = 0b0100_0000 (bit 6 set)
        // B = 0b0010_0000 (bit 5 set)
        let duties = DutyValues {
            one_duty: 50,
            zero_duty: 13,
        };
        let mut buf = [0u16; BITS_PER_LED];
        colors_to_duty_buffer(&colors, duties, &mut buf);

        // Bytes 0..8 = G, MSB first. G's set bit is index 1 (0b0100_0000
        // → second-most-significant position).
        for (i, &d) in buf[0..8].iter().enumerate() {
            let expected = if i == 1 { duties.one_duty } else { duties.zero_duty };
            assert_eq!(d, expected, "G bit {i} mismatch");
        }
        // Bytes 8..16 = R, MSB first. R's set bit is index 0.
        for (i, &d) in buf[8..16].iter().enumerate() {
            let expected = if i == 0 { duties.one_duty } else { duties.zero_duty };
            assert_eq!(d, expected, "R bit {i} mismatch");
        }
        // Bytes 16..24 = B, MSB first. B's set bit is index 2.
        for (i, &d) in buf[16..24].iter().enumerate() {
            let expected = if i == 2 { duties.one_duty } else { duties.zero_duty };
            assert_eq!(d, expected, "B bit {i} mismatch");
        }
    }

    #[test]
    fn duty_buffer_handles_multiple_leds() {
        let colors = [RGB8::new(0xFF, 0, 0), RGB8::new(0, 0xFF, 0)];
        let duties = DutyValues {
            one_duty: 50,
            zero_duty: 13,
        };
        let mut buf = [0u16; BITS_PER_LED * 2];
        colors_to_duty_buffer(&colors, duties, &mut buf);

        // LED 0: G=0, R=0xFF, B=0 → bytes 0..8 zero, 8..16 one, 16..24 zero
        assert!(buf[0..8].iter().all(|&d| d == duties.zero_duty));
        assert!(buf[8..16].iter().all(|&d| d == duties.one_duty));
        assert!(buf[16..24].iter().all(|&d| d == duties.zero_duty));

        // LED 1: G=0xFF, R=0, B=0 → bytes 24..32 one, 32..40 zero, 40..48 zero
        assert!(buf[24..32].iter().all(|&d| d == duties.one_duty));
        assert!(buf[32..40].iter().all(|&d| d == duties.zero_duty));
        assert!(buf[40..48].iter().all(|&d| d == duties.zero_duty));
    }

    #[test]
    fn brightness_45_is_identity() {
        let mut colors = [RGB8::new(100, 50, 200)];
        apply_brightness(&mut colors, 45);
        // tan(45°) = 1 exactly, so colors are preserved.
        assert_eq!(colors[0].r, 100);
        assert_eq!(colors[0].g, 50);
        assert_eq!(colors[0].b, 200);
    }

    #[test]
    fn brightness_clamped_above_45() {
        // Anything > 45 should clamp to 45 → identity, not stretched.
        let mut colors = [RGB8::new(100, 50, 200)];
        apply_brightness(&mut colors, 200);
        assert_eq!(colors[0].r, 100);
        assert_eq!(colors[0].g, 50);
        assert_eq!(colors[0].b, 200);
    }

    #[test]
    fn brightness_below_45_amplifies_colors() {
        // tan(90° - 22°) = tan(68°) ≈ 2.475. A channel value that fits
        // in u8 after scaling should be amplified.
        let mut colors = [RGB8::new(50, 0, 0)];
        apply_brightness(&mut colors, 22);
        assert!(
            colors[0].r > 50,
            "expected amplification, got {}",
            colors[0].r
        );
        // Saturating-cast contract: large values clip at 255, not wrap.
        let mut colors2 = [RGB8::new(200, 0, 0)];
        apply_brightness(&mut colors2, 22);
        assert_eq!(colors2[0].r, 255);
    }
}
