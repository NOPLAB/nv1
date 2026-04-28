#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

#[allow(dead_code)]
impl Vector2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y)
    }

    pub fn angle(&self) -> f32 {
        libm::atan2f(self.y, self.x)
    }

    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
            }
        } else {
            Self::zero()
        }
    }

    pub fn dot(&self, other: &Vector2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn distance_to(&self, other: &Vector2) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        libm::sqrtf(dx * dx + dy * dy)
    }
}

impl core::ops::Add for Vector2 {
    type Output = Vector2;

    fn add(self, other: Vector2) -> Vector2 {
        Vector2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl core::ops::Sub for Vector2 {
    type Output = Vector2;

    fn sub(self, other: Vector2) -> Vector2 {
        Vector2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl core::ops::Mul<f32> for Vector2 {
    type Output = Vector2;

    fn mul(self, scalar: f32) -> Vector2 {
        Vector2 {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl core::ops::Div<f32> for Vector2 {
    type Output = Vector2;

    fn div(self, scalar: f32) -> Vector2 {
        Vector2 {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}
impl core::ops::Neg for Vector2 {
    type Output = Vector2;

    fn neg(self) -> Vector2 {
        Vector2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn new_and_zero() {
        let v = Vector2::new(3.0, 4.0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
        assert_eq!(Vector2::zero(), Vector2::new(0.0, 0.0));
    }

    #[test]
    fn magnitude_3_4_5() {
        assert!(approx(Vector2::new(3.0, 4.0).magnitude(), 5.0));
        assert!(approx(Vector2::new(-3.0, -4.0).magnitude(), 5.0));
        assert!(approx(Vector2::zero().magnitude(), 0.0));
    }

    #[test]
    fn angle_axes() {
        assert!(approx(Vector2::new(1.0, 0.0).angle(), 0.0));
        assert!(approx(Vector2::new(0.0, 1.0).angle(), FRAC_PI_2));
        assert!(approx(Vector2::new(-1.0, 0.0).angle().abs(), PI));
        assert!(approx(Vector2::new(1.0, 1.0).angle(), FRAC_PI_4));
    }

    #[test]
    fn normalized_unit_length() {
        let v = Vector2::new(3.0, 4.0).normalized();
        assert!(approx(v.magnitude(), 1.0));
        assert!(approx(v.x, 0.6));
        assert!(approx(v.y, 0.8));
    }

    #[test]
    fn normalized_zero_returns_zero() {
        // Zero vector has no defined direction; normalized() must return zero,
        // not NaN, so downstream math (angle, dot product) stays finite.
        assert_eq!(Vector2::zero().normalized(), Vector2::zero());
    }

    #[test]
    fn dot_orthogonal_is_zero() {
        let a = Vector2::new(1.0, 0.0);
        let b = Vector2::new(0.0, 1.0);
        assert!(approx(a.dot(&b), 0.0));
    }

    #[test]
    fn dot_parallel_is_product_of_magnitudes() {
        let a = Vector2::new(2.0, 0.0);
        let b = Vector2::new(3.0, 0.0);
        assert!(approx(a.dot(&b), 6.0));
    }

    #[test]
    fn distance_to_pythagorean() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(3.0, 4.0);
        assert!(approx(a.distance_to(&b), 5.0));
        assert!(approx(b.distance_to(&a), 5.0));
    }

    #[test]
    fn add_sub_neg() {
        let a = Vector2::new(1.0, 2.0);
        let b = Vector2::new(3.0, 5.0);
        assert_eq!(a + b, Vector2::new(4.0, 7.0));
        assert_eq!(b - a, Vector2::new(2.0, 3.0));
        assert_eq!(-a, Vector2::new(-1.0, -2.0));
    }

    #[test]
    fn scalar_mul_div() {
        let v = Vector2::new(2.0, -3.0);
        assert_eq!(v * 4.0, Vector2::new(8.0, -12.0));
        assert_eq!(v / 2.0, Vector2::new(1.0, -1.5));
    }
}
