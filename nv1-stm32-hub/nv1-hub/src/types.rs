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
