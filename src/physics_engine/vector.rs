use std::{fmt::Display, ops};

pub type Float = f64;

/// 2-dimensional vector
#[derive(Default, Debug, PartialEq, Copy, Clone)]
pub struct Vec2 {
    pub x: Float,
    pub y: Float,
}

impl Vec2 {
    pub fn new(x: Float, y: Float) -> Self {
        Self { x, y }
    }
    pub fn zero() -> Self {
        Vec2::new(0.0, 0.0)
    }
}

impl Vec2 {
    /// Dot product of 2 vectors
    pub fn dot(&self, other: &Vec2) -> Float {
        self.x * other.x + self.y * other.y
    }
    /// Length/magnitude of vector, squared
    pub fn length_squared(&self) -> Float {
        self.dot(self)
    }

    /// Length/magnitude of vector
    pub fn length(&self) -> Float {
        self.length_squared().sqrt()
    }

    #[allow(dead_code)]
    pub fn normalised(&self) -> Self {
        *self * self.length().recip()
    }
}

pub fn cross_vec_vec(a: Vec2, b: Vec2) -> Float {
    // (ax, ay, 0) x (bx, by, 0) = (0,0, z)
    a.x * b.y - a.y * b.x
}
pub fn cross_scalar_vec(s: Float, v: Vec2) -> Vec2 {
    // (0,0,s) x (vx, vy, 0)
    Vec2::new(-s * v.y, s * v.x)
}

// Implementations of basic operations

impl ops::AddAssign<Vec2> for Vec2 {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}
impl ops::SubAssign<Vec2> for Vec2 {
    fn sub_assign(&mut self, rhs: Vec2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}
impl ops::Add<Vec2> for Vec2 {
    type Output = Vec2;
    fn add(mut self, rhs: Vec2) -> Self::Output {
        self += rhs;
        self
    }
}
impl ops::Sub<Vec2> for Vec2 {
    type Output = Vec2;
    fn sub(mut self, rhs: Vec2) -> Self::Output {
        self -= rhs;
        self
    }
}
impl ops::MulAssign<Float> for Vec2 {
    fn mul_assign(&mut self, rhs: Float) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl ops::Mul<Float> for Vec2 {
    type Output = Vec2;
    fn mul(mut self, rhs: Float) -> Self::Output {
        self *= rhs;
        self
    }
}

impl ops::DivAssign<Float> for Vec2 {
    fn div_assign(&mut self, rhs: Float) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl ops::Div<Float> for Vec2 {
    type Output = Vec2;
    fn div(mut self, rhs: Float) -> Self::Output {
        self /= rhs;
        self
    }
}
impl Display for Vec2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn a() {
        let v = Vec2::new(5.0, 4.0);
        let a = Vec2::new(1.0, 2.0);
        assert_eq!(v + a, Vec2::new(3.0, 3.0) * 2.0)
    }
}
