use math::Scalar;

/// A torus.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Torus {
    major_radius: Scalar,
    minor_radius: Scalar
}

impl Torus {
    /// Creates a new torus with the given radiuses.
    #[inline]
    pub fn new(major_radius: Scalar, minor_radius: Scalar) -> Torus {
        Torus {
            major_radius: major_radius,
            minor_radius: minor_radius
        }
    }
}

impl Torus {
    /// The torus minor radius.
    #[inline]
    pub fn minor_radius(&self) -> Scalar {
        self.minor_radius.clone()
    }

    /// The torus major radius.
    #[inline]
    pub fn major_radius(&self) -> Scalar {
        self.major_radius.clone()
    }
}
