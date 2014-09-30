use math::Scalar;
use volumetric::cylinder_volume;
use volumetric::ball_volume;

/// Computes the volume of a capsule.
pub fn capsule_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    cylinder_volume(half_height, radius) + ball_volume(radius)
}
