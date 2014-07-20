use std::num::Zero;
use nalgebra::na::{Translation, Norm};
use nalgebra::na;
use math::{Scalar, Vect, Matrix};
use bounding_volume::{BoundingVolume, LooseBoundingVolume};
use bounding_volume;

/// Trait implemented by objects having a bounding sphere.
pub trait HasBoundingSphere {
    /// The object bounding sphere.
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere;
}

/// A Bounding Sphere.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct BoundingSphere {
    center: Vect,
    radius: Scalar
}

impl BoundingSphere {
    /// Creates a new bounding sphere.
    pub fn new(center: Vect, radius: Scalar) -> BoundingSphere {
        BoundingSphere {
            center: center,
            radius: radius
        }
    }

    /// The bounding sphere center.
    #[inline]
    pub fn center<'a>(&'a self) -> &'a Vect {
        &self.center
    }

    /// The bounding sphere radius.
    #[inline]
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }
}

impl BoundingVolume for BoundingSphere {
    #[inline]
    fn intersects(&self, other: &BoundingSphere) -> bool {

        // FIXME: refactor that with the code from narrow::ball_ball::collide(...) ?
        let delta_pos  = other.center - self.center;
        let sqdist     = na::sqnorm(&delta_pos);
        let sum_radius = self.radius + other.radius;

        sqdist <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere) -> bool {
        let delta_pos  = other.center - self.center;
        let dist       = na::norm(&delta_pos);

        dist + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere) {
        let _0_5: Scalar = na::cast(0.5f64);
        let a            = self.center;
        let b            = other.center;
        let new_center   = (a + b) * _0_5;

        let mut dir = b - a;
        let norm    = dir.normalize();

        if norm.is_zero() {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        }
        else {
            let pts = [
                self.center  + dir * self.radius,
                self.center  - dir * self.radius,
                other.center + dir * other.radius,
                other.center - dir * other.radius
            ];

            let (center, radius) = bounding_volume::point_cloud_bounding_sphere_with_center(pts, new_center);

            self.center = center;
            self.radius = radius ;
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere) -> BoundingSphere {
        let mut res = self.clone();

        res.merge(other);

        res
    }
}

impl LooseBoundingVolume for BoundingSphere {
    #[inline]
    fn loosen(&mut self, amount: Scalar) {
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: Scalar) -> BoundingSphere {
        BoundingSphere::new(self.center.clone(), self.radius + amount)
    }
}

impl Translation<Vect> for BoundingSphere {
    #[inline]
    fn translation(&self) -> Vect {
        self.center.clone()
    }

    #[inline]
    fn inv_translation(&self) -> Vect {
        -self.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &Vect) {
        self.center = self.center + *dv
    }

    #[inline]
    fn append_translation_cpy(bs: &BoundingSphere, dv: &Vect) -> BoundingSphere {
        BoundingSphere::new(bs.center + *dv, bs.radius)
    }

    #[inline]
    fn prepend_translation(&mut self, dv: &Vect) {
        self.append_translation(dv)
    }

    #[inline]
    fn prepend_translation_cpy(bs: &BoundingSphere, dv: &Vect) -> BoundingSphere {
        Translation::append_translation_cpy(bs, dv)
    }

    #[inline]
    fn set_translation(&mut self, v: Vect) {
        self.center = v
    }
}
