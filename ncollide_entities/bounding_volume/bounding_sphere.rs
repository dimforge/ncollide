//! Bounding sphere.

use na::{Translation, Norm, Transform, Translate};
use na;
use math::{Point, Vector};
use bounding_volume::{BoundingVolume, HasBoundingVolume};

// Seems useful to help type inference. See issue #84.
/// Computes the bounding sphere of a shape `g` transformed by `m`.
///
/// Same as `g.bounding_sphere(m)`.
pub fn bounding_sphere<P, M, G: ?Sized>(g: &G, m: &M) -> BoundingSphere<P>
    where P: Point,
          G: HasBoundingVolume<M, BoundingSphere<P>> {
    g.bounding_volume(m)
}

/// A Bounding Sphere.
#[derive(Debug, PartialEq, Clone, RustcEncodable, RustcDecodable)]
pub struct BoundingSphere<P: Point> {
    center: P,
    radius: <P::Vect as Vector>::Scalar
}

impl<P> BoundingSphere<P>
    where P: Point {
    /// Creates a new bounding sphere.
    pub fn new(center: P, radius: <P::Vect as Vector>::Scalar) -> BoundingSphere<P> {
        BoundingSphere {
            center: center,
            radius: radius
        }
    }

    /// The bounding sphere center.
    #[inline]
    pub fn center(&self) -> &P {
        &self.center
    }

    /// The bounding sphere radius.
    #[inline]
    pub fn radius(&self) -> <P::Vect as Vector>::Scalar {
        self.radius.clone()
    }

    /// Transforms this bounding sphere by `m`.
    #[inline]
    pub fn transform_by<M: Transform<P>>(&self, m: &M) -> BoundingSphere<P> {
        BoundingSphere::new(na::transform(m, &self.center), self.radius)
    }
}

impl<P> BoundingVolume<<P::Vect as Vector>::Scalar> for BoundingSphere<P>
    where P: Point,
          P::Vect: Translate<P> {
    #[inline]
    fn intersects(&self, other: &BoundingSphere<P>) -> bool {
        // FIXME: refactor that with the code from narrow_phase::ball_ball::collide(...) ?
        let delta_pos  = other.center - self.center;
        let distance_squared     = na::norm_squared(&delta_pos);
        let sum_radius = self.radius + other.radius;

        distance_squared <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere<P>) -> bool {
        let delta_pos  = other.center - self.center;
        let distance       = na::norm(&delta_pos);

        distance + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere<P>) {
        let mut dir = *other.center() - *self.center();
        let norm    = dir.normalize_mut();

        if na::is_zero(&norm) {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        }
        else {
            let s_center_dir = na::dot(self.center.as_vector(), &dir);
            let o_center_dir = na::dot(other.center.as_vector(), &dir);

            let right;
            let left;

            if s_center_dir + self.radius > o_center_dir + other.radius {
                right = self.center + dir * self.radius;
            }
            else {
                right = other.center + dir * other.radius;
            }

            if -s_center_dir + self.radius > -o_center_dir + other.radius {
                left = (dir * self.radius).inverse_translate(&self.center);
            }
            else {
                left = (dir * other.radius).inverse_translate(&other.center);
            }

            self.center = na::center(&left, &right);
            self.radius = na::distance(&right, &self.center);
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere<P>) -> BoundingSphere<P> {
        let mut res = self.clone();

        res.merge(other);

        res
    }

    #[inline]
    fn loosen(&mut self, amount: <P::Vect as Vector>::Scalar) {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: <P::Vect as Vector>::Scalar) -> BoundingSphere<P> {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        BoundingSphere::new(self.center.clone(), self.radius + amount)
    }

    #[inline]
    fn tighten(&mut self, amount: <P::Vect as Vector>::Scalar) {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        self.radius = self.radius - amount
    }

    #[inline]
    fn tightened(&self, amount: <P::Vect as Vector>::Scalar) -> BoundingSphere<P> {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        BoundingSphere::new(self.center.clone(), self.radius - amount)
    }
}

impl<P> Translation<P::Vect> for BoundingSphere<P>
    where P: Point {
    #[inline]
    fn translation(&self) -> P::Vect {
        self.center.as_vector().clone()
    }

    #[inline]
    fn inverse_translation(&self) -> P::Vect {
        -self.translation()
    }

    #[inline]
    fn append_translation_mut(&mut self, dv: &P::Vect) {
        self.center = self.center + *dv
    }

    #[inline]
    fn append_translation(&self, dv: &P::Vect) -> BoundingSphere<P> {
        BoundingSphere::new(self.center + *dv, self.radius)
    }

    #[inline]
    fn prepend_translation_mut(&mut self, dv: &P::Vect) {
        self.append_translation_mut(dv)
    }

    #[inline]
    fn prepend_translation(&self, dv: &P::Vect) -> BoundingSphere<P> {
        self.append_translation(dv)
    }

    #[inline]
    fn set_translation(&mut self, v: P::Vect) {
        self.center = na::origin::<P>() + v
    }
}
