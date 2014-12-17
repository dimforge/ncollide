//! Bounding sphere.

use na::{Translation, Norm, Transform, Translate};
use na;
use bounding_volume::BoundingVolume;
use math::{Scalar, Point, Vect};

/// Trait implemented by objects having a bounding sphere.
pub trait HasBoundingSphere<N, P, M> {
    /// The object bounding sphere.
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P>;
}

/// A Bounding Sphere.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct BoundingSphere<N, P> {
    center: P,
    radius: N
}

impl<N, P> BoundingSphere<N, P>
    where N: Scalar {
    /// Creates a new bounding sphere.
    pub fn new(center: P, radius: N) -> BoundingSphere<N, P> {
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
    pub fn radius(&self) -> N {
        self.radius.clone()
    }

    /// Transforms this bounding sphere by `m`.
    #[inline]
    pub fn transform_by<M: Transform<P>>(&self, m: &M) -> BoundingSphere<N, P> {
        BoundingSphere::new(na::transform(m, &self.center), self.radius)
    }
}

impl<N, P, V> BoundingVolume<N> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P> {
    #[inline]
    fn intersects(&self, other: &BoundingSphere<N, P>) -> bool {
        // FIXME: refactor that with the code from narrow_phase::ball_ball::collide(...) ?
        let delta_pos  = other.center - self.center;
        let sqdist     = na::sqnorm(&delta_pos);
        let sum_radius = self.radius + other.radius;

        sqdist <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere<N, P>) -> bool {
        let delta_pos  = other.center - self.center;
        let dist       = na::norm(&delta_pos);

        dist + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere<N, P>) {
        let mut dir = *other.center() - *self.center();
        let norm    = dir.normalize();

        if na::is_zero(&norm) {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        }
        else {
            let s_center_dir = na::dot(self.center.as_vec(), &dir);
            let o_center_dir = na::dot(other.center.as_vec(), &dir);

            let right;
            let left;

            if s_center_dir + self.radius > o_center_dir + other.radius {
                right = self.center + dir * self.radius;
            }
            else {
                right = other.center + dir * other.radius;
            }

            if -s_center_dir + self.radius > -o_center_dir + other.radius {
                left = (dir * self.radius).inv_translate(&self.center);
            }
            else {
                left = (dir * other.radius).inv_translate(&other.center);
            }

            self.center = na::center(&left, &right);
            self.radius = na::dist(&right, &self.center);
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere<N, P>) -> BoundingSphere<N, P> {
        let mut res = self.clone();

        res.merge(other);

        res
    }

    #[inline]
    fn loosen(&mut self, amount: N) {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: N) -> BoundingSphere<N, P> {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        BoundingSphere::new(self.center.clone(), self.radius + amount)
    }

    #[inline]
    fn tighten(&mut self, amount: N) {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        self.radius = self.radius - amount
    }

    #[inline]
    fn tightened(&self, amount: N) -> BoundingSphere<N, P> {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        BoundingSphere::new(self.center.clone(), self.radius - amount)
    }
}

impl<N, P, V> Translation<V> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn translation(&self) -> V {
        self.center.as_vec().clone()
    }

    #[inline]
    fn inv_translation(&self) -> V {
        -self.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &V) {
        self.center = self.center + *dv
    }

    #[inline]
    fn append_translation_cpy(&self, dv: &V) -> BoundingSphere<N, P> {
        BoundingSphere::new(self.center + *dv, self.radius)
    }

    #[inline]
    fn prepend_translation(&mut self, dv: &V) {
        self.append_translation(dv)
    }

    #[inline]
    fn prepend_translation_cpy(&self, dv: &V) -> BoundingSphere<N, P> {
        self.append_translation_cpy(dv)
    }

    #[inline]
    fn set_translation(&mut self, v: V) {
        self.center = na::orig::<P>() + v
    }
}
