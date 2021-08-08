//! Bounding sphere.

use crate::bounding_volume::{BoundingVolume, HasBoundingVolume};
use crate::math::{Isometry, Point};
use na::{self, RealField};

// Seems useful to help type inference. See issue #84.
/// Computes the bounding sphere of a shape `g` transformed by `m`.
///
/// Same as `g.bounding_sphere(m)`.
pub fn bounding_sphere<N, G: ?Sized>(g: &G, m: &Isometry<N>) -> BoundingSphere<N>
where
    N: RealField + Copy,
    G: HasBoundingVolume<N, BoundingSphere<N>>,
{
    g.bounding_volume(m)
}

// Seems useful to help type inference. See issue #84.
/// Computes the bounding sphere of a shape `g`.
///
/// Same as `g.local_bounding_sphere(m)`.
pub fn local_bounding_sphere<N, G: ?Sized>(g: &G) -> BoundingSphere<N>
where
    N: RealField + Copy,
    G: HasBoundingVolume<N, BoundingSphere<N>>,
{
    g.local_bounding_volume()
}

/// A Bounding Sphere.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct BoundingSphere<N: RealField + Copy> {
    center: Point<N>,
    radius: N,
}

impl<N: RealField + Copy> BoundingSphere<N> {
    /// Creates a new bounding sphere.
    pub fn new(center: Point<N>, radius: N) -> BoundingSphere<N> {
        BoundingSphere { center, radius }
    }

    /// The bounding sphere center.
    #[inline]
    pub fn center(&self) -> &Point<N> {
        &self.center
    }

    /// The bounding sphere radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }

    /// Transforms this bounding sphere by `m`.
    #[inline]
    pub fn transform_by(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        BoundingSphere::new(m * self.center, self.radius)
    }
}

impl<N: RealField + Copy> BoundingVolume<N> for BoundingSphere<N> {
    #[inline]
    fn center(&self) -> Point<N> {
        *self.center()
    }

    #[inline]
    fn intersects(&self, other: &BoundingSphere<N>) -> bool {
        // FIXME: refactor that with the code from narrow_phase::ball_ball::collide(...) ?
        let delta_pos = other.center - self.center;
        let distance_squared = delta_pos.norm_squared();
        let sum_radius = self.radius + other.radius;

        distance_squared <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere<N>) -> bool {
        let delta_pos = other.center - self.center;
        let distance = delta_pos.norm();

        distance + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere<N>) {
        let mut dir = *other.center() - *self.center();
        let norm = dir.normalize_mut();

        if norm.is_zero() {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        } else {
            let s_center_dir = self.center.coords.dot(&dir);
            let o_center_dir = other.center.coords.dot(&dir);

            let right;
            let left;

            if s_center_dir + self.radius > o_center_dir + other.radius {
                right = self.center + dir * self.radius;
            } else {
                right = other.center + dir * other.radius;
            }

            if -s_center_dir + self.radius > -o_center_dir + other.radius {
                left = self.center - dir * self.radius;
            } else {
                left = other.center - dir * other.radius;
            }

            self.center = na::center(&left, &right);
            self.radius = na::distance(&right, &self.center);
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere<N>) -> BoundingSphere<N> {
        let mut res = self.clone();

        res.merge(other);

        res
    }

    #[inline]
    fn loosen(&mut self, amount: N) {
        assert!(
            amount >= na::zero(),
            "The loosening margin must be positive."
        );
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: N) -> BoundingSphere<N> {
        assert!(
            amount >= na::zero(),
            "The loosening margin must be positive."
        );
        BoundingSphere::new(self.center, self.radius + amount)
    }

    #[inline]
    fn tighten(&mut self, amount: N) {
        assert!(
            amount >= na::zero(),
            "The tightening margin must be positive."
        );
        assert!(amount <= self.radius, "The tightening margin is to large.");
        self.radius = self.radius - amount
    }

    #[inline]
    fn tightened(&self, amount: N) -> BoundingSphere<N> {
        assert!(
            amount >= na::zero(),
            "The tightening margin must be positive."
        );
        assert!(amount <= self.radius, "The tightening margin is to large.");
        BoundingSphere::new(self.center, self.radius - amount)
    }
}
