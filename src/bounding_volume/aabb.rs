//! Axis Aligned Bounding Box.

use bounding_volume::{BoundingVolume, HasBoundingVolume};
use math::{Isometry, Point, Vector};
use na::{self, Real};

// Seems useful to help type inference. See issue #84.
/// Computes the axis-aligned bounding box of a shape `g` transformed by `m`.
///
/// Same as `g.aabb(m)`.
pub fn aabb<N, G: ?Sized>(g: &G, m: &Isometry<N>) -> AABB<N>
    where
        N: Real,
        G: HasBoundingVolume<N, AABB<N>>,
{
    g.bounding_volume(m)
}

/// An Axis Aligned Bounding Box.
#[derive(Debug, PartialEq, Clone)]
pub struct AABB<N: Real> {
    mins: Point<N>,
    maxs: Point<N>,
}

impl<N: Real> AABB<N> {
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    pub fn new(mins: Point<N>, maxs: Point<N>) -> AABB<N> {
        // assert!(na::partial_le(&mins, &maxs));
        AABB {
            mins: mins,
            maxs: maxs,
        }
    }

    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    pub fn mins(&self) -> &Point<N> {
        &self.mins
    }

    /// Reference to the AABB point with the biggest components along each axis.
    #[inline]
    pub fn maxs(&self) -> &Point<N> {
        &self.maxs
    }

    /// The center of this AABB.
    #[inline]
    pub fn center(&self) -> Point<N> {
        na::center(&self.mins, &self.maxs)
    }

    /// The half extents of this AABB.
    #[inline]
    pub fn half_extents(&self) -> Vector<N> {
        let half: N = na::convert(0.5);
        (self.maxs - self.mins) * half
    }

    /// The extents of this AABB.
    #[inline]
    pub fn extents(&self) -> Vector<N> {
        self.maxs - self.mins
    }
}

impl<N: Real> BoundingVolume<N> for AABB<N> {
    #[inline]
    fn center(&self) -> Point<N> {
        self.center()
    }

    #[inline]
    fn intersects(&self, other: &AABB<N>) -> bool {
        na::partial_le(&self.mins, &other.maxs) && na::partial_ge(&self.maxs, &other.mins)
    }

    #[inline]
    fn contains(&self, other: &AABB<N>) -> bool {
        na::partial_le(&self.mins, &other.mins) && na::partial_ge(&self.maxs, &other.maxs)
    }

    #[inline]
    fn merge(&mut self, other: &AABB<N>) {
        self.mins = na::inf(&self.mins, &other.mins);
        self.maxs = na::sup(&self.maxs, &other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB<N>) -> AABB<N> {
        AABB {
            mins: na::inf(&self.mins, &other.mins),
            maxs: na::sup(&self.maxs, &other.maxs),
        }
    }

    #[inline]
    fn loosen(&mut self, amount: N) {
        assert!(
            amount >= na::zero(),
            "The loosening margin must be positive."
        );
        self.mins = self.mins + Vector::repeat(-amount);
        self.maxs = self.maxs + Vector::repeat(amount);
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB<N> {
        assert!(
            amount >= na::zero(),
            "The loosening margin must be positive."
        );
        AABB {
            mins: self.mins + Vector::repeat(-amount),
            maxs: self.maxs + Vector::repeat(amount),
        }
    }

    #[inline]
    fn tighten(&mut self, amount: N) {
        assert!(
            amount >= na::zero(),
            "The tightening margin must be positive."
        );
        self.mins = self.mins + Vector::repeat(amount);
        self.maxs = self.maxs + Vector::repeat(-amount);
        assert!(
            na::partial_le(&self.mins, &self.maxs),
            "The tightening margin is to large."
        );
    }

    #[inline]
    fn tightened(&self, amount: N) -> AABB<N> {
        assert!(
            amount >= na::zero(),
            "The tightening margin must be positive."
        );

        AABB::new(
            self.mins + Vector::repeat(amount),
            self.maxs + Vector::repeat(-amount),
        )
    }
}