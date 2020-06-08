//! Axis Aligned Bounding Box.

use crate::bounding_volume::{BoundingSphere, BoundingVolume, HasBoundingVolume};
use crate::math::{Isometry, Point, Vector, DIM};
use crate::utils::IsometryOps;
use na::{self, RealField};

// Seems useful to help type inference. See issue #84.
/// Computes the axis-aligned bounding box of a shape `g` transformed by `m`.
///
/// Same as `g.aabb(m)`.
#[inline]
pub fn aabb<N, G: ?Sized>(g: &G, m: &Isometry<N>) -> AABB<N>
where
    N: RealField,
    G: HasBoundingVolume<N, AABB<N>>,
{
    g.bounding_volume(m)
}

// Seems useful to help type inference. See issue #84.
/// Computes the axis-aligned bounding box of a shape `g`.
///
/// Same as `g.local_aabb(m)`.
#[inline]
pub fn local_aabb<N, G: ?Sized>(g: &G) -> AABB<N>
where
    N: RealField,
    G: HasBoundingVolume<N, AABB<N>>,
{
    g.local_bounding_volume()
}

/// An Axis Aligned Bounding Box.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct AABB<N: RealField> {
    pub mins: Point<N>,
    pub maxs: Point<N>,
}

impl<N: RealField> AABB<N> {
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    #[inline]
    pub fn new(mins: Point<N>, maxs: Point<N>) -> AABB<N> {
        AABB { mins, maxs }
    }

    /// Creates a new AABB from its center and its half-extents.
    #[inline]
    pub fn from_half_extents(center: Point<N>, half_extents: Vector<N>) -> Self {
        Self::new(center - half_extents, center + half_extents)
    }

    /// Creates a new AABB from a set of points.
    pub fn from_points<'a, I>(pts: I) -> Self
    where
        I: IntoIterator<Item = &'a Point<N>>,
    {
        super::aabb_utils::local_point_cloud_aabb(pts)
    }

    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    #[deprecated(note = "use the `.mins` public field instead.")]
    pub fn mins(&self) -> &Point<N> {
        &self.mins
    }

    /// Reference to the AABB point with the biggest components along each axis.
    #[inline]
    #[deprecated(note = "use the `.maxs` public field instead.")]
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

    /// Enlarges this AABB so it also contains the point `pt`.
    pub fn take_point(&mut self, pt: Point<N>) {
        self.mins = self.mins.coords.inf(&pt.coords).into();
        self.maxs = self.maxs.coords.sup(&pt.coords).into();
    }

    /// Computes the AABB bounding `self` transformed by `m`.
    #[inline]
    pub fn transform_by(&self, m: &Isometry<N>) -> Self {
        let ls_center = self.center();
        let center = m * ls_center;
        let ws_half_extents = m.absolute_transform_vector(&self.half_extents());

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }

    /// The smallest bounding sphere containing this AABB.
    #[inline]
    pub fn bounding_sphere(&self) -> BoundingSphere<N> {
        let center = self.center();
        let rad = na::distance(&self.mins, &self.maxs);

        BoundingSphere::new(center, rad)
    }

    #[inline]
    pub fn contains_local_point(&self, point: &Point<N>) -> bool {
        for i in 0..DIM {
            if point[i] < self.mins[i] || point[i] > self.maxs[i] {
                return false;
            }
        }

        true
    }
}

impl<N: RealField> BoundingVolume<N> for AABB<N> {
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
        self.mins = self.mins.inf(&other.mins);
        self.maxs = self.maxs.sup(&other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB<N>) -> AABB<N> {
        AABB {
            mins: self.mins.inf(&other.mins),
            maxs: self.maxs.sup(&other.maxs),
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
