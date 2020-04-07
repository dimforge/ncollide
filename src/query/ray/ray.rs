//! Traits and structure needed to cast rays.

use crate::math::{Isometry, Point, Vector};
use crate::shape::FeatureId;
#[cfg(feature = "dim3")]
use na::Point2;
use na::RealField;

/// A Ray.
#[derive(Debug, Clone, Copy)]
pub struct Ray<N: RealField> {
    /// Starting point of the ray.
    pub origin: Point<N>,
    /// Direction of the ray.
    pub dir: Vector<N>,
}

impl<N: RealField> Ray<N> {
    /// Creates a new ray starting from `origin` and with the direction `dir`. `dir` must be
    /// normalized.
    pub fn new(origin: Point<N>, dir: Vector<N>) -> Ray<N> {
        Ray {
            origin: origin,
            dir: dir,
        }
    }

    /// Transforms this ray by the given isometry.
    #[inline]
    pub fn transform_by(&self, m: &Isometry<N>) -> Self {
        Self::new(m * self.origin, m * self.dir)
    }

    /// Transforms this ray by the inverse of the given isometry.
    #[inline]
    pub fn inverse_transform_by(&self, m: &Isometry<N>) -> Self {
        Self::new(
            m.inverse_transform_point(&self.origin),
            m.inverse_transform_vector(&self.dir),
        )
    }

    /// Translates this ray by the given vector. Its direction is left unchanged.
    #[inline]
    pub fn translate_by(&self, v: Vector<N>) -> Self {
        Self::new(self.origin + v, self.dir)
    }

    /// Computes the point at the given parameter on this line.
    ///
    /// This computes `self.origin + self.dir * t`.
    #[inline]
    pub fn point_at(&self, t: N) -> Point<N> {
        self.origin + self.dir * t
    }
}

/// Structure containing the result of a successful ray cast.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RayIntersection<N: RealField> {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with: `ray.point_at(toi)` or equivalently `origin + dir * toi` where `origin` is the origin of the ray;
    /// `dir` is its direction and `toi` is the value of this field.
    pub toi: N,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    // XXX: use a Unit<Vetor<N>> instead.
    pub normal: Vector<N>,

    /// Feature at the intersection point.
    pub feature: FeatureId,

    /// The textures coordinates at the intersection point.  This is an `Option` because some shape
    /// do not support texture coordinates.
    #[cfg(feature = "dim3")]
    pub uvs: Option<Point2<N>>,
}

impl<N: RealField> RayIntersection<N> {
    #[inline]
    /// Creates a new `RayIntersection`.
    #[cfg(feature = "dim3")]
    pub fn new_with_uvs(
        toi: N,
        normal: Vector<N>,
        feature: FeatureId,
        uvs: Option<Point2<N>>,
    ) -> RayIntersection<N> {
        RayIntersection {
            toi,
            normal,
            feature,
            uvs,
        }
    }

    #[inline]
    /// Creates a new `RayIntersection`.
    #[cfg(feature = "dim3")]
    pub fn new(toi: N, normal: Vector<N>, feature: FeatureId) -> RayIntersection<N> {
        RayIntersection {
            toi,
            normal,
            feature,
            uvs: None,
        }
    }

    #[inline]
    /// Creates a new `RayIntersection`.
    #[cfg(feature = "dim2")]
    pub fn new(toi: N, normal: Vector<N>, feature: FeatureId) -> RayIntersection<N> {
        RayIntersection {
            toi,
            normal,
            feature,
        }
    }
}

/// Traits of objects which can be transformed and tested for intersection with a ray.
pub trait RayCast<N: RealField> {
    /// Computes the time of impact between this transform shape and a ray.
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, solid: bool) -> Option<N> {
        self.toi_and_normal_with_ray(m, ray, max_toi, solid)
            .map(|inter| inter.toi)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>>;

    /// Computes time of impact, normal, and texture coordinates (uv) between this transformed
    /// shape and a ray.
    #[cfg(feature = "dim3")]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        self.toi_and_normal_with_ray(m, ray, max_toi, solid)
    }

    /// Tests whether a ray intersects this transformed shape.
    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N) -> bool {
        self.toi_with_ray(m, ray, max_toi, true).is_some()
    }
}
