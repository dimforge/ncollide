//! Traits and structure needed to cast rays.

use na::{Point2, Real};
use utils::IsometryOps;
use math::{Isometry, Point, Vector};

/// A Ray.
#[derive(Debug, Clone, Copy)]
pub struct Ray<N: Real> {
    /// Starting point of the ray.
    pub origin: Point<N>,
    /// Direction of the ray.
    pub dir: Vector<N>,
}

impl<N: Real> Ray<N> {
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
}

/// Structure containing the result of a successful ray cast.
pub struct RayIntersection<N: Real> {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with: `origin + dir * toi` where `origin` is the origin of the ray; `dir` is its direction and
    /// `toi` is the value of this field.
    pub toi: N,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    // XXX: use a Unit<Vetor<N>> instead.
    pub normal: Vector<N>,

    /// The textures coordinates at the intersection point.  This is an `Option` because some shape
    /// do not support texture coordinates.
    pub uvs: Option<Point2<N>>,
}

impl<N: Real> RayIntersection<N> {
    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new_with_uvs(
        toi: N,
        normal: Vector<N>,
        uvs: Option<Point2<N>>,
    ) -> RayIntersection<N> {
        RayIntersection {
            toi: toi,
            normal: normal,
            uvs: uvs,
        }
    }

    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new(toi: N, normal: Vector<N>) -> RayIntersection<N> {
        RayIntersection {
            toi: toi,
            normal: normal,
            uvs: None,
        }
    }
}

/// Traits of objects which can be transformed and tested for intersection with a ray.
pub trait RayCast<N: Real> {
    /// Computes the time of impact between this transform shape and a ray.
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        self.toi_and_normal_with_ray(m, ray, solid)
            .map(|inter| inter.toi)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>;

    /// Computes time of impact, normal, and texture coordinates (uv) between this transformed
    /// shape and a ray.
    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        self.toi_and_normal_with_ray(m, ray, solid)
    }

    /// Tests whether a ray intersects this transformed shape.
    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<N>) -> bool {
        self.toi_with_ray(m, ray, true).is_some()
    }
}
