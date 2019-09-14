use na::{self, RealField};

use crate::math::{Isometry, Point, Vector};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{FeatureId, Plane};

/// Computes the toi of an unbounded line with a plane described by its center and normal.
#[inline]
pub fn line_toi_with_plane<N: RealField>(
    plane_center: &Point<N>,
    plane_normal: &Vector<N>,
    line_origin: &Point<N>,
    line_dir: &Vector<N>,
) -> Option<N> {
    let dpos = *plane_center - *line_origin;
    let denom = plane_normal.dot(line_dir);

    if relative_eq!(denom, N::zero()) {
        None
    } else {
        Some(plane_normal.dot(&dpos) / denom)
    }
}

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn ray_toi_with_plane<N: RealField>(
    center: &Point<N>,
    normal: &Vector<N>,
    ray: &Ray<N>,
) -> Option<N> {
    if let Some(t) = line_toi_with_plane(center, normal, &ray.origin, &ray.dir) {
        if t >= na::zero() {
            return Some(t);
        }
    }

    None
}

impl<N: RealField> RayCast<N> for Plane<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let dpos = -ls_ray.origin;

        let dot_normal_dpos = self.normal().dot(&dpos.coords);

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(
                na::zero(),
                na::zero(),
                FeatureId::Face(0),
            ));
        }

        let t = dot_normal_dpos / self.normal().dot(&ls_ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() {
                -*self.normal()
            } else {
                *self.normal()
            };

            Some(RayIntersection::new(t, m * *n, FeatureId::Face(0)))
        } else {
            None
        }
    }
}
