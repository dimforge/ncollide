use na::{self, Real};

use math::{Isometry, Point, Vector};
use query::{Ray, RayCast, RayIntersection};
use shape::{Plane, FeatureId};

/// Computes the toi of an unbounded line with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_line<N: Real>(
    plane_center: &Point<N>,
    plane_normal: &Vector<N>,
    line_origin: &Point<N>,
    line_dir: &Vector<N>,
) -> Option<N>
{
    let dpos = *plane_center - *line_origin;
    let denom = na::dot(plane_normal, line_dir);

    if relative_eq!(denom, N::zero()) {
        None
    } else {
        Some(na::dot(plane_normal, &dpos) / denom)
    }
}

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<N: Real>(
    center: &Point<N>,
    normal: &Vector<N>,
    ray: &Ray<N>,
) -> Option<N>
{
    if let Some(t) = plane_toi_with_line(center, normal, &ray.origin, &ray.dir) {
        if t >= na::zero() {
            return Some(t);
        }
    }

    None
}

impl<N: Real> RayCast<N> for Plane<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        let dpos = -ls_ray.origin;

        let dot_normal_dpos = na::dot(self.normal().as_ref(), &dpos.coords);

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero(), FeatureId::Face(0)));
        }

        let t = dot_normal_dpos / na::dot(self.normal().as_ref(), &ls_ray.dir);

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
