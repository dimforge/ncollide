use na;

use query::{Ray, RayCast, RayIntersection};
use shape::Plane;
use math::{Isometry, Point};

/// Computes the toi of an unbounded line with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_line<N: Real>(
    plane_center: &P,
    plane_normal: &Vector<N>,
    line_origin: &P,
    line_dir: &Vector<N>,
) -> N {
    let dpos = *plane_center - *line_origin;
    na::dot(plane_normal, &dpos) / na::dot(plane_normal, line_dir)
}

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<N: Real>(
    center: &P,
    normal: &Vector<N>,
    ray: &Ray<N>,
) -> Option<N> {
    let t = plane_toi_with_line(center, normal, &ray.origin, &ray.dir);
    if t >= na::zero() {
        Some(t)
    } else {
        None
    }
}

impl<N: Real> RayCast<P, M> for Plane<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        let ls_ray = ray.inverse_transform_by(m);

        let dpos = -ls_ray.origin;

        let dot_normal_dpos = na::dot(self.normal().as_ref(), &dpos.coords);

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero()));
        }

        let t = dot_normal_dpos / na::dot(self.normal().as_ref(), &ls_ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() {
                -*self.normal()
            } else {
                *self.normal()
            };

            Some(RayIntersection::new(t, m.rotate_vector(&n)))
        } else {
            None
        }
    }
}
