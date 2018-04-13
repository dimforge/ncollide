use na;

use query::{Ray, RayCast, RayIntersection};
use shape::Plane;
use math::{Isometry, Point};

/// Computes the toi of an unbounded line with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_line<P: Point>(
    plane_center: &P,
    plane_normal: &P::Vector,
    line_origin: &P,
    line_dir: &P::Vector,
) -> P::Real {
    let dpos = *plane_center - *line_origin;
    na::dot(plane_normal, &dpos) / na::dot(plane_normal, line_dir)
}

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<P: Point>(
    center: &P,
    normal: &P::Vector,
    ray: &Ray<P>,
) -> Option<P::Real> {
    let t = plane_toi_with_line(center, normal, &ray.origin, &ray.dir);
    if t >= na::zero() {
        Some(t)
    } else {
        None
    }
}

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Plane<P::Vector> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        let dpos = -ls_ray.origin;

        let dot_normal_dpos = na::dot(self.normal().as_ref(), &dpos.coordinates());

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
