use na::{Transform, Rotate};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use shape::Plane;
use math::{Scalar, Point, Vect};

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<N, P, V>(center: &P, normal: &V, ray: &Ray<P, V>) -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let dpos = *center - ray.orig;
    let t    = na::dot(normal, &dpos) / na::dot(normal, &ray.dir);

    if t >= na::zero() {
        Some(t)
    }
    else {
        None
    }
}

impl<N, P, V> LocalRayCast<N, P, V> for Plane<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let dpos = -ray.orig;

        let dot_normal_dpos = na::dot(self.normal(), dpos.as_vec());

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero()))
        }

        let t = dot_normal_dpos / na::dot(self.normal(), &ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() { -*self.normal() } else { self.normal().clone() };

            Some(RayIntersection::new(t, n))
        }
        else {
            None
        }
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Plane<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}
