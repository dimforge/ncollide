use std::num::Zero;
use na::{Identity, Translation, Rotate, Transform};
use na;
use geometry::algorithms::gjk;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use shape::{MinkowskiSum, Segment, Cylinder, Cone, Capsule, Convex};
use support_map::SupportMap;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use math::{Scalar, Point, Vect};


/// Cast a ray on a shape using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<N, P, V, M, S, G>(m:       &M,
                                                          shape:   &G,
                                                          simplex: &mut S,
                                                          ray:     &Ray<P, V>,
                                                          solid:   bool)
                                                          -> Option<RayIntersection<N, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translation<V>,
          S: Simplex<N, P>,
          G: SupportMap<P, V, M> {
    let inter = gjk::cast_ray(m, shape, simplex, ray);

    if !solid {
        match inter {
            None        => None,
            Some((toi, normal)) => {
                if toi.is_zero() {
                    // the ray is inside of the shape.
                    let supp    = shape.support_point(m, &ray.dir);
                    let shift   = na::dot(&(supp - ray.orig), &ray.dir) + na::cast(0.001f64);
                    let new_ray = Ray::new(ray.orig + ray.dir * shift, -ray.dir);

                    // FIXME: replace by? : simplex.translate_by(&(ray.orig - new_ray.orig));
                    simplex.reset(supp + (-*new_ray.orig.as_vec()));

                    gjk::cast_ray(m, shape, simplex, &new_ray).map(|(toi, normal)| {
                        RayIntersection::new(shift - toi, normal)
                    })
                }
                else {
                    Some(RayIntersection::new(toi, normal))
                }
            }
        }
    }
    else {
        inter.map(|(toi, normal)| RayIntersection::new(toi, normal))
    }
}

impl<N, P, V> LocalRayCast<N, P, V> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalRayCast<N, P, V> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalRayCast<N, P, V> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalRayCast<N, P, V> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalRayCast<N, P, V> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        // XXX: optimize if na::dim::<P, V>() == 2
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<'a, N, P, V, M, G1, G2> LocalRayCast<N, P, V> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Transform<P> + Rotate<V>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
    }
}

impl<'a, N, P, V, M, G1, G2> RayCast<N, P, V, M> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Transform<P> + Rotate<V>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
}
