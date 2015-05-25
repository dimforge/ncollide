use na::{Identity, Translation, Rotate, Transform};
use na;
use geometry::algorithms::gjk;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use entities::shape::{MinkowskiSum, Segment, Cylinder, Cone, Capsule, Convex};
use entities::support_map::SupportMap;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use math::{Scalar, Point, Vect};


/// Cast a ray on a shape using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<P, M, S, G: ?Sized>(m:       &M,
                                                            shape:   &G,
                                                            simplex: &mut S,
                                                            ray:     &Ray<P>,
                                                            solid:   bool)
                                                            -> Option<RayIntersection<P::Vect>>
    where P: Point,
          M: Translation<P::Vect>,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    let inter = gjk::cast_ray(m, shape, simplex, ray);

    if !solid {
        match inter {
            None                => None,
            Some((toi, normal)) => {
                if na::is_zero(&toi) {
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

impl<P> LocalRayCast<P> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}

impl<P> LocalRayCast<P> for Cone<<P::Vect as Vect>::Scalar>
    where P: Point {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Cone<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}

impl<P> LocalRayCast<P> for Capsule<<P::Vect as Vect>::Scalar>
    where P: Point {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Capsule<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}

impl<P> LocalRayCast<P> for Convex<P>
    where P: Point {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Convex<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}

impl<P> LocalRayCast<P> for Segment<P>
    where P: Point {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        // XXX: optimize if na::dim::<P>() == 2
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}

impl<'a, P, M, G1: ?Sized, G2: ?Sized> LocalRayCast<P> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          M:  Transform<P> + Rotate<P::Vect>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<P>::new_w_tls(), ray, solid)
    }
}

impl<'a, P, M, G1: ?Sized, G2: ?Sized> RayCast<P, M> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          M:  Transform<P> + Rotate<P::Vect>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
}
