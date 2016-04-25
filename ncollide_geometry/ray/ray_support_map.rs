use na::{Identity, Translate, Rotate, Transform};
use na;
use query::algorithms::gjk;
use query::algorithms::simplex::Simplex;
use query::algorithms::johnson_simplex::JohnsonSimplex;
use shape::{SupportMap, MinkowskiSum, Segment, Cylinder, Cone, Capsule, ConvexHull};
use ray::{Ray, RayCast, RayIntersection};
use math::{Point, Vector};


/// Cast a ray on a shape using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<P, M, S, G: ?Sized>(m:       &M,
                                                            shape:   &G,
                                                            simplex: &mut S,
                                                            ray:     &Ray<P>,
                                                            solid:   bool)
                                                            -> Option<RayIntersection<P::Vect>>
    where P: Point,
          M: Translate<P>,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    let inter = gjk::cast_ray(m, shape, simplex, ray);

    if !solid {
        match inter {
            None                => None,
            Some((toi, normal)) => {
                if na::is_zero(&toi) {
                    // the ray is inside of the shape.
                    let ndir    = na::normalize(&ray.dir);
                    let supp    = shape.support_point(m, &ndir);
                    let shift   = na::dot(&(supp - ray.origin), &ndir) + na::cast(0.001f64);
                    let new_ray = Ray::new(ray.origin + ndir * shift, -ray.dir);

                    // FIXME: replace by? : simplex.translate_by(&(ray.origin - new_ray.origin));
                    simplex.reset(supp + (-*new_ray.origin.as_vector()));

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

impl<P, M> RayCast<P, M> for Cylinder<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}

impl<P, M> RayCast<P, M> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}

impl<P, M> RayCast<P, M> for Capsule<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}

impl<P, M> RayCast<P, M> for ConvexHull<P>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}

impl<P, M> RayCast<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        // XXX: optimize if na::dimension::<P>() == 2
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}

impl<'a, P, M, M2, G1: ?Sized, G2: ?Sized> RayCast<P, M2> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          M2: Transform<P> + Translate<P> + Rotate<P::Vect>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    fn toi_and_normal_with_ray(&self, m: &M2, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        // XXX: optimize if na::dimension::<P>() == 2
        let ls_ray = Ray::new(m.inverse_transform(&ray.origin), m.inverse_rotate(&ray.dir));

        implicit_toi_and_normal_with_ray(&Identity::new(), self,
                                         &mut JohnsonSimplex::<P>::new_w_tls(), &ls_ray,
                                         solid).map(|mut res| {
            res.normal = m.rotate(&res.normal);
            res
        })
    }
}
