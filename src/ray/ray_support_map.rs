use std::num::{Zero, Bounded};
use na::{Identity, Translation, Rotate, Transform, Norm};
use na;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use shape::{MinkowskiSum, Segment, Cylinder, Cone, Capsule, Convex};
use support_map::SupportMap;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use ray;
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
    let inter = gjk_toi_and_normal_with_ray(m, shape, simplex, ray);

    if !solid {
        match inter {
            None        => None,
            Some(inter) => {
                if inter.toi.is_zero() {
                    // the ray is inside of the shape.
                    let supp    = shape.support_point(m, &ray.dir);
                    let shift   = na::dot(&(supp - ray.orig), &ray.dir) + na::cast(0.001f64);
                    let new_ray = Ray::new(ray.orig + ray.dir * shift, -ray.dir);

                    // FIXME: replace by? : simplex.translate_by(&(ray.orig - new_ray.orig));
                    simplex.reset(supp + (-*new_ray.orig.as_vec()));

                    gjk_toi_and_normal_with_ray(m, shape, simplex, &new_ray).map(|new_inter| {
                        RayIntersection::new(shift - new_inter.toi, new_inter.normal)
                    })
                }
                else {
                    Some(inter)
                }
            }
        }
    }
    else {
        inter
    }
}

fn gjk_toi_and_normal_with_ray<N, P, V, M, S, G>(m:       &M,
                                                 shape:   &G,
                                                 simplex: &mut S,
                                                 ray:     &Ray<P, V>)
                                                 -> Option<RayIntersection<N, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translation<V>,
          S: Simplex<N, P>,
          G: SupportMap<P, V, M> {
    let mut ltoi: N = na::zero();

    let _eps: N     = Float::epsilon();
    let _eps_tol: N = _eps * na::cast(100.0f64);
    let _dim             = na::dim::<V>();

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = *curr_ray.orig.as_vec() - m.translation();

    if dir.is_zero() {
        dir[0] = na::one();
    }

    let mut old_sq_len: N = Bounded::max_value();

    let mut ldir = dir.clone();
    // FIXME: this converges in more than 100 iterations… something is wrong here…
    let mut niter = 0u;
    loop {
        niter = niter + 1;

        if dir.normalize().is_zero() {
            return Some(RayIntersection::new(ltoi, ldir))
        }

        let support_point = shape.support_point(m, &dir);

        // Clip the ray on the support plane (None <=> t < 0)
        // The configurations are:
        //   dir.dot(ray.dir)  |   t   |               Action
        // −−−−−−−−−−−−−−−−−−−−+−−−−−−−+−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
        //          < 0        |  < 0  | Continue.
        //          < 0        |  > 0  | New lower bound, move the origin.
        //          > 0        |  < 0  | Miss. No intersection.
        //          > 0        |  > 0  | New higher bound.
        match ray::plane_toi_with_ray(&support_point, &dir, &curr_ray) {
            Some(t) => {
                if na::dot(&dir, &ray.dir) < na::zero() && t > _eps_tol {
                    // new lower bound
                    ldir = dir.clone();
                    ltoi = ltoi + t;
                    curr_ray.orig = ray.orig + ray.dir * ltoi;
                    dir = curr_ray.orig - support_point;
                    // FIXME: could we simply translate the simpex by old_orig - new_orig ?
                    simplex.reset(na::orig::<P>() + (-dir));
                    let _max: N = Bounded::max_value();
                    old_sq_len = _max;
                    continue
                }
            },
            None => {
                if na::dot(&dir, &ray.dir) > na::zero() {
                    // miss
                    return None
                }
            }
        }

        simplex.add_point(na::orig::<P>() + (support_point - curr_ray.orig));

        let proj       = simplex.project_origin_and_reduce().to_vec();
        let sq_len_dir = na::sqnorm(&proj);

        if simplex.dimension() == _dim {
            return Some(RayIntersection::new(ltoi, ldir))
        }
        else if sq_len_dir <= _eps_tol * simplex.max_sq_len() {
            // Return ldir: the last projection plane is tangeant to the intersected surface.
            return Some(RayIntersection::new(ltoi, ldir))
        }
        else if sq_len_dir >= old_sq_len {
            // use dir instead of proj since this situations means that the new projection is less
            // accurate than the last one (which is stored on dir).
            return Some(RayIntersection::new(ltoi, dir))
        }

        old_sq_len = sq_len_dir;
        dir        = -proj;
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
