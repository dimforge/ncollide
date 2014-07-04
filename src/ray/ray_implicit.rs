use std::num::{Zero, Bounded};
use nalgebra::na::{Identity, Translation, Indexable, Norm};
use nalgebra::na;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::{Cylinder, Cone, Capsule, MinkowskiSum, Convex, Segment};
use implicit::Implicit;
use ray::{Ray, RayCast, RayIntersection};
use ray;
use math::{Scalar, Vect, Matrix};

/// Cast a ray on a geometry using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<S: Simplex<Vect>,
                                        G: Implicit<Vect, _M>,
                                        _M: Translation<Vect>>(
                                        m:       &_M,
                                        geom:    &G,
                                        simplex: &mut S,
                                        ray:     &Ray,
                                        solid:   bool)
                                        -> Option<RayIntersection> {
    let inter = gjk_toi_and_normal_with_ray(m, geom, simplex, ray);

    if !solid {
        match inter {
            None        => None,
            Some(inter) => {
                if inter.toi == na::zero() {
                    // the ray is inside of the shape.
                    let supp    = geom.support_point(m, &ray.dir);
                    let shift   = na::dot(&(supp - ray.orig), &ray.dir) + na::cast(0.001f64);
                    let new_ray = Ray::new(ray.orig + ray.dir * shift, -ray.dir);

                    simplex.reset(supp - new_ray.orig); // FIXME: replace by? : simplex.translate_by(&(ray.orig - new_ray.orig));

                    gjk_toi_and_normal_with_ray(m, geom, simplex, &new_ray).map(|new_inter| {
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

fn gjk_toi_and_normal_with_ray<S: Simplex<Vect>, G: Implicit<Vect, _M>, _M: Translation<Vect>>(
                               m:       &_M,
                               geom:    &G,
                               simplex: &mut S,
                               ray:     &Ray)
                               -> Option<RayIntersection> {
    let mut ltoi: Scalar = na::zero();

    let _eps: Scalar     = Float::epsilon();
    let _eps_tol: Scalar = _eps * na::cast(100.0f64);
    let _dim             = na::dim::<Vect>();

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = curr_ray.orig - m.translation();

    if dir.is_zero() {
        dir.set(0, na::one())
    }

    let mut old_sq_len: Scalar = Bounded::max_value();

    let mut ldir = dir.clone();
    // FIXME: this converges in more than 100 iterations… something is wrong here…
    let mut niter = 0u;
    loop {
        niter = niter + 1;

        let _             = dir.normalize();
        let support_point = geom.support_point(m, &dir);

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
                    simplex.reset(-dir); // FIXME: could we simply translate the simpex by old_orig - new_orig ?
                    let _M: Scalar = Bounded::max_value();
                    old_sq_len = _M;
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

        simplex.add_point(support_point - curr_ray.orig);

        let proj       = simplex.project_origin_and_reduce();
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

impl RayCast for Cylinder {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}

impl RayCast for Cone {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}

impl RayCast for Capsule {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}

impl RayCast for Convex {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}

impl RayCast for Segment {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        // XXX: optimize if na::dim::<Vect>() == 2 && self.margin().is_zero()
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}

impl<'a, G1: Implicit<Vect, Matrix>, G2: Implicit<Vect, Matrix>>
RayCast for MinkowskiSum<'a, G1, G2> {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Vect>::new_w_tls(), ray, solid)
    }
}
