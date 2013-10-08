use std::num::{Zero, One, from_f32};
use nalgebra::na::{AlgebraicVecExt, Dim, Identity, Translation, Rotate, Transform};
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::{Cylinder, Cone, Capsule, MinkowskiSum, Implicit};
use ray::{Ray, RayCast, RayCastWithTransform};
use ray;

/// Projects the origin on a geometry unsing the GJK algorithm.
///
/// # Arguments:
///     * geom - the geometry to project the origin on
///     * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///     with at least one point on the geometry boundary.
pub fn gjk_toi_and_normal_with_ray<S: Simplex<N, V>,
                                   G: Implicit<N, V, M>,
                                   N: Ord + Num + Float + FromPrimitive + ToStr,
                                   V: AlgebraicVecExt<N> + Clone + ToStr,
                                   M: Translation<V>>(
                                   m:       &M,
                                   geom:    &G,
                                   simplex: &mut S,
                                   ray:     &Ray<V>)
                                   -> Option<(N, V)> {
    let mut ltoi: N = Zero::zero();

    let _eps: N  = Float::epsilon();
    let _eps_tol = _eps * from_f32(100.0).unwrap();
    let _eps_rel = _eps.sqrt();
    let _dim     = Dim::dim(None::<V>);

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = curr_ray.orig - m.translation();

    if dir.is_zero() {
        dir.set(0, One::one())
    }

    let mut old_sq_len: N = Bounded::max_value();

    // FIXME: this converges in more than 100 iterations… something is wrong here…
    loop {
        dir.normalize();

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
                if dir.dot(&ray.dir) < Zero::zero() {
                    // new lower bound
                    if t <= _eps_rel * ltoi {
                        return Some((ltoi, dir))
                    }

                    ltoi   = ltoi + t;
                    curr_ray.orig = ray.orig + ray.dir * ltoi;
                    dir    = curr_ray.orig - support_point;
                    simplex.reset(-dir);
                    let _M: N = Bounded::max_value();
                    old_sq_len = _M;
                    continue
                }
            },
            None => {
                if dir.dot(&ray.dir) > Zero::zero() {
                    // miss
                    return None
                }
            }
        }

        simplex.add_point(support_point - curr_ray.orig);

        let proj = simplex.project_origin_and_reduce();
        let sq_len_dir = proj.sqnorm();

        if (simplex.dimension() == _dim ||
            sq_len_dir >= old_sq_len    || // FIXME: hacky way to prevent infinite loop…
            sq_len_dir <= _eps_tol * simplex.max_sq_len()
           ) {
            return Some((ltoi, dir)) // FIXME: dir or -proj ?
        }

        old_sq_len = sq_len_dir;
        dir        = -proj;
    }
}

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr>
RayCast<N, V> for Cylinder<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Cylinder<N> { }

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr>
RayCast<N, V> for Cone<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Cone<N> { }

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr>
RayCast<N, V> for Capsule<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Capsule<N> { }

impl<'self,
     N:  Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V:  AlgebraicVecExt<N> + Clone + ToStr,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>,
     M>
RayCast<N, V> for MinkowskiSum<'self, M, G1, G2> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<'self,
     N:  Ord + Num + Float + FromPrimitive + Clone + ToStr,
     V:  AlgebraicVecExt<N> + Clone + ToStr,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>,
     M:  Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for MinkowskiSum<'self, M, G1, G2> { }
