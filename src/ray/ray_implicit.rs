use std::num::{Zero, One};
use nalgebra::na::{AlgebraicVecExt, Cast, Identity, Translation, Rotate, Transform};
use nalgebra::na;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::{Cylinder, Cone, Capsule, MinkowskiSum, Convex, Segment};
use implicit::Implicit;
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
                                   N: Ord + Num + Float + Cast<f32>,
                                   V: AlgebraicVecExt<N> + Clone,
                                   M: Translation<V>>(
                                   m:       &M,
                                   geom:    &G,
                                   simplex: &mut S,
                                   ray:     &Ray<V>)
                                   -> Option<(N, V)> {
    let mut ltoi: N = Zero::zero();

    let _eps: N     = Float::epsilon();
    let _eps_tol: N = _eps * na::cast(100.0);
    let _dim        = na::dim::<V>();

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = curr_ray.orig - m.translation();

    if dir.is_zero() {
        dir.set(0, One::one())
    }

    let mut old_sq_len: N = Bounded::max_value();

    let mut ldir = dir.clone();
    // FIXME: this converges in more than 100 iterations… something is wrong here…
    let mut niter = 0;
    loop {
        niter = niter + 1;
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
                if na::dot(&dir, &ray.dir) < na::zero() && t > _eps_tol {
                    // new lower bound
                    ldir = dir.clone();
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
                if na::dot(&dir, &ray.dir) > na::zero() {
                    // miss
                    return None
                }
            }
        }

        simplex.add_point(support_point - curr_ray.orig);

        let proj       = simplex.project_origin_and_reduce();
        let sq_len_dir = na::sqnorm(&proj);

        if (simplex.dimension() == _dim) {
            return Some((ltoi, ldir)) // FIXME: dir or -proj ?
        }
        else if (sq_len_dir <= _eps_tol * simplex.max_sq_len()) {
            // Return ldir: the last projection plane is tangeant to the intersected surface.
            return Some((ltoi, ldir))
        }
        else if (sq_len_dir >= old_sq_len) {
            // use dir instead of proj since this situations means that the new projection is less
            // accurate than the last one (which is stored on dir).
            return Some((ltoi, dir)) // FIXME: dir or -proj ?
        }

        old_sq_len = sq_len_dir;
        dir        = -proj;
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Cylinder<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Cylinder<N> { }

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Cone<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Cone<N> { }

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Capsule<N> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Capsule<N> { }

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Convex<N, V> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Convex<N, V> { }

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Segment<N, V> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        // XXX: optimize if na::dim::<V>() == 2 && self.margin().is_zero()
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Segment<N, V> { }

impl<'a,
     N:  Ord + Num + Float + Cast<f32> + Clone,
     V:  AlgebraicVecExt<N> + Clone,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>,
     M>
RayCast<N, V> for MinkowskiSum<'a, M, G1, G2> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
    }
}

impl<'a,
     N:  Ord + Num + Float + Cast<f32> + Clone,
     V:  AlgebraicVecExt<N> + Clone,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>,
     M:  Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for MinkowskiSum<'a, M, G1, G2> { }
