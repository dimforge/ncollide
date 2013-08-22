use std::num::Zero;
use nalgebra::traits::vector::{AlgebraicVec, AlgebraicVecExt};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::implicit::Implicit;
use geom::cylinder::Cylinder;
use geom::cone::Cone;
use geom::capsule::Capsule;
use geom::minkowski_sum::NonTransformableMinkowskiSum;
use ray::ray::{Ray, RayCast};
use ray::ray_plane;

/// Projects the origin on a geometry unsing the GJK algorithm.
///
/// # Arguments:
///     * geom - the geometry to project the origin on
///     * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///     with at least one point on the geometry boundary.
pub fn gjk_toi_with_ray<S: Simplex<N, V>,
                        G: Implicit<N, V, M>,
                        N: Ord + Num + Float + NumCast + ToStr,
                        V: AlgebraicVec<N> + Clone + ToStr,
                        M: Translation<V>>(
                        m:       &M,
                        geom:    &G,
                        simplex: &mut S,
                        ray:     &Ray<V>)
                        -> Option<N> {
    let mut ltoi   = Zero::zero::<N>();

    let _eps_tol = Float::epsilon::<N>() * NumCast::from(100.0f64);
    let _eps_rel = Float::epsilon::<N>().sqrt();
    let _dim     = Dim::dim::<V>();

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = curr_ray.orig - m.translation();
    // XXX: will fail if curr_ray.orig - m.translation() == Zero::zero()
    let mut old_sq_len = Bounded::max_value::<N>();

    // FIXME: this converges in more than 100 iterations… something is wrong here…
    loop {
        dir.normalize();

        let support_point = geom.support_point(m, &dir);

        // Clip the ray on the support plane (None <=> t < 0)
        // The configurations are:
        //   dir.dot(ray.dir)  |   t   |               Action
        // −−−−−−−−−−−−−−−−−−−−+−−−−−−−+−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
        //          < 0        |  < 0  | Continue. ?????
        //          < 0        |  > 0  | New lower bound, move the origin.
        //          > 0        |  < 0  | Miss. No intersection.
        //          > 0        |  > 0  | New higher bound.
        match ray_plane::plane_toi_with_ray(&support_point, &dir, &curr_ray) {
            Some(t) => {
                if dir.dot(&ray.dir) < Zero::zero() {
                    // new lower bound
                    if t <= _eps_rel * ltoi {
                        return Some(ltoi)
                    }

                    ltoi   = ltoi + t;
                    curr_ray.orig = ray.orig + ray.dir * ltoi;
                    dir    = curr_ray.orig - support_point;
                    simplex.reset(-dir);
                    old_sq_len = Bounded::max_value::<N>();
                    loop
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
            return Some(ltoi)
        }

        old_sq_len = sq_len_dir;
        dir        = -proj;
    }
}

impl<N: Ord + Num + Float + NumCast + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Translation<V> + Transform<V> + Rotate<V>>
RayCast<N, V, M> for Cylinder<N> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        // FIXME: too bad we have to do _a lot_ of allocations to create the simplex solver…
        gjk_toi_with_ray(m, self, &mut JohnsonSimplex::new_w_tls::<N, V>(), ray)
    }
}

impl<N: Ord + Num + Float + NumCast + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Translation<V> + Transform<V> + Rotate<V>>
RayCast<N, V, M> for Cone<N> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        // FIXME: too bad we have to do _a lot_ of allocations to create the simplex solver…
        gjk_toi_with_ray(m, self, &mut JohnsonSimplex::new_w_tls::<N, V>(), ray)
    }
}

impl<N: Ord + Num + Float + NumCast + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Translation<V> + Transform<V> + Rotate<V>>
RayCast<N, V, M> for Capsule<N> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        // FIXME: too bad we have to do _a lot_ of allocations to create the simplex solver…
        gjk_toi_with_ray(m, self, &mut JohnsonSimplex::new_w_tls::<N, V>(), ray)
    }
}

impl<'self,
     N:  Ord + Num + Float + NumCast + Clone + ToStr,
     V:  AlgebraicVecExt<N> + Clone + ToStr,
     M:  Translation<V> + Transform<V> + Rotate<V>,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>>
RayCast<N, V, M> for NonTransformableMinkowskiSum<'self, M, G1, G2> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        // FIXME: too bad we have to do _a lot_ of allocations to create the simplex solver…
        gjk_toi_with_ray(m, self, &mut JohnsonSimplex::new_w_tls::<N, V>(), ray)
    }
}
