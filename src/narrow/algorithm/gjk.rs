use std::num::NumCast;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use geom::implicit::Implicit;
use geom::reflection::Reflection;
use geom::minkowski_sum::{AnnotatedPoint, AnnotatedNonTransformableMinkowskiSum};
use narrow::algorithm::simplex::Simplex;

///  Computes the closest points between two convex geometries unsing the GJK algorithm.
///
///  # Arguments:
///     * `g1`      - first geometry.
///     * `g2`      - second geometry.
///     * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///     with at least one point on the geometries CSO. See `minkowski_sum::cso_support_point` to
///     compute such point.
pub fn closest_points<S:  Simplex<N, AnnotatedPoint<V>>,
                      G1: Implicit<V, M>,
                      G2: Implicit<V, M>,
                      N:  Sub<N, N> + Ord + Mul<N, N> + Float + NumCast + ToStr,
                      V:  Clone + Norm<N> + Neg<V> + Add<V, V> + Dot<N> + Dim,
                      M>(
                      m1:      &M,
                      g1:      &G1,
                      m2:      &M,
                      g2:      &G2,
                      simplex: &mut S) -> Option<(V, V)> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedNonTransformableMinkowskiSum::new(m1, g1, m2, &reflect2);

    // NOTE: we pass `m1` or whatever, it will be ignored by the NonTransformableMinkowskiSum anyway.
    project_origin(m1, &cso, simplex).map(|p| (p.orig1().clone(), -p.orig2()))
}

/// Projects the origin on a geometry unsing the GJK algorithm.
///
/// # Arguments:
///     * geom - the geometry to project the origin on
///     * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///     with at least one point on the geometry boundary.
pub fn project_origin<S: Simplex<N, V>,
                      G: Implicit<V, M>,
                      N: Sub<N, N> + Ord + Mul<N, N> + Float + NumCast + ToStr,
                      V: Norm<N> + Neg<V> + Dot<N> + Dim,
                      M>(
                      m:       &M,
                      geom:    &G,
                      simplex: &mut S)
                      -> Option<V> {
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = proj.sqnorm();

    let _eps_tol = Float::epsilon::<N>() * NumCast::from(100.0f64);
    let _eps_rel = Float::epsilon::<N>().sqrt();
    let _dim     = Dim::dim::<V>();

    loop {
        let support_point = geom.support_point(m, &-proj);

        if (sq_len_dir - proj.dot(&support_point) <= _eps_rel * sq_len_dir) {
            return Some(proj) // the distance found has a good enough precision 
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = proj.sqnorm();

        if (simplex.dimension() == _dim || sq_len_dir <= _eps_tol * simplex.max_sq_len()) {
            return None // point inside of the cso
        }

        if (sq_len_dir >= old_sq_len_dir) {
            return Some(old_proj) // upper bounds inconsistencies
        }
    }
}
