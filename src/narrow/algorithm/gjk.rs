//! The Gilbert–Johnson–Keerthi distance algorithm.

use na::{FloatVec, Identity};
use na;
use geom::{Reflection, AnnotatedPoint, AnnotatedMinkowskiSum};
use implicit::Implicit;
use narrow::algorithm::simplex::Simplex;
use math::{Scalar, Vect, Matrix};

/// Results of the GJK algorithm.
#[deriving(Encodable, Decodable, Clone)]
pub enum GJKResult<_V, Dir> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    Projection(_V),
    /// Result of the GJK algorithm when the origin is to far away from the polytope.
    NoIntersection(Dir)
}

/// Computes the closest points between two convex geometries unsing the GJK
/// algorithm.
///
/// # Arguments:
/// * `g1`      - first geometry.
/// * `g2`      - second geometry.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the geometries CSO. See
///               `minkowski_sum::cso_support_point` to compute such point.
pub fn closest_points<S:  Simplex<AnnotatedPoint>,
                      G1: Implicit<Vect, Matrix>,
                      G2: Implicit<Vect, Matrix>>(
                      m1:      &Matrix,
                      g1:      &G1,
                      m2:      &Matrix,
                      g2:      &G2,
                      simplex: &mut S) -> Option<(Vect, Vect)> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    project_origin(&Identity::new(), &cso, simplex).map(|p| (p.orig1().clone(), -p.orig2()))
}

/// Computes the closest points between two convex geometries without their margins unsing the GJK
/// algorithm.
///
/// # Arguments:
/// * `g1`      - first geometry.
/// * `g2`      - second geometry.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the geometries CSO. See `minkowski_sum::cso_support_point` to
///               compute such point.
pub fn closest_points_without_margin_with_max_dist<S:  Simplex<AnnotatedPoint>,
                                                   G1: Implicit<Vect, Matrix>,
                                                   G2: Implicit<Vect, Matrix>>(
                                                   m1:       &Matrix,
                                                   g1:       &G1,
                                                   m2:       &Matrix,
                                                   g2:       &G2,
                                                   max_dist: &Scalar,
                                                   simplex:  &mut S) -> GJKResult<(Vect, Vect), Vect> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    match project_origin_with_max_dist(&Identity::new(), &cso, max_dist, simplex) {
        Projection(p)       => Projection((p.orig1().clone(), -p.orig2())),
        Intersection        => Intersection,
        NoIntersection(dir) => NoIntersection(dir.point().clone())
    }
}

/*
 * Distance GJK
 */
/// Projects the origin on a geometry unsing the GJK algorithm.
///
/// # Arguments:
/// * geom - the geometry to project the origin on
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the geometry boundary.
pub fn project_origin<S: Simplex<_V>, G: Implicit<_V, _M>, _V: FloatVec<Scalar>, _M>(
                      m:       &_M,
                      geom:    &G,
                      simplex: &mut S)
                      -> Option<_V> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(&proj);

    let _eps: Scalar  = Float::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<Vect>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return None // point inside of the cso
        }

        let support_point = geom.support_point(m, &-proj);

        if sq_len_dir - na::dot(&proj, &support_point) <= _eps_rel * sq_len_dir {
            return Some(proj) // the distance found has a good enough precision 
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = na::sqnorm(&proj);

        if sq_len_dir >= old_sq_len_dir {
            return Some(old_proj) // upper bounds inconsistencies
        }
    }
}

/*
 * Separating Axis GJK
 */
/// Projects the origin on a geometry using the Separating Axis GJK algorithm.
/// The algorithm will stop as soon as the polytope can be proven to be at least `max_dist` away
/// from the origin.
///
/// # Arguments:
/// * geom - the geometry to project the origin on
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the geometry boundary.
pub fn project_origin_with_max_dist<S: Simplex<_V>, G: Implicit<_V, _M>, _V: FloatVec<Scalar>, _M>(
                                    m:        &_M,
                                    geom:     &G,
                                    max_dist: &Scalar,
                                    simplex:  &mut S)
                                    -> GJKResult<_V, _V> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(&proj);

    let _eps: Scalar  = Float::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<Vect>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return Intersection // point inside of the cso
        }

        let support_point = geom.support_point(m, &-proj);

        let dot = na::dot(&proj, &support_point);

        // FIXME: find a way to avoid the sqrt here
        if dot > *max_dist * na::norm(&proj) {
            return NoIntersection(proj);
        }

        if sq_len_dir - dot <= _eps_rel * sq_len_dir {
            return Projection(proj) // the distance found has a good enough precision 
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = na::sqnorm(&proj);

        if sq_len_dir >= old_sq_len_dir {
            return Projection(old_proj) // upper bounds inconsistencies
        }
    }
}
