//! The Gilbert–Johnson–Keerthi distance algorithm.

use na::Identity;
use na;
use geom::{AnnotatedPoint, AnnotatedMinkowskiSum, Reflection};
use implicit::Implicit;
use narrow::algorithm::simplex::Simplex;
use math::{Scalar, Point, Vect};


/// Results of the GJK algorithm.
#[deriving(Encodable, Decodable, Clone)]
pub enum GJKResult<P, V> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    Projection(P),
    /// Result of the GJK algorithm when the origin is to far away from the polytope.
    NoIntersection(V)
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
pub fn closest_points<N, P, V, M, S, G1, G2>(m1:      &M,
                                             g1:      &G1,
                                             m2:      &M,
                                             g2:      &G2,
                                             simplex: &mut S)
                                             -> Option<(P, P)>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          S:  Simplex<N, AnnotatedPoint<P>>,
          G1: Implicit<P, V, M>,
          G2: Implicit<P, V, M> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    // XXX: we need to specify S because of a bug on the compiler.
    project_origin::<_, _, _, _, S, _>(&Identity::new(), &cso, simplex).map(|p| (p.orig1().clone(), -*p.orig2()))
}

/// Computes the closest points between two convex geometries unsing the GJK algorithm.
///
/// # Arguments:
/// * `g1`      - first geometry.
/// * `g2`      - second geometry.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the geometries CSO. See `minkowski_sum::cso_support_point` to
///               compute such point.
pub fn closest_points_with_max_dist<N, P, V, M, S, G1, G2>(m1:       &M,
                                                           g1:       &G1,
                                                           m2:       &M,
                                                           g2:       &G2,
                                                           max_dist: N,
                                                           simplex:  &mut S)
                                                           -> GJKResult<(P, P), V>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          S:  Simplex<N, AnnotatedPoint<P>>,
          G1: Implicit<P, V, M>,
          G2: Implicit<P, V, M> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    // XXX: we need to specify S because of a bug on the compiler.
    match project_origin_with_max_dist::<_, _, _, _, S, _>(&Identity::new(), &cso, max_dist, simplex) {
        Projection(p)       => Projection((p.orig1().clone(), -*p.orig2())),
        Intersection        => Intersection,
        NoIntersection(dir) => NoIntersection(dir.clone())
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
pub fn project_origin<N, P, V, M, S, G>(m: &M, geom: &G, simplex: &mut S) -> Option<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          S: Simplex<N, P>,
          G: Implicit<P, V, M> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(proj.as_vec());

    let _eps: N  = Float::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<V>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return None // point inside of the cso
        }

        let support_point = geom.support_point(m, &-*proj.as_vec());

        if sq_len_dir - na::dot(proj.as_vec(), support_point.as_vec()) <= _eps_rel * sq_len_dir {
            return Some(proj) // the distance found has a good enough precision
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = na::sqnorm(proj.as_vec());

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
pub fn project_origin_with_max_dist<N, P, V, M, S, G>(m:        &M,
                                                      geom:     &G,
                                                      max_dist: N,
                                                      simplex:  &mut S)
                                                      -> GJKResult<P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          S: Simplex<N, P>,
          G: Implicit<P, V, M> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(proj.as_vec());

    let _eps: N  = Float::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<V>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return Intersection // point inside of the cso
        }

        let support_point = geom.support_point(m, &-*proj.as_vec());

        let dot = na::dot(proj.as_vec(), support_point.as_vec());

        // FIXME: find a way to avoid the sqrt here
        if dot > max_dist * na::norm(proj.as_vec()) {
            return NoIntersection(proj.to_vec());
        }

        if sq_len_dir - dot <= _eps_rel * sq_len_dir {
            return Projection(proj) // the distance found has a good enough precision
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = na::sqnorm(proj.as_vec());

        if sq_len_dir >= old_sq_len_dir {
            return Projection(old_proj) // upper bounds inconsistencies
        }
    }
}
