//! The Gilbert–Johnson–Keerthi distance algorithm.

use num::{Float, Zero};
use na::{Identity, Translate, Bounded, Norm};
use na;
use entities::shape::{AnnotatedPoint, AnnotatedMinkowskiSum, MinkowskiSum, Reflection};
use entities::support_map::SupportMap;
use geometry::algorithms::simplex::Simplex;
use ray::Ray;
use ray;
use math::{Point, Vect, FloatError};


/// Results of the GJK algorithm.
#[derive(RustcEncodable, RustcDecodable, Clone)]
pub enum GJKResult<P, V> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    Projection(P),
    /// Result of the GJK algorithm when the origin is to far away from the polytope.
    NoIntersection(V)
}

/// Computes the closest points between two convex shapes unsing the GJK
/// algorithm.
///
/// # Arguments:
/// * `g1`      - first shape.
/// * `g2`      - second shape.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the shapes CSO. See
///               `minkowski_sum::cso_support_point` to compute such point.
pub fn closest_points<P, M, S, G1: ?Sized, G2: ?Sized>(m1:      &M,
                                                       g1:      &G1,
                                                       m2:      &M,
                                                       g2:      &G2,
                                                       simplex: &mut S)
                                                       -> Option<(P, P)>
    where P:  Point,
          S:  Simplex<AnnotatedPoint<P>>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    // XXX: we need to specify S because of a bug on the compiler.
    project_origin::<_, _, S, _>(&Identity::new(), &cso, simplex).map(|p| (p.orig1().clone(), -*p.orig2()))
}

/// Computes the closest points between two convex shapes unsing the GJK algorithm.
///
/// # Arguments:
/// * `g1`      - first shape.
/// * `g2`      - second shape.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the shapes CSO. See `minkowski_sum::cso_support_point`
///               to compute such point.
pub fn closest_points_with_max_dist<P, M, S, G1: ?Sized, G2: ?Sized>(m1:       &M,
                                                                     g1:       &G1,
                                                                     m2:       &M,
                                                                     g2:       &G2,
                                                                     max_dist: <P::Vect as Vect>::Scalar,
                                                                     simplex:  &mut S)
                                                                     -> GJKResult<(P, P), P::Vect>
    where P:  Point,
          S:  Simplex<AnnotatedPoint<P>>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let reflect2 = Reflection::new(g2);
    let cso      = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    // XXX: we need to specify S because of a bug on the compiler.
    match project_origin_with_max_dist::<_, _, S, _>(&Identity::new(), &cso, max_dist, simplex) {
        GJKResult::Projection(p)       => GJKResult::Projection((p.orig1().clone(), -*p.orig2())),
        GJKResult::Intersection        => GJKResult::Intersection,
        GJKResult::NoIntersection(dir) => GJKResult::NoIntersection(dir.clone())
    }
}

/// Computes the exact distance separating two convex shapes unsing the GJK.
/// algorithm.
///
/// # Arguments:
/// * `g1`      - first shape.
/// * `g2`      - second shape.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the shapes CSO. See `minkowski_sum::cso_support_point`
///               to compute such point.
pub fn distance<P, M, S, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1, m2: &M, g2: &G2, simplex: &mut S) -> <P::Vect as Vect>::Scalar
    where P:  Point,
          S:  Simplex<P>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let reflect2 = Reflection::new(g2);
    let cso      = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // XXX: we need to specify S because of a bug on the compiler.
    match project_origin::<_, _, S, _>(&Identity::new(), &cso, simplex) {
        Some(c) => na::norm(c.as_vec()),
        None    => na::zero()
    }
}

/*
 * Distance GJK
 */
/// Projects the origin on a shape unsing the GJK algorithm.
///
/// # Arguments:
/// * shape - the shape to project the origin on
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the shape boundary.
pub fn project_origin<P, M, S, G: ?Sized>(m: &M, shape: &G, simplex: &mut S) -> Option<P>
    where P: Point,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(proj.as_vec());

    let _eps: <P::Vect as Vect>::Scalar = FloatError::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<P>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return None // point inside of the cso
        }

        let support_point = shape.support_point(m, &-*proj.as_vec());

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
/// Projects the origin on a shape using the Separating Axis GJK algorithm.
/// The algorithm will stop as soon as the polytope can be proven to be at least `max_dist` away
/// from the origin.
///
/// # Arguments:
/// * shape - the shape to project the origin on
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the shape boundary.
pub fn project_origin_with_max_dist<P, M, S, G: ?Sized>(m:        &M,
                                                        shape:    &G,
                                                        max_dist: <P::Vect as Vect>::Scalar,
                                                        simplex:  &mut S)
                                                        -> GJKResult<P, P::Vect>
    where P: Point,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    // FIXME: reset the simplex if it is empty?
    let mut proj       = simplex.project_origin_and_reduce();
    let mut sq_len_dir = na::sqnorm(proj.as_vec());

    let _eps: <P::Vect as Vect>::Scalar = FloatError::epsilon();
    let _eps_tol = _eps * na::cast(100.0f64);
    let _eps_rel = _eps.sqrt();
    let _dim     = na::dim::<P>();

    loop {
        if simplex.dimension() == _dim || sq_len_dir <= _eps_tol /* * simplex.max_sq_len()*/ {
            return GJKResult::Intersection // point inside of the cso
        }

        let support_point = shape.support_point(m, &-*proj.as_vec());

        let dot = na::dot(proj.as_vec(), support_point.as_vec());

        // FIXME: find a way to avoid the sqrt here
        if dot > max_dist * na::norm(proj.as_vec()) {
            return GJKResult::NoIntersection(proj.to_vec());
        }

        if sq_len_dir - dot <= _eps_rel * sq_len_dir {
            return GJKResult::Projection(proj) // the distance found has a good enough precision
        }

        simplex.add_point(support_point);

        let old_proj = proj;

        proj = simplex.project_origin_and_reduce();

        let old_sq_len_dir = sq_len_dir;

        sq_len_dir = na::sqnorm(proj.as_vec());

        if sq_len_dir >= old_sq_len_dir {
            return GJKResult::Projection(old_proj) // upper bounds inconsistencies
        }
    }
}

/// Casts a ray on a support map using the GJK algorithm.
pub fn cast_ray<P, M, S, G: ?Sized>(m:       &M,
                                    shape:   &G,
                                    simplex: &mut S,
                                    ray:     &Ray<P>)
                                    -> Option<(<P::Vect as Vect>::Scalar, P::Vect)>
    where P: Point,
          M: Translate<P>,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    let mut ltoi: <P::Vect as Vect>::Scalar = na::zero();

    let _eps: <P::Vect as Vect>::Scalar     = FloatError::epsilon();
    let _eps_tol: <P::Vect as Vect>::Scalar = _eps * na::cast(100.0f64);
    let _dim                                = na::dim::<P>();

    // initialization
    let mut curr_ray   = Ray::new(ray.orig.clone(), ray.dir.clone());
    let mut dir        = m.inv_translate(&curr_ray.orig).as_vec().clone();

    if dir.is_zero() {
        dir[0] = na::one();
    }

    let mut old_sq_len: <P::Vect as Vect>::Scalar = Bounded::max_value();

    let mut ldir = dir.clone();
    // FIXME: this converges in more than 100 iterations… something is wrong here…
    let mut niter = 0usize;
    loop {
        niter = niter + 1;

        if dir.normalize_mut().is_zero() {
            return Some((ltoi, ldir))
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
                    let _max: <P::Vect as Vect>::Scalar = Bounded::max_value();
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
            return Some((ltoi, ldir))
        }
        else if sq_len_dir <= _eps_tol * simplex.max_sq_len() {
            // Return ldir: the last projection plane is tangeant to the intersected surface.
            return Some((ltoi, ldir))
        }
        else if sq_len_dir >= old_sq_len {
            // use dir instead of proj since this situations means that the new projection is less
            // accurate than the last one (which is stored on dir).
            return Some((ltoi, dir))
        }

        old_sq_len = sq_len_dir;
        dir        = -proj;
    }
}
