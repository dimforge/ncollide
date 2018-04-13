//! The Gilbert–Johnson–Keerthi distance algorithm.

use num::{Bounded, Zero};
use approx::ApproxEq;

use alga::general::{Id, Real};
use alga::linear::NormedSpace;
use na::{self, Unit};

use shape::{AnnotatedMinkowskiSum, AnnotatedPoint, MinkowskiSum, Reflection, SupportMap};
use query::algorithms::simplex::Simplex;
use query::Proximity;
use query::{ray_internal, Ray};
use math::{Isometry, Point};

/// Results of the GJK algorithm.
#[derive(Clone, Debug)]
pub enum GJKResult<P, V> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    Projection(P, Unit<V>),
    /// Result of the GJK algorithm when the origin is to close to the polytope but not inside of it.
    Proximity(V),
    /// Result of the GJK algorithm when the origin is too far away from the polytope.
    NoIntersection(V),
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
pub fn closest_points<P, M, S, G1: ?Sized, G2: ?Sized>(
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    simplex: &mut S,
) -> Option<(P, P)>
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    project_origin(&Id::new(), &cso, simplex).map(|p| (*p.orig1(), -*p.orig2()))
}

/// Computes the closest points between two convex shapes unsing the GJK algorithm.
///
/// # Arguments:
/// * `g1`      - first shape.
/// * `g2`      - second shape.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the shapes CSO. See `minkowski_sum::cso_support_point`
///               to compute such point.
pub fn closest_points_with_max_dist<P, M, S, G1: ?Sized, G2: ?Sized>(
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    max_dist: P::Real,
    simplex: &mut S,
) -> GJKResult<(P, P), P::Vector>
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    match project_origin_with_max_dist(&Id::new(), &cso, max_dist, true, simplex) {
        GJKResult::Projection(p, n) => GJKResult::Projection((*p.orig1(), -*p.orig2()), n),
        GJKResult::Intersection => GJKResult::Intersection,
        GJKResult::NoIntersection(dir) => GJKResult::NoIntersection(dir),
        GJKResult::Proximity(_) => unreachable!(),
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
pub fn distance<P, M, S, G1: ?Sized, G2: ?Sized>(
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    simplex: &mut S,
) -> P::Real
where
    P: Point,
    S: Simplex<P>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &reflect2);

    match project_origin(&Id::new(), &cso, simplex) {
        Some(c) => na::norm(&c.coordinates()),
        None => na::zero(),
    }
}

/// Computes the closest points between two convex shapes unsing the GJK algorithm.
///
/// # Arguments:
/// * `g1`      - first shape.
/// * `g2`      - second shape.
/// * `simplex` - the simplex to be used by the GJK algorithm. It must be already initialized
///               with at least one point on the shapes CSO. See `minkowski_sum::cso_support_point`
///               to compute such point.
pub fn proximity<P, M, S, G1: ?Sized, G2: ?Sized>(
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    max_dist: P::Real,
    simplex: &mut S,
) -> (Proximity, P::Vector)
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    match project_origin_with_max_dist(&Id::new(), &cso, max_dist, false, simplex) {
        GJKResult::NoIntersection(data) => (Proximity::Disjoint, data),
        GJKResult::Proximity(data) => (Proximity::WithinMargin, data),
        GJKResult::Intersection => (Proximity::Intersecting, na::zero()),
        GJKResult::Projection(..) => unreachable!(),
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
where
    P: Point,
    S: Simplex<P>,
    G: SupportMap<P, M>,
{
    let _eps = P::Real::default_epsilon();
    let _eps_tol: P::Real = eps_tol();
    let _eps_rel: P::Real = _eps_tol.sqrt();
    let _dimension = na::dimension::<P::Vector>();

    // FIXME: reset the simplex if it is empty?
    let mut proj = simplex.project_origin_and_reduce();
    let mut old_proj = proj;
    let mut old_dir = Unit::new_normalize(proj.coordinates());
    let mut max_bound = P::Real::max_value();
    let mut dir;
    let mut niter = 0;

    loop {
        let old_max_bound = max_bound;

        if let Some((new_dir, dist)) = Unit::try_new_and_get(proj.coordinates(), _eps_tol) {
            dir = new_dir;
            max_bound = dist;
        } else {
            // The origin is on the simplex.
            return None;
        }

        if max_bound >= old_max_bound {
            return Some(old_proj);
        }

        let support_point = shape.support_point(m, &-dir);
        let min_bound = na::dot(dir.as_ref(), &support_point.coordinates());

        assert!(min_bound == min_bound);

        if max_bound - min_bound <= _eps_rel * max_bound {
            return Some(proj);
        }

        if !simplex.add_point(support_point) {
            return Some(proj);
        }

        old_proj = proj;
        old_dir = dir;
        proj = simplex.project_origin_and_reduce();

        if simplex.dimension() == _dimension {
            if min_bound >= _eps_tol {
                return Some(old_proj);
            } else {
                return None; // Point inside of the cso.
            }
        }
        niter += 1;
        if niter == 1000 {
            panic!("GJK did not converge.");
        }
    }
}

/// The absolute tolerence used by the GJK algorithm.
pub fn eps_tol<N: Real>() -> N {
    let _eps = N::default_epsilon();
    _eps * na::convert(100.0f64)
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
/// * exact_dist - if `false`, the gjk will stop as soon as it can prove that the origin is at
/// a distance smaller than `max_dist` but not inside of `shape`. In that case, it returns a
/// `GJKResult::Proximity(sep_axis)` where `sep_axis` is a separating axis. If `false` the gjk will
/// compute the exact distance and return `GJKResult::Projection(point)` if the origin is closer
/// than `max_dist` but not inside `shape`.
pub fn project_origin_with_max_dist<P, M, S, G: ?Sized>(
    m: &M,
    shape: &G,
    max_dist: P::Real,
    exact_dist: bool,
    simplex: &mut S,
) -> GJKResult<P, P::Vector>
where
    P: Point,
    S: Simplex<P>,
    G: SupportMap<P, M>,
{
    let _eps = P::Real::default_epsilon();
    let _eps_tol: P::Real = eps_tol();
    let _eps_rel: P::Real = _eps_tol.sqrt();
    let _dimension = na::dimension::<P::Vector>();

    // FIXME: reset the simplex if it is empty?
    let mut proj = simplex.project_origin_and_reduce();
    let mut old_proj = proj;
    let mut old_dir = Unit::new_normalize(proj.coordinates());
    let mut max_bound = P::Real::max_value();
    let mut dir;
    let mut niter = 0;

    loop {
        let old_max_bound = max_bound;

        if let Some((new_dir, dist)) = Unit::try_new_and_get(proj.coordinates(), _eps_tol) {
            dir = new_dir;
            max_bound = dist;
        } else {
            // The origin is on the simplex.
            return GJKResult::Intersection;
        }

        if max_bound >= old_max_bound {
            if exact_dist {
                return GJKResult::Projection(old_proj, old_dir); // upper bounds inconsistencies
            } else {
                return GJKResult::Proximity(old_proj.coordinates());
            }
        }

        let support_point = shape.support_point(m, &-dir);
        let min_bound = na::dot(dir.as_ref(), &support_point.coordinates());

        assert!(min_bound == min_bound);

        if min_bound > max_dist {
            return GJKResult::NoIntersection(proj.coordinates());
        } else if !exact_dist && min_bound > na::zero() {
            return GJKResult::Proximity(old_proj.coordinates());
        } else if max_bound - min_bound <= _eps_rel * max_bound {
            if exact_dist {
                return GJKResult::Projection(proj, dir); // the distance found has a good enough precision
            } else {
                return GJKResult::Proximity(proj.coordinates());
            }
        }

        if !simplex.add_point(support_point) {
            if exact_dist {
                return GJKResult::Projection(proj, dir);
            } else {
                return GJKResult::Proximity(proj.coordinates());
            }
        }

        old_proj = proj;
        old_dir = dir;
        proj = simplex.project_origin_and_reduce();

        if simplex.dimension() == _dimension {
            if min_bound >= _eps_tol {
                if exact_dist {
                    return GJKResult::Projection(old_proj, old_dir);
                } else {
                    return GJKResult::Proximity(old_proj.coordinates());
                }
            } else {
                return GJKResult::Intersection; // Point inside of the cso.
            }
        }
        niter += 1;
        if niter == 1000 {
            panic!("GJK did not converge.");
        }
    }
}

/// Casts a ray on a support map using the GJK algorithm.
pub fn cast_ray<P, M, S, G: ?Sized>(
    m: &M,
    shape: &G,
    simplex: &mut S,
    ray: &Ray<P>,
) -> Option<(P::Real, P::Vector)>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<P>,
    G: SupportMap<P, M>,
{
    let mut ltoi: P::Real = na::zero();

    let _eps_tol = eps_tol();
    let _dimension = na::dimension::<P::Vector>();

    // initialization
    let mut curr_ray = *ray;
    let mut dir = m.inverse_translate_point(&curr_ray.origin).coordinates();

    if dir == na::zero() {
        dir[0] = na::one();
    }

    let mut old_max_bound: P::Real = Bounded::max_value();

    let mut ldir = dir;
    // FIXME: this converges in more than 100 iterations… something is wrong here…
    let mut niter = 0usize;
    loop {
        niter = niter + 1;

        if dir.normalize_mut().is_zero() {
            return Some((ltoi, ldir));
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
        match ray_internal::plane_toi_with_ray(&support_point, &dir, &curr_ray) {
            Some(t) => {
                if na::dot(&dir, &ray.dir) < na::zero() && t > _eps_tol {
                    // new lower bound
                    ldir = dir;
                    ltoi = ltoi + t;
                    curr_ray.origin = ray.origin + ray.dir * ltoi;
                    dir = curr_ray.origin - support_point;
                    // FIXME: could we simply translate the simpex by old_origin - new_origin ?
                    simplex.reset(P::origin() + (-dir));
                    let _max: P::Real = Bounded::max_value();
                    old_max_bound = _max;
                    continue;
                }
            }
            None => {
                if na::dot(&dir, &ray.dir) > na::zero() {
                    // miss
                    return None;
                }
            }
        }

        if !simplex.add_point(P::origin() + (support_point - curr_ray.origin)) {
            return Some((ltoi, dir));
        }

        let proj = simplex.project_origin_and_reduce().coordinates();
        let max_bound = na::norm_squared(&proj);

        if simplex.dimension() == _dimension {
            return Some((ltoi, ldir));
        } else if max_bound <= _eps_tol * simplex.max_sq_len() {
            // Return ldir: the last projection plane is tangeant to the intersected surface.
            return Some((ltoi, ldir));
        } else if max_bound >= old_max_bound {
            // Use dir instead of proj since this situations means that the new projection is less
            // accurate than the last one (which is stored on dir).
            return Some((ltoi, dir));
        }

        old_max_bound = max_bound;
        dir = -proj;
    }
}
