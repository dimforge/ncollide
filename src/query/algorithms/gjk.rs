//! The Gilbert–Johnson–Keerthi distance algorithm.

use num::Bounded;

use alga::general::Real;
use na::{self, Unit};

use shape::{ConstantOrigin, SupportMap};
use query::algorithms::{CSOPoint, VoronoiSimplex};
// use query::Proximity;
use query::{ray_internal, Ray};
use math::{DIM, Isometry, Point, Vector};

/// Results of the GJK algorithm.
#[derive(Clone, Debug, PartialEq)]
pub enum GJKResult<N: Real> {
    /// Result of the GJK algorithm when the origin is inside of the polytope.
    Intersection,
    /// Result of the GJK algorithm when a projection of the origin on the polytope is found.
    ClosestPoints(Point<N>, Point<N>, Unit<Vector<N>>),
    /// Result of the GJK algorithm when the origin is too close to the polytope but not inside of it.
    Proximity(Unit<Vector<N>>),
    /// Result of the GJK algorithm when the origin is too far away from the polytope.
    NoIntersection(Unit<Vector<N>>),
}

/// The absolute tolerence used by the GJK algorithm.
pub fn eps_tol<N: Real>() -> N {
    let _eps = N::default_epsilon();
    _eps * na::convert(100.0f64)
}

/// Projects the origin on the boundary of the given shape.
///
/// The origin is assumed to be outside of the shape. If it is inside,
/// use the EPA algorithm instead.
/// Return `None` if the origin is not inside of the shape or if
/// the EPA algorithm failed to compute the projection.
pub fn project_origin<N, G: ?Sized>(
    m: &Isometry<N>,
    g: &G,
    simplex: &mut VoronoiSimplex<N>,
) -> Option<Point<N>>
where
    N: Real,
    G: SupportMap<N>,
{
    match closest_points(
        m,
        g,
        &Isometry::identity(),
        &ConstantOrigin,
        N::max_value(),
        true,
        simplex,
    ) {
        GJKResult::Intersection => None,
        GJKResult::ClosestPoints(p, _, _) => Some(p),
        _ => unreachable!(),
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
/// * simplex - the simplex to be used by the GJK algorithm. It must be already initialized
///             with at least one point on the shape boundary.
/// * exact_dist - if `false`, the gjk will stop as soon as it can prove that the origin is at
/// a distance smaller than `max_dist` but not inside of `shape`. In that case, it returns a
/// `GJKResult::Proximity(sep_axis)` where `sep_axis` is a separating axis. If `false` the gjk will
/// compute the exact distance and return `GJKResult::Projection(point)` if the origin is closer
/// than `max_dist` but not inside `shape`.
pub fn closest_points<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    max_dist: N,
    exact_dist: bool,
    simplex: &mut VoronoiSimplex<N>,
) -> GJKResult<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let _eps = N::default_epsilon();
    let _eps_tol: N = eps_tol();
    let _eps_rel: N = _eps_tol.sqrt();

    fn result<N: Real>(simplex: &VoronoiSimplex<N>, prev: bool) -> (Point<N>, Point<N>) {
        let mut res = (Point::origin(), Point::origin());
        if prev {
            for i in 0..simplex.prev_dimension() + 1 {
                let coord = simplex.prev_proj_coord(i);
                let point = simplex.prev_point(i);
                res.0 += point.orig1.coords * coord;
                res.1 += point.orig2.coords * coord;
            }

            res
        } else {
            for i in 0..simplex.dimension() + 1 {
                let coord = simplex.proj_coord(i);
                let point = simplex.point(i);
                res.0 += point.orig1.coords * coord;
                res.1 += point.orig2.coords * coord;
            }

            res
        }
    }

    // FIXME: reset the simplex if it is empty?
    let mut proj = simplex.project_origin_and_reduce();
    let mut old_dir = -Unit::new_normalize(proj.coords);
    let mut max_bound = N::max_value();
    let mut dir;
    let mut niter = 0;

    loop {
        let old_max_bound = max_bound;

        if let Some((new_dir, dist)) = Unit::try_new_and_get(-proj.coords, _eps_tol) {
            dir = new_dir;
            max_bound = dist;
        } else {
            // The origin is on the simplex.
            return GJKResult::Intersection;
        }

        if max_bound >= old_max_bound {
            if exact_dist {
                let (p1, p2) = result(simplex, true);
                return GJKResult::ClosestPoints(p1, p2, old_dir); // upper bounds inconsistencies
            } else {
                return GJKResult::Proximity(old_dir);
            }
        }

        let cso_point = CSOPoint::from_shapes(m1, g1, m2, g2, &dir);
        let min_bound = -na::dot(dir.as_ref(), &cso_point.point.coords);

        assert!(min_bound == min_bound);

        if min_bound > max_dist {
            return GJKResult::NoIntersection(dir);
        } else if !exact_dist && min_bound > na::zero() {
            return GJKResult::Proximity(old_dir);
        } else if max_bound - min_bound <= _eps_rel * max_bound {
            if exact_dist {
                let (p1, p2) = result(simplex, false);
                return GJKResult::ClosestPoints(p1, p2, dir); // the distance found has a good enough precision
            } else {
                return GJKResult::Proximity(dir);
            }
        }

        if !simplex.add_point(cso_point) {
            if exact_dist {
                let (p1, p2) = result(simplex, false);
                return GJKResult::ClosestPoints(p1, p2, dir);
            } else {
                return GJKResult::Proximity(dir);
            }
        }

        old_dir = dir;
        proj = simplex.project_origin_and_reduce();

        if simplex.dimension() == DIM {
            if min_bound >= _eps_tol {
                if exact_dist {
                    let (p1, p2) = result(simplex, true);
                    return GJKResult::ClosestPoints(p1, p2, old_dir);
                } else {
                    // NOTE: previous implementation used old_proj here.
                    return GJKResult::Proximity(old_dir);
                }
            } else {
                return GJKResult::Intersection; // Point inside of the cso.
            }
        }
        niter += 1;
        if niter == 10000 {
            println!("Error: GJK did not converge.");
            return GJKResult::NoIntersection(Vector::x_axis());
        }
    }
}

/// Casts a ray on a support map using the GJK algorithm.
pub fn cast_ray<N, G: ?Sized>(
    m: &Isometry<N>,
    shape: &G,
    simplex: &mut VoronoiSimplex<N>,
    ray: &Ray<N>,
) -> Option<(N, Vector<N>)>
where
    N: Real,
    G: SupportMap<N>,
{
    let mut ltoi: N = na::zero();

    let _eps_tol = eps_tol();

    // FIXME: initialization if the simplex is empty?
    let proj = simplex.project_origin_and_reduce().coords;
    let mut curr_ray = *ray;
    let mut dir = -proj;
    let mut old_max_bound: N = Bounded::max_value();


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
                    simplex.reset(CSOPoint::single_point(Point::from_coordinates(-dir)));
                    let _max: N = Bounded::max_value();
                    old_max_bound = _max;
                    continue;
                }
            }
            None => {
                if na::dot(&dir, &ray.dir) >= -N::default_epsilon() {
                    // miss
                    return None;
                }
            }
        }

        if !simplex.add_point(CSOPoint::single_point(Point::from_coordinates(
            support_point - curr_ray.origin,
        ))) {
            return Some((ltoi, dir));
        }

        let proj = simplex.project_origin_and_reduce().coords;
        let max_bound = na::norm_squared(&proj);

        if simplex.dimension() == DIM {
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
