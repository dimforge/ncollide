//! The Gilbert–Johnson–Keerthi distance algorithm.


use alga::general::RealField;
use na::{self, Unit};

use crate::query::algorithms::{CSOPoint, VoronoiSimplex};
use crate::shape::{ConstantOrigin, SupportMap};
// use query::Proximity;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::query::{ray_internal, Ray};

/// Results of the GJK algorithm.
#[derive(Clone, Debug, PartialEq)]
pub enum GJKResult<N: RealField> {
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
pub fn eps_tol<N: RealField>() -> N {
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
    N: RealField,
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
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let _eps = N::default_epsilon();
    let _eps_tol: N = eps_tol();
    let _eps_rel: N = _eps_tol.sqrt();

    fn result<N: RealField>(simplex: &VoronoiSimplex<N>, prev: bool) -> (Point<N>, Point<N>) {
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
        let min_bound = -dir.dot(&cso_point.point.coords);

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
//            println!("Error: GJK did not converge.");
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
    N: RealField,
    G: SupportMap<N>,
{
    let m2 = Isometry::identity();
    let g2 = ConstantOrigin;
    minkowski_ray_cast(m, shape, &m2, &g2, ray, simplex)
}

/// Compute the distance that can travel `g1` along the direction `dir` so that
/// `g1` and `g2` just touch.
pub fn directional_distance<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    dir: &Vector<N>,
    simplex: &mut VoronoiSimplex<N>,
) -> Option<N>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    directional_distance_and_normal(m1, g1, m2, g2, dir, simplex).map(|res| res.0)
}

/// Compute the normal and the distance that can travel `g1` along the direction
/// `dir` so that `g1` and `g2` just touch.
pub fn directional_distance_and_normal<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    dir: &Vector<N>,
    simplex: &mut VoronoiSimplex<N>,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let ray = Ray::new(Point::origin(), *dir);
    minkowski_ray_cast(m1, g1, m2, g2, &ray, simplex)
}

// Ray-cast on the Minkowski Difference `m1 * g1 - m2 * g2`.
fn minkowski_ray_cast<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    ray: &Ray<N>,
    simplex: &mut VoronoiSimplex<N>,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let mut ltoi: N = na::zero();
    let mut old_max_bound = N::max_value();
    let _eps_tol = eps_tol();

    if relative_eq!(ray.dir.norm_squared(), N::zero()) {
        return None;
    }

    let mut curr_ray = *ray;
    let mut dir = -ray.dir;
    let mut ldir = dir;
    let mut simplex_init = false;

    loop {
        let mut ray_advanced = false;

        if dir.normalize_mut().is_zero() {
            return Some((ltoi, ldir));
        }

        let support_point = CSOPoint::from_shapes(m1, g1, m2, g2, &dir);

        // Clip the ray on the support plane (None <=> t < 0)
        // The configurations are:
        //   dir.dot(curr_ray.dir)  |   t   |               Action
        // −−−−−−−−−−−−−−−−−−−−+−−−−−−−+−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
        //          < 0        |  < 0  | Continue.
        //          < 0        |  > 0  | New lower bound, move the origin.
        //          > 0        |  < 0  | Miss. No intersection.
        //          > 0        |  > 0  | New higher bound.
        match ray_internal::plane_toi_with_ray(&support_point.point, &dir, &curr_ray) {
            Some(t) => {
                if dir.dot(&curr_ray.dir) < na::zero() && t > N::zero() {
                    // new lower bound
                    ldir = dir;
                    ltoi += t;
                    let shift = curr_ray.dir * t;
                    curr_ray.origin = curr_ray.origin + shift;
                    // FIXME: could we simply translate the simplex by old_origin - new_origin ?
                    simplex.modify_pnts(&|pt| pt.translate1_mut(&-shift));
                    ray_advanced = true;
                }
            }
            None => {
                if dir.dot(&curr_ray.dir) > N::default_epsilon() {
                    // miss
                    return None;
                }
            }
        }

        if !simplex_init {
            simplex.reset(support_point.translate1(&-curr_ray.origin.coords));
            simplex_init = true;
        } else if !simplex.add_point(support_point.translate1(&-curr_ray.origin.coords)) && !ray_advanced {
            return Some((ltoi, ldir));
        }

        let proj = simplex.project_origin_and_reduce().coords;
        let max_bound = proj.norm_squared();

        if simplex.dimension() == DIM {
            return Some((ltoi, ldir));
        } else if max_bound <= _eps_tol || max_bound <= _eps_tol * simplex.max_sq_len() {
            // FIXME: we use the same tolerance for absolute and relative epsilons. This could be improved.
            // Return ldir: the last projection plane is tangent to the intersected surface.
            return Some((ltoi, ldir));
        } else if max_bound >= old_max_bound {
            if max_bound <= old_max_bound + _eps_tol {
                return Some((ltoi, ldir))
            } else {
                return None;
            }
        }

        old_max_bound = max_bound;
        dir = -proj;
    }
}
