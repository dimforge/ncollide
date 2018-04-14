use num::Zero;

use alga::linear::Translation;
use na::{self, Unit};
use shape::{self, AnnotatedPoint, SupportMap};
use query::algorithms::gjk::GJKResult;
use query::algorithms::gjk;
use query::algorithms::minkowski_sampling;
use query::algorithms::epa3;
use query::algorithms::epa2;
use query::algorithms::{EPA2, EPA3, JohnsonSimplex, Simplex, VoronoiSimplex2, VoronoiSimplex3};
use query::Contact;
use math::{Isometry, Point};

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<P>>
where
    N: Real,
    M: Isometry<P>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    if na::dimension::<Vector<N>>() == 2 {
        let simplex = &mut VoronoiSimplex2::new();
        match support_map_against_support_map_with_params(m1, g1, m2, g2, prediction, simplex, None)
        {
            GJKResult::Projection(c, _) => Some(c),
            GJKResult::NoIntersection(_) => None,
            GJKResult::Intersection => unreachable!(),
            GJKResult::Proximity(_) => unreachable!(),
        }
    } else if na::dimension::<Vector<N>>() == 3 {
        let simplex = &mut VoronoiSimplex3::new();
        match support_map_against_support_map_with_params(m1, g1, m2, g2, prediction, simplex, None)
        {
            GJKResult::Projection(c, _) => Some(c),
            GJKResult::NoIntersection(_) => None,
            GJKResult::Intersection => unreachable!(),
            GJKResult::Proximity(_) => unreachable!(),
        }
    } else {
        let simplex = &mut JohnsonSimplex::new_w_tls();
        match support_map_against_support_map_with_params(m1, g1, m2, g2, prediction, simplex, None)
        {
            GJKResult::Projection(c, _) => Some(c),
            GJKResult::NoIntersection(_) => None,
            GJKResult::Intersection => unreachable!(),
            GJKResult::Proximity(_) => unreachable!(),
        }
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<P, M, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut S,
    init_dir: Option<Vector<N>>,
) -> GJKResult<Contact<P>, Vector<N>>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let mut dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation().to_vector() - m2.translation().to_vector(),
        Some(dir) => dir,
    };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(shape::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points_with_max_dist(m1, g1, m2, g2, prediction, simplex) {
        GJKResult::Projection((p1, p2), n) => {
            let p1p2 = p2 - p1;
            let sqn = na::norm_squared(&p1p2);

            if !sqn.is_zero() {
                let (normal, depth) = Unit::new_and_get(p1p2);
                return GJKResult::Projection(Contact::new(p1, p2, normal, -depth), normal);
            }
        }
        GJKResult::NoIntersection(dir) => return GJKResult::NoIntersection(dir),
        GJKResult::Intersection => {} // fallback
        GJKResult::Proximity(_) => unreachable!(),
    }

    // The point is inside of the CSO: use the fallback algorithm
    if na::dimension::<Vector<N>>() == 2 {
        let mut epa = EPA2::new();
        if let Some((p1, p2, n)) = epa2::closest_points(&mut epa, m1, g1, m2, g2, simplex) {
            // XXX: if the depth is exactly zero, we should retrieve the normal by intersectiong the
            // inverse Gauss maps at p1 and p2.
            if let Some((normal, depth)) = Unit::try_new_and_get(p1 - p2, gjk::eps_tol()) {
                return GJKResult::Projection(Contact::new(p1, p2, normal, depth), normal);
            }
        }
    } else if na::dimension::<Vector<N>>() == 3 {
        let mut epa = EPA3::new();
        if let Some((p1, p2, n)) = epa3::closest_points(&mut epa, m1, g1, m2, g2, simplex) {
            // XXX: if the depth is exactly zero, we should retrieve the normal by intersectiong the
            // inverse Gauss maps at p1 and p2.
            if let Some((normal, depth)) = Unit::try_new_and_get(p1 - p2, gjk::eps_tol()) {
                return GJKResult::Projection(Contact::new(p1, p2, normal, depth), normal);
            }
        }
    }

    // When all else fail (e.g. because of roundup errors or for penetration in dimension
    // higher than 3), default to minkowski sampling.
    match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2, normal)) => {
            let depth = na::dot(&(p1 - p2), &normal);
            GJKResult::Projection(Contact::new(p1, p2, normal, depth), normal)
        }
        None => GJKResult::NoIntersection(na::zero()), // panic!("Both GJK and fallback algorithm failed.")
    }
}
