use math::{Isometry, Vector};
use na::{self, Real, Unit};
use query::Contact;
use query::algorithms::{gjk, CSOPoint, gjk::GJKResult};
use query::algorithms::{Simplex, VoronoiSimplex, EPA};
use shape::SupportMap;

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<N>>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let simplex = &mut VoronoiSimplex::new();
    match support_map_against_support_map_with_params(m1, g1, m2, g2, prediction, simplex, None) {
        GJKResult::ClosestPoints(world1, world2, dir21) => {
            Some(Contact::new_wo_depth(world1, world2, -dir21))
        }
        GJKResult::NoIntersection(_) => None,
        GJKResult::Intersection => unreachable!(),
        GJKResult::Proximity(_) => unreachable!(),
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut S,
    init_dir: Option<Vector<N>>,
) -> GJKResult<N>
where
    N: Real,
    S: Simplex<N>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation.vector - m2.translation.vector,
        Some(dir) => dir,
    };

    if let Some(dir) = Unit::try_new(dir, N::default_epsilon()) {
        simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &Vector::x_axis()));
    }

    let cpts = gjk::closest_points(m1, g1, m2, g2, prediction, true, simplex);
    if cpts != GJKResult::Intersection {
        println!("GJK found closest.");
        return cpts;
    }

    // The point is inside of the CSO: use the fallback algorithm
    let mut epa = EPA::new();
    if let Some((p1, p2, n)) = epa.closest_points(m1, g1, m2, g2, simplex) {
        println!("EPA found closest.");
        return GJKResult::ClosestPoints(p1, p2, -n);
    }

    // Everything failed
    GJKResult::NoIntersection(na::zero())

    // // When all else fail (e.g. because of roundup errors, default to minkowski sampling.
    // match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
    //     Some((p1, p2, normal)) => {
    //         let depth = na::dot(&(p1 - p2), &normal);
    //         GJKResult::Projection(Contact::new(p1, p2, normal, depth), normal)
    //     }
    //     None => GJKResult::NoIntersection(na::zero()), // panic!("Both GJK and fallback algorithm failed.")
    // }
}
