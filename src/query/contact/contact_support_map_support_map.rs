use crate::math::{Isometry, Vector};
use na::{RealField, Unit};
use crate::query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use crate::query::algorithms::{VoronoiSimplex, EPA};
use crate::query::Contact;
use crate::shape::SupportMap;

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn contact_support_map_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<N>>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let simplex = &mut VoronoiSimplex::new();
    match contact_support_map_support_map_with_params(m1, g1, m2, g2, prediction, simplex, None) {
        GJKResult::ClosestPoints(world1, world2, normal) => {
            Some(Contact::new_wo_depth(world1, world2, normal))
        }
        GJKResult::NoIntersection(_) => None,
        GJKResult::Intersection => unreachable!(),
        GJKResult::Proximity(_) => unreachable!(),
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
/// The vector-typed result is the vector that should be passed as `init` for
/// subsequent executions of the algorithm. It is also the contact
/// normal (that points toward the outside of the first solid).
pub fn contact_support_map_support_map_with_params<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut VoronoiSimplex<N>,
    init_dir: Option<Unit<Vector<N>>>,
) -> GJKResult<N>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dir = if let Some(init_dir) = init_dir {
        init_dir
    } else if let Some(init_dir) = Unit::try_new(
        m2.translation.vector - m1.translation.vector,
        N::default_epsilon(),
    ) {
        init_dir
    } else {
        Vector::x_axis()
    };

    simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &dir));

    let cpts = gjk::closest_points(m1, g1, m2, g2, prediction, true, simplex);
    if cpts != GJKResult::Intersection {
        return cpts;
    }

    // The point is inside of the CSO: use the fallback algorithm
    let mut epa = EPA::new();
    if let Some((p1, p2, n)) = epa.closest_points(m1, g1, m2, g2, simplex) {
        // FIXME: the n here,
        return GJKResult::ClosestPoints(p1, p2, n);
    }

    // Everything failed
    GJKResult::NoIntersection(Vector::x_axis())

    // // When all else fail (e.g. because of roundup errors, default to minkowski sampling.
    // match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
    //     Some((p1, p2, normal)) => {
    //         let depth = na::dot(&(p1 - p2), &normal);
    //         GJKResult::Projection(Contact::new(p1, p2, normal, depth), normal)
    //     }
    //     None => GJKResult::NoIntersection(na::zero()), // panic!("Both GJK and fallback algorithm failed.")
    // }
}
