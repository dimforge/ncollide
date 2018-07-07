use math::{Isometry, Vector};
use na::{Real, Unit};
use query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use query::algorithms::{VoronoiSimplex, EPA};
use query::Contact;
use shape::SupportMap;

fn initial_cso_dir<N: Real>(m12: &Isometry<N>) -> Unit<Vector<N>> {
    if let Some(dir) = Unit::try_new(-m12.translation.vector, N::default_epsilon()) {
        dir
    } else {
        Vector::x_axis()
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.), in the local-space of `g1`.
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    g1: &G1,
    m12: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<N>>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let mut simplex = VoronoiSimplex::new();
    match support_map_against_support_map_with_simplex(g1, m12, g2, prediction, &mut simplex) {
        GJKResult::ClosestPoints(local1, local1_2, normal1) => {
            Some(Contact::new_wo_depth(local1, local1_2, normal1))
        }
        GJKResult::NoIntersection(_) => None,
        GJKResult::Intersection => unreachable!(),
        GJKResult::Proximity(_) => unreachable!(),
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// All computations are performed on the local-space of `g1`.
/// This allows a more fine grained control other the underlying GJK algorigtm.
/// The vector-typed result is the vector that should be passed as `init` for
/// subsequent executions of the algorithm. It is also the contact
/// normal (that points toward the outside of the first solid).
pub fn support_map_against_support_map_with_dir<N, G1: ?Sized, G2: ?Sized>(
    g1: &G1,
    m12: &Isometry<N>,
    g2: &G2,
    prediction: N,
    init_dir: &Unit<Vector<N>>,
) -> GJKResult<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let mut simplex = VoronoiSimplex::new();
    simplex.reset(CSOPoint::from_shapes_local1(g1, m12, g2, &init_dir));
    support_map_against_support_map_with_simplex(g1, m12, g2, prediction, &mut simplex)
}
/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// All computations are performed on the local-space of `g1`.
/// This allows a more fine grained control other the underlying GJK algorigtm.
/// Here, the provided simplex is used as-is by the GJK algorithm. Useful for warm-starting.
/// The vector-typed result is the vector that should be passed as `init` for
/// subsequent executions of the algorithm. It is also the contact
/// normal (that points toward the outside of the first solid).
pub fn support_map_against_support_map_with_simplex<N, G1: ?Sized, G2: ?Sized>(
    g1: &G1,
    m12: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut VoronoiSimplex<N>,
) -> GJKResult<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    if !simplex.initialized() {
        let init_dir = initial_cso_dir(m12);
        simplex.reset(CSOPoint::from_shapes_local1(g1, m12, g2, &init_dir));
    }

    let cpts = gjk::closest_points(g1, m12, g2, prediction, true, simplex);
    if cpts != GJKResult::Intersection {
        return cpts;
    }

    // The point is inside of the CSO: use the fallback algorithm
    let mut epa = EPA::new();
    if let Some((p1, p2, n)) = epa.closest_points(g1, m12, g2, simplex) {
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
