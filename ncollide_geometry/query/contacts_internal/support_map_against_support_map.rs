use num::Zero;

use alga::linear::{NormedSpace, Translation};
use na;
use shape::{self, SupportMap, AnnotatedPoint};
use query::algorithms::gjk::GJKResult;
use query::algorithms::gjk;
use query::algorithms::minkowski_sampling;
use query::algorithms::simplex::Simplex;
use query::algorithms::johnson_simplex::JohnsonSimplex;
use query::Contact;
use math::{Point, Isometry};


/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(
                                       m1:         &M,
                                       g1:         &G1,
                                       m2:         &M,
                                       g2:         &G2,
                                       prediction: P::Real)
                                       -> Option<Contact<P>>
    where P:  Point,
          M:  Isometry<P>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    match support_map_against_support_map_with_params(
        m1, g1, m2, g2, prediction, &mut JohnsonSimplex::new_w_tls(), None) {
        GJKResult::Projection(c)     => Some(c),
        GJKResult::NoIntersection(_) => None,
        GJKResult::Intersection      => unreachable!(),
        GJKResult::Proximity(_)      => unreachable!()
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<P, M, S, G1: ?Sized, G2: ?Sized>(
                                                   m1:         &M,
                                                   g1:         &G1,
                                                   m2:         &M,
                                                   g2:         &G2,
                                                   prediction: P::Real,
                                                   simplex:    &mut S,
                                                   init_dir:   Option<P::Vector>)
                                                   -> GJKResult<Contact<P>, P::Vector>
    where P:  Point,
          M:  Isometry<P>,
          S:  Simplex<AnnotatedPoint<P>>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let mut dir =
        match init_dir {
            // FIXME: or m2.translation - m1.translation ?
            None      => m1.translation().to_vector() - m2.translation().to_vector(),
            Some(dir) => dir
        };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(shape::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points_with_max_dist(m1, g1, m2, g2, prediction, simplex) {
        GJKResult::Projection((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = na::norm_squared(&p1p2);

            if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize_mut();

                return GJKResult::Projection(Contact::new(p1, p2, normal, -depth));
            }
        },
        GJKResult::NoIntersection(dir) => return GJKResult::NoIntersection(dir),
        GJKResult::Intersection        => { }, // fallback
        GJKResult::Proximity(_)        => unreachable!()
    }

    // The point is inside of the CSO: use the fallback algorithm
    match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2, normal)) => {
            let depth = na::dot(&(p1 - p2), &normal);

            GJKResult::Projection(Contact::new(p1, p2, normal, depth))
        }
        None => GJKResult::NoIntersection(na::zero()) // panic!("Both GJK and fallback algorithm failed.")
    }
}
