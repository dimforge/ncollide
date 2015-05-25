use num::Zero;
use na::{Translation, Translate, Norm};
use na;
use entities::support_map::SupportMap;
use entities::support_map;
use entities::shape::AnnotatedPoint;
use geometry::algorithms::gjk::GJKResult;
use geometry::algorithms::gjk;
use geometry::algorithms::minkowski_sampling;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use geometry::Contact;
use math::{Scalar, Point, Vect};


/// Contact between support-mapped shapes (`Cuboid`, `Convex`, etc.)
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(
                                       m1:         &M,
                                       g1:         &G1,
                                       m2:         &M,
                                       g2:         &G2,
                                       prediction: <P::Vect as Vect>::Scalar)
                                       -> Option<Contact<P>>
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Translation<P::Vect>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    match support_map_against_support_map_with_params(
        m1, g1, m2, g2, prediction, &mut JohnsonSimplex::new_w_tls(), None) {
        GJKResult::Projection(c)     => Some(c),
        GJKResult::NoIntersection(_) => None,
        GJKResult::Intersection      => unreachable!()
    }
}

/// Contact between support-mapped shapes (`Cuboid`, `Convex`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<P, M, S, G1: ?Sized, G2: ?Sized>(
                                                   m1:         &M,
                                                   g1:         &G1,
                                                   m2:         &M,
                                                   g2:         &G2,
                                                   prediction: <P::Vect as Vect>::Scalar,
                                                   simplex:    &mut S,
                                                   init_dir:   Option<P::Vect>)
                                                   -> GJKResult<Contact<P>, P::Vect>
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Translation<P::Vect>,
          S:  Simplex<AnnotatedPoint<P>>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let mut dir =
        match init_dir {
            None      => m1.translation() - m2.translation(), // FIXME: or m2.translation - m1.translation ?
            Some(dir) => dir
        };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(support_map::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points_with_max_dist(m1, g1, m2, g2, prediction, simplex) {
        GJKResult::Projection((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = na::sqnorm(&p1p2);

            if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize_mut();

                return GJKResult::Projection(Contact::new(p1, p2, normal, -depth));
            }
        },
        GJKResult::NoIntersection(dir) => return GJKResult::NoIntersection(dir),
        GJKResult::Intersection        => { } // fallback
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
