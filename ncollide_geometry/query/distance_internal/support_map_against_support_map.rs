use num::Zero;

use alga::linear::Translation;
use na;
use query::algorithms::gjk;
use query::algorithms::simplex::Simplex;
use query::algorithms::johnson_simplex::JohnsonSimplex;
use shape::{self, SupportMap};
use math::{Point, Isometry};


/// Distance between support-mapped shapes.
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1,
                                                                     m2: &M, g2: &G2)
                                                                     -> P::Real
    where P:  Point,
          M:  Isometry<P>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    support_map_against_support_map_with_params(m1, g1, m2, g2, &mut JohnsonSimplex::new_w_tls(), None)
}

/// Distance between support-mapped shapes.
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<P, M, S, G1: ?Sized, G2: ?Sized>(
                                                   m1:       &M,
                                                   g1:       &G1,
                                                   m2:       &M,
                                                   g2:       &G2,
                                                   simplex:  &mut S,
                                                   init_dir: Option<P::Vector>)
                                                   -> P::Real
    where P:  Point,
          M:  Isometry<P>,
          S:  Simplex<P>,
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

    simplex.reset(*shape::cso_support_point(m1, g1, m2, g2, dir).point());

    gjk::distance(m1, g1, m2, g2, simplex)
}
