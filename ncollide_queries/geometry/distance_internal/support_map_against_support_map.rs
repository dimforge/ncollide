use na::{Translation, Translate};
use na;
use geometry::algorithms::gjk;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use entities::support_map::SupportMap;
use entities::support_map;
use math::{Scalar, Point, Vect};


/// Distance between support-mapped shapes.
pub fn support_map_against_support_map<N, P, V, M, Sized ? G1, Sized ? G2>(m1: &M, g1: &G1,
                                                                           m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Translation<V>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    support_map_against_support_map_with_params(m1, g1, m2, g2, &mut JohnsonSimplex::new_w_tls(), None)
}

/// Distance between support-mapped shapes.
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, P, V, M, S, Sized? G1, Sized? G2>(
                                                   m1:         &M,
                                                   g1:         &G1,
                                                   m2:         &M,
                                                   g2:         &G2,
                                                   simplex:    &mut S,
                                                   init_dir:   Option<V>)
                                                   -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Translation<V>,
          S:  Simplex<N, P>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    let mut dir =
        match init_dir {
            None      => m1.translation() - m2.translation(), // FIXME: or m2.translation - m1.translation ?
            Some(dir) => dir
        };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(support_map::cso_support_point(m1, g1, m2, g2, dir).point().clone());

    gjk::distance(m1, g1, m2, g2, simplex)
}
