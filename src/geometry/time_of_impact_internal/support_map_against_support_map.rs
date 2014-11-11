use na::{Rotate, Transform};
use na;
use support_map::SupportMap;
use shape::{Reflection, MinkowskiSum};
use ray::{LocalRayCast, Ray};
use math::{Scalar, Point, Vect};


/// Time of impacts between two support-mapped shapes under translational movement.
pub fn support_map_against_support_map<N, P, V, M, G1, G2>(m1: &M, dir1: &V, g1: &G1,
                                                           m2: &M, dir2: &V, g2: &G2)
                                                           -> Option<N>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Rotate<V> + Transform<P>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    let dir = *dir1 - *dir2;
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_with_ray(&Ray::new(na::orig(), -dir), true)
}
