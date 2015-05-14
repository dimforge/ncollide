use na::{Rotate, Transform};
use na;
use entities::support_map::SupportMap;
use entities::shape::{Reflection, MinkowskiSum};
use ray::{LocalRayCast, Ray};
use math::{Scalar, Point, Vect};


/// Time of impacts between two support-mapped shapes under translational movement.
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, vel1: &P::Vect, g1: &G1,
                                                                     m2: &M, vel2: &P::Vect, g2: &G2)
                                                                     -> Option<<P::Vect as Vect>::Scalar>
    where P:  Point,
          M:  Rotate<P::Vect> + Transform<P>,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let vel = *vel1 - *vel2;
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_with_ray(&Ray::new(na::orig(), -vel), true)
}
