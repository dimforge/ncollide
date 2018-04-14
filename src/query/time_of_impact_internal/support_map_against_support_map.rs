use alga::general::Id;
use shape::{MinkowskiSum, Reflection, SupportMap};
use query::{Ray, RayCast};
use math::{Isometry, Point};

/// Time of impacts between two support-mapped shapes under translational movement.
pub fn support_map_against_support_map<P, M, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: Real,
    M: Isometry<P>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let vel = *vel1 - *vel2;
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_with_ray(&Isometry::identity(), &Ray::new(Point::origin(), -vel), true)
}
