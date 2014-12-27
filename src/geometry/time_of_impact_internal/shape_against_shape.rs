use std::intrinsics::TypeId;
use std::any::AnyRefExt;
use na::Translate;
use geometry::time_of_impact;
use shape::{Shape, Ball, Plane, Cuboid, Capsule, Cone, Cylinder, Convex, Compound, Mesh, Segment,
            Triangle};
use math::{Scalar, Point, Vect, Isometry};


macro_rules! dispatch_part(
    ($fname2: ident,
     $m1: ident, $d1: ident, $g1: ident, $id1: ident,
     $m2: ident, $d2: ident, $g2: ident, $id2: ident
     | $tg1: ty, $tg2: ty) => {
        if $id1 == TypeId::of::<$tg1>() && $id2 == TypeId::of::<$tg2>() {
            let exact_g1: &$tg1 = $g1.downcast_ref::<$tg1>().unwrap();
            let exact_g2: &$tg2 = $g2.downcast_ref::<$tg2>().unwrap();

            return $fname2($m1, $d1, exact_g1, $m2, $d2, exact_g2);
        }
    }
);

/// Time of impacts between two shapes (trait objects) under translational movement.
#[inline]
pub fn shape_against_shape<N, P, V, M>(m1: &M, vel1: &V, g1: &Shape<N, P, V, M>,
                                       m2: &M, vel2: &V, g2: &Shape<N, P, V, M>)
                                       -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V> {
    let tg1 = g1.get_type_id();
    let tg2 = g2.get_type_id();

    // FIXME: use a hash-map instead of if-elses ?
    apply_to_all_shape_pair!(dispatch_part, time_of_impact, m1, vel1, g1, tg1, m2, vel2, g2, tg2);

    None
}
