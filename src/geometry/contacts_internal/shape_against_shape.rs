use std::intrinsics::TypeId;
use std::any::AnyRefExt;
use std::default::Default;
use na::{Translate, Rotation, Cross};
use geometry::{Contact, contact, contacts};
use shape::{Shape, Ball, Plane, Cuboid, Capsule, Cone, Cylinder, Convex, Compound, Mesh, Segment,
            Triangle};
use math::{Scalar, Point, Vect, Isometry, HasInertiaMatrix};


macro_rules! dispatch_part(
    ($fname2: ident,
     $m1: ident, $g1: ident, $id1: ident,
     $m2: ident, $g2: ident, $id2: ident
     $(, $args: ident)*
     | $tg1: ty, $tg2: ty) => {
        if $id1 == TypeId::of::<$tg1>() && $id2 == TypeId::of::<$tg2>() {
            let exact_g1: &$tg1 = $g1.downcast_ref::<$tg1>().unwrap();
            let exact_g2: &$tg2 = $g2.downcast_ref::<$tg2>().unwrap();

            return $fname2($m1, exact_g1, $m2, exact_g2 $(, $args)*);
        }
    }
)

macro_rules! dispatch(
    ($dispatcher_name: ident, $fname: ident $(, $args: ident: $types: ty)* -> $result: ty) => {
        /// FIXME
        #[inline]
        pub fn $dispatcher_name<N, P, V, AV, M, I>(m1: &M, g1: &Shape<N, P, V, M>,
                                                   m2: &M, g2: &Shape<N, P, V, M>
                                                   $(, $args: $types)*)
                                                   -> $result
            where N:  Scalar,
                  P:  Point<N, V>,
                  V:  Vect<N> + Translate<P> + HasInertiaMatrix<I> + Cross<AV>,
                  AV: Vect<N>,
                  M:  Isometry<N, P, V> + Rotation<AV>,
                  I:  Send + Sync + Clone {
            let tg1 = g1.get_type_id();
            let tg2 = g2.get_type_id();

            // FIXME: use a hash-map instead of if-elses ?


            apply_to_all_shape_pair!(dispatch_part, $fname, m1, g1, tg1, m2, g2, tg2$(, $args)*)

            Default::default()
        }
    }
)

dispatch!(shape_against_shape, contact, prediction: N -> Option<Contact<N, P, V>>)
dispatch!(manifold_shape_against_shape, contacts, prediction: N, out: &mut Vec<Contact<N, P, V>> -> ())
