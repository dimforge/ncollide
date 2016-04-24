use std::mem;
use std::any::TypeId;
use math::{Point, Vector, Isometry};
use shape::{Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, Triangle};
use support_map::SupportMap;
use inspection::{Shape, ShapeDesc};

#[macro_export]
macro_rules! impl_support_map_desc(
    ($t: ty) => {
        impl<P, M> Shape<P, M> for $t
            where P: Point,
                  M: Isometry<P, P::Vect> {
                #[inline(always)]
                fn desc(&self) -> ShapeDesc<P, M> {
                    unsafe {
                        ShapeDesc::new(
                            TypeId::of::<$t>(),
                            TypeId::of::<SupportMap<P, M>>(),
                            mem::transmute(self as &SupportMap<P, M>)
                        )
                    }
                }
            }
    }
);

impl_support_map_desc!(Ball<<P::Vect as Vector>::Scalar>);
impl_support_map_desc!(Capsule<<P::Vect as Vector>::Scalar>);
impl_support_map_desc!(Cone<<P::Vect as Vector>::Scalar>);
impl_support_map_desc!(ConvexHull<P>);
impl_support_map_desc!(Cuboid<P::Vect>);
impl_support_map_desc!(Cylinder<<P::Vect as Vector>::Scalar>);
impl_support_map_desc!(Segment<P>);
impl_support_map_desc!(Triangle<P>);
