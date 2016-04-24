use std::mem;
use std::any::TypeId;
use math::{Point, Isometry};
use shape::{Compound, TriMesh, Polyline, CompositeShape};
use inspection::{Shape, ShapeDesc};

#[macro_export]
macro_rules! impl_composite_shape_desc(
    ($shape: ty) => {
        impl<P, M> Shape<P, M> for $shape
            where P: Point,
                  M: Isometry<P, P::Vect> {
                #[inline(always)]
                fn desc(&self) -> ShapeDesc<P, M> {
                    unsafe {
                        ShapeDesc::new(
                            TypeId::of::<$shape>(),
                            TypeId::of::<&CompositeShape<P, M>>(),
                            mem::transmute(self as &CompositeShape<P, M>)
                        )
                    }
                }
            }
    };
    ($shape: ty, $point: ty, $isometry: ty) => {
        impl Shape<$point, $isometry> for $shape {
                #[inline(always)]
                fn desc(&self) -> ShapeDesc<$point, $isometry> {
                    unsafe {
                        ShapeDesc::new(
                            TypeId::of::<$shape>(),
                            TypeId::of::<&CompositeShape<$point, $isometry>>(),
                            mem::transmute(self as &CompositeShape<$point, $isometry>)
                        )
                    }
                }
            }
    }
);

impl_composite_shape_desc!(Compound<P, M>);
impl_composite_shape_desc!(TriMesh<P>);
impl_composite_shape_desc!(Polyline<P>);
