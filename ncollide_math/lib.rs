//! Trait implemented by the types used by ncollide.

extern crate "nalgebra" as na;

use std::rand::Rand;
use std::fmt::Show;
use std::ops::{IndexMut, Mul};
use std::num::FromPrimitive;
use na::{Pnt1, Pnt2, Pnt3, Pnt4, Vec1, Vec2, Vec3, Vec4, Mat1, Mat3, Iso2, Iso3, Iso4, Identity};
use na::{ApproxEq, Cast, POrd, FloatVec, Translate, UniformSphereSample, Translation,
         Rotate, Transform, AbsoluteRotate, Inv, ScalarSub, ScalarAdd, ScalarMul, ScalarDiv,
         FloatPnt, Shape, Absolute, Iterable, BaseFloat, Bounded, One};

/// Trait implemented by scalar types.
pub trait Scalar: Copy + Send + Sync + Show +
                  BaseFloat + FromPrimitive + ApproxEq<Self> + Cast<f64> + Rand + Bounded {
}

/// Trait implemented by point types.
pub trait Point<N, V>: Send         + Sync              + FloatPnt<N, V> +
                       POrd         + Bounded           + ScalarSub<N> +
                       ScalarAdd<N> + ScalarMul<N>      + ScalarDiv<N> +
                       IndexMut<uint, Output = N> + Clone + Copy + Show {
}


/// Trait implemented by vector types.
pub trait Vect<N>: Send                + Sync  + FloatVec<N> +
                   UniformSphereSample + Clone + IndexMut<uint, Output = N> +
                   Rand                + Shape<uint> + POrd +
                   Absolute<Self>      + Iterable<N> + Copy + Show {
}

/// Trait implemented by transformation matrices types.
pub trait Isometry<N, P, V>: Send           + Sync              + One          +
                             Translation<V> + Rotate<V>         + Translate<P> +
                             Transform<P>   + AbsoluteRotate<V> + Inv          +
                             Clone + Mul<Self, Output = Self> + Copy + Show {
}

/// Trait implement by vectors that are transformable by the inertia matrix `I`.
pub trait HasInertiaMatrix<I> { }

impl Scalar for f32 { }
impl Scalar for f64 { }

impl<N: Scalar> Point<N, Vec1<N>> for Pnt1<N> { }
impl<N: Scalar> Point<N, Vec2<N>> for Pnt2<N> { }
impl<N: Scalar> Point<N, Vec3<N>> for Pnt3<N> { }
impl<N: Scalar> Point<N, Vec4<N>> for Pnt4<N> { }

impl<N: Scalar> Vect<N> for Vec1<N> { }
impl<N: Scalar> Vect<N> for Vec2<N> { }
impl<N: Scalar> Vect<N> for Vec3<N> { }
impl<N: Scalar> Vect<N> for Vec4<N> { }

impl<N: Scalar> Isometry<N, Pnt2<N>, Vec2<N>> for Iso2<N> { }
impl<N: Scalar> Isometry<N, Pnt3<N>, Vec3<N>> for Iso3<N> { }
impl<N: Scalar> Isometry<N, Pnt4<N>, Vec4<N>> for Iso4<N> { }

impl<N: Scalar> Isometry<N, Pnt2<N>, Vec2<N>> for Identity { }
impl<N: Scalar> Isometry<N, Pnt3<N>, Vec3<N>> for Identity { }
impl<N: Scalar> Isometry<N, Pnt4<N>, Vec4<N>> for Identity { }

impl<N> HasInertiaMatrix<Mat1<N>> for Vec2<N> { }
impl<N> HasInertiaMatrix<Mat3<N>> for Vec3<N> { }
