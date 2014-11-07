//! Trait implemented by the types used by ncollide.

use std::num::{Bounded, One};
use std::rand::Rand;
use na::overload::{Pnt4MulRhs, Pnt4DivRhs, Vec4MulRhs, Vec4DivRhs,
                   Pnt4AddRhs, Pnt4SubRhs, Vec4AddRhs, Vec4SubRhs,
                   Pnt3MulRhs, Pnt3DivRhs, Vec3MulRhs, Vec3DivRhs,
                   Pnt3AddRhs, Pnt3SubRhs, Vec3AddRhs, Vec3SubRhs,
                   Pnt2MulRhs, Pnt2DivRhs, Vec2MulRhs, Vec2DivRhs,
                   Pnt2AddRhs, Pnt2SubRhs, Vec2AddRhs, Vec2SubRhs,
                   Pnt1MulRhs, Pnt1DivRhs, Vec1MulRhs, Vec1DivRhs,
                   Pnt1AddRhs, Pnt1SubRhs, Vec1AddRhs, Vec1SubRhs,
                   Mat4MulRhs, Mat4DivRhs, Mat4AddRhs, Mat4SubRhs,
                   Mat3MulRhs, Mat3DivRhs, Mat3AddRhs, Mat3SubRhs,
                   Mat2MulRhs, Mat2DivRhs, Mat2AddRhs, Mat2SubRhs};
use na::{Pnt1, Pnt2, Pnt3, Pnt4, Vec1, Vec2, Vec3, Vec4, Mat2, Mat1, Mat3, Mat4, Iso2, Iso3, Iso4, Identity};
use na::{ApproxEq, Cast, POrd, FloatVec, Translate, UniformSphereSample, Translation,
         Rotate, Transform, AbsoluteRotate, Inv, ScalarSub, ScalarAdd, ScalarMul, ScalarDiv,
         FloatPnt, Shape, Absolute, Iterable};

/// Trait implemented by scalar types.
pub trait Scalar: Send + Sync + FloatMath + FromPrimitive + ApproxEq<Self> + Cast<f64> + Rand +
                  Pnt4MulRhs<Self, Pnt4<Self>> + Pnt4DivRhs<Self, Pnt4<Self>> +
                  Pnt4AddRhs<Self, Pnt4<Self>> + Pnt4SubRhs<Self, Pnt4<Self>> +
                  Pnt3MulRhs<Self, Pnt3<Self>> + Pnt3DivRhs<Self, Pnt3<Self>> +
                  Pnt3AddRhs<Self, Pnt3<Self>> + Pnt3SubRhs<Self, Pnt3<Self>> +
                  Pnt2MulRhs<Self, Pnt2<Self>> + Pnt2DivRhs<Self, Pnt2<Self>> +
                  Pnt2AddRhs<Self, Pnt2<Self>> + Pnt2SubRhs<Self, Pnt2<Self>> +
                  Pnt1MulRhs<Self, Pnt1<Self>> + Pnt1DivRhs<Self, Pnt1<Self>> +
                  Pnt1AddRhs<Self, Pnt1<Self>> + Pnt1SubRhs<Self, Pnt1<Self>> +
                  Vec4MulRhs<Self, Vec4<Self>> + Vec4DivRhs<Self, Vec4<Self>> +
                  Vec4AddRhs<Self, Vec4<Self>> + Vec4SubRhs<Self, Vec4<Self>> +
                  Vec3MulRhs<Self, Vec3<Self>> + Vec3DivRhs<Self, Vec3<Self>> +
                  Vec3AddRhs<Self, Vec3<Self>> + Vec3SubRhs<Self, Vec3<Self>> +
                  Vec2MulRhs<Self, Vec2<Self>> + Vec2DivRhs<Self, Vec2<Self>> +
                  Vec2AddRhs<Self, Vec2<Self>> + Vec2SubRhs<Self, Vec2<Self>> +
                  Vec1MulRhs<Self, Vec1<Self>> + Vec1DivRhs<Self, Vec1<Self>> +
                  Vec1AddRhs<Self, Vec1<Self>> + Vec1SubRhs<Self, Vec1<Self>> +
                  Mat4MulRhs<Self, Mat4<Self>> + Mat4DivRhs<Self, Mat4<Self>> +
                  Mat4AddRhs<Self, Mat4<Self>> + Mat4SubRhs<Self, Mat4<Self>> +
                  Mat3MulRhs<Self, Mat3<Self>> + Mat3DivRhs<Self, Mat3<Self>> +
                  Mat3AddRhs<Self, Mat3<Self>> + Mat3SubRhs<Self, Mat3<Self>> +
                  Mat2MulRhs<Self, Mat2<Self>> + Mat2DivRhs<Self, Mat2<Self>> +
                  Mat2AddRhs<Self, Mat2<Self>> + Mat2SubRhs<Self, Mat2<Self>> {
}

/// Trait implemented by point types.
pub trait Point<N, V>: Send         + Sync              + FloatPnt<N, V> +
                       POrd         + Bounded           + ScalarSub<N> +
                       ScalarAdd<N> + ScalarMul<N>      + ScalarDiv<N> +
                       IndexMut<uint, N> + Clone {
}


/// Trait implemented by vector types.
pub trait Vect<N>: Send                + Sync  + FloatVec<N> +
                   UniformSphereSample + Clone + IndexMut<uint, N> +
                   Rand                + Shape<uint, N> + POrd +
                   Absolute<Self>      + Iterable<N> {
}

/// Trait implemented by transformation matrices types.
pub trait Isometry<N, P, V>: Send           + Sync              + One          +
                             Translation<V> + Rotate<V>         + Translate<P> +
                             Transform<P>   + AbsoluteRotate<V> + Inv          +
                             Clone {
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
