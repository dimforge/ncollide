//! Trait implemented by the primitive algebraic types used by ncollide.

extern crate rustc_serialize;
extern crate num;
extern crate rand;
extern crate nalgebra as na;

use std::f32;
use std::f64;
use rustc_serialize::{Encodable, Decodable};
use rand::Rand;
use std::fmt::Debug;
use std::ops::{IndexMut, Mul, Neg};
use std::any::Any;
use num::{Signed, One};
use na::{Pnt1, Pnt2, Pnt3, Pnt4, Vec1, Vec2, Vec3, Vec4, Mat1, Mat3, Iso2, Iso3, Iso4, Identity};
use na::{ApproxEq, Cast, POrd, FloatVec, Translate, UniformSphereSample, Translation,
         Rotate, Transform, AbsoluteRotate, Inv, FloatPnt, Shape, Absolute, Iterable, BaseFloat,
         Bounded, Repeat};

/// Trait for constant helping handling floating point computations.
pub trait FloatError {
    /// Epsilon value used to perform fuzzy comparisons with zero.
    fn epsilon() -> Self;
}

/// Trait implemented by scalar types.
pub trait Scalar: Copy + Send + Sync + 'static + Debug + Signed +
                  BaseFloat + ApproxEq<Self> + Cast<f64> + Rand + Bounded +
                  FloatError + Decodable + Encodable + Any {
}

/// Trait implemented by point types.
pub trait Point: Send + Sync + 'static + Clone + Copy + Debug +
                 FloatPnt<<<Self as Point>::Vect as Vect>::Scalar, <Self as Point>::Vect> +
                 POrd    +
                 Bounded +
                 IndexMut<usize, Output = <<Self as Point>::Vect as Vect>::Scalar> +
                 Neg<Output = Self> +
                 Decodable + Encodable +
                 Repeat<<<Self as Point>::Vect as Vect>::Scalar> +
                 Any {
    type Vect: Vect;
}


/// Trait implemented by vector types.
pub trait Vect: Send + Sync + 'static +
                FloatVec<<Self as Vect>::Scalar> + UniformSphereSample + Clone +
                IndexMut<usize, Output = <Self as Vect>::Scalar> + Rand + Shape<usize> +
                POrd + Absolute<Self> + Iterable<<Self as Vect>::Scalar> +
                Copy + Neg<Output = Self> + Debug + Any + Decodable + Encodable +
                Repeat<<Self as Vect>::Scalar> {
    type Scalar: Scalar;
}

/// Trait implemented by transformation matrices types.
pub trait Isometry<P, V>: // FIXME: we actually want associated types here.
                          Send           + Sync           + 'static                  +
                          One            + Translation<V> + Rotate<V>                +
                          Translate<P>   + Transform<P>   + AbsoluteRotate<V>        +
                          Inv            + Clone          + Mul<Self, Output = Self> +
                          Copy           + Debug          + Any                      +
                          Decodable      + Encodable {
}

/// Trait implement by vectors that are transformable by the inertia matrix `I`.
pub trait HasInertiaMatrix<I> { } // FIXME: we actually want associated types here.

impl FloatError for f32 {
    fn epsilon() -> f32 {
        f32::EPSILON
    }
}

impl FloatError for f64 {
    fn epsilon() -> f64 {
        f64::EPSILON
    }
}

impl Scalar for f32 { }
impl Scalar for f64 { }

impl<N: Scalar> Point for Pnt1<N> { type Vect = Vec1<N>; }
impl<N: Scalar> Point for Pnt2<N> { type Vect = Vec2<N>; }
impl<N: Scalar> Point for Pnt3<N> { type Vect = Vec3<N>; }
impl<N: Scalar> Point for Pnt4<N> { type Vect = Vec4<N>; }

impl<N: Scalar> Vect for Vec1<N> { type Scalar = N; }
impl<N: Scalar> Vect for Vec2<N> { type Scalar = N; }
impl<N: Scalar> Vect for Vec3<N> { type Scalar = N; }
impl<N: Scalar> Vect for Vec4<N> { type Scalar = N; }

impl<N: Scalar> Isometry<Pnt2<N>, Vec2<N>> for Iso2<N> { }
impl<N: Scalar> Isometry<Pnt3<N>, Vec3<N>> for Iso3<N> { }
impl<N: Scalar> Isometry<Pnt4<N>, Vec4<N>> for Iso4<N> { }

impl<N: Scalar> Isometry<Pnt2<N>, Vec2<N>> for Identity { }
impl<N: Scalar> Isometry<Pnt3<N>, Vec3<N>> for Identity { }
impl<N: Scalar> Isometry<Pnt4<N>, Vec4<N>> for Identity { }

impl<N> HasInertiaMatrix<Mat1<N>> for Vec2<N> { }
impl<N> HasInertiaMatrix<Mat3<N>> for Vec3<N> { }
