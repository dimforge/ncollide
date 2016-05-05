//! Trait implemented by the primitive algebraic types used by ncollide.

extern crate rustc_serialize;
extern crate num;
extern crate rand;
extern crate nalgebra as na;

use std::f32;
use std::f64;
use rustc_serialize::{Encodable, Decodable};
use rand::Rand;
use std::fmt::{Debug, Display};
use std::ops::{IndexMut, Mul, Neg};
use std::any::Any;
use num::{Signed, One};
use na::{Point1, Point2, Point3, Vector1, Vector2, Vector3, Matrix1, Matrix3, Isometry2, Isometry3, Identity};
use na::{ApproxEq, Cast, PartialOrder, FloatVector, Translate, UniformSphereSample,
         Rotate, Transform, AbsoluteRotate, Inverse, FloatPoint, Shape, Absolute, Iterable, BaseFloat,
         Bounded, Repeat};

/// Trait for constant helping handling floating point computations.
pub trait FloatError {
    /// Epsilon value used to perform fuzzy comparisons with zero.
    fn epsilon() -> Self;
}

/// Trait implemented by scalar types.
pub trait Scalar: Copy + Send + Sync + 'static + Debug + Display + Signed +
                  BaseFloat + ApproxEq<Self> + Cast<f64> + Rand + Bounded +
                  FloatError + Decodable + Encodable + Any {
}

/// Trait implemented by point types.
pub trait Point: Send + Sync + 'static + Clone + Copy + Debug + Display +
                 FloatPoint<<<Self as Point>::Vect as Vector>::Scalar, Vector = <Self as Point>::Vect> +
                 PartialOrder +
                 Bounded +
                 IndexMut<usize, Output = <<Self as Point>::Vect as Vector>::Scalar> +
                 Neg<Output = Self> +
                 Decodable + Encodable +
                 ApproxEq<<<Self as Point>::Vect as Vector>::Scalar> +
                 Repeat<<<Self as Point>::Vect as Vector>::Scalar> +
                 Any {
    /// Type of a point's tangent space element, i.e., the vector type.
    type Vect: Vector;
}


/// Trait implemented by vector types.
pub trait Vector: Send + Sync + 'static +
                  FloatVector<<Self as Vector>::Scalar> + UniformSphereSample + Clone +
                  IndexMut<usize, Output = <Self as Vector>::Scalar> + Rand + Shape<usize> +
                  PartialOrder + Absolute<Self> + Iterable<<Self as Vector>::Scalar> +
                  Copy + Neg<Output = Self> + Debug + Display + Any + Decodable + Encodable +
                  Repeat<<Self as Vector>::Scalar> {
    type Scalar: Scalar;
}

/// Trait implemented by transformation matrices types.
pub trait Isometry<P: Point>: Send         + Sync            + 'static                  +
                              One          + Rotate<P::Vect> +
                              Translate<P> + Transform<P>    + AbsoluteRotate<P::Vect>  +
                              Inverse      + Clone           + Mul<Self, Output = Self> +
                              Copy         + Debug           + Display                  +
                              Any          + Decodable       + Encodable {
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

impl<N: Scalar> Point for Point1<N> { type Vect = Vector1<N>; }
impl<N: Scalar> Point for Point2<N> { type Vect = Vector2<N>; }
impl<N: Scalar> Point for Point3<N> { type Vect = Vector3<N>; }

impl<N: Scalar> Vector for Vector1<N> { type Scalar = N; }
impl<N: Scalar> Vector for Vector2<N> { type Scalar = N; }
impl<N: Scalar> Vector for Vector3<N> { type Scalar = N; }

impl<N: Scalar> Isometry<Point2<N>> for Isometry2<N> { }
impl<N: Scalar> Isometry<Point3<N>> for Isometry3<N> { }

impl<N: Scalar> Isometry<Point2<N>> for Identity { }
impl<N: Scalar> Isometry<Point3<N>> for Identity { }

impl<N> HasInertiaMatrix<Matrix1<N>> for Vector2<N> { }
impl<N> HasInertiaMatrix<Matrix3<N>> for Vector3<N> { }
