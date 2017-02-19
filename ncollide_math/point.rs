use std::fmt::Display;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg, Index, IndexMut};
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Lattice, Real};
use alga::linear::EuclideanSpace;
use na::{Axpy, ColumnVector, PointBase};
use na::dimension::{DimName, U1};
use na::storage::OwnedStorage;
use na::allocator::OwnedAllocator;
use ::Vector;


/// Trait implemented by point types usable by ncollide.
pub trait Point: Copy + Send + Sync + 'static +
                 Display + Lattice + Bounded +
                 Index<usize, Output = <Self as EuclideanSpace>::Real> +
                 IndexMut<usize, Output = <Self as EuclideanSpace>::Real> +
                 Axpy<<Self as EuclideanSpace>::Real> +
                 ApproxEq<Epsilon = <Self as EuclideanSpace>::Real> +
                 EuclideanSpace<Coordinates = <Self as Point>::Vector> {
    type Vector: Vector<Real = Self::Real> +
                 Add<Self::Vector, Output = Self::Vector> +
                 AddAssign<Self::Vector> +
                 Sub<Self::Vector, Output = Self::Vector> +
                 SubAssign<Self::Vector> +
                 Mul<Self::Real, Output = Self::Vector> +
                 MulAssign<Self::Real>          +
                 Div<Self::Real, Output = Self::Vector> +
                 DivAssign<Self::Real>          +
                 Neg<Output = Self::Vector>;
}

impl<N, D, S> Point for PointBase<N, D, S>
    where N: Real,
          D: DimName,
          S: Send + Sync + 'static + Copy + OwnedStorage<N, D, U1>,
          S::Alloc: OwnedAllocator<N, D, U1, S>,
          ColumnVector<N, D, S>: Vector<Real = N> {
    type Vector = Self::Coordinates;
}
