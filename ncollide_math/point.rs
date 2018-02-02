use std::fmt::Display;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub,
               SubAssign};
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Lattice, Real};
use alga::linear::EuclideanSpace;
use na::{self, DefaultAllocator, Unit, VectorN};
use na::storage::Owned;
use na::dimension::DimName;
use na::allocator::Allocator;
use Vector;

/// Trait implemented by point types usable by ncollide.
pub trait Point
    : Copy
    + Send
    + Sync
    + 'static
    + Display
    + Lattice
    + Bounded
    + Index<usize, Output = <Self as EuclideanSpace>::Real>
    + IndexMut<usize, Output = <Self as EuclideanSpace>::Real>
    + ApproxEq<Epsilon = <Self as EuclideanSpace>::Real>
    + EuclideanSpace<Coordinates = <Self as Point>::Vector> {
    type Vector: Vector<Real = Self::Real>
        + Add<Self::Vector, Output = Self::Vector>
        + AddAssign<Self::Vector>
        + Sub<Self::Vector, Output = Self::Vector>
        + SubAssign<Self::Vector>
        + Mul<Self::Real, Output = Self::Vector>
        + MulAssign<Self::Real>
        + Div<Self::Real, Output = Self::Vector>
        + DivAssign<Self::Real>
        + Neg<Output = Self::Vector>;

    fn axpy(
        &mut self,
        a: <Self as EuclideanSpace>::Real,
        rhs: &Self,
        b: <Self as EuclideanSpace>::Real,
    );
    fn ccw_face_normal(pts: &[&Self]) -> Option<Unit<Self::Vector>>;
}

impl<N, D> Point for na::Point<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
    Owned<N, D>: Copy + Sync + Send + 'static,
    VectorN<N, D>: Vector<Real = N>,
{
    type Vector = Self::Coordinates;

    #[inline]
    fn axpy(&mut self, a: N, rhs: &Self, b: N) {
        self.coords.axpy(a, &rhs.coords, b);
    }

    #[inline]
    fn ccw_face_normal(pts: &[&Self]) -> Option<Unit<VectorN<N, D>>> {
        let mut res = VectorN::zeros();

        if D::dim() == 2 {
            let ab = pts[1] - pts[0];
            res[0] = ab[1];
            res[1] = -ab[0];
        } else if D::dim() == 3 {
            let ab = pts[1] - pts[0];
            let ac = pts[2] - pts[0];
            res = ab.cross(&ac);
        } else {
            unimplemented!()
        }

        Unit::try_new(res, N::default_epsilon())
    }
}
