use std::fmt::{Debug, Display};
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Lattice, Real};
use alga::linear::{FiniteDimInnerSpace, InnerSpace};
use na::{self, DefaultAllocator, Vector2, Vector3};
use na::dimension::{U2, U3};
use na::storage::Owned;
use na::allocator::Allocator;

/// Trait implemented by vector types usable by ncollide.
pub trait Vector
    : Copy
    + Send
    + Sync
    + 'static
    + Debug
    + Display
    + Lattice
    + Bounded
    + ApproxEq<Epsilon = <Self as InnerSpace>::Real>
    + FiniteDimInnerSpace {
    /// Iterate through the samples.
    fn sample_sphere<F: FnMut(Self)>(F);
}

impl<N> Vector for Vector2<N>
where
    N: Real + Display,
    DefaultAllocator: Allocator<N, U2> + Allocator<usize, U2>,
    Owned<N, U2>: Copy + Sync + Send + 'static,
{
    #[inline(always)]
    fn sample_sphere<F: FnMut(Self)>(mut f: F) {
        for e in SAMPLES_2_F64.iter() {
            let mut s = unsafe { Self::new_uninitialized() };
            s[0] = na::convert(e[0]);
            s[1] = na::convert(e[1]);
            f(s)
        }
    }
}

impl<N> Vector for Vector3<N>
where
    N: Real + Display,
    DefaultAllocator: Allocator<N, U3> + Allocator<usize, U3>,
    Owned<N, U3>: Copy + Sync + Send + 'static,
{
    #[inline(always)]
    fn sample_sphere<F: FnMut(Self)>(mut f: F) {
        for e in SAMPLES_3_F64.iter() {
            let mut s = unsafe { Self::new_uninitialized() };
            s[0] = na::convert(e[0]);
            s[1] = na::convert(e[1]);
            s[2] = na::convert(e[2]);
            f(s)
        }
    }
}

// FIXME: this is bad: this fixes definitely the number of samples…
static SAMPLES_2_F64: [[f64; 2]; 21] = [
    [1.0, 0.0],
    [0.95557281, 0.29475517],
    [0.82623877, 0.56332006],
    [0.6234898, 0.78183148],
    [0.36534102, 0.93087375],
    [0.07473009, 0.9972038],
    [-0.22252093, 0.97492791],
    [-0.5, 0.8660254],
    [-0.73305187, 0.68017274],
    [-0.90096887, 0.43388374],
    [-0.98883083, 0.14904227],
    [-0.98883083, -0.14904227],
    [-0.90096887, -0.43388374],
    [-0.73305187, -0.68017274],
    [-0.5, -0.8660254],
    [-0.22252093, -0.97492791],
    [0.07473009, -0.9972038],
    [0.36534102, -0.93087375],
    [0.6234898, -0.78183148],
    [0.82623877, -0.56332006],
    [0.95557281, -0.29475517],
];

// Those vectors come from bullet 3d
static SAMPLES_3_F64: [[f64; 3]; 42] = [
    [0.000000, -0.000000, -1.000000],
    [0.723608, -0.525725, -0.447219],
    [-0.276388, -0.850649, -0.447219],
    [-0.894426, -0.000000, -0.447216],
    [-0.276388, 0.850649, -0.447220],
    [0.723608, 0.525725, -0.447219],
    [0.276388, -0.850649, 0.447220],
    [-0.723608, -0.525725, 0.447219],
    [-0.723608, 0.525725, 0.447219],
    [0.276388, 0.850649, 0.447219],
    [0.894426, 0.000000, 0.447216],
    [-0.000000, 0.000000, 1.000000],
    [0.425323, -0.309011, -0.850654],
    [-0.162456, -0.499995, -0.850654],
    [0.262869, -0.809012, -0.525738],
    [0.425323, 0.309011, -0.850654],
    [0.850648, -0.000000, -0.525736],
    [-0.525730, -0.000000, -0.850652],
    [-0.688190, -0.499997, -0.525736],
    [-0.162456, 0.499995, -0.850654],
    [-0.688190, 0.499997, -0.525736],
    [0.262869, 0.809012, -0.525738],
    [0.951058, 0.309013, 0.000000],
    [0.951058, -0.309013, 0.000000],
    [0.587786, -0.809017, 0.000000],
    [0.000000, -1.000000, 0.000000],
    [-0.587786, -0.809017, 0.000000],
    [-0.951058, -0.309013, -0.000000],
    [-0.951058, 0.309013, -0.000000],
    [-0.587786, 0.809017, -0.000000],
    [-0.000000, 1.000000, -0.000000],
    [0.587786, 0.809017, -0.000000],
    [0.688190, -0.499997, 0.525736],
    [-0.262869, -0.809012, 0.525738],
    [-0.850648, 0.000000, 0.525736],
    [-0.262869, 0.809012, 0.525738],
    [0.688190, 0.499997, 0.525736],
    [0.525730, 0.000000, 0.850652],
    [0.162456, -0.499995, 0.850654],
    [-0.425323, -0.309011, 0.850654],
    [-0.425323, 0.309011, 0.850654],
    [0.162456, 0.499995, 0.850654],
];
