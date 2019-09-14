use crate::procedural::path::PolylineCompatibleCap;
use alga::general::RealField;
use na::{Point3, Vector3};

/// A cap that renders nothing.
pub struct NoCap;

impl NoCap {
    /// Creates a new `NoCap`.
    #[inline]
    pub fn new() -> NoCap {
        NoCap
    }
}

impl<N: RealField> PolylineCompatibleCap<N> for NoCap {
    fn gen_start_cap(
        &self,
        _: u32,
        _: &[Point3<N>],
        _: &Point3<N>,
        _: &Vector3<N>,
        _: bool,
        _: &mut Vec<Point3<N>>,
        _: &mut Vec<Point3<u32>>,
    ) {
    }

    fn gen_end_cap(
        &self,
        _: u32,
        _: &[Point3<N>],
        _: &Point3<N>,
        _: &Vector3<N>,
        _: bool,
        _: &mut Vec<Point3<N>>,
        _: &mut Vec<Point3<u32>>,
    ) {
    }
}
