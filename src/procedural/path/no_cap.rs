use crate::procedural::path::PolylineCompatibleCap;
use na::{Point3, Vector3};
use simba::scalar::RealField;

/// A cap that renders nothing.
pub struct NoCap;

impl NoCap {
    /// Creates a new `NoCap`.
    #[inline]
    pub fn new() -> NoCap {
        NoCap
    }
}

impl<N: RealField + Copy> PolylineCompatibleCap<N> for NoCap {
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
