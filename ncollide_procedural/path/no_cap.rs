use na::{Pnt3, Vec3};
use polyline::Polyline;
use path::PolylineCompatibleCap;
use math::Scalar;

/// A cap that renders nothing.
pub struct NoCap;

impl NoCap {
    /// Creates a new `NoCap`.
    #[inline]
    pub fn new() -> NoCap {
        NoCap
    }
}

impl<N: Scalar> PolylineCompatibleCap<N> for NoCap {
    fn gen_start_cap(&self,
                     _: u32,
                     _: &Polyline<Pnt3<N>>,
                     _: &Pnt3<N>,
                     _: &Vec3<N>,
                     _: bool,
                     _: &mut Vec<Pnt3<N>>,
                     _: &mut Vec<Pnt3<u32>>) {
    }

    fn gen_end_cap(&self,
                   _: u32,
                   _: &Polyline<Pnt3<N>>,
                   _: &Pnt3<N>,
                   _: &Vec3<N>,
                   _: bool,
                   _: &mut Vec<Pnt3<N>>,
                   _: &mut Vec<Pnt3<u32>>) {
    }
}
