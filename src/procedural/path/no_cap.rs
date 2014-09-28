use na::Vec3;
use procedural::Polyline;
use procedural::path::PolylineCompatibleCap;

/// A cap that renders nothing.
pub struct NoCap;

impl NoCap {
    /// Creates a new `NoCap`.
    #[inline]
    pub fn new() -> NoCap {
        NoCap
    }
}

impl<N> PolylineCompatibleCap<N> for NoCap {
    fn gen_start_cap(&self,
                     _: u32,
                     _: &Polyline<N, Vec3<N>>,
                     _: &Vec3<N>,
                     _: &Vec3<N>,
                     _: bool,
                     _: &mut Vec<Vec3<N>>,
                     _: &mut Vec<Vec3<u32>>) {
    }

    fn gen_end_cap(&self,
                   _: u32,
                   _: &Polyline<N, Vec3<N>>,
                   _: &Vec3<N>,
                   _: &Vec3<N>,
                   _: bool,
                   _: &mut Vec<Vec3<N>>,
                   _: &mut Vec<Vec3<u32>>) {
    }
}
