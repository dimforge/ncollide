use bounding_volume::BoundingVolume;
use na::Real;
use partitioning::{Visitor, VisitStatus};
use std::marker::PhantomData;


/// Bounding Volume Tree visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeInterferencesCollector<'a, N: 'a, T: 'a, BV: 'a> {
    pub bv: &'a BV,
    pub collector: &'a mut Vec<T>,
    _point: PhantomData<N>,
}

impl<'a, N, T, BV> BoundingVolumeInterferencesCollector<'a, N, T, BV>
    where
        N: Real,
        BV: BoundingVolume<N>,
{
    /// Creates a new `BoundingVolumeInterferencesCollector`.
    #[inline]
    pub fn new(
        bv: &'a BV,
        buffer: &'a mut Vec<T>,
    ) -> BoundingVolumeInterferencesCollector<'a, N, T, BV> {
        BoundingVolumeInterferencesCollector {
            bv: bv,
            collector: buffer,
            _point: PhantomData,
        }
    }
}

impl<'a, N, T, BV> Visitor<T, BV> for BoundingVolumeInterferencesCollector<'a, N, T, BV>
    where
        N: Real,
        T: Clone,
        BV: BoundingVolume<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.intersects(self.bv) {
            if let Some(t) = t {
                self.collector.push(t.clone())
            }

            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}
