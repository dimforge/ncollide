use math::Isometry;
use na::Real;
use pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use query::proximity_internal;
use query::Proximity;
use shape::{Plane, Shape};

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapProximityDetector {
    proximity: Proximity,
}

impl PlaneSupportMapProximityDetector {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapProximityDetector {
        PlaneSupportMapProximityDetector {
            proximity: Proximity::Disjoint,
        }
    }
}

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct SupportMapPlaneProximityDetector {
    subdetector: PlaneSupportMapProximityDetector,
}

impl SupportMapPlaneProximityDetector {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneProximityDetector {
        SupportMapPlaneProximityDetector {
            subdetector: PlaneSupportMapProximityDetector::new(),
        }
    }
}

impl<N: Real> ProximityDetector<N> for PlaneSupportMapProximityDetector {
    #[inline]
    fn update(
        &mut self,
        _: &ProximityDispatcher<N>,
        ma: &Isometry<N>,
        plane: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool {
        if let (Some(p), Some(sm)) = (plane.as_shape::<Plane<N>>(), b.as_support_map()) {
            self.proximity = proximity_internal::plane_against_support_map(ma, p, mb, sm, margin);

            true
        } else {
            false
        }
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.proximity
    }
}

impl<N: Real> ProximityDetector<N> for SupportMapPlaneProximityDetector {
    #[inline]
    fn update(
        &mut self,
        disp: &ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool {
        self.subdetector.update(disp, mb, b, ma, a, margin)
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        ProximityDetector::<N>::proximity(&self.subdetector)
    }
}
