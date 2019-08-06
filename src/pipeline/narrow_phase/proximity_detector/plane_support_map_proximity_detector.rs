use crate::math::Isometry;
use na::RealField;
use crate::pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use crate::query::{self, Proximity};
use crate::shape::{Plane, Shape};

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapProximityDetector {
}

impl PlaneSupportMapProximityDetector {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapProximityDetector {
        PlaneSupportMapProximityDetector {
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

impl<N: RealField> ProximityDetector<N> for PlaneSupportMapProximityDetector {
    #[inline]
    fn update(
        &mut self,
        _: &dyn ProximityDispatcher<N>,
        ma: &Isometry<N>,
        plane: &dyn Shape<N>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        margin: N,
    ) -> Option<Proximity>
    {
        let p = plane.as_shape::<Plane<N>>()?;
        let sm = b.as_support_map()?;
        Some(query::proximity_plane_support_map(ma, p, mb, sm, margin))
    }
}

impl<N: RealField> ProximityDetector<N> for SupportMapPlaneProximityDetector {
    #[inline]
    fn update(
        &mut self,
        disp: &dyn ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        margin: N,
    ) -> Option<Proximity>
    {
        self.subdetector.update(disp, mb, b, ma, a, margin)
    }
}
