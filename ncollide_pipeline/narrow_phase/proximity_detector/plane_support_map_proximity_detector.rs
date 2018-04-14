use std::marker::PhantomData;
use math::{Isometry, Point};
use geometry::shape::{Plane, Shape};
use geometry::query::Proximity;
use geometry::query::proximity_internal;
use narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapProximityDetector<N: Real, M> {
    proximity: Proximity,
    pt_type: PhantomData<P>,  // FIXME: can we avoid this?
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<N: Real, M> PlaneSupportMapProximityDetector<P, M> {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapProximityDetector<P, M> {
        PlaneSupportMapProximityDetector {
            proximity: Proximity::Disjoint,
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct SupportMapPlaneProximityDetector<N: Real, M> {
    subdetector: PlaneSupportMapProximityDetector<P, M>,
}

impl<N: Real, M> SupportMapPlaneProximityDetector<P, M> {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneProximityDetector<P, M> {
        SupportMapPlaneProximityDetector {
            subdetector: PlaneSupportMapProximityDetector::new(),
        }
    }
}

impl<N: Real> ProximityDetector<P, M> for PlaneSupportMapProximityDetector<P, M> {
    #[inline]
    fn update(
        &mut self,
        _: &ProximityDispatcher<P, M>,
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

impl<N: Real> ProximityDetector<P, M> for SupportMapPlaneProximityDetector<P, M> {
    #[inline]
    fn update(
        &mut self,
        disp: &ProximityDispatcher<P, M>,
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
        self.subdetector.proximity()
    }
}
