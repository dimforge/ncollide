use std::marker::PhantomData;
use math::{Isometry, Point};
use geometry::shape::{Plane, Shape};
use geometry::query::Proximity;
use geometry::query::proximity_internal;
use narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapProximityDetector<P: Point, M> {
    proximity: Proximity,
    pt_type: PhantomData<P>,  // FIXME: can we avoid this?
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapProximityDetector<P, M> {
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
pub struct SupportMapPlaneProximityDetector<P: Point, M> {
    subdetector: PlaneSupportMapProximityDetector<P, M>,
}

impl<P: Point, M> SupportMapPlaneProximityDetector<P, M> {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneProximityDetector<P, M> {
        SupportMapPlaneProximityDetector {
            subdetector: PlaneSupportMapProximityDetector::new(),
        }
    }
}

impl<P: Point, M: Isometry<P>> ProximityDetector<P, M> for PlaneSupportMapProximityDetector<P, M> {
    #[inline]
    fn update(
        &mut self,
        _: &ProximityDispatcher<P, M>,
        ma: &M,
        plane: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        margin: P::Real,
    ) -> bool {
        if let (Some(p), Some(sm)) = (plane.as_shape::<Plane<P::Vector>>(), b.as_support_map()) {
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

impl<P: Point, M: Isometry<P>> ProximityDetector<P, M> for SupportMapPlaneProximityDetector<P, M> {
    #[inline]
    fn update(
        &mut self,
        disp: &ProximityDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        margin: P::Real,
    ) -> bool {
        self.subdetector.update(disp, mb, b, ma, a, margin)
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.subdetector.proximity()
    }
}
