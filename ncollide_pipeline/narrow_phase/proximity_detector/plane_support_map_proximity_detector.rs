use std::any::Any;
use std::marker::PhantomData;
use na::{Translate, Rotate};
use math::{Point, Vect};
use entities::shape::Plane;
use entities::inspection;
use entities::inspection::Repr;
use queries::geometry::Proximity;
use queries::geometry::proximity_internal;
use narrow_phase::{ProximityDetector, ProximityDispatcher};


/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapProximityDetector<P: Point, M> {
    proximity:  Proximity,
    pt_type:    PhantomData<P>, // FIXME: can we avoid this?
    mat_type:   PhantomData<M>  // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapProximityDetector<P, M> {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapProximityDetector<P, M> {
        PlaneSupportMapProximityDetector {
            proximity: Proximity::Disjoint,
            pt_type:   PhantomData,
            mat_type:  PhantomData
        }
    }
}

/// Proximity detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct SupportMapPlaneProximityDetector<P: Point, M> {
    subdetector: PlaneSupportMapProximityDetector<P, M>
}

impl<P: Point, M> SupportMapPlaneProximityDetector<P, M> {
    /// Creates a new persistent proximity detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneProximityDetector<P, M> {
        SupportMapPlaneProximityDetector {
            subdetector: PlaneSupportMapProximityDetector::new()
        }
    }
}

impl<P, M> ProximityDetector<P, M> for PlaneSupportMapProximityDetector<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self, _: &ProximityDispatcher<P, M>,
              ma: &M, plane: &Repr<P, M>,
              mb: &M, b: &Repr<P, M>,
              margin: <P::Vect as Vect>::Scalar)
              -> bool {
        let rp = plane.repr();

        if let (Some(p), Some(sm)) =
            (rp.downcast_ref::<Plane<P::Vect>>(), inspection::maybe_as_support_map::<P, M, _>(b)) {
                self.proximity = proximity_internal::plane_against_support_map(ma, p, mb, sm, margin);

                true
        }
        else {
            false
        }
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.proximity
    }
}

impl<P, M> ProximityDetector<P, M> for SupportMapPlaneProximityDetector<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self, disp: &ProximityDispatcher<P, M>,
              ma: &M, a: &Repr<P, M>,
              mb: &M, b: &Repr<P, M>,
              margin: <P::Vect as Vect>::Scalar)
              -> bool {
        self.subdetector.update(disp, mb, b, ma, a, margin)
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.subdetector.proximity()
    }
}
