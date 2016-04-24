use std::any::Any;
use std::marker::PhantomData;
use na::{self, Translate, Translation};
use math::{Point, Vector};
use entities::inspection::Shape;
use entities::shape::AnnotatedPoint;
use queries::geometry::algorithms::simplex::Simplex;
use queries::geometry::proximity_internal;
use queries::geometry::Proximity;
use narrow_phase::{ProximityDetector, ProximityDispatcher};


/// Persistent proximity detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.
#[derive(Clone)]
pub struct SupportMapSupportMapProximityDetector<P: Point, M, S> {
    simplex:   S,
    proximity: Proximity,
    sep_axis:  P::Vect,
    pt_type:   PhantomData<P>, // FIXME: can we avoid this?
    mat_type:  PhantomData<M>  // FIXME: can we avoid this?
}

impl<P, M, S> SupportMapSupportMapProximityDetector<P, M, S>
    where P: Point,
          S: Simplex<AnnotatedPoint<P>> {
    /// Creates a new persistant proximity detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapProximityDetector<P, M, S> {
        SupportMapSupportMapProximityDetector {
            simplex:   simplex,
            proximity: Proximity::Disjoint,
            sep_axis:  na::zero(),
            pt_type:   PhantomData,
            mat_type:  PhantomData
        }
    }
}

impl<P, M, S> ProximityDetector<P, M> for SupportMapSupportMapProximityDetector<P, M, S>
    where P: Point,
          P::Vect: Translate<P>,
          M: Translation<P::Vect> + Any,
          S: Simplex<AnnotatedPoint<P>> {
    #[inline]
    fn update(&mut self, _: &ProximityDispatcher<P, M>,
              ma: &M, a: &Shape<P, M>,
              mb: &M, b: &Shape<P, M>,
              margin: <P::Vect as Vector>::Scalar)
              -> bool {
        if let (Some(sma), Some(smb)) = (a.desc().as_support_map(), b.desc().as_support_map()) {
            let initial_direction;
            if self.proximity == Proximity::Disjoint {
                initial_direction = None
            }
            else {
                initial_direction = Some(self.sep_axis)
            }

            let res = proximity_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                margin,
                &mut self.simplex,
                initial_direction);

            self.proximity = res.0;
            self.sep_axis  = res.1;

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
