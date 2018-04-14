use std::marker::PhantomData;
use na;
use math::{Isometry, Point};
use shape::{AnnotatedPoint, Shape};
use query::algorithms::Simplex;
use query::proximity_internal;
use query::Proximity;
use pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Persistent proximity detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.
#[derive(Clone)]
pub struct SupportMapSupportMapProximityDetector<N, S> {
    simplex: S,
    proximity: Proximity,
    sep_axis: Vector<N>,
    pt_type: PhantomData<P>,  // FIXME: can we avoid this?
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<N, S> SupportMapSupportMapProximityDetector<N, S>
where
    N: Real,
    S: Simplex<AnnotatedPoint<P>>,
{
    /// Creates a new persistant proximity detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapProximityDetector<N, S> {
        SupportMapSupportMapProximityDetector {
            simplex: simplex,
            proximity: Proximity::Disjoint,
            sep_axis: na::zero(),
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

impl<N, S> ProximityDetector<N> for SupportMapSupportMapProximityDetector<N, S>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
{
    #[inline]
    fn update(
        &mut self,
        _: &ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool {
        if let (Some(sma), Some(smb)) = (a.as_support_map(), b.as_support_map()) {
            let initial_direction;
            if self.proximity == Proximity::Disjoint {
                initial_direction = None
            } else {
                initial_direction = Some(self.sep_axis)
            }

            let res = proximity_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                margin,
                &mut self.simplex,
                initial_direction,
            );

            self.proximity = res.0;
            self.sep_axis = res.1;

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
