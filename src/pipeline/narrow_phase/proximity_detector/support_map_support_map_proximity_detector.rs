use crate::math::{Isometry, Vector};
use na::{RealField, Unit};
use crate::pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use crate::query::algorithms::VoronoiSimplex;
use crate::query::proximity_internal;
use crate::query::Proximity;
use crate::shape::Shape;

/// Persistent proximity detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.
#[derive(Clone)]
pub struct SupportMapSupportMapProximityDetector<N: RealField> {
    simplex: VoronoiSimplex<N>,
    proximity: Proximity,
    sep_axis: Unit<Vector<N>>,
}

impl<N: RealField> SupportMapSupportMapProximityDetector<N> {
    /// Creates a new persistant proximity detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new() -> SupportMapSupportMapProximityDetector<N> {
        SupportMapSupportMapProximityDetector {
            simplex: VoronoiSimplex::new(),
            proximity: Proximity::Disjoint,
            sep_axis: Vector::x_axis(),
        }
    }
}

impl<N: RealField> ProximityDetector<N> for SupportMapSupportMapProximityDetector<N> {
    #[inline]
    fn update(
        &mut self,
        _: &ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool
    {
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
