use math::{Isometry, Vector};
use na::{Real, Unit};
use pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use query::algorithms::VoronoiSimplex;
use query::proximity_internal;
use query::Proximity;
use shape::Shape;

/// Persistent proximity detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.
#[derive(Clone)]
pub struct SupportMapSupportMapProximityDetector<N: Real> {
    simplex: VoronoiSimplex<N>,
    proximity: Proximity,
    sep_axis: Unit<Vector<N>>,
}

impl<N: Real> SupportMapSupportMapProximityDetector<N> {
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

impl<N: Real> ProximityDetector<N> for SupportMapSupportMapProximityDetector<N> {
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
