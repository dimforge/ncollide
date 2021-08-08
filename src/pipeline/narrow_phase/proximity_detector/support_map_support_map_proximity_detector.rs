use crate::math::{Isometry, Vector};
use crate::pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use crate::query::algorithms::VoronoiSimplex;
use crate::query::{self, Proximity};
use crate::shape::Shape;
use na::{RealField, Unit};

/// Persistent proximity detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.
#[derive(Clone)]
pub struct SupportMapSupportMapProximityDetector<N: RealField + Copy> {
    simplex: VoronoiSimplex<N>,
    sep_axis: Option<Unit<Vector<N>>>,
}

impl<N: RealField + Copy> SupportMapSupportMapProximityDetector<N> {
    /// Creates a new persistant proximity detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new() -> SupportMapSupportMapProximityDetector<N> {
        SupportMapSupportMapProximityDetector {
            simplex: VoronoiSimplex::new(),
            sep_axis: None,
        }
    }
}

impl<N: RealField + Copy> ProximityDetector<N> for SupportMapSupportMapProximityDetector<N> {
    #[inline]
    fn update(
        &mut self,
        _: &dyn ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        margin: N,
    ) -> Option<Proximity> {
        let sma = a.as_support_map()?;
        let smb = b.as_support_map()?;

        let res = query::proximity_support_map_support_map_with_params(
            ma,
            sma,
            mb,
            smb,
            margin,
            &mut self.simplex,
            self.sep_axis,
        );

        if res.0 != Proximity::Intersecting {
            self.sep_axis = Some(res.1);
        } else {
            self.sep_axis = None;
        }

        Some(res.0)
    }
}
