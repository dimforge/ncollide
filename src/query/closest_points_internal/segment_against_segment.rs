use approx::AbsDiffEq;

use alga::linear::EuclideanSpace;
use crate::math::Isometry;
use na::{self, Real};
use crate::query::ClosestPoints;
use crate::shape::{Segment, SegmentPointLocation};

/// Closest points between segments.
#[inline]
pub fn segment_against_segment<N: Real>(
    m1: &Isometry<N>,
    seg1: &Segment<N>,
    m2: &Isometry<N>,
    seg2: &Segment<N>,
    margin: N,
) -> ClosestPoints<N>
{
    let (loc1, loc2) = segment_against_segment_with_locations(m1, seg1, m2, seg2);
    let p1 = seg1.point_at(&loc1);
    let p2 = seg2.point_at(&loc2);

    if na::distance_squared(&p1, &p2) <= margin * margin {
        ClosestPoints::WithinMargin(p1, p2)
    } else {
        ClosestPoints::Disjoint
    }
}

// FIXME: use this specialized procedure for distance/interference/contact determination as well.
/// Closest points between two segments.
#[inline]
pub fn segment_against_segment_with_locations<N: Real>(
    m1: &Isometry<N>,
    seg1: &Segment<N>,
    m2: &Isometry<N>,
    seg2: &Segment<N>,
) -> (SegmentPointLocation<N>, SegmentPointLocation<N>)
{
    let seg1 = seg1.transformed(m1);
    let seg2 = seg2.transformed(m2);

    segment_against_segment_with_locations_nD((seg1.a(), seg1.b()), (seg2.a(), seg2.b()))
}

/// Segment-segment closest points computation in an arbitrary dimension.
#[allow(non_snake_case)]
#[inline]
pub fn segment_against_segment_with_locations_nD<P>(
    seg1: (&P, &P),
    seg2: (&P, &P),
) -> (SegmentPointLocation<P::Real>, SegmentPointLocation<P::Real>)
where
    P: EuclideanSpace + Copy,
{
    // Inspired by Real-time collision detection by Christer Ericson.
    let d1 = *seg1.1 - *seg1.0;
    let d2 = *seg2.1 - *seg2.0;
    let r = *seg1.0 - *seg2.0;

    let a = na::norm_squared(&d1);
    let e = na::norm_squared(&d2);
    let f = na::dot(&d2, &r);

    let _0: P::Real = na::zero();
    let _1: P::Real = na::one();

    let mut s;
    let mut t;

    let _eps = P::Real::default_epsilon();
    if a <= _eps && e <= _eps {
        s = _0;
        t = _0;
    } else if a <= _eps {
        s = _0;
        t = na::clamp(f / e, _0, _1);
    } else {
        let c = na::dot(&d1, &r);
        if e <= _eps {
            t = _0;
            s = na::clamp(-c / a, _0, _1);
        } else {
            let b = na::dot(&d1, &d2);
            let ae = a * e;
            let bb = b * b;
            let denom = ae - bb;

            // Use absolute and ulps error to test collinearity.
            if denom > _eps && !ulps_eq!(ae, bb) {
                s = na::clamp((b * f - c * e) / denom, _0, _1);
            } else {
                s = _0;
            }

            t = (b * s + f) / e;

            if t < _0 {
                t = _0;
                s = na::clamp(-c / a, _0, _1);
            } else if t > _1 {
                t = _1;
                s = na::clamp((b - c) / a, _0, _1);
            }
        }
    }

    let loc1 = if s == _0 {
        SegmentPointLocation::OnVertex(0)
    } else if s == _1 {
        SegmentPointLocation::OnVertex(1)
    } else {
        SegmentPointLocation::OnEdge([_1 - s, s])
    };

    let loc2 = if t == _0 {
        SegmentPointLocation::OnVertex(0)
    } else if t == _1 {
        SegmentPointLocation::OnVertex(1)
    } else {
        SegmentPointLocation::OnEdge([_1 - t, t])
    };

    (loc1, loc2)
}
