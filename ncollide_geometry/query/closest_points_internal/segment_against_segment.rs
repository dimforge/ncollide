use approx::ApproxEq;

use na;
use math::{Isometry, Point};
use shape::{Segment, SegmentPointLocation};
use query::ClosestPoints;

/// Closest points between segments.
#[inline]
pub fn segment_against_segment<P, M>(
    m1: &M,
    seg1: &Segment<P>,
    m2: &M,
    seg2: &Segment<P>,
    margin: P::Real,
) -> ClosestPoints<P>
where
    P: Point,
    M: Isometry<P>,
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
pub fn segment_against_segment_with_locations<P, M>(
    m1: &M,
    seg1: &Segment<P>,
    m2: &M,
    seg2: &Segment<P>,
) -> (SegmentPointLocation<P::Real>, SegmentPointLocation<P::Real>)
where
    P: Point,
    M: Isometry<P>,
{
    // Inspired by Real-time collision detection by Christer Ericson.
    let seg1 = seg1.transformed(m1);
    let seg2 = seg2.transformed(m2);
    let d1 = seg1.scaled_direction();
    let d2 = seg2.scaled_direction();
    let r = *seg1.a() - *seg2.a();

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
            let denom = a * e - b * b;

            if denom != _0 {
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
