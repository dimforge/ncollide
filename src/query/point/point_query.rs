use crate::math::{Isometry, Point};
use crate::shape::FeatureId;
use na::{self, RealField};

/// Description of the projection of a point on a shape.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointProjection<N: RealField> {
    /// Whether or not the point to project was inside of the shape.
    pub is_inside: bool,
    /// The projection result.
    pub point: Point<N>,
}

impl<N: RealField> PointProjection<N> {
    /// Initializes a new `PointProjection`.
    pub fn new(is_inside: bool, point: Point<N>) -> PointProjection<N> {
        PointProjection {
            is_inside: is_inside,
            point: point,
        }
    }
}

/// Trait of objects that can be tested for point inclusion and projection.
pub trait PointQuery<N: RealField> {
    /// Projects a point on `self` transformed by `m`.
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N>;

    /// Computes the minimal distance between a point and `self` transformed by `m`.
    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let proj = self.project_point(m, pt, solid);
        let dist = na::distance(pt, &proj.point);

        if solid || !proj.is_inside {
            dist
        } else {
            -dist
        }
    }

    /// Projects a point on the boundary of `self` transformed by `m` and retuns the id of the
    /// feature the point was projected on.
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId);

    /// Tests if the given point is inside of `self` transformed by `m`.
    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        self.project_point(m, pt, false).is_inside
    }
}

/// Returns shape-specific info in addition to generic projection information
///
/// One requirement for the `PointQuery` trait is to be usable as a trait
/// object. Unfortunately this precludes us from adding an associated type to it
/// that might allow us to return shape-specific information in addition to the
/// general information provided in `PointProjection`. This is where
/// `PointQueryWithLocation` comes in. It forgoes the ability to be used as a trait
/// object in exchange for being able to provide shape-specific projection
/// information.
///
/// Any shapes that implement `PointQuery` but are able to provide extra
/// information, can implement `PointQueryWithLocation` in addition and have their
/// `PointQuery::project_point` implementation just call out to
/// `PointQueryWithLocation::project_point_with_location`.
pub trait PointQueryWithLocation<N: RealField> {
    /// Additional shape-specific projection information
    ///
    /// In addition to the generic projection information returned in
    /// `PointProjection`, implementations might provide shape-specific
    /// projection info. The type of this shape-specific information is defined
    /// by this associated type.
    type Location;

    /// Projects a point on `self` transformed by `m`.
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
        solid: bool,
    ) -> (PointProjection<N>, Self::Location);
}
