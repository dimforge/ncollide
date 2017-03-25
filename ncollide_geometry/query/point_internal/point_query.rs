use na;
use math::Point;

/// Description of the projection of a point on a shape.
pub struct PointProjection<P: Point, I = ()> {
    /// Whether or not the point to project was inside of the shape.
    pub is_inside: bool,
    /// The projection result.
    pub point: P,
    /// Additional projection results whose type is dependent on the shape type.
    pub info: I,
}

impl<P: Point, I> PointProjection<P, I> {
    /// Initializes a new `PointProjection`.
    pub fn new(is_inside: bool, point: P, info: I) -> PointProjection<P, I> {
        PointProjection {
            is_inside: is_inside,
            point:     point,
            info:      info,
        }
    }
}

/// Trait of objects that can be tested for point inclusion and projection.
pub trait PointQuery<P: Point, M> {
    /// Projects a point on `self` transformed by `m`.
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P>;

    /// Computes the minimal distance between a point and `self` transformed by `m`.
    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let proj = self.project_point(m, pt, solid);
        let dist = na::distance(pt, &proj.point);

        if solid || !proj.is_inside {
            dist
        }
        else {
            -dist
        }
    }

    /// Tests if the given point is inside of `self` transformed by `m`.
    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        self.project_point(m, pt, false).is_inside
    }
}
