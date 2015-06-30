use math::{Point, Vect};

/// Trait of objects that can be tested for point inclusion and projection.
pub trait PointQuery<P: Point, M> {
    /// Projects a point on `self` transformed by `m`.
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P;

    /// Computes the minimal distance between a point and `self` transformed by `m`.
    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar;

    /// Tests if the given point is inside of `self` transformed by `m`.
    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool;
}
