use na::{Transform, Bounded};
use point::{LocalPointQuery, PointQuery};
use shape::BezierSurface;
use math::Scalar;

impl<N: Scalar, P: Clone> LocalPointQuery<N, P> for BezierSurface<P> {
    fn project_point(&self, point: &P, _: bool) -> P {
        println!("Point projection on BezierSurface, is not yet supported.");
        // XXX: not yet implemented.
        point.clone()
    }

    fn distance_to_point(&self, _: &P) -> N {
        println!("Point distance with BezierSurface, is not yet supported.");
        // XXX: not yet implemented.
        Bounded::max_value()
    }

    fn contains_point(&self, _: &P) -> bool {
        println!("Point inclusion on BezierSurface, is not yet supported.");
        // XXX: not yet implemented.
        false
    }
}

impl<N: Scalar, P: Clone, M: Transform<P>> PointQuery<N, P, M> for BezierSurface<P> {
}
