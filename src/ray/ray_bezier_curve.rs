use na::{Transform, Rotate};
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use shape::BezierCurve;

impl<N, P, V> LocalRayCast<N, P, V> for BezierCurve<P> {
    fn toi_and_normal_with_ray(&self, _: &Ray<P, V>, _: bool) -> Option<RayIntersection<N, V>> {
        // XXX: not yet implemented
        None
    }
}

impl<N, P, V, M: Transform<P> + Rotate<V>> RayCast<N, P, V, M> for BezierCurve<P> {
}
