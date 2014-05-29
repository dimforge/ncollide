use ray::{Ray, RayCast, RayIntersection};
use geom::BezierSurface;

impl RayCast for BezierSurface {
    fn toi_and_normal_with_ray(&self, _: &Ray, _: bool) -> Option<RayIntersection> {
        // XXX: not yet implemented
        None
    }
}
