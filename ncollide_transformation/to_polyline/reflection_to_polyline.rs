use geometry::shape::Reflection;
use math::Point;
use procedural::Polyline;
use super::ToPolyline;

impl<'a, P, G: ToPolyline<P, I>, I> ToPolyline<P, I> for Reflection<'a, G>
where
    N: Real,
{
    fn to_polyline(&self, parameter: I) -> Polyline<P> {
        let mut res = self.shape().to_polyline(parameter);

        for c in res.coords_mut().iter_mut() {
            *c = -*c;
        }

        res
    }
}
