use procedural::{ToPolyline, Polyline};
use procedural;
use math::{Scalar, Point, Vect};
use geom::BezierCurve;

impl ToPolyline<u32> for BezierCurve {
    fn to_polyline(&self, nsubdivs: u32) -> Polyline<Scalar, Point, Vect> {
        procedural::bezier_curve(self.control_points(), nsubdivs as uint)
    }
}
