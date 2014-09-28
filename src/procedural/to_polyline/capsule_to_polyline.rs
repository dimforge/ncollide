use na::Vec2;
use na;
use geom::Capsule;
use procedural::{ToPolyline, Polyline};
use procedural::utils;
use math::{Scalar, Vect};

#[dim2]
impl ToPolyline<u32> for Capsule {
    fn to_polyline(&self, nsubdivs: u32) -> Polyline<Scalar, Vect> {
        let pi: Scalar = Float::pi();
        let dtheta     = pi / na::cast(nsubdivs as f64);

        let mut points: Vec<Vect> = Vec::with_capacity(nsubdivs as uint);

        utils::push_xy_arc(self.radius(), nsubdivs, dtheta, &mut points);

        let npoints = points.len();

        for i in range(0, npoints) {
            let new_point = points[i] + Vec2::new(na::zero(), self.half_height());

            points.push(-new_point);
            *points.get_mut(i) = new_point;
        }

        Polyline::new(points, None)
    }
}
