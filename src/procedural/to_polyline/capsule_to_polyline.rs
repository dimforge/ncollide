use nalgebra::na;
use geom::Capsule;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Vect};

impl ToPolyline<u32> for Capsule {
    fn to_polyline(&self, nsubdivs: u32) -> Polyline<Scalar, Vect> {
        let pi: Scalar = Float::pi();
        let dtheta     = pi / na::cast(nsubdivs as f64);

        let mut points = Vec::with_capacity(nsubdivs);

        utils::push_xy_arc(self.radius(), nsubdivs, dtheta, &mut top_pts);

        let npoints = points.len();

        for i in range(0, npoints) {
            let new_point = points.at(i) + Vec3::new(na::zero(), self.half_height(), na::zero());

            points.push(-new_point);
            *points.at_mut(i) = new_point;
        }

        Polyline::new(points, None)
    }
}
