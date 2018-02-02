use alga::linear::NormedSpace;
use path::{CurveSampler, PathSample};
use polyline::Polyline;
use math::Point;

/// A path with its sample points given by a polyline.
///
/// This will return sequencially each vertex of the polyline.
pub struct PolylinePath<'a, P: 'a + Point> {
    curr_len: P::Real,
    curr_dir: P::Vector,
    curr_pt_id: usize,
    curr_pt: P,
    polyline: &'a Polyline<P>,
}

impl<'a, P: Point> PolylinePath<'a, P> {
    /// Creates a new polyline-based path.
    pub fn new(polyline: &'a Polyline<P>) -> PolylinePath<'a, P> {
        assert!(
            polyline.coords().len() > 1,
            "The polyline must have at least two points."
        );

        let mut dir: P::Vector = polyline.coords()[1] - polyline.coords()[0];
        let len: P::Real = dir.normalize_mut();

        PolylinePath {
            curr_len: len,
            curr_dir: dir,
            curr_pt_id: 0,
            curr_pt: polyline.coords()[0].clone(),
            polyline: polyline,
        }
    }
}

impl<'a, P: Point> CurveSampler<P> for PolylinePath<'a, P> {
    fn next(&mut self) -> PathSample<P> {
        let poly_coords = self.polyline.coords();

        let result = if self.curr_pt_id == 0 {
            PathSample::StartPoint(self.curr_pt.clone(), self.curr_dir.clone())
        } else if self.curr_pt_id < poly_coords.len() - 1 {
            PathSample::InnerPoint(self.curr_pt.clone(), self.curr_dir.clone())
        } else if self.curr_pt_id == poly_coords.len() - 1 {
            PathSample::EndPoint(self.curr_pt.clone(), self.curr_dir.clone())
        } else {
            PathSample::EndOfSample
        };

        self.curr_pt_id = self.curr_pt_id + 1;

        if self.curr_pt_id < poly_coords.len() {
            self.curr_pt = poly_coords[self.curr_pt_id].clone();

            if self.curr_pt_id < poly_coords.len() - 1 {
                let mut curr_diff = poly_coords[self.curr_pt_id + 1] - poly_coords[self.curr_pt_id];
                self.curr_len = curr_diff.normalize_mut();
                self.curr_dir = curr_diff;
            }
        }

        result
    }
}
