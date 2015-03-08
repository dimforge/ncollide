use path::{PathSample, CurveSampler};
use polyline::Polyline;
use math::{Scalar, Point, Vect};

/// A path with its sample points given by a polyline.
///
/// This will return sequencially each vertex of the polyline.
pub struct PolylinePath<'a, N: 'a, P: 'a, V: 'a> {
    curr_len:               N,
    curr_dir:               V,
    curr_pt_id:             usize,
    curr_pt:                P,
    polyline:               &'a Polyline<N, P, V>
}

impl<'a, N: Scalar, P: Point<N, V>, V: Vect<N>> PolylinePath<'a, N, P, V> {
    /// Creates a new polyline-based path.
    pub fn new(polyline: &'a Polyline<N, P, V>) -> PolylinePath<'a, N, P, V> {
        assert!(polyline.coords.len() > 1, "The polyline must have at least two points.");

        let mut dir: V  = polyline.coords[1] - polyline.coords[0];
        let len: N      = dir.normalize_mut();

        PolylinePath {
            curr_len:   len,
            curr_dir:   dir,
            curr_pt_id: 0,
            curr_pt:    polyline.coords[0].clone(),
            polyline:   polyline
        }
    }
}

impl<'a, N, P, V> CurveSampler<P, V> for PolylinePath<'a, N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn next(&mut self) -> PathSample<P, V> {
        let result =
            if self.curr_pt_id == 0 {
                PathSample::StartPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else if self.curr_pt_id < self.polyline.coords.len() - 1 {
                PathSample::InnerPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else if self.curr_pt_id == self.polyline.coords.len() - 1 {
                PathSample::EndPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else {
                PathSample::EndOfSample
            };

        self.curr_pt_id = self.curr_pt_id + 1;

        if self.curr_pt_id < self.polyline.coords.len() {
            self.curr_pt = self.polyline.coords[self.curr_pt_id].clone();

            if self.curr_pt_id < self.polyline.coords.len() - 1 {
                let mut curr_diff = self.polyline.coords[self.curr_pt_id + 1] -
                                    self.polyline.coords[self.curr_pt_id];
                self.curr_len = curr_diff.normalize_mut();
                self.curr_dir = curr_diff;
            }
        }

        result
    }
}
