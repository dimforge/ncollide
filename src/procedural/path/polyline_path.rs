use nalgebra::na::FloatVec;
use procedural::path::{PathSample, CurveSampler, StartPoint, InnerPoint, EndPoint, EndOfSample};
use procedural::Polyline;

/// A path with its sample points given by a polyline.
///
/// This will return sequencially each vertex of the polyline.
pub struct PolylinePath<'a, N, V: 'a> {
    curr_len:               N,
    curr_dir:               V,
    curr_pt_id:             uint,
    curr_pt:                V,
    polyline:               &'a Polyline<N, V>
}

impl<'a, N: Float, V: FloatVec<N> + Clone> PolylinePath<'a, N, V> {
    /// Creates a new polyline-based path.
    pub fn new(polyline: &'a Polyline<N, V>) -> PolylinePath<'a, N, V> {
        assert!(polyline.coords.len() > 1, "The polyline must have at least two points.");

        let mut dir = polyline.coords[1] - polyline.coords[0];
        let len     = dir.normalize();
        PolylinePath {
            curr_len:               len,
            curr_dir:               dir,
            curr_pt_id:             0,
            curr_pt:                polyline.coords[0].clone(),
            polyline:               polyline
        }
    }
}

impl<'a, N: Float, V: FloatVec<N> + Clone> CurveSampler<N, V>  for PolylinePath<'a, N, V> {
    fn next(&mut self) -> PathSample<V> {
        let result =
            if self.curr_pt_id == 0 {
                StartPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else if self.curr_pt_id < self.polyline.coords.len() - 1 {
                InnerPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else if self.curr_pt_id == self.polyline.coords.len() - 1 {
                EndPoint(self.curr_pt.clone(), self.curr_dir.clone())
            }
            else {
                EndOfSample
            };

        self.curr_pt_id = self.curr_pt_id + 1;

        if self.curr_pt_id < self.polyline.coords.len() {
            self.curr_pt = self.polyline.coords[self.curr_pt_id].clone();

            if self.curr_pt_id < self.polyline.coords.len() - 1 {
                let mut curr_diff = self.polyline.coords[self.curr_pt_id + 1] -
                                    self.polyline.coords[self.curr_pt_id];
                self.curr_len = curr_diff.normalize();
                self.curr_dir = curr_diff;
            }
        }

        result
    }
}
