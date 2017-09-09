use alga::general::Real;
use na::{Point2, Point3, Vector3, Isometry3};
use na;
use polyline::Polyline;
use trimesh::{TriMesh, IndexBuffer};
use utils;
use path::{StrokePattern, CurveSampler, PathSample};

/// A pattern composed of polyline and two caps.
pub struct PolylinePattern<N: Real, C1, C2> {
    pattern:       Polyline<Point3<N>>,
    closed:        bool,
    last_start_id: u32,
    start_cap:     C1,
    end_cap:       C2,
}

/// Trait to be implemented by caps compatible with a `PolylinePattern`.
pub trait PolylineCompatibleCap<N: Real> {
    /// Generates the mesh for the cap at the beginning of a path.
    fn gen_start_cap(&self,
                     attach_id: u32,
                     pattern:   &Polyline<Point3<N>>,
                     pt:        &Point3<N>,
                     dir:       &Vector3<N>,
                     closed:    bool,
                     coords:    &mut Vec<Point3<N>>,
                     indices:   &mut Vec<Point3<u32>>);

    /// Generates the mesh for the cap at the end of a path.
    fn gen_end_cap(&self,
                   attach_id: u32,
                   pattern:   &Polyline<Point3<N>>,
                   pt:        &Point3<N>,
                   dir:       &Vector3<N>,
                   closed:    bool,
                   coords:    &mut Vec<Point3<N>>,
                   indices:   &mut Vec<Point3<u32>>);
}

impl<N, C1, C2> PolylinePattern<N, C1, C2>
    where N:  Real,
          C1: PolylineCompatibleCap<N>,
          C2: PolylineCompatibleCap<N> {
    /// Creates a new polyline pattern.
    pub fn new(pattern:   &Polyline<Point2<N>>,
               closed:    bool,
               start_cap: C1,
               end_cap:   C2)
               -> PolylinePattern<N, C1, C2> {
        let mut coords3d = Vec::with_capacity(pattern.coords().len());

        for v in pattern.coords().iter() {
            coords3d.push(Point3::new(v.x.clone(), v.y.clone(), na::zero()));
        }

        PolylinePattern {
            pattern:       Polyline::new(coords3d, None),
            closed:        closed,
            last_start_id: 0,
            start_cap:     start_cap,
            end_cap:       end_cap
        }
    }
}

impl<N, C1, C2> StrokePattern<Point3<N>> for PolylinePattern<N, C1, C2>
    where N:  Real,
          C1: PolylineCompatibleCap<N>,
          C2: PolylineCompatibleCap<N>{
    fn stroke<C>(&mut self, sampler: &mut C) -> TriMesh<Point3<N>>
        where C: CurveSampler<Point3<N>> {
        let mut vertices = Vec::new();
        let mut indices  = Vec::new();
        let npts         = self.pattern.coords().len() as u32;
        // FIXME: collect the normals too.
        // let mut normals  = Vec::new();

        loop {
            let next = sampler.next();

            // second match to add the inner triangles.
            match next {
                PathSample::StartPoint(ref pt, ref dir) |
                PathSample::InnerPoint(ref pt, ref dir) |
                PathSample::EndPoint(ref pt, ref dir)   => {
                    let mut new_polyline = self.pattern.clone();
                    let transform;

                    if dir.x.is_zero() && dir.z.is_zero() { // FIXME: this might not be enough to avoid singularities.
                        transform = Isometry3::new_observer_frame(pt, &(*pt + *dir), &Vector3::x());
                    }

                    else {
                        transform = Isometry3::new_observer_frame(pt, &(*pt + *dir), &Vector3::y());
                    }

                    new_polyline.transform_by(&transform);

                    let new_start_id = vertices.len() as u32;

                    vertices.extend(new_polyline.unwrap().0.into_iter());

                    if new_start_id != 0 {
                        if self.closed {
                            utils::push_ring_indices(new_start_id, self.last_start_id, npts, &mut indices);
                        }
                        else {
                            utils::push_open_ring_indices(new_start_id, self.last_start_id, npts, &mut indices);
                        }

                        self.last_start_id = new_start_id;
                    }
                },
                PathSample::EndOfSample =>
                    return TriMesh::new(vertices, None, None, Some(IndexBuffer::Unified(indices)))
            }

            // third match to add the end cap
            // FIXME: this will fail with patterns having multiple starting and end points!
            match next {
                PathSample::StartPoint(ref pt, ref dir) => {
                    self.start_cap.gen_start_cap(0, &self.pattern,
                                                 pt, dir, self.closed, &mut vertices, &mut indices);
                },
                PathSample::EndPoint(ref pt, ref dir) => {
                    self.end_cap.gen_end_cap(vertices.len() as u32 - npts, &self.pattern,
                                             pt, dir, self.closed, &mut vertices, &mut indices);
                },
                _ => { }
            }
        }
    }
}
