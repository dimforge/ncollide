use na::{Pnt2, Pnt3, Vec2, Vec3, Iso3};
use na;
use procedural::{Polyline, TriMesh, IndexBuffer};
use procedural::utils;
use procedural::path::{StrokePattern, CurveSampler, StartPoint, InnerPoint, EndPoint, EndOfSample};
use math::Scalar;

/// A pattern composed of polyline and two caps.
pub struct PolylinePattern<N, C1, C2> {
    pattern:       Polyline<N, Pnt3<N>, Vec3<N>>,
    closed:        bool,
    last_start_id: u32,
    start_cap:     C1,
    end_cap:       C2,
}

/// Trait to be implemented by caps compatible with a `PolylinePattern`.
pub trait PolylineCompatibleCap<N> {
    /// Generates the mesh for the cap at the beginning of a path.
    fn gen_start_cap(&self,
                     attach_id: u32,
                     pattern:   &Polyline<N, Pnt3<N>, Vec3<N>>,
                     pt:        &Pnt3<N>,
                     dir:       &Vec3<N>,
                     closed:    bool,
                     coords:    &mut Vec<Pnt3<N>>,
                     indices:   &mut Vec<Vec3<u32>>);

    /// Generates the mesh for the cap at the end of a path.
    fn gen_end_cap(&self,
                   attach_id: u32,
                   pattern:   &Polyline<N, Pnt3<N>, Vec3<N>>,
                   pt:        &Pnt3<N>,
                   dir:       &Vec3<N>,
                   closed:    bool,
                   coords:    &mut Vec<Pnt3<N>>,
                   indices:   &mut Vec<Vec3<u32>>);
}

impl<N, C1, C2> PolylinePattern<N, C1, C2>
    where N: Scalar,
          C1: PolylineCompatibleCap<N>,
          C2: PolylineCompatibleCap<N> {
    /// Creates a new polyline pattern.
    pub fn new(pattern:   &Polyline<N, Pnt2<N>, Vec2<N>>,
               closed:    bool,
               start_cap: C1,
               end_cap:   C2)
               -> PolylinePattern<N, C1, C2> {
        let mut coords3d = Vec::with_capacity(pattern.coords.len());

        for v in pattern.coords.iter() {
            coords3d.push(Pnt3::new(v.x.clone(), v.y.clone(), na::zero()));
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

impl<N, C1, C2> StrokePattern<N, Pnt3<N>, Vec3<N>> for PolylinePattern<N, C1, C2>
    where N: Scalar,
          C1: PolylineCompatibleCap<N>,
          C2: PolylineCompatibleCap<N>{
    fn stroke<C>(&mut self, sampler: &mut C) -> TriMesh<N, Pnt3<N>, Vec3<N>>
        where C: CurveSampler<N, Pnt3<N>, Vec3<N>> {
        let mut vertices = Vec::new();
        let mut indices  = Vec::new();
        let npts         = self.pattern.coords.len() as u32;
        // FIXME: collect the normals too.
        // let mut normals  = Vec::new();

        loop {
            let next = sampler.next();

            // second match to add the inner triangles.
            match next {
                StartPoint(ref pt, ref dir) |
                InnerPoint(ref pt, ref dir) |
                EndPoint(ref pt, ref dir)   => {
                    let mut new_polyline = self.pattern.clone();
                    let mut transform    = Iso3::new(na::zero(), na::zero());

                    if dir.x.is_zero() && dir.z.is_zero() { // FIXME: this might not be enough to avoid singularities.
                        transform.look_at_z(pt, &(*pt + *dir), &Vec3::x());
                    }

                    else {
                        transform.look_at_z(pt, &(*pt + *dir), &Vec3::y());
                    }

                    new_polyline.transform_by(&transform);

                    let new_start_id = vertices.len() as u32;

                    vertices.extend(new_polyline.coords.into_iter());

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
                EndOfSample =>
                    return TriMesh::new(vertices, None, None, Some(IndexBuffer::Unified(indices)))
            }

            // third match to add the end cap
            // FIXME: this will fail with patterns having multiple starting and end points!
            match next {
                StartPoint(ref pt, ref dir) => {
                    self.start_cap.gen_start_cap(0, &self.pattern,
                                                 pt, dir, self.closed, &mut vertices, &mut indices);
                },
                EndPoint(ref pt, ref dir) => {
                    self.end_cap.gen_end_cap(vertices.len() as u32 - npts, &self.pattern,
                                             pt, dir, self.closed, &mut vertices, &mut indices);
                },
                _ => { }
            }
        }
    }
}
