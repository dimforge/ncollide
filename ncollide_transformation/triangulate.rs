//! Point cloud triangulation.

use std::collections::HashMap;
use std::collections::hash_map::Entry;

use alga::general::Real;
use na::Point3;
use na;
use math::Point;
use utils;
use geometry::bounding_volume;
use procedural::{IndexBuffer, TriMesh};

struct Triangle<P: Point> {
    idx: Point3<usize>,
    circumcircle_center: P,
    circumcircle_sq_radius: P::Real,
}

impl<P: Point> Triangle<P> {
    pub fn new(idx: Point3<usize>, pts: &[P]) -> Triangle<P> {
        let pa = &pts[idx.x];
        let pb = &pts[idx.y];
        let pc = &pts[idx.z];

        let (center, radius) = utils::circumcircle(pa, pb, pc);

        Triangle {
            idx: idx,
            circumcircle_center: center,
            circumcircle_sq_radius: radius * radius,
        }
    }

    pub fn circumcircle_contains_point(&self, pt: &P) -> bool {
        na::distance_squared(pt, &self.circumcircle_center) <= self.circumcircle_sq_radius
    }
}

/// Incremental triangulation utility.
pub struct Triangulator<P: Point> {
    vertices: Vec<P>,
    triangles: Vec<Triangle<P>>,
    edges: HashMap<(usize, usize), usize>,
}

impl<P: Point> Triangulator<P> {
    /// Creates a new Triangulator.
    pub fn new(supertriangle_a: P, supertriangle_b: P, supertriangle_c: P) -> Triangulator<P> {
        let vertices = vec![supertriangle_a, supertriangle_b, supertriangle_c];

        Triangulator {
            // FIXME: why do we have to specify the type explicitely here ?
            triangles: vec![Triangle::<P>::new(Point3::new(0, 1, 2), &vertices[..])],
            vertices: vertices,
            edges: HashMap::new(),
        }
    }

    /// Adds a point to the triangulated set.
    pub fn add_point(&mut self, pt: P) {
        self.remove_containing_triangles_and_register_edges(&pt);

        let ipt = self.vertices.len();
        self.vertices.push(pt);

        for (&(ia, ib), num) in self.edges.iter() {
            if *num == 1 {
                // FIXME: why do we have to specify the type explicitely here ?
                let t = Triangle::<P>::new(Point3::new(ia, ib, ipt), &self.vertices[..]);

                self.triangles.push(t)
            }
        }
    }

    /// Returns the result of the triangulation.
    pub fn to_trimesh(mut self) -> TriMesh<P> {
        let mut idx = Vec::with_capacity(self.triangles.len());

        let _ = self.vertices.swap_remove(2);
        let _ = self.vertices.swap_remove(1);
        let _ = self.vertices.swap_remove(0);

        let num_ids = self.vertices.len();

        for t in self.triangles.iter() {
            if t.idx.x > 2 && t.idx.y > 2 && t.idx.z > 2 {
                let mut shifted_idx = t.idx;

                if shifted_idx.x >= num_ids {
                    shifted_idx.x -= num_ids;
                }

                if shifted_idx.y >= num_ids {
                    shifted_idx.y -= num_ids;
                }

                if shifted_idx.z >= num_ids {
                    shifted_idx.z -= num_ids;
                }

                idx.push(na::convert(shifted_idx));
            }
        }

        TriMesh::new(self.vertices, None, None, Some(IndexBuffer::Unified(idx)))
    }

    fn remove_containing_triangles_and_register_edges(&mut self, pt: &P) {
        self.edges.clear();

        let mut i = 0;

        while i != self.triangles.len() {
            // the len might change inside of the loop
            if self.triangles[i].circumcircle_contains_point(pt) {
                {
                    let t = &self.triangles[i];

                    fn s(a: usize, b: usize) -> (usize, usize) {
                        if a > b {
                            (b, a)
                        } else {
                            (a, b)
                        }
                    }

                    let edge_keys = [
                        s(t.idx.x, t.idx.y),
                        s(t.idx.y, t.idx.z),
                        s(t.idx.z, t.idx.x),
                    ];

                    for edge_key in edge_keys.iter() {
                        match self.edges.entry(*edge_key) {
                            Entry::Occupied(mut entry) => *entry.get_mut() += 1,
                            Entry::Vacant(entry) => {
                                let _ = entry.insert(1);
                            }
                        };
                    }
                }

                let _ = self.triangles.swap_remove(i);
            } else {
                i = i + 1;
            }
        }
    }
}

/// Triangulates a set of point (sort of) lying on the same 2d plane.
///
/// If the points do not lie on the same 2d plane, strange things might happends (triangle might be
/// attached together in an unnatural way). Though, if they are only slighly perturbated on the
/// directions orthogonal to the plane, this should be fine.
pub fn triangulate<P: Point>(pts: &[P]) -> TriMesh<P> {
    //// Compute the super-triangle
    let (center, radius) = bounding_volume::point_cloud_bounding_sphere(pts);
    let radius = radius * na::convert(2.0);

    // Compute a triangle with (center, radius) as its inscribed circle.
    let pi = P::Real::pi();
    let right_shift = radius / (pi / na::convert(6.0)).tan();
    let up_shift = (right_shift * right_shift + radius * radius).sqrt();

    let mut up = na::zero::<P::Vector>();
    up[0] = na::one();

    let mut right = na::zero::<P::Vector>();
    right[1] = na::one();

    // Triangle:
    //
    //              top
    //
    //
    //         bleft    bright
    //
    let top = center + up * up_shift;
    // FIXME: use `-` instead of `+ (-` when the trait refor is done.
    let bright = center + (-up * radius + right * right_shift);
    let bleft = center + (-up * radius - right * right_shift);

    //// Build the triangulator.
    let mut triangulator = Triangulator::new(top, bright, bleft);

    for pt in pts.iter() {
        triangulator.add_point(pt.clone());
    }

    triangulator.to_trimesh()
}
