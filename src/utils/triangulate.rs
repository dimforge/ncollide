//! Point cloud triangulation.

use std::collections::hashmap::HashMap;
use nalgebra::na::{FloatVec, FloatVecExt, Cast, Vec3};
use nalgebra::na;
use procedural::{TriMesh, UnifiedIndexBuffer};
use utils;
use bounding_volume;

struct Triangle<N, V> {
    idx:                    Vec3<uint>,
    circumcircle_center:    V,
    circumcircle_sq_radius: N
}

impl<N: Float + Cast<f64>, V: FloatVec<N>> Triangle<N, V> {
    pub fn new(idx: Vec3<uint>, pts: &[V]) -> Triangle<N, V> {
        let pa = &pts[idx.x];
        let pb = &pts[idx.y];
        let pc = &pts[idx.z];

        let (center, radius) = utils::circumcircle(pa, pb, pc);

        Triangle {
            idx:                    idx,
            circumcircle_center:    center,
            circumcircle_sq_radius: radius * radius
        }

    }

    pub fn circumcircle_contains_point(&self, pt: &V) -> bool {
        na::sqnorm(&(*pt - self.circumcircle_center)) <= self.circumcircle_sq_radius
    }
}

/// Incremental triangulation utility.
pub struct Triangulator<N, V> {
    vertices:  Vec<V>,
    triangles: Vec<Triangle<N, V>>,
    edges:     HashMap<(uint, uint), uint>
}

impl<N: Float + Cast<f64>, V: FloatVec<N>> Triangulator<N, V> {
    /// Creates a new Triangulator.
    pub fn new(supertriangle_a: V, supertriangle_b: V, supertriangle_c: V) -> Triangulator<N, V> {
        let vertices = vec!(supertriangle_a, supertriangle_b, supertriangle_c);

        Triangulator {
            triangles: vec!(Triangle::new(Vec3::new(0, 1, 2), vertices.as_slice())),
            vertices:  vertices,
            edges:     HashMap::new()
        }
    }

    /// Adds a point to the triangulated set.
    pub fn add_point(&mut self, pt: V) {
        self.remove_containing_triangles_and_register_edges(&pt);

        let ipt = self.vertices.len();
        self.vertices.push(pt);

        for (&(ia, ib), num) in self.edges.iter() {
            if *num == 1 {
                let t = Triangle::new(Vec3::new(ia, ib, ipt), self.vertices.as_slice());

                self.triangles.push(t)
            }
        }
    }

    /// Returns the result of the triangulation.
    pub fn to_trimesh(mut self) -> TriMesh<N, V> {
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

                idx.push(na::cast(shifted_idx));
            }
        }

        TriMesh::new(self.vertices, None, None, Some(UnifiedIndexBuffer(idx)))
    }

    fn remove_containing_triangles_and_register_edges(&mut self, pt: &V) {
        self.edges.clear();

        let mut i = 0;

        while i != self.triangles.len() { // the len might change inside of the loop
            if self.triangles.get(i).circumcircle_contains_point(pt) {
                {
                    let t = self.triangles.get(i);

                    fn s(a: uint, b: uint) -> (uint, uint) {
                        if a > b { (b, a) } else { (a, b) }
                    }

                    *self.edges.find_or_insert(s(t.idx.x, t.idx.y), 0) += 1;
                    *self.edges.find_or_insert(s(t.idx.y, t.idx.z), 0) += 1;
                    *self.edges.find_or_insert(s(t.idx.z, t.idx.x), 0) += 1;
                }

                let _ = self.triangles.swap_remove(i);
            }
            else {
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
pub fn triangulate<N: FloatMath + Cast<f64>, V: FloatVecExt<N> + Clone>(pts: &[V]) -> TriMesh<N, V> {
    //// Compute the super-triangle
    let (center, radius) = bounding_volume::bounding_sphere(pts);
    let radius           = radius * na::cast(2.0);

    // Compute a triangle with (center, radius) as its inscribed circle.
    let pi: N       = Float::pi();
    let right_shift = radius / (pi / na::cast(6.0)).tan();
    let up_shift    = (right_shift * right_shift + radius * radius).sqrt();

    let mut up = na::zero::<V>();
    up.set(0, na::one());

    let mut right = na::zero::<V>();
    right.set(1, na::one());

    // Triangle:
    //
    //              top
    //
    //
    //         bleft    bright
    //
    let top    = center + up * up_shift;
    let bright = center - up * radius + right * right_shift;
    let bleft  = center - up * radius - right * right_shift;

    //// Build the triangulator.
    let mut triangulator = Triangulator::new(top, bright, bleft);

    for pt in pts.iter() {
        triangulator.add_point(pt.clone());
    }

    triangulator.to_trimesh()
}
