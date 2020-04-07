//! Definition of the triangle shape.

use crate::math::{Isometry, Point, Vector};
use crate::shape::Segment;
use crate::shape::SupportMap;
#[cfg(feature = "dim3")]
use crate::shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId};
use na::RealField;
use na::{self, Unit};
#[cfg(feature = "dim3")]
use std::f64;
use std::mem;

/// A triangle shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
#[derive(PartialEq, Debug, Clone)]
pub struct Triangle<N: RealField> {
    a: Point<N>,
    b: Point<N>,
    c: Point<N>,
}

/// Description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TrianglePointLocation<N: RealField> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on an edge.
    ///
    /// The 0-st edge is the segment AB.
    /// The 1-st edge is the segment BC.
    /// The 2-nd edge is the segment AC.
    // XXX: it appears the conversion of edge indexing here does not match the
    // convension of edge indexing for the `fn edge` method (from the ConvexPolyhedron impl).
    OnEdge(usize, [N; 2]),
    /// The point lies on the triangle interior.
    ///
    /// The integer indicates on which side of the face the point is. 0 indicates the point
    /// is on the half-space toward the CW normal of the triangle. 1 indicates the point is on the other
    /// half-space. This is always set to 0 in 2D.
    OnFace(usize, [N; 3]),
    /// The point lies on the triangle interior (for "solid" point queries).
    OnSolid,
}

impl<N: RealField> TrianglePointLocation<N> {
    /// The barycentric coordinates corresponding to this point location.
    ///
    /// Returns `None` if the location is `TrianglePointLocation::OnSolid`.
    pub fn barycentric_coordinates(&self) -> Option<[N; 3]> {
        let mut bcoords = [N::zero(); 3];

        match self {
            TrianglePointLocation::OnVertex(i) => bcoords[*i] = N::one(),
            TrianglePointLocation::OnEdge(i, uv) => {
                let idx = match i {
                    0 => (0, 1),
                    1 => (1, 2),
                    2 => (0, 2),
                    _ => unreachable!(),
                };

                bcoords[idx.0] = uv[0];
                bcoords[idx.1] = uv[1];
            }
            TrianglePointLocation::OnFace(_, uvw) => {
                bcoords[0] = uvw[0];
                bcoords[1] = uvw[1];
                bcoords[2] = uvw[2];
            }
            TrianglePointLocation::OnSolid => {
                return None;
            }
        }

        Some(bcoords)
    }

    /// Returns `true` if the point is located on the relative interior of the triangle.
    pub fn is_on_face(&self) -> bool {
        if let TrianglePointLocation::OnFace(..) = *self {
            true
        } else {
            false
        }
    }
}

impl<N: RealField> Triangle<N> {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: Point<N>, b: Point<N>, c: Point<N>) -> Triangle<N> {
        Triangle { a, b, c }
    }

    /// Creates the reference to a triangle from the reference to an array of three points.
    pub fn from_array(arr: &[Point<N>; 3]) -> &Triangle<N> {
        unsafe { mem::transmute(arr) }
    }

    /// The fist point of this triangle.
    #[inline]
    pub fn a(&self) -> &Point<N> {
        &self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b(&self) -> &Point<N> {
        &self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c(&self) -> &Point<N> {
        &self.c
    }

    /// Reference to an array containing the three vertices of this triangle.
    #[inline]
    pub fn vertices(&self) -> &[Point<N>; 3] {
        unsafe { mem::transmute(self) }
    }

    /// The normal of this triangle assuming it is oriented ccw.
    ///
    /// The normal points such that it is collinear to `AB × AC` (where `×` denotes the cross
    /// product).
    #[inline]
    pub fn normal(&self) -> Option<Unit<Vector<N>>> {
        Unit::try_new(self.scaled_normal(), N::default_epsilon())
    }

    /// The three edges of this triangle: [AB, BC, CA].
    #[inline]
    pub fn edges(&self) -> [Segment<N>; 3] {
        [
            Segment::new(self.a, self.b),
            Segment::new(self.b, self.c),
            Segment::new(self.c, self.a),
        ]
    }

    /// Returns a new triangle with vertices transformed by `m`.
    #[inline]
    pub fn transformed(&self, m: &Isometry<N>) -> Self {
        Triangle::new(m * self.a, m * self.b, m * self.c)
    }
    /// The three edges scaled directions of this triangle: [B - A, C - B, A - C].
    #[inline]
    pub fn edges_scaled_directions(&self) -> [Vector<N>; 3] {
        [self.b - self.a, self.c - self.b, self.a - self.c]
    }

    /// A vector normal of this triangle.
    ///
    /// The vector points such that it is collinear to `AB × AC` (where `×` denotes the cross
    /// product).
    #[inline]
    pub fn scaled_normal(&self) -> Vector<N> {
        let ab = self.b - self.a;
        let ac = self.c - self.a;
        ab.cross(&ac)
    }

    /// Computes the extents of this triangle on the given direction.
    ///
    /// This computes the min and max values of the dot products between each
    /// vertex of this triangle and `dir`.
    #[inline]
    pub fn extents_on_dir(&self, dir: &Unit<Vector<N>>) -> (N, N) {
        let a = self.a.coords.dot(dir);
        let b = self.b.coords.dot(dir);
        let c = self.c.coords.dot(dir);

        if a > b {
            if b > c {
                (c, a)
            } else if a > c {
                (b, a)
            } else {
                (b, c)
            }
        } else {
            // b >= a
            if a > c {
                (c, b)
            } else if b > c {
                (a, b)
            } else {
                (a, c)
            }
        }
    }

    /// Checks that the given direction in world-space is on the tangent cone of the given `feature`.
    #[cfg(feature = "dim3")]
    #[inline]
    pub fn tangent_cone_contains_dir(
        &self,
        feature: FeatureId,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
    ) -> bool {
        let ls_dir = m.inverse_transform_vector(dir);

        if let Some(normal) = self.normal() {
            match feature {
                FeatureId::Vertex(_) => {
                    // FIXME: for now we assume since the triangle has no thickness,
                    // the case where `dir` is coplanar with the triangle never happens.
                    false
                }
                FeatureId::Edge(_) => {
                    // FIXME: for now we assume since the triangle has no thickness,
                    // the case where `dir` is coplanar with the triangle never happens.
                    false
                }
                FeatureId::Face(0) => ls_dir.dot(&normal) <= N::zero(),
                FeatureId::Face(1) => ls_dir.dot(&normal) >= N::zero(),
                _ => panic!("Invalid feature ID."),
            }
        } else {
            false
        }
    }

    #[cfg(feature = "dim3")]
    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>, eps: N) -> FeatureId {
        if let Some(normal) = self.normal() {
            let (seps, ceps) = eps.sin_cos();

            let normal_dot = local_dir.dot(&*normal);
            if normal_dot >= ceps {
                FeatureId::Face(0)
            } else if normal_dot <= -ceps {
                FeatureId::Face(1)
            } else {
                let edges = self.edges();
                let mut dots = [N::zero(); 3];

                let dir1 = edges[0].direction();
                if let Some(dir1) = dir1 {
                    dots[0] = dir1.dot(local_dir);

                    if dots[0].abs() < seps {
                        return FeatureId::Edge(0);
                    }
                }

                let dir2 = edges[1].direction();
                if let Some(dir2) = dir2 {
                    dots[1] = dir2.dot(local_dir);

                    if dots[1].abs() < seps {
                        return FeatureId::Edge(1);
                    }
                }

                let dir3 = edges[2].direction();
                if let Some(dir3) = dir3 {
                    dots[2] = dir3.dot(local_dir);

                    if dots[2].abs() < seps {
                        return FeatureId::Edge(2);
                    }
                }

                if dots[0] > N::zero() && dots[1] < N::zero() {
                    FeatureId::Vertex(1)
                } else if dots[1] > N::zero() && dots[2] < N::zero() {
                    FeatureId::Vertex(2)
                } else {
                    FeatureId::Vertex(0)
                }
            }
        } else {
            FeatureId::Vertex(0)
        }
    }
}

impl<N: RealField> SupportMap<N> for Triangle<N> {
    #[inline]
    fn support_point(&self, m: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        let local_dir = m.inverse_transform_vector(dir);

        let d1 = self.a().coords.dot(&local_dir);
        let d2 = self.b().coords.dot(&local_dir);
        let d3 = self.c().coords.dot(&local_dir);

        let res = if d1 > d2 {
            if d1 > d3 {
                self.a()
            } else {
                self.c()
            }
        } else {
            if d2 > d3 {
                self.b()
            } else {
                self.c()
            }
        };

        m * res
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> ConvexPolyhedron<N> for Triangle<N> {
    fn vertex(&self, id: FeatureId) -> Point<N> {
        match id.unwrap_vertex() {
            0 => self.a,
            1 => self.b,
            2 => self.c,
            _ => panic!("Triangle vertex index out of bounds."),
        }
    }
    fn edge(&self, id: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId) {
        match id.unwrap_edge() {
            0 => (self.a, self.b, FeatureId::Vertex(0), FeatureId::Vertex(1)),
            1 => (self.b, self.c, FeatureId::Vertex(1), FeatureId::Vertex(2)),
            2 => (self.c, self.a, FeatureId::Vertex(2), FeatureId::Vertex(0)),
            _ => panic!("Triangle edge index out of bounds."),
        }
    }

    fn face(&self, id: FeatureId, face: &mut ConvexPolygonalFeature<N>) {
        face.clear();

        if let Some(normal) = self.normal() {
            face.set_feature_id(id);

            match id.unwrap_face() {
                0 => {
                    face.push(self.a, FeatureId::Vertex(0));
                    face.push(self.b, FeatureId::Vertex(1));
                    face.push(self.c, FeatureId::Vertex(2));
                    face.push_edge_feature_id(FeatureId::Edge(0));
                    face.push_edge_feature_id(FeatureId::Edge(1));
                    face.push_edge_feature_id(FeatureId::Edge(2));
                    face.set_normal(normal);
                }
                1 => {
                    face.push(self.a, FeatureId::Vertex(0));
                    face.push(self.c, FeatureId::Vertex(2));
                    face.push(self.b, FeatureId::Vertex(1));
                    face.push_edge_feature_id(FeatureId::Edge(2));
                    face.push_edge_feature_id(FeatureId::Edge(1));
                    face.push_edge_feature_id(FeatureId::Edge(0));
                    face.set_normal(-normal);
                }
                _ => unreachable!(),
            }

            face.recompute_edge_normals();
        } else {
            face.push(self.a, FeatureId::Vertex(0));
            face.set_feature_id(FeatureId::Vertex(0));
        }
    }

    fn feature_normal(&self, _: FeatureId) -> Unit<Vector<N>> {
        if let Some(normal) = self.normal() {
            // FIXME: We should be able to do much better here.
            normal
        } else {
            Vector::y_axis()
        }
    }

    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        face: &mut ConvexPolygonalFeature<N>,
    ) {
        let normal = self.scaled_normal();

        if normal.dot(&*dir) >= na::zero() {
            ConvexPolyhedron::<N>::face(self, FeatureId::Face(0), face);
        } else {
            ConvexPolyhedron::<N>::face(self, FeatureId::Face(1), face);
        }
        face.transform_by(m)
    }

    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        eps: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        out.clear();
        let tri = self.transformed(transform);
        let feature = tri.support_feature_id_toward(dir, eps);

        match feature {
            FeatureId::Vertex(_) => {
                let v = tri.vertex(feature);
                out.push(v, feature);
                out.set_feature_id(feature);
            }
            FeatureId::Edge(_) => {
                let (a, b, fa, fb) = tri.edge(feature);
                out.push(a, fa);
                out.push(b, fb);
                out.push_edge_feature_id(feature);
                out.set_feature_id(feature);
            }
            FeatureId::Face(_) => tri.face(feature, out),
            _ => unreachable!(),
        }
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        self.support_feature_id_toward(local_dir, na::convert(f64::consts::PI / 180.0))
    }
}
