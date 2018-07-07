//! Definition of the triangle shape.

#[cfg(feature = "dim3")]
use std::f64;
use std::mem;
use na::{self, Unit};
use na::Real;
use shape::SupportMap;
use math::{Isometry, Point, Vector};
use utils::IsometryOps;

#[cfg(feature = "dim3")]
use smallvec::SmallVec;
#[cfg(feature = "dim3")]
use shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId};
#[cfg(feature = "dim3")]
use bounding_volume::PolyhedralCone;

/// A triangle shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Triangle<N: Real> {
    a: Point<N>,
    b: Point<N>,
    c: Point<N>,
}

/// Description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TrianglePointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on an edge.
    OnEdge(usize, [N; 2]),
    /// The point lies on the triangle interior.
    OnFace([N; 3]),
    /// The point lies on the triangle interior (for "solid" point queries).
    OnSolid,
}

impl<N: Real> TrianglePointLocation<N> {
    /// Returns `true` if the point is located on the relative interior of the triangle.
    pub fn is_on_face(&self) -> bool {
        if let TrianglePointLocation::OnFace(_) = *self {
            true
        } else {
            false
        }
    }
}

impl<N: Real> Triangle<N> {
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
    pub fn as_array(&self) -> &[Point<N>; 3] {
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
}

impl<N: Real> SupportMap<N> for Triangle<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        let d1 = na::dot(&self.a().coords, dir);
        let d2 = na::dot(&self.b().coords, dir);
        let d3 = na::dot(&self.c().coords, dir);

        if d1 > d2 {
            if d1 > d3 {
                self.a
            } else {
                self.c
            }
        } else {
            if d2 > d3 {
                self.b
            } else {
                self.c
            }
        }
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> ConvexPolyhedron<N> for Triangle<N> {
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

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        if let Some(normal) = self.normal() {
            match feature {
                FeatureId::Vertex(id2) => {
                    let vtx = self.as_array();
                    let mut generators = SmallVec::new();
                    let id1 = if id2 == 0 { 2 } else { id2 - 1 };
                    let id3 = (id2 + 1) % 3;

                    if let Some(side1) = Unit::try_new(vtx[id2] - vtx[id1], N::default_epsilon()) {
                        generators.push(side1);
                    }
                    generators.push(-normal);
                    if let Some(side2) = Unit::try_new(vtx[id3] - vtx[id2], N::default_epsilon()) {
                        generators.push(side2);
                    }
                    generators.push(normal);

                    // FIXME: is it meaningful not to push the sides if the triangle is degenerate?

                    PolyhedralCone::Span(generators)
                }
                FeatureId::Edge(id1) => {
                    // FIXME: We should be able to do much better here.
                    let id2 = (id1 + 1) % 3;
                    let vtx = self.as_array();
                    let mut generators = SmallVec::new();

                    if let Some(side) = Unit::try_new(vtx[id2] - vtx[id1], N::default_epsilon()) {
                        generators.push(side);
                        generators.push(-normal);
                        generators.push(side);
                        generators.push(normal);
                    } else {
                        // FIXME: is this meaningful?
                        generators.push(-normal);
                        generators.push(normal);
                    }

                    PolyhedralCone::Span(generators)
                }
                FeatureId::Face(0) => PolyhedralCone::HalfLine(normal),
                FeatureId::Face(1) => PolyhedralCone::HalfLine(-normal),
                _ => panic!("Invalid feature ID."),
            }
        } else {
            PolyhedralCone::Full
        }
    }

    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        face: &mut ConvexPolygonalFeature<N>,
    ) {
        let normal = self.scaled_normal();

        if na::dot(&normal, &*dir) >= na::zero() {
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
        _angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        if let Some(normal) = self.normal() {
            let eps: N = na::convert(f64::consts::PI / 180.0);
            let ceps = eps.cos();

            let normal_dot = local_dir.dot(&*normal);
            if normal_dot >= ceps {
                FeatureId::Face(0)
            } else if normal_dot <= -ceps {
                FeatureId::Face(1)
            } else {
                let dot1 = local_dir.dot(&self.a.coords);
                let dot2 = local_dir.dot(&self.b.coords);
                let dot3 = local_dir.dot(&self.c.coords);

                if dot1 > dot2 {
                    if dot1 > dot3 {
                        FeatureId::Vertex(0)
                    } else {
                        FeatureId::Vertex(2)
                    }
                } else if dot2 > dot3 {
                    FeatureId::Vertex(1)
                } else {
                    FeatureId::Vertex(2)
                }
            }
        } else {
            FeatureId::Vertex(0)
        }
    }
}
