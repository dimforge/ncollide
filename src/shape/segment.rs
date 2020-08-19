//! Definition of the segment shape.

use crate::math::{Isometry, Point, Vector};
use crate::shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId, SupportMap};
#[cfg(feature = "dim2")]
use crate::utils;
use crate::utils::IsometryOps;
use na::{self, RealField, Unit};
use std::f64;
use std::mem;

/// A segment shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Segment<N: RealField> {
    /// The segment first point.
    pub a: Point<N>,
    /// The segment second point.
    pub b: Point<N>,
}

/// Logical description of the location of a point on a triangle.
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SegmentPointLocation<N: RealField> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on the segment interior.
    OnEdge([N; 2]),
}

impl<N: RealField> SegmentPointLocation<N> {
    /// The barycentric coordinates corresponding to this point location.
    pub fn barycentric_coordinates(&self) -> [N; 2] {
        let mut bcoords = [N::zero(); 2];

        match self {
            SegmentPointLocation::OnVertex(i) => bcoords[*i] = N::one(),
            SegmentPointLocation::OnEdge(uv) => {
                bcoords[0] = uv[0];
                bcoords[1] = uv[1];
            }
        }

        bcoords
    }
}

impl<N: RealField> Segment<N> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: Point<N>, b: Point<N>) -> Segment<N> {
        Segment { a, b }
    }

    /// Creates the reference to a segment from the reference to an array of two points.
    pub fn from_array(arr: &[Point<N>; 2]) -> &Segment<N> {
        unsafe { mem::transmute(arr) }
    }
}

impl<N: RealField> Segment<N> {
    /// The first point of this segment.
    #[inline]
    #[deprecated(note = "use the `self.a` public field directly.")]
    pub fn a(&self) -> &Point<N> {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    #[deprecated(note = "use the `self.b` public field directly.")]
    pub fn b(&self) -> &Point<N> {
        &self.b
    }
}

impl<N: RealField> Segment<N> {
    /// The direction of this segment scaled by its length.
    ///
    /// Points from `self.a` toward `self.b`.
    pub fn scaled_direction(&self) -> Vector<N> {
        self.b - self.a
    }

    /// The length of this segment.
    pub fn length(&self) -> N {
        self.scaled_direction().norm()
    }

    /// Swaps the two vertices of this segment.
    pub fn swap(&mut self) {
        mem::swap(&mut self.a, &mut self.b)
    }

    /// The unit direction of this segment.
    ///
    /// Points from `self.a()` toward `self.b()`.
    /// Returns `None` is both points are equal.
    pub fn direction(&self) -> Option<Unit<Vector<N>>> {
        Unit::try_new(self.scaled_direction(), N::default_epsilon())
    }

    /// In 2D, the not-normalized counterclockwise normal of this segment.
    #[cfg(feature = "dim2")]
    pub fn scaled_normal(&self) -> Vector<N> {
        let dir = self.scaled_direction();
        Vector::new(dir.y, -dir.x)
    }

    /// In 2D, the normalized counterclockwise normal of this segment.
    #[cfg(feature = "dim2")]
    pub fn normal(&self) -> Option<Unit<Vector<N>>> {
        Unit::try_new(self.scaled_normal(), N::default_epsilon())
    }

    /// Returns `None`. Exists only for API similarity with the 2D ncollide.
    #[cfg(feature = "dim3")]
    pub fn normal(&self) -> Option<Unit<Vector<N>>> {
        None
    }

    /// Applies the isometry `m` to the vertices of this segment and returns the resulting segment.
    pub fn transformed(&self, m: &Isometry<N>) -> Self {
        Segment::new(m * self.a, m * self.b)
    }

    /// Computes the point at the given location.
    pub fn point_at(&self, location: &SegmentPointLocation<N>) -> Point<N> {
        match *location {
            SegmentPointLocation::OnVertex(0) => self.a,
            SegmentPointLocation::OnVertex(1) => self.b,
            SegmentPointLocation::OnEdge(bcoords) => {
                self.a * bcoords[0] + self.b.coords * bcoords[1]
            }
            _ => panic!(),
        }
    }

    /// Checks that the given direction in world-space is on the tangent cone of the given `feature`.
    pub fn tangent_cone_contains_dir(
        &self,
        feature: FeatureId,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
    ) -> bool {
        let ls_dir = m.inverse_transform_unit_vector(dir);

        if let Some(direction) = self.direction() {
            match feature {
                FeatureId::Vertex(id) => {
                    let dot = ls_dir.dot(&direction);
                    if id == 0 {
                        dot >= N::one() - N::default_epsilon()
                    } else {
                        -dot >= N::one() - N::default_epsilon()
                    }
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge(_) => {
                    ls_dir.dot(&direction).abs() >= N::one() - N::default_epsilon()
                }
                FeatureId::Face(id) => {
                    let mut dir = Vector::zeros();
                    if id == 0 {
                        dir[0] = direction[1];
                        dir[1] = -direction[0];
                    } else {
                        dir[0] = -direction[1];
                        dir[1] = direction[0];
                    }

                    ls_dir.dot(&dir) <= N::zero()
                }
                _ => true,
            }
        } else {
            false
        }
    }
}

impl<N: RealField> SupportMap<N> for Segment<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        if self.a.coords.dot(dir) > self.b.coords.dot(dir) {
            self.a
        } else {
            self.b
        }
    }
}

impl<N: RealField> ConvexPolyhedron<N> for Segment<N> {
    fn vertex(&self, id: FeatureId) -> Point<N> {
        if id.unwrap_vertex() == 0 {
            self.a
        } else {
            self.b
        }
    }

    #[cfg(feature = "dim3")]
    fn edge(&self, _: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId) {
        (self.a, self.b, FeatureId::Vertex(0), FeatureId::Vertex(1))
    }

    #[cfg(feature = "dim3")]
    fn face(&self, _: FeatureId, _: &mut ConvexPolygonalFeature<N>) {
        panic!("A segment does not have any face in dimensions higher than 2.")
    }

    #[cfg(feature = "dim2")]
    fn face(&self, id: FeatureId, face: &mut ConvexPolygonalFeature<N>) {
        face.clear();

        if let Some(normal) = utils::ccw_face_normal([&self.a, &self.b]) {
            face.set_feature_id(id);

            match id.unwrap_face() {
                0 => {
                    face.push(self.a, FeatureId::Vertex(0));
                    face.push(self.b, FeatureId::Vertex(1));
                    face.set_normal(normal);
                }
                1 => {
                    face.push(self.b, FeatureId::Vertex(1));
                    face.push(self.a, FeatureId::Vertex(0));
                    face.set_normal(-normal);
                }
                _ => unreachable!(),
            }
        } else {
            face.push(self.a, FeatureId::Vertex(0));
            face.set_feature_id(FeatureId::Vertex(0));
        }
    }

    fn feature_normal(&self, feature: FeatureId) -> Unit<Vector<N>> {
        if let Some(direction) = self.direction() {
            match feature {
                FeatureId::Vertex(id) => {
                    if id == 0 {
                        direction
                    } else {
                        -direction
                    }
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge(_) => {
                    let iamin = direction.iamin();
                    let mut normal = Vector::zeros();
                    normal[iamin] = N::one();
                    normal -= *direction * direction[iamin];
                    Unit::new_normalize(normal)
                }
                FeatureId::Face(id) => {
                    let mut dir = Vector::zeros();
                    if id == 0 {
                        dir[0] = direction[1];
                        dir[1] = -direction[0];
                    } else {
                        dir[0] = -direction[1];
                        dir[1] = direction[0];
                    }
                    Unit::new_unchecked(dir)
                }
                _ => panic!("Invalid feature ID: {:?}", feature),
            }
        } else {
            Vector::y_axis()
        }
    }

    #[cfg(feature = "dim2")]
    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        face: &mut ConvexPolygonalFeature<N>,
    ) {
        let seg_dir = self.scaled_direction();

        if dir.perp(&seg_dir) >= na::zero() {
            self.face(FeatureId::Face(0), face);
        } else {
            self.face(FeatureId::Face(1), face);
        }
        face.transform_by(m)
    }

    #[cfg(feature = "dim3")]
    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        _: &Unit<Vector<N>>,
        face: &mut ConvexPolygonalFeature<N>,
    ) {
        face.clear();
        face.push(self.a, FeatureId::Vertex(0));
        face.push(self.b, FeatureId::Vertex(1));
        face.push_edge_feature_id(FeatureId::Edge(0));
        face.set_feature_id(FeatureId::Edge(0));
        face.transform_by(m)
    }

    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        eps: N,
        face: &mut ConvexPolygonalFeature<N>,
    ) {
        face.clear();
        let seg = self.transformed(transform);
        let ceps = eps.sin();

        if let Some(seg_dir) = seg.direction() {
            let cang = dir.dot(&seg_dir);

            if cang > ceps {
                face.set_feature_id(FeatureId::Vertex(1));
                face.push(seg.b, FeatureId::Vertex(1));
            } else if cang < -ceps {
                face.set_feature_id(FeatureId::Vertex(0));
                face.push(seg.a, FeatureId::Vertex(0));
            } else {
                #[cfg(feature = "dim3")]
                {
                    face.push(seg.a, FeatureId::Vertex(0));
                    face.push(seg.b, FeatureId::Vertex(1));
                    face.push_edge_feature_id(FeatureId::Edge(0));
                    face.set_feature_id(FeatureId::Edge(0));
                }
                #[cfg(feature = "dim2")]
                {
                    if dir.perp(&seg_dir) >= na::zero() {
                        seg.face(FeatureId::Face(0), face);
                    } else {
                        seg.face(FeatureId::Face(1), face);
                    }
                }
            }
        }
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        if let Some(seg_dir) = self.direction() {
            let eps: N = na::convert(f64::consts::PI / 180.0);
            let seps = eps.sin();
            let dot = seg_dir.dot(local_dir.as_ref());

            if dot <= seps {
                #[cfg(feature = "dim2")]
                {
                    if local_dir.perp(seg_dir.as_ref()) >= na::zero() {
                        FeatureId::Face(0)
                    } else {
                        FeatureId::Face(1)
                    }
                }
                #[cfg(feature = "dim3")]
                {
                    FeatureId::Edge(0)
                }
            } else if dot >= na::zero() {
                FeatureId::Vertex(1)
            } else {
                FeatureId::Vertex(0)
            }
        } else {
            FeatureId::Vertex(0)
        }
    }
}
