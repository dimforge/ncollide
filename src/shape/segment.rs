//! Definition of the segment shape.

use bounding_volume::PolyhedralCone;
use math::{Isometry, Point, Vector};
use na::{self, Real, Unit};
use shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId, SupportMap};
use std::f64;
use std::mem;
#[cfg(feature = "dim2")]
use utils;
use utils::IsometryOps;

/// A segment shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Segment<N: Real> {
    a: Point<N>,
    b: Point<N>,
}

/// Logical description of the location of a point on a triangle.
pub enum SegmentPointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on the segment interior.
    OnEdge([N; 2]),
}

impl<N: Real> Segment<N> {
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

impl<N: Real> Segment<N> {
    /// The first point of this segment.
    #[inline]
    pub fn a(&self) -> &Point<N> {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b(&self) -> &Point<N> {
        &self.b
    }
}

impl<N: Real> Segment<N> {
    /// The direction of this segment scaled by its length.
    ///
    /// Points from `self.a()` toward `self.b()`.
    pub fn scaled_direction(&self) -> Vector<N> {
        self.b - self.a
    }

    /// The length of this segment.
    pub fn length(&self) -> N {
        na::norm(&self.scaled_direction())
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
            _ => unreachable!(),
        }
    }
}

impl<N: Real> SupportMap<N> for Segment<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        if na::dot(&self.a.coords, dir) > na::dot(&self.b.coords, dir) {
            self.a
        } else {
            self.b
        }
    }
}

impl<N: Real> ConvexPolyhedron<N> for Segment<N> {
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
        panic!("A segment does not have any face indimensions higher than 2.")
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

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        if let Some(direction) = self.direction() {
            match feature {
                FeatureId::Vertex(id) => {
                    if id == 0 {
                        PolyhedralCone::HalfSpace(direction)
                    } else {
                        PolyhedralCone::HalfSpace(-direction)
                    }
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge(_) => PolyhedralCone::OrthogonalSubspace(direction),
                FeatureId::Face(id) => {
                    let mut dir = Vector::zeros();
                    if id == 0 {
                        dir[0] = direction[1];
                        dir[1] = -direction[0];
                    } else {
                        dir[0] = -direction[1];
                        dir[1] = direction[0];
                    }
                    PolyhedralCone::HalfLine(Unit::new_unchecked(dir))
                }
                _ => PolyhedralCone::Empty,
            }
        } else {
            PolyhedralCone::Full
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
        _angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        if let Some(seg_dir) = self.direction() {
            let eps: N = na::convert(f64::consts::PI / 180.0);
            let seps = eps.sin();
            let dot = na::dot(seg_dir.as_ref(), local_dir.as_ref());

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
