use crate::bounding_volume::ConicalApproximation;
use crate::math::{Isometry, Point, Vector};
use na::{Real, Unit};
use crate::shape::{ConvexPolygonalFeature, SupportMap};

/// An identifier of a feature of a convex polyhedron.
///
/// This identifier is shape-dependent and is seach that it
/// allows an efficient retrieval of the geometric information of the
/// feature.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub enum FeatureId {
    /// Shape-dependent identifier of a vertex.
    Vertex(usize),
    #[cfg(feature = "dim3")]
    /// Shape-dependent identifier of an edge.
    Edge(usize),
    /// Shape-dependent identifier of a face.
    Face(usize),
    // XXX: remove this variant.
    /// Unknown identifier.
    Unknown,
}

impl FeatureId {
    /// Revries the value of the identifier if `self` is a vertex.
    pub fn unwrap_vertex(self) -> usize {
        match self {
            FeatureId::Vertex(id) => id,
            _ => panic!("The feature id does not identify a vertex."),
        }
    }

    /// Revries the value of the identifier if `self` is an edge.
    #[cfg(feature = "dim3")]
    pub fn unwrap_edge(self) -> usize {
        match self {
            FeatureId::Edge(id) => id,
            _ => panic!("The feature id does not identify an edge."),
        }
    }

    /// Retrieves the value of the identifier if `self` is a face.
    pub fn unwrap_face(self) -> usize {
        match self {
            FeatureId::Face(id) => id,
            _ => panic!("The feature id does not identify a face."),
        }
    }
}

/// Trait implemented by all convex polyhedron.
pub trait ConvexPolyhedron<N: Real>: SupportMap<N> {
    /// Gets the specified vertex in the shape local-space.
    fn vertex(&self, id: FeatureId) -> Point<N>;
    /// Fill `face` with the geometric description of the specified face, in the shape's local-space.
    fn face(&self, id: FeatureId, face: &mut ConvexPolygonalFeature<N>);
    #[cfg(feature = "dim3")]
    /// Get the specified edge's vertices (in the shape local-space) and the vertices' identifiers.
    fn edge(&self, id: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId);

    /// Get the normal cone of the specified feature, in the shape's local-space.
    fn normal_cone(&self, feature: FeatureId) -> ConicalApproximation<N>;

    /// Retrieve the face (in world-space) with a normal that maximizes the scalar product with `dir`.
    fn support_face_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        out: &mut ConvexPolygonalFeature<N>,
    );

    /// Retrieve the feature (in world-space) which normal cone contains `dir`.
    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        _angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    );

    /// Retrieve the identifier of the feature which normal cone contains `dir`.
    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId;
}
