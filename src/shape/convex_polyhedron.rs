use bounding_volume::PolyhedralCone;
use math::{Isometry, Point, Vector};
use na::{Real, Unit};
use shape::{ConvexPolyface, SupportMap};

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub enum FeatureId {
    Vertex(usize),
    #[cfg(feature = "dim3")]
    Edge(usize),
    Face(usize),
    // XXX: remove this variant.
    Unknown,
}

impl FeatureId {
    pub fn unwrap_vertex(self) -> usize {
        match self {
            FeatureId::Vertex(id) => id,
            _ => panic!("The feature id does not identify a vertex."),
        }
    }

    #[cfg(feature = "dim3")]
    pub fn unwrap_edge(self) -> usize {
        match self {
            FeatureId::Edge(id) => id,
            _ => panic!("The feature id does not identify an edge."),
        }
    }

    pub fn unwrap_face(self) -> usize {
        match self {
            FeatureId::Face(id) => id,
            _ => panic!("The feature id does not identify a face."),
        }
    }
}

pub trait ConvexPolyhedron<N: Real>: SupportMap<N> {
    fn vertex(&self, id: FeatureId) -> Point<N>;
    fn face(&self, id: FeatureId, face: &mut ConvexPolyface<N>);
    #[cfg(feature = "dim3")]
    fn edge(&self, id: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId);

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N>;

    fn support_face_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        out: &mut ConvexPolyface<N>,
    );

    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        _angle: N,
        out: &mut ConvexPolyface<N>,
    );

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId;
}
