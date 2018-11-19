use na::Real;
use math::Isometry;
use shape::FeatureId;
use utils::IsometryOps;
#[cfg(feature = "dim3")]
use shape::{TriMesh, TriMeshFace};
use query::{Contact, ContactPrediction};


/// Shape-specific contact validation.
pub trait ContactGeneratorShapeContext<N: Real> {
    /// Returns the feature ID corresponding to this shape's parent's feature.
    fn remap_feature(&self, feature: FeatureId) -> FeatureId;

    /// Checks that the given contact is valid for this shape, i.e., that it actually is an LMD.
    ///
    /// If `c.depth > 0` (penetration) this will return `true` without performing any other check.
    fn validate_lmd(&self, m: &Isometry<N>, feature: FeatureId, c: &Contact<N>, is_first: bool, prediction: &ContactPrediction<N>) -> bool;
}

#[cfg(feature = "dim3")]
pub struct TriMeshFaceContext<'a, N: Real> {
    mesh: &'a TriMesh<N>,
    face_id: usize,
}

#[cfg(feature = "dim3")]
impl<'a, N: Real> TriMeshFaceContext<'a, N> {
    pub fn new(mesh: &'a TriMesh<N>, face_id: usize) -> Self {
        TriMeshFaceContext {
            mesh, face_id
        }
    }
}

#[cfg(feature = "dim3")]
impl<'a, N: Real> ContactGeneratorShapeContext<N> for TriMeshFaceContext<'a, N> {
    fn remap_feature(&self, feature: FeatureId) -> FeatureId {
        let face = &self.mesh.faces()[self.face_id];
        match feature {
            FeatureId::Vertex(i) => FeatureId::Vertex(face.indices[i]),
            FeatureId::Edge(i) => FeatureId::Edge(face.edges[i]),
            FeatureId::Face(i) => {
                if i == 0 {
                    FeatureId::Face(self.face_id)
                } else {
                    FeatureId::Face(self.face_id + self.mesh.faces().len())
                }
            }
            FeatureId::Unknown => FeatureId::Unknown,
        }
    }

    fn validate_lmd(&self, m: &Isometry<N>, feature: FeatureId, c: &Contact<N>, is_first: bool, prediction: &ContactPrediction<N>) -> bool {
        if c.depth > N::zero() {
            true
        } else {
            let local_dir = m.inverse_transform_unit_vector(&c.normal);

            if is_first {
                self.mesh.tangent_cone_polar_contains_dir(feature, &local_dir, prediction.sin_angular1(), prediction.cos_angular1())
            } else {
                self.mesh.tangent_cone_polar_contains_dir(feature, &-local_dir, prediction.sin_angular2(), prediction.cos_angular2())
            }
        }
    }
}