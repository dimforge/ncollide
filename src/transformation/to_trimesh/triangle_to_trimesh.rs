use super::ToTriMesh;
use crate::procedural::TriMesh;
use crate::shape::Triangle;
use simba::scalar::RealField;

impl<N: RealField> ToTriMesh<N> for Triangle<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        TriMesh::new(vec![self.a, self.b, self.c], None, None, None)
    }
}
