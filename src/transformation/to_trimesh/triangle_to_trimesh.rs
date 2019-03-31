use super::ToTriMesh;
use alga::general::RealField;
use crate::procedural::TriMesh;
use crate::shape::Triangle;

impl<N: RealField> ToTriMesh<N> for Triangle<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        TriMesh::new(
            vec![self.a().clone(), self.b().clone(), self.c().clone()],
            None,
            None,
            None,
        )
    }
}
