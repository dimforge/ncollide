use super::ToTriMesh;
use alga::general::Real;
use procedural::TriMesh;
use shape::Triangle;

impl<N: Real> ToTriMesh<N> for Triangle<N> {
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
