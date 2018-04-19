use alga::general::Real;
use na::Point;
use shape::Triangle;
use procedural::TriMesh;
use super::ToTriMesh;

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
