use super::ToTriMesh;
use crate::procedural::TriMesh;
use crate::shape;
use simba::scalar::RealField;

impl<N: RealField> ToTriMesh<N> for shape::HeightField<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let mut vertices = Vec::new();

        for tri in self.triangles() {
            vertices.push(tri.a);
            vertices.push(tri.b);
            vertices.push(tri.c);
        }

        TriMesh::new(vertices, None, None, None)
    }
}
