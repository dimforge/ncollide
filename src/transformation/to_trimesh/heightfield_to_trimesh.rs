use alga::general::Real;
use crate::shape;
use crate::procedural::TriMesh;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for shape::HeightField<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let mut vertices = Vec::new();

        for tri in self.triangles() {
            vertices.push(*tri.a());
            vertices.push(*tri.b());
            vertices.push(*tri.c());
        }

        /*
        for i in 0..self.nrows() {
            for j in 0..self.ncols() {
                let triangles = self.triangles_at(i, j);

                if let Some(tri) = triangles.0 {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }

                if let Some(tri) = triangles.1 {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }
            }
        }
        */

        TriMesh::new(vertices, None, None, None)
    }
}
