use alga::general::Real;
use na;
use crate::shape;
use crate::procedural::TriMesh;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for shape::HeightField<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let mut vertices = Vec::new();

        for i in 0..self.nrows() {
            for j in 0..self.ncols() {
                if let Some(tri) = self.triangles_at(i, j).0 {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }

                if let Some(tri) = self.triangles_at(i, j).1 {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }
            }
        }

        TriMesh::new(vertices, None, None, None)
    }
}
