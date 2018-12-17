use alga::general::Real;
use na;
use na::Point3;
use shape;
use procedural::{self, IndexBuffer, TriMesh};
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for shape::HeightField<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let mut vertices = Vec::new();

        for i in 0..self.nrows() {
            for j in 0..self.ncols() {
                if let Some(tri) = self.triangle_at(i, j, true) {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }

                if let Some(tri) = self.triangle_at(i, j, false) {
                    vertices.push(*tri.a());
                    vertices.push(*tri.b());
                    vertices.push(*tri.c());
                }
            }
        }

        TriMesh::new(vertices, None, None, None)
    }
}
