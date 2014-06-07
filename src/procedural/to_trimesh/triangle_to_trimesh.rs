use std::num::Zero;
use geom::Triangle;
use procedural::{ToTriMesh, TriMesh};
use math::{Scalar, Vect};

impl ToTriMesh<()> for Triangle {
    fn to_trimesh(&self, _: ()) -> TriMesh<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Mesh generation for triangles with non-zero margin is not yet implemented.");

        TriMesh::new(
            vec!(self.a().clone(), self.b().clone(), self.c().clone()),
            None,
            None,
            None)
    }
}
