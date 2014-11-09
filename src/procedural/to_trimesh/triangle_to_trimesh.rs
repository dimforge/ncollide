use na::{Pnt3, Vec3};
use shape::Triangle3;
use procedural::{ToTriMesh, TriMesh};
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, ()> for Triangle3<N> {
    fn to_trimesh(&self, _: ()) -> TriMesh<N, Pnt3<N>, Vec3<N>> {
        TriMesh::new(
            vec!(self.a().clone(), self.b().clone(), self.c().clone()),
            None,
            None,
            None)
    }
}
