use geom::GeomWithMargin;
use procedural::{ToTriMesh, TriMesh};
use math::{Scalar, Vect};


impl<'a, G: ToTriMesh<P>, P> ToTriMesh<P> for GeomWithMargin<'a, G> {
    fn to_trimesh(&self, parameter: P) -> TriMesh<Scalar, Vect> {
        self.geom().to_trimesh(parameter)
    }
}
