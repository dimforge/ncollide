use geom::Reflection;
use procedural::{ToTriMesh, TriMesh};
use math::{Scalar, Point, Vect};


impl<'a, G: ToTriMesh<P>, P> ToTriMesh<P> for Reflection<'a, G> {
    fn to_trimesh(&self, parameter: P) -> TriMesh<Scalar, Point, Vect> {
        let mut res = self.geom().to_trimesh(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
