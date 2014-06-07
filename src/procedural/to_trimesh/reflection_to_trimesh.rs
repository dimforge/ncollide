use geom::Reflection;
use procedural::{ToTriMesh, TriMesh};
use math::{Scalar, Vect};


impl<'a, G: ToTriMesh<P>, P> ToTriMesh<P> for Reflection<'a, G> {
    fn to_trimesh(&self, parameter: P) -> TriMesh<Scalar, Vect> {
        let mut res = self.geom().to_trimesh(parameter);

        for c in res.coords.mut_iter() {
            *c = -*c;
        }

        res
    }
}
