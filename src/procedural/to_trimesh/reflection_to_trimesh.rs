use shape::Reflection;
use procedural::{ToTriMesh, TriMesh};
use math::{Point, Vect};


impl<'a, N, P, V, G: ToTriMesh<N, P, V, I>, I> ToTriMesh<N, P, V, I> for Reflection<'a, G>
    where P: Point<N, V>,
          V: Vect<N> {
    fn to_trimesh(&self, parameter: I) -> TriMesh<N, P, V> {
        let mut res = self.shape().to_trimesh(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
