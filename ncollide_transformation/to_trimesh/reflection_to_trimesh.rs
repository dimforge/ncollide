use entities::shape::Reflection;
use procedural::TriMesh;
use super::ToTriMesh;
use math::Point;


impl<'a, P, G: ToTriMesh<P, I>, I> ToTriMesh<P, I> for Reflection<'a, G>
    where P: Point {
    fn to_trimesh(&self, parameter: I) -> TriMesh<P> {
        let mut res = self.shape().to_trimesh(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
