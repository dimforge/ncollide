use geom::MinkowskiSum;
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Vect};


// XXX: Implemented this for other dimensions (harder because of the concavities.
#[dim2]
impl<'a, G1: ToTriMesh<A>, G2: ToTriMesh<B>, A, B> ToTriMesh<(A, B)> for MinkowskiSum<'a, G1, G2> {
    fn to_polyline(&self, (a, b): (A, B)) -> TriMesh<Scalar, Vect> {
        let poly1 = self.g1().to_polyline(a);
        let poly2 = self.g2().to_polyline(b);

        // FIXME: this will work only for convex polyhedras.
        let mut all_points = Vec::with_capacity(poly1.coords.len() * poly2.coords.len());

        let p1;
        let p2;

        if poly1.coords.len() > poly2.coords.len() {
            p1 = poly1;
            p2 = poly2;
        }
        else {
            p1 = poly2;
            p2 = poly1;
        }

        // XXX: We are that assuming all the vertices of both meshes are actually used.
        for pt in p2.coords.iter() {
            let mut cpy = p1.clone();

            cpy.translate_by(pt);

            all_points.push_all_move(cpy.coords);
        }

        procedural::convex_hull3d(all_points.as_slice())
    }
}
