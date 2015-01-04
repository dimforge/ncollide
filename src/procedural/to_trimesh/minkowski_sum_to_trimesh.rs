use std::ops::Add;
use std::ops::Mul;
use na::{Outer, EigenQR, Translate, Zero};
use na;
use shape::MinkowskiSum;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::{Scalar, Point, Vect};


// XXX: Implemente this for other dimensions (harder because of the concavities.
impl<'a, N, P, V, M1, M2, G1, G2, A, B> ToTriMesh<N, P, V, (A, B)> for MinkowskiSum<'a, M1, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Outer<M2>,
          M2: EigenQR<N, V> + Mul<P, P> + Add<M2, M2> + Zero + Copy,
          G1: ToTriMesh<N, P, V, A>,
          G2: ToTriMesh<N, P, V, B> {
    fn to_trimesh(&self, (a, b): (A, B)) -> TriMesh<N, P, V> {
        assert!(na::dim::<P>() == 3);

        let poly1 = self.g1().to_trimesh(a);
        let poly2 = self.g2().to_trimesh(b);

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

            cpy.translate_by(pt.as_vec());

            all_points.extend(cpy.coords.into_iter());
        }

        procedural::convex_hull3(all_points.as_slice())
    }
}
