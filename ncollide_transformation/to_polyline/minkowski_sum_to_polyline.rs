use na::Translate;
use na;
use math::{Scalar, Point, Vect};
use entities::shape::MinkowskiSum;
use procedural::Polyline;
use super::ToPolyline;


// XXX: Implemented this for other dimensions (harder because of the concavities.
impl<'a, N, P, V, M, G1, G2, A, B> ToPolyline<N, P, V, (A, B)> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          G1: ToPolyline<N, P, V, A>,
          G2: ToPolyline<N, P, V, B> {
    fn to_polyline(&self, (a, b): (A, B)) -> Polyline<N, P, V> {
        assert!(na::dim::<P>() == 2);

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

        for pt in p2.coords.iter() {
            let mut cpy = p1.clone();

            cpy.translate_by(pt.as_vec());

            all_points.extend(cpy.coords.into_iter());
        }

        ::convex_hull2(all_points.as_slice())
    }
}
