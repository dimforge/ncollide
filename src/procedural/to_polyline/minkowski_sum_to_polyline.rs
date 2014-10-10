use geom::MinkowskiSum;
use procedural::{Polyline, ToPolyline};
use procedural;
use math::{Scalar, Point, Vect};


// XXX: Implemented this for other dimensions (harder because of the concavities.
#[cfg(feature = "2d")]
impl<'a, G1: ToPolyline<A>, G2: ToPolyline<B>, A, B> ToPolyline<(A, B)> for MinkowskiSum<'a, G1, G2> {
    fn to_polyline(&self, (a, b): (A, B)) -> Polyline<Scalar, Point, Vect> {
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

        procedural::convex_hull2d(all_points.as_slice())
    }
}
