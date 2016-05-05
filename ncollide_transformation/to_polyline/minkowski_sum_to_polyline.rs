use na::Translate;
use na;
use math::Point;
use geometry::shape::MinkowskiSum;
use procedural::Polyline;
use super::ToPolyline;


// XXX: Implemented this for other dimensions (harder because of the concavities.
impl<'a, P, M, G1, G2, A, B> ToPolyline<P, (A, B)> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          P::Vect: Translate<P>,
          G1: ToPolyline<P, A>,
          G2: ToPolyline<P, B> {
    fn to_polyline(&self, (a, b): (A, B)) -> Polyline<P> {
        assert!(na::dimension::<P>() == 2);

        let poly1 = self.g1().to_polyline(a);
        let poly2 = self.g2().to_polyline(b);

        // FIXME: this will work only for convex polyhedras.
        let mut all_points = Vec::with_capacity(poly1.coords().len() * poly2.coords().len());

        let p1;
        let p2;

        if poly1.coords().len() > poly2.coords().len() {
            p1 = poly1;
            p2 = poly2;
        }
        else {
            p1 = poly2;
            p2 = poly1;
        }

        for pt in p2.coords().iter() {
            let mut cpy = p1.clone();

            cpy.translate_by(pt.as_vector());

            all_points.extend(cpy.coords().into_iter());
        }

        ::convex_hull2(&all_points[..])
    }
}
