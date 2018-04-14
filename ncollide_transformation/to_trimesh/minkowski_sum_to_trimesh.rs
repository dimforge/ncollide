use std::ops::{Mul, Add};
use num::Zero;
use na::{Outer, EigenQR, Translate };
use na;
use shape::MinkowskiSum;
use procedural::TriMesh;
use super::ToTriMesh;
use math::{Point, Vector};


// XXX: Implemente this for other dimensions (harder because of the concavities.
impl<'a, P, M1, G1, G2, A, B> ToTriMesh<P, (A, B)> for MinkowskiSum<'a, M1, G1, G2>
    where P:  Point,
          Vector<N>: Translate<P> + Outer + Mul<<<P as Point>::Vect as Outer>::OuterProductType, Output = <P as Point>::Vect>,
          G1: ToTriMesh<P, A>,
          G2: ToTriMesh<P, B>,
          <Vector<N> as Outer>::OuterProductType: EigenQR<N, Vector<N>> +
                                                Mul<P, Output = P> +
                                                Add<<Vector<N> as Outer>::OuterProductType, Output = <Vector<N> as Outer>::OuterProductType>
                                                + Zero + Copy {
    fn to_trimesh(&self, (a, b): (A, B)) -> TriMesh<P> {
        assert!(na::dimension::<P>() == 3);

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

            cpy.translate_by(pt.as_vector());

            all_points.extend(cpy.coords.into_iter());
        }

        ::convex_hull3(&all_points[..])
    }
}
