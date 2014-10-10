use na::Identity;
use geom::Reflection;
use implicit::Implicit;
use geom::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
use math::{Point, Vect, Matrix};

impl<'a, G1: Implicit<Point, Vect, Matrix>, G2: Implicit<Point, Vect, Matrix>>
Implicit<Point, Vect, Identity> for MinkowskiSum<'a, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &Vect) -> Point {
        self.g1().support_point(self.m1(), dir) + *self.g2().support_point(self.m2(), dir).as_vec()
    }
}

impl<'a, G1: Implicit<Point, Vect, Matrix>, G2: Implicit<Point, Vect, Matrix>>
Implicit<AnnotatedPoint, Vect, Identity> for AnnotatedMinkowskiSum<'a, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &Vect) -> AnnotatedPoint {
        let orig1 = self.g1().support_point(self.m1(), dir);
        let orig2 = self.g2().support_point(self.m2(), dir);
        let point = orig1 + *orig2.as_vec();

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of a CSO on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<G1: Implicit<Point, Vect, Matrix>,
                         G2: Implicit<Point, Vect, Matrix>>(
                         m1:  &Matrix,
                         g1:  &G1,
                         m2:  &Matrix,
                         g2:  &G2,
                         dir: Vect)
                         -> AnnotatedPoint {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Identity::new(), &dir)
}
