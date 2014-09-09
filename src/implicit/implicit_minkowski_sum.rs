use nalgebra::na::Identity;
use geom::Reflection;
use implicit::Implicit;
use geom::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
use math::{Scalar, Vect, Matrix};

impl<'a, G1: Implicit<Vect, Matrix>, G2: Implicit<Vect, Matrix>>
Implicit<Vect, Identity> for MinkowskiSum<'a, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &Vect) -> Vect {
        self.g1().support_point(self.m1(), dir) + self.g2().support_point(self.m2(), dir)
    }
}

impl<'a, G1: Implicit<Vect, Matrix>, G2: Implicit<Vect, Matrix>>
Implicit<AnnotatedPoint, Identity> for AnnotatedMinkowskiSum<'a, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &AnnotatedPoint) -> AnnotatedPoint {
        let orig1 = self.g1().support_point(self.m1(), dir.point());
        let orig2 = self.g2().support_point(self.m2(), dir.point());
        let point = orig1 + orig2;

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of a CSO on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<G1: Implicit<Vect, Matrix>,
                         G2: Implicit<Vect, Matrix>>(
                         m1:  &Matrix,
                         g1:  &G1,
                         m2:  &Matrix,
                         g2:  &G2,
                         dir: Vect)
                         -> AnnotatedPoint {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Identity::new(), &AnnotatedPoint::new_invalid(dir))
}
