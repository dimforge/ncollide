use na::Identity;
use support_map::SupportMap;
use shape::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint, Reflection};
use math::Point;


impl<'a, P, M, G1: ?Sized, G2: ?Sized> SupportMap<P, Identity> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &P::Vect) -> P {
        self.g1().support_point(self.m1(), dir) + *self.g2().support_point(self.m2(), dir).as_vector()
    }
}

impl<'a, P, M, G1: ?Sized, G2: ?Sized>
SupportMap<AnnotatedPoint<P>, Identity> for AnnotatedMinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &P::Vect) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point(self.m1(), dir);
        let orig2 = self.g2().support_point(self.m2(), dir);
        let point = orig1 + *orig2.as_vector();

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of the CSO `g1 - g2` on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1,
                                                       m2: &M, g2: &G2,
                                                       dir: P::Vect)
                                                       -> AnnotatedPoint<P>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Identity::new(), &dir)
}
