use na::Identity;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint, Reflection};
use math::{Scalar, Point, Vect};


impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized> SupportMap<P, V, Identity> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &V) -> P {
        self.g1().support_point(self.m1(), dir) + *self.g2().support_point(self.m2(), dir).as_vec()
    }
}

impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized>
SupportMap<AnnotatedPoint<P>, V, Identity> for AnnotatedMinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &V) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point(self.m1(), dir);
        let orig2 = self.g2().support_point(self.m2(), dir);
        let point = orig1 + *orig2.as_vec();

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

impl<'a, V, M, G1: ?Sized, G2: ?Sized> PreferedSamplingDirections<V, Identity> for MinkowskiSum<'a, M, G1, G2> {
    #[inline(always)]
    fn sample(&self, _: &Identity, _: &mut FnMut(V)) {
    }
}

impl<'a, V, M, G1: ?Sized, G2: ?Sized> PreferedSamplingDirections<V, Identity> for AnnotatedMinkowskiSum<'a, M, G1, G2> {
    #[inline(always)]
    fn sample(&self, _: &Identity, _: &mut FnMut(V)) {
    }
}

/// Computes the support point of the CSO `g1 - g2` on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1,
                                                             m2: &M, g2: &G2,
                                                             dir: V)
                                                             -> AnnotatedPoint<P>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Identity::new(), &dir)
}
