use nalgebra::na::{AlgebraicVec, Identity};
use geom::Reflection;
use implicit::{Implicit, HasMargin};
use geom::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};

impl<'a, N: Add<N, N>, M, G1: HasMargin<N>, G2: HasMargin<N>>
HasMargin<N> for MinkowskiSum<'a, M, G1, G2> {
    #[inline]
    fn margin(&self) -> N {
        self.g1().margin() + self.g2().margin()
    }
}

impl<'a,
     N: Num + Algebraic,
     V: AlgebraicVec<N>,
     M,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>>
Implicit<N, V, Identity> for MinkowskiSum<'a, M, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &V) -> V {
        self.g1().support_point(self.m1(), dir) + self.g2().support_point(self.m2(), dir)
    }

    #[inline]
    fn support_point_without_margin(&self, _: &Identity, dir: &V) -> V {
        self.g1().support_point_without_margin(self.m1(), dir) +
        self.g2().support_point_without_margin(self.m2(), dir)
    }
}

impl<'a, N: Add<N, N>, M, G1: HasMargin<N>, G2: HasMargin<N>>
HasMargin<N> for AnnotatedMinkowskiSum<'a, M, G1, G2> {
    #[inline]
    fn margin(&self) -> N {
        self.g1().margin() + self.g2().margin()
    }
}

impl<'a,
     N: Algebraic + Num,
     V: AlgebraicVec<N> + Clone,
     M,
     G1: Implicit<N, V, M>,
     G2: Implicit<N, V, M>>
Implicit<N, AnnotatedPoint<V>, Identity> for AnnotatedMinkowskiSum<'a, M, G1, G2> {
    #[inline]
    fn support_point(&self, _: &Identity, dir: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        let orig1 = self.g1().support_point(self.m1(), dir.point());
        let orig2 = self.g2().support_point(self.m2(), dir.point());
        let point = orig1 + orig2;

        AnnotatedPoint::new(orig1, orig2, point)
    }

    #[inline]
    fn support_point_without_margin(&self,
                                    _:   &Identity,
                                    dir: &AnnotatedPoint<V>)
                                    -> AnnotatedPoint<V> {
        let orig1 = self.g1().support_point_without_margin(self.m1(), dir.point());
        let orig2 = self.g2().support_point_without_margin(self.m2(), dir.point());
        let point = orig1 + orig2;

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of a CSO on a given direction.
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<G1: Implicit<N, V, M>,
                         G2: Implicit<N, V, M>,
                         N:  Algebraic + Num,
                         V:  AlgebraicVec<N> + Clone,
                         M>(
                         m1:  &M,
                         g1:  &G1,
                         m2:  &M,
                         g2:  &G2,
                         dir: V)
                         -> AnnotatedPoint<V> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Identity::new(), &AnnotatedPoint::new_invalid(dir))
}

/// Computes the support point of a CSO on a given direction.
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point_without_margin<G1: Implicit<N, V, M>,
                                        G2: Implicit<N, V, M>,
                                        N:  Algebraic + Num,
                                        V:  AlgebraicVec<N> + Clone,
                                        M>(
                                        m1:  &M,
                                        g1:  &G1,
                                        m2:  &M,
                                        g2:  &G2,
                                        dir: V)
                                        -> AnnotatedPoint<V> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point_without_margin(&Identity::new(), &AnnotatedPoint::new_invalid(dir))
}
