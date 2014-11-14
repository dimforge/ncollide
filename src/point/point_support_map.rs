use na::{Identity, Translation, Rotate, Transform};
use na;
use geometry::algorithms::gjk;
use geometry::algorithms::minkowski_sampling;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use shape::{MinkowskiSum, Cylinder, Cone, Capsule, Convex};
use support_map::{SupportMap, PreferedSamplingDirections};
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};

/// Projects a point on a shape using the GJK algorithm.
pub fn support_map_point_projection<N, P, V, M, S, G>(m:       &M,
                                                      shape:   &G,
                                                      simplex: &mut S,
                                                      point:   &P,
                                                      solid:   bool) -> P
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translation<V>,
          S: Simplex<N, P>,
          G: SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {
    let m = na::append_translation(m, &-*point.as_vec());

    match gjk::project_origin(&m, shape, simplex) {
        Some(p) => {
            p + *point.as_vec()
        },
        None => {
            // Fallback algorithm.
            // FIXME: we use the Minkowski Sampling for now, but this should be changed for the EPA
            // in the future.
            if !solid {
                match minkowski_sampling::project_origin(&m, shape, simplex) {
                    Some(p) => p + *point.as_vec(),
                    None    => point.clone()
                }
            }
            else {
                point.clone()
            }
        }
    }
}

impl<N, P, V> LocalPointQuery<N, P> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        support_map_point_projection(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.project_point(pt, true) == *pt
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalPointQuery<N, P> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        support_map_point_projection(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.project_point(pt, true) == *pt
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalPointQuery<N, P> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        support_map_point_projection(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.project_point(pt, true) == *pt
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalPointQuery<N, P> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        support_map_point_projection(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.project_point(pt, true) == *pt
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<'a, N, P, V, M, G1, G2> LocalPointQuery<N, P> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Transform<P> + Rotate<V>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        support_map_point_projection(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.project_point(pt, true) == *pt
    }
}

impl<'a, N, P, V, M, G1, G2> PointQuery<N, P, M> for MinkowskiSum<'a, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Transform<P> + Rotate<V>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
}
