use na::{Translation, Rotate, Transform};
use na;
use geometry::algorithms::gjk;
use geometry::algorithms::minkowski_sampling;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::johnson_simplex::JohnsonSimplex;
use entities::shape::{Cylinder, Cone, Capsule, Convex};
use entities::support_map::SupportMap;
use point::PointQuery;
use math::{Scalar, Point, Vect};

/// Projects a point on a shape using the GJK algorithm.
pub fn support_map_point_projection<P, M, S, G>(m:       &M,
                                                shape:   &G,
                                                simplex: &mut S,
                                                point:   &P,
                                                solid:   bool)
                                                -> P
    where P: Point,
          M: Translation<P::Vect>,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    let m = na::append_translation(m, &-*point.as_vec());

    let support_point = shape.support_point(&m, &-*point.as_vec());

    simplex.reset(support_point);

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

impl<P, M> PointQuery<P, M> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> + Translation<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(pt, &self.project_point(m, pt, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        self.project_point(m, pt, true) == *pt
    }
}

impl<P, M> PointQuery<P, M> for Cone<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> + Translation<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(pt, &self.project_point(m, pt, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        self.project_point(m, pt, true) == *pt
    }
}

impl<P, M> PointQuery<P, M> for Capsule<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> + Translation<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(pt, &self.project_point(m, pt, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        self.project_point(m, pt, true) == *pt
    }
}

impl<P, M> PointQuery<P, M> for Convex<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> + Translation<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(pt, &self.project_point(m, pt, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        self.project_point(m, pt, true) == *pt
    }
}
