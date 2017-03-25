use alga::linear::Translation;

use query::algorithms::gjk;
use query::algorithms::minkowski_sampling;
use query::algorithms::simplex::Simplex;
use query::algorithms::johnson_simplex::JohnsonSimplex;
use query::{PointQuery, PointProjection};
use shape::{SupportMap, Cylinder, Cone, Capsule, ConvexHull};
use math::{Point, Isometry};

/// Projects a point on a shape using the GJK algorithm.
pub fn support_map_point_projection<P, M, S, G>(m:       &M,
                                                shape:   &G,
                                                simplex: &mut S,
                                                point:   &P,
                                                solid:   bool)
                                                -> PointProjection<P>
    where P: Point,
          M: Isometry<P>,
          S: Simplex<P>,
          G: SupportMap<P, M> {
    let m = m.append_translation(&M::Translation::from_vector(-point.coordinates()).unwrap());

    let support_point = shape.support_point(&m, &-point.coordinates());

    simplex.reset(support_point);

    match gjk::project_origin(&m, shape, simplex) {
        Some(p) => {
            PointProjection::new(false, p + point.coordinates(), ())
        },
        None => {
            let proj;

            // Fallback algorithm.
            // FIXME: we use the Minkowski Sampling for now, but this should be changed for the EPA
            // in the future.
            if !solid {
                match minkowski_sampling::project_origin(&m, shape, simplex) {
                    Some(p) => proj = p + point.coordinates(),
                    None    => proj = *point
                }
            }
            else {
                proj = *point
            }

            PointProjection::new(true, proj, ())
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Cylinder<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Cone<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Capsule<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for ConvexHull<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        support_map_point_projection(m, self, &mut JohnsonSimplex::<P>::new_w_tls(), point, solid)
    }
}
