use approx::ApproxEq;
use alga::linear::Translation;
use na::{self, Unit};

use query::algorithms::{gjk, minkowski_sampling, EPA2, EPA3, JohnsonSimplex, Simplex,
                        VoronoiSimplex2, VoronoiSimplex3};
use query::{PointProjection, PointQuery};
use shape::{Capsule, Cone, ConvexHull, ConvexPolygon, ConvexPolyhedron, Cylinder, FeatureId,
            SupportMap};
use math::{Isometry, Point};

/// Projects a point on a shape using the GJK algorithm.
pub fn support_map_point_projection<P, M, S, G>(
    m: &M,
    shape: &G,
    simplex: &mut S,
    point: &P,
    solid: bool,
) -> PointProjection<P>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<P>,
    G: SupportMap<P, M>,
{
    let m = m.append_translation(&M::Translation::from_vector(-point.coordinates()).unwrap());

    let support_point = shape.support_point(&m, &-point.coordinates());

    simplex.reset(support_point);

    match gjk::project_origin(&m, shape, simplex) {
        Some(p) => PointProjection::new(false, p + point.coordinates()),
        None => {
            if !solid {
                let dim = na::dimension::<P::Vector>();
                if dim == 2 {
                    let mut epa2 = EPA2::new();
                    if let Some((pt, _)) = epa2.project_origin(&m, shape, simplex) {
                        return PointProjection::new(true, pt + point.coordinates());
                    }
                } else if dim == 3 {
                    let mut epa3 = EPA3::new();
                    if let Some((pt, _)) = epa3.project_origin(&m, shape, simplex) {
                        return PointProjection::new(true, pt + point.coordinates());
                    }
                }

                return match minkowski_sampling::project_origin(&m, shape, simplex) {
                    Some(p) => PointProjection::new(true, p + point.coordinates()),
                    None => PointProjection::new(true, *point),
                };
            } else {
                PointProjection::new(true, *point)
            }
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Cylinder<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        if na::dimension::<P::Vector>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<P::Vector>() == 3 {
            support_map_point_projection(m, self, &mut VoronoiSimplex3::<P>::new(), point, solid)
        } else {
            support_map_point_projection(
                m,
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                point,
                solid,
            )
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, point: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Cone<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        if na::dimension::<P::Vector>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<P::Vector>() == 3 {
            support_map_point_projection(m, self, &mut VoronoiSimplex3::<P>::new(), point, solid)
        } else {
            support_map_point_projection(
                m,
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                point,
                solid,
            )
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, point: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for ConvexHull<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        if na::dimension::<P::Vector>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<P::Vector>() == 3 {
            support_map_point_projection(m, self, &mut VoronoiSimplex3::<P>::new(), point, solid)
        } else {
            support_map_point_projection(
                m,
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                point,
                solid,
            )
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, point: &P) -> (PointProjection<P>, FeatureId) {
        let proj = self.project_point(m, point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside {
            m.inverse_transform_vector(&-dpt)
        } else {
            m.inverse_transform_vector(&dpt)
        };

        if let Some(local_dir) = Unit::try_new(local_dir, P::Real::default_epsilon()) {
            let feature = ConvexPolyhedron::<P, M>::support_feature_id_toward(self, &local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for ConvexPolygon<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        if na::dimension::<P::Vector>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<P::Vector>() == 3 {
            support_map_point_projection(m, self, &mut VoronoiSimplex3::<P>::new(), point, solid)
        } else {
            support_map_point_projection(
                m,
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                point,
                solid,
            )
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, point: &P) -> (PointProjection<P>, FeatureId) {
        let proj = self.project_point(m, point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside {
            m.inverse_transform_vector(&-dpt)
        } else {
            m.inverse_transform_vector(&dpt)
        };

        if let Some(local_dir) = Unit::try_new(local_dir, P::Real::default_epsilon()) {
            let feature = ConvexPolyhedron::<P, M>::support_feature_id_toward(self, &local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}
