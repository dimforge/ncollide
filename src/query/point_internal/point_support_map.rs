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
pub fn support_map_point_projection<N, S, G>(
    m: &Isometry<N>,
    shape: &G,
    simplex: &mut S,
    point: &Point<N>,
    solid: bool,
) -> PointProjection<N>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<N>,
    G: SupportMap<N>,
{
    let m = m.append_translation(&Isometry<N>::Translation::from_vector(-point.coords).unwrap());

    let support_point = shape.support_point(&m, &-point.coords);

    simplex.reset(support_point);

    match gjk::project_origin(&m, shape, simplex) {
        Some(p) => PointProjection::new(false, p + point.coords),
        None => {
            if !solid {
                let dim = na::dimension::<Vector<N>>();
                if dim == 2 {
                    let mut epa2 = EPA2::new();
                    if let Some((pt, _)) = epa2.project_origin(&m, shape, simplex) {
                        return PointProjection::new(true, pt + point.coords);
                    }
                } else if dim == 3 {
                    let mut epa3 = EPA3::new();
                    if let Some((pt, _)) = epa3.project_origin(&m, shape, simplex) {
                        return PointProjection::new(true, pt + point.coords);
                    }
                }

                return match minkowski_sampling::project_origin(&m, shape, simplex) {
                    Some(p) => PointProjection::new(true, p + point.coords),
                    None => PointProjection::new(true, *point),
                };
            } else {
                PointProjection::new(true, *point)
            }
        }
    }
}

impl<N: Real> PointQuery<N> for Cylinder<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        if na::dimension::<Vector<N>>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<Vector<N>>() == 3 {
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
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &Point<N>) -> (PointProjection<N>, FeatureId) {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

impl<N: Real> PointQuery<N> for Cone<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        if na::dimension::<Vector<N>>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<Vector<N>>() == 3 {
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
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &Point<N>) -> (PointProjection<N>, FeatureId) {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

impl<N: Real> PointQuery<N> for ConvexHull<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        if na::dimension::<Vector<N>>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<Vector<N>>() == 3 {
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
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &Point<N>) -> (PointProjection<N>, FeatureId) {
        let proj = self.project_point(m, point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside {
            m.inverse_transform_vector(&-dpt)
        } else {
            m.inverse_transform_vector(&dpt)
        };

        if let Some(local_dir) = Unit::try_new(local_dir, N::default_epsilon()) {
            let feature = ConvexPolyhedron::<N>::support_feature_id_toward(self, &local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}

impl<N: Real> PointQuery<N> for ConvexPolygon<P> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        if na::dimension::<Vector<N>>() == 2 {
            support_map_point_projection(m, self, &mut VoronoiSimplex2::<P>::new(), point, solid)
        } else if na::dimension::<Vector<N>>() == 3 {
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
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &Point<N>) -> (PointProjection<N>, FeatureId) {
        let proj = self.project_point(m, point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside {
            m.inverse_transform_vector(&-dpt)
        } else {
            m.inverse_transform_vector(&dpt)
        };

        if let Some(local_dir) = Unit::try_new(local_dir, N::default_epsilon()) {
            let feature = ConvexPolyhedron::<N>::support_feature_id_toward(self, &local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}
