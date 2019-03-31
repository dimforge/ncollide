use na::{RealField, Unit};

use crate::math::{Isometry, Point, Translation, Vector};
use crate::query::algorithms::{gjk, CSOPoint, VoronoiSimplex, EPA};
use crate::query::{PointProjection, PointQuery};
#[cfg(feature = "dim2")]
use crate::shape::ConvexPolygon;
#[cfg(feature = "dim3")]
use crate::shape::{Cone, ConvexHull, Cylinder};
use crate::shape::{ConstantOrigin, ConvexPolyhedron, FeatureId, SupportMap};
use crate::utils::IsometryOps;

/// Projects a point on a shape using the GJK algorithm.
pub fn support_map_point_projection<N, G>(
    m: &Isometry<N>,
    shape: &G,
    simplex: &mut VoronoiSimplex<N>,
    point: &Point<N>,
    solid: bool,
) -> PointProjection<N>
where
    N: RealField,
    G: SupportMap<N>,
{
    let id = Isometry::identity();
    let m = Translation::from(-point.coords) * m;

    let dir =
        Unit::try_new(-m.translation.vector, N::default_epsilon()).unwrap_or(Vector::x_axis());
    let support_point = CSOPoint::from_shapes(&m, shape, &id, &ConstantOrigin, &dir);

    simplex.reset(support_point);

    if let Some(proj) = gjk::project_origin(&m, shape, simplex) {
        PointProjection::new(false, proj + point.coords)
    } else if solid {
        PointProjection::new(true, *point)
    } else {
        let mut epa = EPA::new();
        if let Some(pt) = epa.project_origin(&m, shape, simplex) {
            return PointProjection::new(true, pt + point.coords);
        } else {
            // return match minkowski_sampling::project_origin(&m, shape, simplex) {
            //     Some(p) => PointProjection::new(true, p + point.coords),
            //     None => PointProjection::new(true, *point),
            // };

            //// All failed.
            PointProjection::new(true, *point)
        }
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> PointQuery<N> for Cylinder<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        support_map_point_projection(m, self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> PointQuery<N> for Cone<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        support_map_point_projection(m, self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> PointQuery<N> for ConvexHull<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        support_map_point_projection(m, self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
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

#[cfg(feature = "dim2")]
impl<N: RealField> PointQuery<N> for ConvexPolygon<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        support_map_point_projection(m, self, &mut VoronoiSimplex::new(), point, solid)
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        let proj = self.project_point(m, point, false);
        let dpt = *point - proj.point;
        let local_dir = if proj.is_inside {
            m.inverse_transform_vector(&-dpt)
        } else {
            m.inverse_transform_vector(&dpt)
        };

        if let Some(local_dir) = Unit::try_new(local_dir, N::default_epsilon()) {
            let feature = self.support_feature_id_toward(&local_dir);
            (proj, feature)
        } else {
            (proj, FeatureId::Unknown)
        }
    }
}
