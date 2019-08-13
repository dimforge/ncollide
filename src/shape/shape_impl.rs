use crate::bounding_volume::{self, BoundingSphere, AABB, BoundingVolume};
use crate::math::{Isometry, Vector};
use na::{RealField, Unit};
use crate::query::{PointQuery, RayCast};
#[cfg(feature = "dim2")]
use crate::shape::ConvexPolygon;
use crate::shape::{
    Ball, CompositeShape, Compound, ConvexPolyhedron, Cuboid, FeatureId, Plane, Polyline, Segment,
    Capsule, Shape, SupportMap, DeformableShape, HeightField, Multiball
};
#[cfg(feature = "dim3")]
use crate::shape::{ConvexHull, TriMesh, Triangle};
use crate::utils::IsometryOps;

macro_rules! impl_as_convex_polyhedron (
    () => {
        #[inline]
        fn as_convex_polyhedron(&self) -> Option<&dyn ConvexPolyhedron<N>> {
            Some(self)
        }

        #[inline]
        fn is_convex_polyhedron(&self) -> bool {
            true
        }

        #[inline]
        fn tangent_cone_contains_dir(&self, feature: FeatureId, m: &Isometry<N>, _: Option<&[N]>, dir: &Unit<Vector<N>>) -> bool {
            self.tangent_cone_contains_dir(feature, m, dir)
        }
    }
);

macro_rules! impl_as_support_map (
    () => {
        #[inline]
        fn as_support_map(&self) -> Option<&dyn SupportMap<N>> {
            Some(self)
        }

        #[inline]
        fn is_support_map(&self) -> bool {
            true
        }
    }
);

macro_rules! impl_as_composite_shape (
    () => {
        #[inline]
        fn as_composite_shape(&self) -> Option<&dyn CompositeShape<N>> {
            Some(self)
        }

        #[inline]
        fn is_composite_shape(&self) -> bool {
            true
        }
    }
);

macro_rules! impl_as_deformable_shape (
    () => {
        #[inline]
        fn as_deformable_shape(&self) -> Option<&dyn DeformableShape<N>> {
            Some(self)
        }

        #[inline]
        fn as_deformable_shape_mut(&mut self) -> Option<&mut dyn DeformableShape<N>> {
            Some(self)
        }

        #[inline]
        fn is_deformable_shape(&self) -> bool {
            true
        }
    }
);

macro_rules! impl_shape_common (
    () => {
        #[inline]
        fn aabb(&self, m: &Isometry<N>) -> AABB<N> {
            bounding_volume::aabb(self, m)
        }

        #[inline]
        fn local_aabb(&self) -> AABB<N> {
            bounding_volume::local_aabb(self)
        }

        #[inline]
        fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
            bounding_volume::bounding_sphere(self, m)
        }

        #[inline]
        fn as_ray_cast(&self) -> Option<&dyn RayCast<N>> {
            Some(self)
        }

        #[inline]
        fn as_point_query(&self) -> Option<&dyn PointQuery<N>> {
            Some(self)
        }
    }
);

#[cfg(feature = "dim3")]
impl<N: RealField> Shape<N> for Triangle<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: RealField> Shape<N> for Segment<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: RealField> Shape<N> for Ball<N> {
    impl_shape_common!();
    impl_as_support_map!();

    // FIXME: this is wrong in theory but keep it this
    // way for now because of the way the ContactKinematic
    // currently works.
    fn tangent_cone_contains_dir(
        &self,
        _: FeatureId,
        _: &Isometry<N>,
        _: Option<&[N]>,
        _: &Unit<Vector<N>>,
    ) -> bool
    {
        false
    }
}

impl<N: RealField> Shape<N> for Cuboid<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: RealField> Shape<N> for Capsule<N> {
    impl_shape_common!();
    impl_as_support_map!();

    // FIXME: this is wrong in theory but keep it this
    // way for now because of the way the ContactKinematic
    // currently works.
    fn tangent_cone_contains_dir(
        &self,
        _: FeatureId,
        _: &Isometry<N>,
        _: Option<&[N]>,
        _: &Unit<Vector<N>>,
    ) -> bool
    {
    false
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> Shape<N> for ConvexHull<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

#[cfg(feature = "dim2")]
impl<N: RealField> Shape<N> for ConvexPolygon<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: RealField> Shape<N> for Compound<N> {
    impl_shape_common!();
    impl_as_composite_shape!();

    fn tangent_cone_contains_dir(
        &self,
        feature: FeatureId,
        m: &Isometry<N>,
        _: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        let (i, fid) = self.subshape_feature_id(feature);
        let shape = &self.shapes()[i];
        let ls_dir = m.inverse_transform_unit_vector(dir);
        shape.1.as_ref().tangent_cone_contains_dir(fid, &shape.0, None, &ls_dir)
    }

    fn subshape_containing_feature(&self, feature: FeatureId) -> usize {
        self.subshape_feature_id(feature).0
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> Shape<N> for TriMesh<N> {
    impl_shape_common!();
    impl_as_composite_shape!();
    impl_as_deformable_shape!();

    fn tangent_cone_contains_dir(
        &self,
        fid: FeatureId,
        m: &Isometry<N>,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        let ls_dir = m.inverse_transform_unit_vector(dir);

        match fid {
            FeatureId::Face(i) => self.face_tangent_cone_contains_dir(i, deformations, &ls_dir),
            FeatureId::Edge(i) => self.edge_tangent_cone_contains_dir(i, deformations, &ls_dir),
            FeatureId::Vertex(i) => self.vertex_tangent_cone_contains_dir(i, deformations, &ls_dir),
            FeatureId::Unknown => false,
        }
    }

    fn subshape_containing_feature(&self, id: FeatureId) -> usize {
        self.face_containing_feature(id)
    }
}

impl<N: RealField> Shape<N> for Polyline<N> {
    impl_shape_common!();
    impl_as_composite_shape!();
    impl_as_deformable_shape!();

    fn tangent_cone_contains_dir(
        &self,
        _feature: FeatureId,
        _m: &Isometry<N>,
        _deformations: Option<&[N]>,
        _dir: &Unit<Vector<N>>,
    ) -> bool
    {
        // FIXME
        false
    }


    fn subshape_containing_feature(&self, id: FeatureId) -> usize {
        self.edge_containing_feature(id)
    }
}

impl<N: RealField> Shape<N> for HeightField<N> {
    impl_shape_common!();

    fn tangent_cone_contains_dir(
        &self,
        _fid: FeatureId,
        _m: &Isometry<N>,
        _deformations: Option<&[N]>,
        _dir: &Unit<Vector<N>>,
    ) -> bool
    {
        // FIXME
        false
    }

    fn subshape_containing_feature(&self, _id: FeatureId) -> usize {
        // FIXME
        0
    }
}


impl<N: RealField> Shape<N> for Multiball<N> {
    impl_as_deformable_shape!();

    #[inline]
    fn aabb(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::point_cloud_aabb(m, self.centers())
            .loosened(self.radius())
    }

    #[inline]
    fn local_aabb(&self) -> AABB<N> {
        bounding_volume::local_point_cloud_aabb(self.centers())
            .loosened(self.radius())
    }

    #[inline]
    fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let (c, r) = bounding_volume::point_cloud_bounding_sphere(self.centers());
        BoundingSphere::new(c, r + self.radius())
    }

    #[inline]
    fn as_ray_cast(&self) -> Option<&dyn RayCast<N>> {
        Some(self as &dyn RayCast<N>)
    }

    #[inline]
    fn as_point_query(&self) -> Option<&dyn PointQuery<N>> {
        Some(self as &dyn PointQuery<N>)
    }

    fn tangent_cone_contains_dir(
        &self,
        feature: FeatureId,
        m: &Isometry<N>,
        _: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        // NOTE: the actual center of the concerned ball does not have
        // to be taken into account since the result only depends on the orientation
        // of the ball.
        Ball::new(self.radius())
            .tangent_cone_contains_dir(FeatureId::Face(0), m, None, dir)
    }

    fn subshape_containing_feature(&self, feature: FeatureId) -> usize {
        self.subshape_feature_id(feature).0
    }
}

impl<N: RealField> Shape<N> for Plane<N> {
    impl_shape_common!();

    fn tangent_cone_contains_dir(
        &self,
        _: FeatureId,
        m: &Isometry<N>,
        _: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        let world_normal = m * self.normal();
        dir.dot(&world_normal) <= N::zero()
    }
}
