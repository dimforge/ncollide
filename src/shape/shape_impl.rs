use bounding_volume::{self, BoundingSphere, AABB};
use math::{Isometry, Point, Vector};
use na::{Real, Unit};
use query::{PointQuery, RayCast};
#[cfg(feature = "dim2")]
use shape::ConvexPolygon;
use shape::{
    Ball, CompositeShape, Compound, ConvexPolyhedron, Cuboid, FeatureId, Plane, Polyline, Segment,
    Shape, SupportMap,
};
#[cfg(feature = "dim3")]
use shape::{ConvexHull, DeformableShape, TriMesh, Triangle};
use utils::IsometryOps;

macro_rules! impl_as_convex_polyhedron (
    () => {
        #[inline]
        fn as_convex_polyhedron(&self) -> Option<&ConvexPolyhedron<N>> {
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
        fn as_support_map(&self) -> Option<&SupportMap<N>> {
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
        fn as_composite_shape(&self) -> Option<&CompositeShape<N>> {
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
        fn as_deformable_shape(&self) -> Option<&DeformableShape<N>> {
            Some(self)
        }

        #[inline]
        fn as_deformable_shape_mut(&mut self) -> Option<&mut DeformableShape<N>> {
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
        fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
            bounding_volume::bounding_sphere(self, m)
        }

        #[inline]
        fn as_ray_cast(&self) -> Option<&RayCast<N>> {
            Some(self)
        }

        #[inline]
        fn as_point_query(&self) -> Option<&PointQuery<N>> {
            Some(self)
        }

    }
);

#[cfg(feature = "dim3")]
impl<N: Real> Shape<N> for Triangle<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: Real> Shape<N> for Segment<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: Real> Shape<N> for Ball<N> {
    impl_shape_common!();
    impl_as_support_map!();

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

impl<N: Real> Shape<N> for Cuboid<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

#[cfg(feature = "dim3")]
impl<N: Real> Shape<N> for ConvexHull<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

#[cfg(feature = "dim2")]
impl<N: Real> Shape<N> for ConvexPolygon<N> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
}

impl<N: Real> Shape<N> for Compound<N> {
    impl_shape_common!();
    impl_as_composite_shape!();

    #[inline]
    fn subshape_transform(&self, subshape_id: usize) -> Option<Isometry<N>> {
        let idx = self.start_idx();
        let mut shape_id = 0;

        while shape_id < idx.len() && idx[shape_id] <= subshape_id {
            shape_id += 1;
        }

        let shape = &self.shapes()[shape_id - 1];

        if let Some(subtransform) = shape.1.subshape_transform(subshape_id - idx[shape_id - 1]) {
            Some(shape.0.clone() * subtransform)
        } else {
            Some(shape.0.clone())
        }
    }

    fn tangent_cone_contains_dir(
        &self,
        _feature: FeatureId,
        _m: &Isometry<N>,
        _deformations: Option<&[N]>,
        _dir: &Unit<Vector<N>>,
    ) -> bool
    {
        unimplemented!()
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> Shape<N> for TriMesh<N> {
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

impl<N: Real> Shape<N> for Polyline<N> {
    impl_shape_common!();
    impl_as_composite_shape!();

    fn tangent_cone_contains_dir(
        &self,
        _feature: FeatureId,
        _m: &Isometry<N>,
        _deformations: Option<&[N]>,
        _dir: &Unit<Vector<N>>,
    ) -> bool
    {
        unimplemented!()
    }
}

impl<N: Real> Shape<N> for Plane<N> {
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
