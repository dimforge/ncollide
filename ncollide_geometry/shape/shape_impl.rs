use bounding_volume::{self, AABB, BoundingSphere};
use query::{PointQuery, RayCast};
use shape::{Shape, Triangle, Segment, Ball, Plane, Cuboid, Cylinder, Cone, ConvexHull, Compound,
            TriMesh, Polyline, CompositeShape, SupportMap};
use math::{Point, Isometry};

macro_rules! impl_as_support_map(
    () => {
        #[inline]
        fn as_support_map(&self) -> Option<&SupportMap<P, M>> {
            Some(self)
        }

        #[inline]
        fn is_support_map(&self) -> bool {
            true
        }
    }
);

macro_rules! impl_as_composite_shape(
    () => {
        #[inline]
        fn as_composite_shape(&self) -> Option<&CompositeShape<P, M>> {
            Some(self)
        }

        #[inline]
        fn is_composite_shape(&self) -> bool {
            true
        }
    }
);

macro_rules! impl_shape_common(
    () => {
        #[inline]
        fn aabb(&self, m: &M) -> AABB<P> {
            bounding_volume::aabb(self, m)
        }

        #[inline]
        fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
            bounding_volume::bounding_sphere(self, m)
        }

        #[inline]
        fn as_ray_cast(&self) -> Option<&RayCast<P, M>> {
            Some(self)
        }

        #[inline]
        fn as_point_query(&self) -> Option<&PointQuery<P, M>> {
            Some(self)
        }
    }
);

impl<P: Point, M: Isometry<P>> Shape<P, M> for Triangle<P> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Segment<P> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Ball<P::Real> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Cuboid<P::Vector> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Cylinder<P::Real> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Cone<P::Real> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for ConvexHull<P> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: 'static + Send + Sync + Isometry<P>> Shape<P, M> for Compound<P, M> {
    impl_shape_common!();
    impl_as_composite_shape!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for TriMesh<P> {
    impl_shape_common!();
    impl_as_composite_shape!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Polyline<P> {
    impl_shape_common!();
    impl_as_composite_shape!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Plane<P::Vector> {
    impl_shape_common!();
}
