use bounding_volume::{self, BoundingSphere, AABB};
use query::{PointQuery, RayCast};
use shape::{Ball, CompositeShape, Compound, Cone, ConvexHull, ConvexPolyhedron, Cuboid, Cylinder,
            Plane, Polyline, Segment, Shape, SupportMap, TriMesh, Triangle, FeatureId};
use math::{Isometry, Point};

macro_rules! impl_as_convex_polyhedron(
    () => {
        #[inline]
        fn as_convex_polyhedron(&self) -> Option<&ConvexPolyhedron<P, M>> {
            Some(self)
        }

        #[inline]
        fn is_convex_polyhedron(&self) -> bool {
            true
        }
    }
);

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
    // impl_as_convex_polyhedron!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Segment<P> {
    impl_shape_common!();
    impl_as_support_map!();
    // impl_as_convex_polyhedron!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Ball<P::Real> {
    impl_shape_common!();
    impl_as_support_map!();
}

impl<P: Point, M: Isometry<P>> Shape<P, M> for Cuboid<P::Vector> {
    impl_shape_common!();
    impl_as_support_map!();
    impl_as_convex_polyhedron!();
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
    // impl_as_conve_polyhedron!();
}

impl<P: Point, M: 'static + Send + Sync + Isometry<P>> Shape<P, M> for Compound<P, M> {
    impl_shape_common!();
    impl_as_composite_shape!();

    #[inline]
    fn subshape_transform(&self, mut feature: FeatureId) -> M {
        let idx = self.start_idx();
        let subshape_id = feature.subshape_id();

        let mut shape_id = 0;

        while shape_id < idx.len() && idx[shape_id] <= subshape_id {
            shape_id += 1;
        }

        let shape = &self.shapes()[shape_id - 1];
        feature.set_subshape_id(subshape_id - idx[shape_id - 1]);

        shape.0.clone() * shape.1.subshape_transform(feature)
    }
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
