use bounding_volume::{self, AABB, BoundingSphere};
use math::Isometry;
use na::Real;
use query::{PointQuery, RayCast};
use shape::{Ball, CompositeShape, Compound, ConvexPolyhedron, Cuboid, DeformableShape, Plane, Polyline,
            Segment, Shape, SupportMap};
#[cfg(feature = "dim3")]
use shape::{ConvexHull, Triangle, TriMesh};
#[cfg(feature = "dim2")]
use shape::ConvexPolygon;

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
}

#[cfg(feature = "dim3")]
impl<N: Real> Shape<N> for TriMesh<N> {
    impl_shape_common!();
    impl_as_composite_shape!();
    impl_as_deformable_shape!();
}

impl<N: Real> Shape<N> for Polyline<N> {
    impl_shape_common!();
    impl_as_composite_shape!();
}

impl<N: Real> Shape<N> for Plane<N> {
    impl_shape_common!();
}
