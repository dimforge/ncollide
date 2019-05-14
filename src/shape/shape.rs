// Queries.
use crate::bounding_volume::{BoundingSphere, AABB};
use crate::math::{Isometry, Vector};
use na::{self, RealField, Unit};
use crate::query::{PointQuery, RayCast};
use crate::shape::{CompositeShape, ConvexPolyhedron, DeformableShape, FeatureId, SupportMap};
use std::ops::Deref;
use std::sync::Arc;
use downcast_rs::Downcast;

pub trait ShapeClone<N: RealField> {
    fn clone_box(&self) -> Box<Shape<N>> {
        unimplemented!()
    }
}

impl<N: RealField, T: 'static + Shape<N> + Clone> ShapeClone<N> for T {
    fn clone_box(&self) -> Box<Shape<N>> {
        Box::new(self.clone())
    }
}

/// Trait implemented by all shapes supported by ncollide.
///
/// This allows dynamic inspection of the shape capabilities.
pub trait Shape<N: RealField>: Send + Sync + Downcast + ShapeClone<N> {
    /// The AABB of `self` transformed by `m`.
    #[inline]
    fn aabb(&self, m: &Isometry<N>) -> AABB<N>;

    /// The AABB of `self`.
    #[inline]
    fn local_aabb(&self) -> AABB<N> {
        self.aabb(&Isometry::identity())
    }

    /// The bounding sphere of `self` transformed by `m`.
    #[inline]
    fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let aabb = self.aabb(m);
        BoundingSphere::new(aabb.center(), aabb.half_extents().norm_squared())
    }

    /// The bounding sphere of `self`.
    #[inline]
    fn local_bounding_sphere(&self) -> BoundingSphere<N> {
        let aabb = self.local_aabb();
        BoundingSphere::new(aabb.center(), aabb.half_extents().norm())
    }

    /// Check if if the feature `_feature` of the `i-th` subshape of `self` transformed by `m` has a tangent
    /// cone that contains `dir` at the point `pt`.
    // NOTE: for the moment, we assume the tangent cone is the same for the whole feature.
    #[inline]
    fn tangent_cone_contains_dir(
        &self,
        _feature: FeatureId,
        _m: &Isometry<N>,
        _deformations: Option<&[N]>,
        _dir: &Unit<Vector<N>>,
    ) -> bool;

    /// Returns the id of the subshape containing the specified feature.
    ///
    /// If several subshape contains the same feature, any one is returned.
    fn subshape_containing_feature(&self, _i: FeatureId) -> usize {
        0
    }

    /// The `RayCast` implementation of `self`.
    #[inline]
    fn as_ray_cast(&self) -> Option<&RayCast<N>> {
        None
    }

    /// The `PointQuery` implementation of `self`.
    #[inline]
    fn as_point_query(&self) -> Option<&PointQuery<N>> {
        None
    }

    /// The convex polyhedron representation of `self` if applicable.
    #[inline]
    fn as_convex_polyhedron(&self) -> Option<&ConvexPolyhedron<N>> {
        None
    }

    /// The support mapping of `self` if applicable.
    #[inline]
    fn as_support_map(&self) -> Option<&SupportMap<N>> {
        None
    }

    /// The composite shape representation of `self` if applicable.
    #[inline]
    fn as_composite_shape(&self) -> Option<&CompositeShape<N>> {
        None
    }

    /// The deformable shape representation of `self` if applicable.
    #[inline]
    fn as_deformable_shape(&self) -> Option<&DeformableShape<N>> {
        None
    }

    /// The mutable deformable shape representation of `self` if applicable.
    #[inline]
    fn as_deformable_shape_mut(&mut self) -> Option<&mut DeformableShape<N>> {
        None
    }

    /// Whether `self` uses a convex polyhedron representation.
    #[inline]
    fn is_convex_polyhedron(&self) -> bool {
        self.as_convex_polyhedron().is_some()
    }

    /// Whether `self` uses a support-mapping based representation.
    #[inline]
    fn is_support_map(&self) -> bool {
        self.as_support_map().is_some()
    }

    /// Whether `self` uses a composite shape-based representation.
    #[inline]
    fn is_composite_shape(&self) -> bool {
        self.as_composite_shape().is_some()
    }

    /// Whether `self` uses a composite shape-based representation.
    #[inline]
    fn is_deformable_shape(&self) -> bool {
        self.as_deformable_shape().is_some()
    }
}

impl_downcast!(Shape<N> where N: RealField);

/// Trait for casting shapes to its exact represetation.
impl<N: RealField> Shape<N> {
    /// Tests if this shape has a specific type `T`.
    #[inline]
    pub fn is_shape<T: Shape<N>>(&self) -> bool {
        self.is::<T>()
    }

    /// Performs the cast.
    #[inline]
    pub fn as_shape<T: Shape<N>>(&self) -> Option<&T> {
        self.downcast_ref()
    }
}

impl<N: RealField> Clone for Box<Shape<N>> {
    fn clone(&self) -> Box<Shape<N>> {
        self.clone_box()
    }
}

/// A shared handle to an abstract shape.
///
/// This can be mutated using COW.
#[derive(Clone)]
pub struct ShapeHandle<N: RealField>(Arc<Box<Shape<N>>>);

impl<N: RealField> ShapeHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<N> + Clone>(shape: S) -> ShapeHandle<N> {
        ShapeHandle(Arc::new(Box::new(shape)))
    }

    pub(crate) fn make_mut(&mut self) -> &mut Shape<N> {
        &mut **Arc::make_mut(&mut self.0)
    }
}

impl<N: RealField> AsRef<Shape<N>> for ShapeHandle<N> {
    #[inline]
    fn as_ref(&self) -> &Shape<N> {
        &*self.deref()
    }
}

impl<N: RealField> Deref for ShapeHandle<N> {
    type Target = Shape<N>;

    #[inline]
    fn deref(&self) -> &Shape<N> {
        &**self.0.deref()
    }
}