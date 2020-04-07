// Queries.
use crate::bounding_volume::{BoundingSphere, AABB};
use crate::math::{Isometry, Vector};
use crate::query::{PointQuery, RayCast};
use crate::shape::{CompositeShape, ConvexPolyhedron, DeformableShape, FeatureId, SupportMap};
use downcast_rs::Downcast;
use na::{self, RealField, Unit};
use std::ops::Deref;
use std::sync::Arc;

pub trait ShapeClone<N: RealField> {
    /// Construct an `Arc` that refers to a uniquely-owned copy of `self`
    fn clone_arc(&self) -> Arc<dyn Shape<N>>;
}

impl<N: RealField, T: 'static + Shape<N> + Clone> ShapeClone<N> for T {
    fn clone_arc(&self) -> Arc<dyn Shape<N>> {
        Arc::new(self.clone())
    }
}

/// Trait implemented by all shapes supported by ncollide.
///
/// This allows dynamic inspection of the shape capabilities.
pub trait Shape<N: RealField>: Send + Sync + Downcast + ShapeClone<N> {
    /// The AABB of `self` transformed by `m`.
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
        BoundingSphere::new(aabb.center(), aabb.half_extents().norm())
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
    fn as_ray_cast(&self) -> Option<&dyn RayCast<N>> {
        None
    }

    /// The `PointQuery` implementation of `self`.
    #[inline]
    fn as_point_query(&self) -> Option<&dyn PointQuery<N>> {
        None
    }

    /// The convex polyhedron representation of `self` if applicable.
    #[inline]
    fn as_convex_polyhedron(&self) -> Option<&dyn ConvexPolyhedron<N>> {
        None
    }

    /// The support mapping of `self` if applicable.
    #[inline]
    fn as_support_map(&self) -> Option<&dyn SupportMap<N>> {
        None
    }

    /// The composite shape representation of `self` if applicable.
    #[inline]
    fn as_composite_shape(&self) -> Option<&dyn CompositeShape<N>> {
        None
    }

    /// The deformable shape representation of `self` if applicable.
    #[inline]
    fn as_deformable_shape(&self) -> Option<&dyn DeformableShape<N>> {
        None
    }

    /// The mutable deformable shape representation of `self` if applicable.
    #[inline]
    fn as_deformable_shape_mut(&mut self) -> Option<&mut dyn DeformableShape<N>> {
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
impl<N: RealField> dyn Shape<N> {
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

/// A shared handle to an abstract shape.
///
/// This can be mutated using COW.
#[derive(Clone)]
pub struct ShapeHandle<N: RealField>(Arc<dyn Shape<N>>);

impl<N: RealField> ShapeHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<N>>(shape: S) -> ShapeHandle<N> {
        ShapeHandle(Arc::new(shape))
    }

    /// Creates a sharable shape handle from a shape trait object.
    pub fn from_arc(shape: Arc<dyn Shape<N>>) -> ShapeHandle<N> {
        ShapeHandle(shape)
    }

    pub(crate) fn make_mut(&mut self) -> &mut dyn Shape<N> {
        if Arc::get_mut(&mut self.0).is_none() {
            let unique_self = self.0.clone_arc();
            self.0 = unique_self;
        }
        Arc::get_mut(&mut self.0).unwrap()
    }
}

impl<N: RealField> AsRef<dyn Shape<N>> for ShapeHandle<N> {
    #[inline]
    fn as_ref(&self) -> &dyn Shape<N> {
        &*self.0
    }
}

impl<N: RealField> Deref for ShapeHandle<N> {
    type Target = dyn Shape<N>;

    #[inline]
    fn deref(&self) -> &dyn Shape<N> {
        &*self.0
    }
}
