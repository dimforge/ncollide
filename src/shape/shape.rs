// Queries.
use crate::bounding_volume::{BoundingSphere, AABB};
use crate::math::{Isometry, Vector};
use na::{self, RealField, Unit};
use crate::query::{PointQuery, RayCast};
use crate::shape::{CompositeShape, ConvexPolyhedron, DeformableShape, FeatureId, SupportMap};
use std::ops::{Deref, DerefMut};
use std::convert::AsRef;
use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};
use downcast_rs::Downcast;

pub trait ShapeClone<N: RealField> {
    fn clone_box(&self) -> Box<dyn Shape<N>> {
        unimplemented!()
    }
}

impl<N: RealField, T: 'static + Shape<N> + Clone> ShapeClone<N> for T {
    fn clone_box(&self) -> Box<dyn Shape<N>> {
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

impl<N: RealField> Clone for Box<dyn Shape<N>> {
    fn clone(&self) -> Box<dyn Shape<N>> {
        self.clone_box()
    }
}

/// An to an abstract shape.
///
/// The shape can either be uniquely owned, immutable shared, or mutably shared.
#[derive(Clone)]
pub enum ShapeHandle<N: RealField> {
    Owned(Box<dyn Shape<N>>),
    Shared(Arc<Box<dyn Shape<N>>>), // FIXME: get rid of the box inside of the Arc.
    SharedMutable(Arc<RwLock<(u64, Box<dyn Shape<N>>)>>) // FIXME: get rid of the Box once the `unsized_tuple_coercion` rust feature is stabilized.
}

impl<N: RealField> ShapeHandle<N> {
    #[deprecated = "Use `Self::new_shared` instead to make the choice of ownership clear."]
    pub fn new(shape: impl Shape<N>) -> Self {
        Self::new_shared(shape)
    }
    pub fn new_owned(shape: impl Shape<N>) -> Self {
        ShapeHandle::Owned(Box::new(shape))
    }

    pub fn new_shared(shape: impl Shape<N>) -> Self {
        ShapeHandle::Shared(Arc::new(Box::new(shape)))
    }

    pub fn new_shared_mutable(shape: impl Shape<N>) -> Self {
        ShapeHandle::SharedMutable(Arc::new(RwLock::new((0, Box::new(shape)))))
    }
}

pub enum ShapeRef<'a, N: RealField> {
    OwnedRef(&'a dyn Shape<N>),
    SharedRef(RwLockReadGuard<'a, (u64, Box<dyn Shape<N>>)>)
}

pub enum ShapeMut<'a, N: RealField> {
    OwnedRef(&'a mut dyn Shape<N>),
    SharedRef(RwLockWriteGuard<'a, (u64, Box<dyn Shape<N>>)>)
}

impl<N: RealField> ShapeHandle<N> {
    pub fn as_ref(&self) -> ShapeRef<N> {
        match self {
            ShapeHandle::Owned(shape) => ShapeRef::OwnedRef(&**shape),
            ShapeHandle::Shared(shape) => ShapeRef::OwnedRef(&***shape),
            ShapeHandle::SharedMutable(shape) => ShapeRef::SharedRef(shape.read().unwrap()),
        }
    }

    pub fn make_mut(&mut self) -> ShapeMut<N> {
        match self {
            ShapeHandle::Owned(shape) => ShapeMut::OwnedRef(&mut **shape),
            ShapeHandle::Shared(shape) => ShapeMut::OwnedRef(&mut **Arc::make_mut(shape)),
            ShapeHandle::SharedMutable(shape) => {
                let mut write = shape.write().unwrap();
                write.0 += 1; // Register the fact there was a mutable borrow (and thus a possible shape modification).
                ShapeMut::SharedRef(write)
            },
        }
    }

    pub(crate) fn is_shared_and_modified(&self, unmodified_timestamp: u64) -> bool {
        if let ShapeHandle::SharedMutable(shape) = self {
            shape.read().unwrap().0 == unmodified_timestamp
        } else {
            false
        }
    }

    pub(crate) fn modification_timestamp(&self) -> u64 {
        if let ShapeHandle::SharedMutable(shape) = self {
            shape.read().unwrap().0
        } else {
            0
        }
    }
}

impl<'a, N: RealField> Deref for ShapeRef<'a, N> {
    type Target = dyn Shape<N>;

    fn deref(&self) -> &dyn Shape<N> {
        match self {
            ShapeRef::OwnedRef(s) => s.deref(),
            ShapeRef::SharedRef(s) => &*s.1,
        }
    }
}

impl<'a, N: RealField> Deref for ShapeMut<'a, N> {
    type Target = dyn Shape<N>;

    fn deref(&self) -> &dyn Shape<N> {
        match self {
            ShapeMut::OwnedRef(s) => s.deref(),
            ShapeMut::SharedRef(s) => &*s.1,
        }
    }
}

impl<'a, N: RealField> DerefMut for ShapeMut<'a, N> {
    fn deref_mut(&mut self) -> &mut dyn Shape<N> {
        match self {
            ShapeMut::OwnedRef(s) => s.deref_mut(),
            ShapeMut::SharedRef(s) => &mut *s.1,
        }
    }
}