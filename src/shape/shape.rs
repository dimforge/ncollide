// Queries.
use bounding_volume::{AABB, BoundingSphere};
use math::{Isometry, Point, Vector};
use na::{self, Real, Unit};
use query::{PointQuery, RayCast};
// Repr.
use shape::{CompositeShape, ConvexPolyhedron, DeformableShape, FeatureId, SupportMap};
use std::any::{Any, TypeId};
use std::mem;
use std::ops::Deref;
use std::sync::Arc;


pub trait ShapeClone<N: Real> {
    fn clone_box(&self) -> Box<Shape<N>> {
        unimplemented!()
    }
}

impl<N: Real, T: 'static + Shape<N> + Clone> ShapeClone<N> for T
{
    fn clone_box(&self) -> Box<Shape<N>> {
        Box::new(self.clone())
    }
}

/// Trait implemented by all shapes supported by ncollide.
///
/// This allows dynamic inspection of the shape capabilities.
pub trait Shape<N: Real>: Send + Sync + Any + GetTypeId + ShapeClone<N> {
    /// The AABB of `self`.
    #[inline]
    fn aabb(&self, m: &Isometry<N>) -> AABB<N>;

    /// The bounding sphere of `self`.
    #[inline]
    fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let aabb = self.aabb(m);
        BoundingSphere::new(aabb.center(), na::norm_squared(&aabb.half_extents()))
    }

    /// Check if if the feature `_feature` of the `i-th` subshape of `self` transformed by `m` has a tangent
    /// cone that contains `dir` at the point `pt`.
    // NOTE: for the moment, we assume the tangent cone is the same for the whole feature.
    #[inline]
    fn subshape_tangent_cone_contains_dir(&self, _feature: FeatureId, _m: &Isometry<N>, _dir: &Unit<Vector<N>>) -> bool;

    /// The transform of a specific subshape.
    ///
    /// Return `None` if the transform is known to be the identity.
    #[inline]
    fn subshape_transform(&self, _: usize) -> Option<Isometry<N>> {
        None
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
    fn as_deformable_shape(&self) -> Option<&DeformableShape<N>> { None }

    /// The mutable deformable shape representation of `self` if applicable.
    #[inline]
    fn as_deformable_shape_mut(&mut self) -> Option<&mut DeformableShape<N>> { None }

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

// FIXME: use the downcast-rs crate instead
// Define our own because it is unstable.
/// Raw representation of a trait-object.
#[repr(C)]
#[derive(Clone, Copy)]
struct TraitObject {
    /// Raw pointer to the trait object data.
    pub data: *mut (),
    /// Raw pointer to the trait object virtual table.
    pub vtable: *mut (),
}

/// Trait for casting shapes to its exact represetation.
impl<N: Real> Shape<N> {
    /// Tests if this shape has a specific type `T`.
    #[inline]
    pub fn is_shape<T: Shape<N>>(&self) -> bool {
        self.type_id() == TypeId::of::<T>()
    }

    /// Performs the cast.
    #[inline]
    pub fn as_shape<T: Shape<N>>(&self) -> Option<&T> {
        if self.is_shape::<T>() {
            unsafe {
                let to: TraitObject = mem::transmute(self);
                mem::transmute(to.data)
            }
        } else {
            None
        }
    }
}

impl<N: Real> Clone for Box<Shape<N>> {
    fn clone(&self) -> Box<Shape<N>> {
        self.clone_box()
    }
}

/// A shared immutable handle to an abstract shape.
#[derive(Clone)]
pub struct ShapeHandle<N: Real> {
    handle: Arc<Box<Shape<N>>>,
}

impl<N: Real> ShapeHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<N> + Clone>(shape: S) -> ShapeHandle<N> {
        ShapeHandle {
            handle: Arc::new(Box::new(shape)),
        }
    }

    pub(crate) fn make_mut(&mut self) -> &mut Shape<N> {
        &mut **Arc::make_mut(&mut self.handle)
    }
}

impl<N: Real> AsRef<Shape<N>> for ShapeHandle<N> {
    #[inline]
    fn as_ref(&self) -> &Shape<N> {
        &*self.deref()
    }
}

impl<N: Real> Deref for ShapeHandle<N> {
    type Target = Shape<N>;

    #[inline]
    fn deref(&self) -> &Shape<N> {
        &**self.handle.deref()
    }
}

/// Trait to retrieve the `TypeId` of a shape.
///
/// This exists only because `Any::get_type_id()` is unstable.
pub unsafe trait GetTypeId {
    /// Gets the dynamic type identifier of this shape.
    #[inline]
    fn type_id(&self) -> TypeId;
}

unsafe impl<T: Any> GetTypeId for T {
    #[inline]
    fn type_id(&self) -> TypeId {
        TypeId::of::<Self>()
    }
}
