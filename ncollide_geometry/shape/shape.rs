use std::mem;
use std::any::{Any, TypeId};
use std::ops::Deref;
use std::sync::Arc;

use na;

// Repr.
use shape::{CompositeShape, SupportMap};
// Queries.
use bounding_volume::{AABB, BoundingSphere};
use query::{RayCast, PointQuery};
use math::Point;

/// Trait implemented by all shapes supported by ncollide.
///
/// This allows dynamic inspection of the shape capabilities.
pub trait Shape<P: Point, M> : Send + Sync + Any + GetTypeId {
    /// The AABB of `self`.
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P>;

    /// The bounding sphere of `self`.
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
        let aabb = self.aabb(m);

        BoundingSphere::new(aabb.center(), na::norm_squared(&aabb.half_extents()))
    }

    /// The `RayCast` implementation of `self`.
    #[inline]
    fn as_ray_cast(&self) -> Option<&RayCast<P, M>> {
        None
    }

    /// The `PointQuery` implementation of `self`.
    #[inline]
    fn as_point_query(&self) -> Option<&PointQuery<P, M>> {
        None
    }


    /// The support mapping of `self` if applicable.
    #[inline]
    fn as_support_map(&self) -> Option<&SupportMap<P, M>> {
        None
    }

    /// The composite shape representation of `self` if applicable.
    #[inline]
    fn as_composite_shape(&self) -> Option<&CompositeShape<P, M>> {
        None
    }

    /// Whether `self` uses a supportmapping-based representation.
    #[inline]
    fn is_support_map(&self) -> bool {
        self.as_support_map().is_some()
    }

    /// Whether `self` uses a composite shape-based representation.
    #[inline]
    fn is_composite_shape(&self) -> bool {
        self.as_composite_shape().is_some()
    }
}

// Define our own because it is unstable.
/// Raw representation of a trait-object.
#[repr(C)]
#[derive(Clone, Copy)]
struct TraitObject {
    /// Raw pointer to the trait object data.
    pub data:   *mut (),
    /// Raw pointer to the trait object virtual table.
    pub vtable: *mut ()
}

/// Trait for casting shapes to its exact represetation.
impl<P: Point, M: 'static> Shape<P, M> {
    /// Tests if this shape has a specific type `T`.
    #[inline]
    pub fn is_shape<T: Shape<P, M>>(&self) -> bool {
        self.type_id() == TypeId::of::<T>()
    }

    /// Performs the cast.
    #[inline]
    pub fn as_shape<T: Shape<P, M>>(&self) -> Option<&T> {
        if self.is_shape::<T>() {
            unsafe {
                let to: TraitObject = mem::transmute(self);
                mem::transmute(to.data)
            }
        }
        else {
            None
        }
    }
}

/// A shared immutable handle to an abstract shape.
#[derive(Clone)]
pub struct ShapeHandle<P: Point, M> {
    handle: Arc<Box<Shape<P, M>>>
}

impl<P: Point, M> ShapeHandle<P, M> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<P, M>>(shape: S) -> ShapeHandle<P, M> {
        ShapeHandle {
            handle: Arc::new(Box::new(shape) as Box<Shape<P, M>>)
        }
    }
}

impl<P: Point, M> AsRef<Shape<P, M>> for ShapeHandle<P, M> {
    #[inline]
    fn as_ref(&self) -> &Shape<P, M> {
        self.deref()
    }
}

impl<P: Point, M> Deref for ShapeHandle<P, M> {
    type Target = Shape<P, M>;

    #[inline]
    fn deref(&self) -> &Shape<P, M> {
        &**self.handle
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
