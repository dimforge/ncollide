use std::mem;
use std::any::{Any, TypeId};
use std::ops::Deref;
use std::sync::Arc;

use na::{self, Real};

// Repr.
use shape::{CompositeShape, ConvexPolyhedron, SupportMap};
// Queries.
use bounding_volume::{BoundingSphere, AABB};
use query::{PointQuery, RayCast};
use math::Isometry;
#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};
#[cfg(feature = "serde")]
use serde::de::DeserializeOwned;

//serialize_trait_object!(Shape<f32>);

/// Trait implemented by all shapes supported by ncollide.
///
/// This allows dynamic inspection of the shape capabilities.
pub trait Shape<N: Real>: Send + Sync + Any + GetTypeId {
    /// The AABB of `self`.
    #[inline]
    fn aabb(&self, m: &Isometry<N>) -> AABB<N>;

    /// The bounding sphere of `self`.
    #[inline]
    fn bounding_sphere(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let aabb = self.aabb(m);
        BoundingSphere::new(aabb.center(), na::norm_squared(&aabb.half_extents()))
    }

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

    /// Whether `self` uses a conve polyhedron representation.
    #[inline]
    fn is_convex_polyhedron(&self) -> bool {
        self.as_convex_polyhedron().is_some()
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

/// A shared immutable handle to an abstract shape.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ShapeHandle<N: Real> {
    handle: Arc<Shape<N>>,
}

impl<N: Real> ShapeHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<N>>(shape: S) -> ShapeHandle<N> {
        ShapeHandle {
            handle: Arc::new(shape),
        }
    }
}

impl<N: Real> AsRef<Shape<N>> for ShapeHandle<N> {
    #[inline]
    fn as_ref(&self) -> &Shape<N> {
        self.deref()
    }
}

impl<N: Real> Deref for ShapeHandle<N> {
    type Target = Shape<N>;

    #[inline]
    fn deref(&self) -> &Shape<N> {
        self.handle.deref()
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
