use std::mem;
use std::any::{Any, TypeId};
use std::marker::PhantomData;
use shape::CompositeShape;
use support_map::SupportMap;

// Define our own because it is unstable.
/// Raw representation of a trait-object.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TraitObject {
    /// Raw pointer to the trait object data.
    pub data:   *mut (),
    /// Raw pointer to the trait object virtual table.
    pub vtable: *mut ()
}

#[derive(Clone, Copy)]
pub struct ShapeDesc<'a, P, M> {
    type_id:      TypeId,
    repr_id:      TypeId,
    repr:         TraitObject,
    life:         PhantomData<fn() -> &'a ()>,
    _point_type:  PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<'a, P, M> ShapeDesc<'a, P, M> {
    /// Creates a new representation descriptor.
    ///
    /// This is unsafe as there is no way to check that the given triple of data are valid.
    #[inline]
    pub unsafe fn new(type_id: TypeId, repr_id: TypeId, repr: TraitObject) -> ShapeDesc<'a, P, M> {
        ShapeDesc {
            type_id:      type_id,
            repr_id:      repr_id,
            repr:         repr,
            life:         PhantomData,
            _point_type:  PhantomData,
            _matrix_type: PhantomData,
        }
    }

    /// `TypeId` of this shape's exact type.
    #[inline]
    pub fn type_id(&self) -> TypeId {
        self.type_id
    }

    /// `TypeId` of this shape's representation.
    #[inline]
    pub fn repr_id(&self) -> TypeId {
        self.repr_id
    }

    /// This shape's representation.
    #[inline]
    pub fn repr(&self) -> TraitObject {
        self.repr
    }

    /// Tests if `self` is a shape of type `T`.
    #[inline]
    pub fn is_shape<T: Shape<P, M>>(&self) -> bool {
        self.type_id == TypeId::of::<T>()
    }

    /// Converts `self` to a reference to a shape of type `T`.
    #[inline]
    pub fn as_shape<T: Shape<P, M>>(&self) -> Option<&T> {
        if self.is_shape::<T>() {
            Some(unsafe { mem::transmute(self.repr.data) })
        }
        else {
            None
        }
    }
}

impl<'a, P: Any, M: Any> ShapeDesc<'a, P, M> {
    /// Tests if `self` is represented as a support mapping.
    #[inline]
    pub fn is_support_map(&self) -> bool {
        self.repr_id == TypeId::of::<SupportMap<P, M>>()
    }

    /// Converts `self` to a reference to a support mapping.
    #[inline]
    pub fn as_support_map(&self) -> Option<&SupportMap<P, M>> {
        if self.is_support_map() {
            Some(unsafe { mem::transmute(self.repr) })
        }
        else {
            None
        }
    }

    /// Tests if `self` is represented as a composite shape.
    #[inline]
    pub fn is_composite_shape(&self) -> bool {
        self.repr_id == TypeId::of::<CompositeShape<P, M>>()
    }

    /// Converts `self` to a reference to a composite shape.
    #[inline]
    pub fn as_composite_shape(&self) -> Option<&CompositeShape<P, M>> {
        if self.is_composite_shape() {
            Some(unsafe { mem::transmute(self.repr) })
        }
        else {
            None
        }
    }
}

/// An object with a unique runtime geometric representation.
pub trait Shape<P, M>: Send + Sync + Any + 'static {
    /// Gets this object's shape descriptor.
    fn desc<'a>(&'a self) -> ShapeDesc<'a, P, M>;
}
