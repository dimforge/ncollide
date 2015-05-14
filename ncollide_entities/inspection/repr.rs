use std::mem;
use std::any::{Any, TypeId};
use std::marker::PhantomData;

// Define our own because it is unstable.
/// Raw representation of a trait-object.
#[allow(raw_pointer_derive)]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TraitObject {
    /// Raw pointer to the trait object data.
    pub data:   *mut (),
    /// Raw pointer to the trait object virtual table.
    pub vtable: *mut ()
}

#[derive(Clone, Copy)]
pub struct ReprDesc<'a> {
    type_id: TypeId,
    repr_id: TypeId,
    repr:    TraitObject,
    life:    PhantomData<fn() -> &'a ()>
}

impl<'a> ReprDesc<'a> {
    /// Creates a new representation descriptor.
    ///
    /// This is unsafe as there is no way to check that the given triple of data are valid.
    #[inline]
    pub unsafe fn new(type_id: TypeId, repr_id: TypeId, repr: TraitObject) -> ReprDesc<'a> {
        ReprDesc {
            type_id: type_id,
            repr_id: repr_id,
            repr:    repr,
            life:    PhantomData
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

    /// Converts this repr as an exact shape.
    #[inline]
    pub fn downcast_ref<T: 'static + Any>(&self) -> Option<&T> {
        if self.type_id == TypeId::of::<T>() {
            Some(unsafe { mem::transmute(self.repr.data) })
        }
        else {
            None
        }
    }
}

/// An object with a unique runtime geometric representation.
pub trait Repr<P, M>: Send + Sync + 'static {
    /// Gets a reference to this object's main representation.
    fn repr<'a>(&'a self) -> ReprDesc<'a>;
}
