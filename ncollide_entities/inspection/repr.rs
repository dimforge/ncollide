use std::mem;
use std::any::TypeId;
use std::raw::TraitObject;
use std::marker::ContravariantLifetime;

#[derive(Copy)]
pub struct ReprDesc<'a> {
    type_id: TypeId,
    repr_id: TypeId,
    repr:    TraitObject,
    life:    ContravariantLifetime<'a>
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
            life:    ContravariantLifetime
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
    pub fn downcast_ref<T: 'static>(&self) -> Option<&T> {
        if self.type_id == TypeId::of::<T>() {
            Some(unsafe { mem::transmute(self.repr.data) })
        }
        else {
            None
        }
    }
}

/// An object with a unique runtime geometric representation.
pub trait Repr<N, P, V, M>: Send + Sync {
    /// Gets a reference to this object's main representation.
    fn repr<'a>(&'a self) -> ReprDesc<'a>;
}
