use std::any::{Any, TypeId};

pub trait AnyPrivate : Any {
    /// The type id of `Self`.
    ///
    /// NOTE: this exists only because `Any::get_type_id()` is private for some reason…
    fn get_dyn_type_id(&self) -> TypeId;
}

impl<T: Any> AnyPrivate for T {
    #[inline]
    fn get_dyn_type_id(&self) -> TypeId {
        TypeId::of::<T>()
    }
}

