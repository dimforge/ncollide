//! Objects with an unique identifier.

use std::rc::Rc;
use std::sync::Arc;

/// Trait of objects having an unique identifier.
pub trait HasUid {
    /// An unique identifier. It should be O(1).
    fn uid(&self) -> uint;
}

impl<T> HasUid for Rc<T> {
    #[inline]
    fn uid(&self) -> uint {
        &**self as *const T as uint
    }
}

impl<T: Sync + Send> HasUid for Arc<T> {
    #[inline]
    fn uid(&self) -> uint {
        self.deref() as *const T as uint
    }
}

impl HasUid for uint {
    #[inline]
    fn uid(&self) -> uint {
        *self
    }
}
