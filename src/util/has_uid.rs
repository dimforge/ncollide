//! Objects with an unique identifier.

use std::gc::Gc;
use std::rc::Rc;
use sync::{Arc, RWArc};

/// Trait of objects having an unique identifier.
pub trait HasUid {
    /// An unique identifier. It should be O(1).
    fn uid(&self) -> uint;
}

impl<T: 'static> HasUid for Gc<T> {
    fn uid(&self) -> uint {
        self.borrow() as *T as uint
    }
}

impl<T> HasUid for Rc<T> {
    fn uid(&self) -> uint {
        &**self as *T as uint
    }
}

impl<T: Share + Send> HasUid for Arc<T> {
    fn uid(&self) -> uint {
        self.get() as *T as uint
    }
}

impl<T: Share + Send> HasUid for RWArc<T> {
    fn uid(&self) -> uint {
        self.read(|t| t as *T as uint)
    }
}

impl HasUid for int {
    fn uid(&self) -> uint {
        *self as uint
    }
}

impl HasUid for uint {
    fn uid(&self) -> uint {
        *self
    }
}
