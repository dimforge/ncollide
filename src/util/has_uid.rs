use std::gc::Gc;
use std::rc::Rc;
use std::ptr;
use extra::arc::{Arc, RWArc};

/// Trait of objects having an unique identifier.
pub trait HasUid {
    /// An unique identifier. It should be O(1).
    fn uid(&self) -> uint;
}

impl<T: 'static> HasUid for Gc<T> {
    fn uid(&self) -> uint {
        ptr::to_unsafe_ptr(self.borrow()) as uint
    }
}

impl<T> HasUid for Rc<T> {
    fn uid(&self) -> uint {
        ptr::to_unsafe_ptr(self.borrow()) as uint
    }
}

impl<T: Freeze + Send> HasUid for Arc<T> {
    fn uid(&self) -> uint {
        ptr::to_unsafe_ptr(self.get()) as uint
    }
}

impl<T: Freeze + Send> HasUid for RWArc<T> {
    fn uid(&self) -> uint {
        self.read(|t| ptr::to_unsafe_ptr(t) as uint)
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
