//! Allocation cache for owned objects.

// FIXME: add a limit to the cache?
/// Cache for owned objects.
///
/// Useful if fast allocation/deallocation of small owned objects is needed.
pub struct OwnedAllocationCache<T> {
    cache: Vec<Box<T>>
}

impl<T> OwnedAllocationCache<T> {
    /// Initializes the cache.
    #[inline]
    pub fn new() -> OwnedAllocationCache<T> {
        OwnedAllocationCache {
            cache: Vec::new()
        }
    }

    /// Box a value into a potentially already allocated box.
    #[inline]
    pub fn alloc(&mut self, value: T) -> Box<T> {
        if !self.cache.is_empty() {
            let mut res = self.cache.pop().unwrap();
            *res = value;

            res
        }
        else {
            box value
        }
    }

    /// Retains a box which can be re-used by the `box` method.
    #[inline]
    pub fn retain(&mut self, elem: Box<T>) {
        self.cache.push(elem)
    }

    /// Clears the cache, destroying any stored pointer.
    #[inline]
    pub fn clear(&mut self) {
        self.cache.clear()
    }
}
