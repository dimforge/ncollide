// FIXME: add a limit to the cache?
/// Cache for owned objects. Useful if fast allocation/deallocation of small owned objects is
/// needed.
pub struct OwnedAllocationCache<T> {
    priv cache: ~[~T]
}

impl<T> OwnedAllocationCache<T> {
    /// Initializes the cache.
    #[inline]
    pub fn new() -> OwnedAllocationCache<T> {
        OwnedAllocationCache {
            cache: ~[]
        }
    }

    /// Box a value into a potentially already allocated box.
    #[inline]
    pub fn box(&mut self, value: T) -> ~T {
        if !self.cache.is_empty() {
            let mut res = self.cache.pop();
            *res = value;

            res
        }
        else {
            ~value
        }
    }

    /// Retains a box which can be re-used by the `box` method.
    #[inline]
    pub fn retain(&mut self, elem: ~T) {
        self.cache.push(elem)
    }

    /// Clears the cache, destroying any stored pointer.
    #[inline]
    pub fn clear(&mut self) {
        self.cache.clear()
    }
}
