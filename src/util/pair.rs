use std::ptr;
use std::managed;
use util::hash;
use util::hash::HashFun;

/// Pair of managed boxes. It is accompagned with a fast hash function.
pub struct Pair<B> {
    /// first object of the pair
    first: @mut B,
    /// second object of the pair
    second: @mut B,

    // FIXME is is really more efficient than checking all possibilitios on the `Eq`
    // implementation?
    priv sfirst:  @mut B,
    priv ssecond: @mut B
}

impl<B> Clone for Pair<B> {
    fn clone(&self) -> Pair<B> {
        Pair::new(self.first, self.second)
    }
}

impl<B> Pair<B> {
    /// Builds a new pair of managed boxes.
    pub fn new(a: @mut B, b: @mut B) -> Pair<B> {
        if ptr::to_mut_unsafe_ptr(a) < ptr::to_mut_unsafe_ptr(b) {
            Pair {
                first: a, second: b, sfirst: a, ssecond: b
            }
        }
        else {
            Pair {
                first: a, second: b, sfirst: b, ssecond: a
            }
        }
    }
}

impl<B> Eq for Pair<B> {
    fn eq(&self, other: &Pair<B>) -> bool {
        managed::mut_ptr_eq(self.sfirst, other.sfirst) &&
        managed::mut_ptr_eq(self.ssecond, other.ssecond)
    }
}

/// Tomas Wang based hash function for a `Pair` object.
pub struct PairTWHash;

impl<B> HashFun<Pair<B>> for PairTWHash {
    fn hash(p: &Pair<B>) -> uint {
        hash::tomas_wang_hash(
            hash::key_from_pair(
                ptr::to_mut_unsafe_ptr(p.sfirst) as uint, ptr::to_mut_unsafe_ptr(p.ssecond) as uint
            )
        )
    }
}
