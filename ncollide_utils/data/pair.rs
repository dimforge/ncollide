//! Hashable pair of objects implementing `HasUid`.

use data::hash;
use data::hash::HashFun;
use data::uid_remap::FastKey;

// XXX: Rename this `FastKeyPair`.

/// An unordered pair of elements implementing `HasUid`.
#[derive(Clone, Copy, RustcEncodable, RustcDecodable)]
pub struct Pair {
    /// first object of the pair
    pub first: FastKey,
    /// second object of the pair
    pub second: FastKey,

    ifirst: usize,
    isecond: usize,
}

impl Pair {
    /// Builds a new `Pair`.
    pub fn new(a: FastKey, b: FastKey) -> Pair {
        let ia = a.uid();
        let ib = b.uid();

        if a.uid() < b.uid() {
            Pair {
                first: a,
                second: b,
                ifirst: ia,
                isecond: ib,
            }
        } else {
            Pair {
                first: a,
                second: b,
                ifirst: ib,
                isecond: ia,
            }
        }
    }
}

impl PartialEq for Pair {
    #[inline]
    fn eq(&self, other: &Pair) -> bool {
        self.ifirst == other.ifirst && self.isecond == other.isecond
    }
}

/// Tomas Wang based hash function for a `Pair` object.
#[derive(RustcEncodable, RustcDecodable)]
pub struct PairTWHash {
    unused: usize,
} // FIXME: ICE with zero-sized structs

impl PairTWHash {
    /// Creates a new PairTWHash
    pub fn new() -> PairTWHash {
        PairTWHash { unused: 0 }
    }
}

impl HashFun<Pair> for PairTWHash {
    #[inline]
    fn hash(&self, p: &Pair) -> usize {
        hash::tomas_wang_hash(hash::key_from_pair(p.ifirst, p.isecond))
    }
}
