//! Hashable pair of objects implementing `HasUid`.

use utils::data::hash;
use utils::data::hash::HashFun;
use utils::data::has_uid::HasUid;

/// An unordered pair of elements implementing `HasUid`.
#[deriving(Clone, Encodable, Decodable)]
pub struct Pair<B> {
    /// first object of the pair
    pub first:  B,
    /// second object of the pair
    pub second: B,

    ifirst:  uint,
    isecond: uint
}

impl<B: HasUid> Pair<B> {
    /// Builds a new `Pair`.
    pub fn new(a: B, b: B) -> Pair<B> {
        let ia = a.uid();
        let ib = b.uid();

        if a.uid() < b.uid() {
            Pair {
                first: a, second: b, ifirst: ia, isecond: ib
            }
        }
        else {
            Pair {
                first: a, second: b, ifirst: ib, isecond: ia
            }
        }
    }
}

impl<B> PartialEq for Pair<B> {
    #[inline]
    fn eq(&self, other: &Pair<B>) -> bool {
        self.ifirst == other.ifirst && self.isecond == other.isecond
    }
}

/// Tomas Wang based hash function for a `Pair` object.
#[deriving(Encodable, Decodable)]
pub struct PairTWHash { unused: uint } // FIXME: ICE with zero-sized structs

impl PairTWHash {
    /// Creates a new PairTWHash
    pub fn new() -> PairTWHash {
        PairTWHash { unused: 0 }
    }
}

impl<B> HashFun<Pair<B>> for PairTWHash {
    #[inline]
    fn hash(&self, p: &Pair<B>) -> uint {
        hash::tomas_wang_hash(
            hash::key_from_pair(
                p.ifirst, p.isecond
            )
        )
    }
}
