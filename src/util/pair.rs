//! Pair of managed boxes.

use util::hash;
use util::hash::HashFun;
use util::has_uid::HasUid;

/// Pair of managed boxes. It is accompagned with a fast hash function.
#[deriving(Clone, Encodable, Decodable)]
pub struct Pair<B> {
    /// first object of the pair
    first:  B,
    /// second object of the pair
    second: B,

    priv ifirst:  uint,
    priv isecond: uint
}

impl<B: HasUid> Pair<B> {
    /// Builds a new pair of managed boxes.
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

impl<B> Eq for Pair<B> {
    fn eq(&self, other: &Pair<B>) -> bool {
        self.ifirst == other.ifirst && self.isecond == other.isecond
    }
}

/// Tomas Wang based hash function for a `Pair` object.
#[deriving(Encodable, Decodable)]
pub struct PairTWHash { priv unused: uint } // FIXME: ICE with zero-sized structs

impl PairTWHash {
    /// Creates a new PairTWHash
    pub fn new() -> PairTWHash {
        PairTWHash { unused: 0 }
    }
}

impl<B> HashFun<Pair<B>> for PairTWHash {
    fn hash(&self, p: &Pair<B>) -> uint {
        hash::tomas_wang_hash(
            hash::key_from_pair(
                p.ifirst, p.isecond
            )
        )
    }
}
