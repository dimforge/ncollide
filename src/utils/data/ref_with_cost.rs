//! A reference packed with a cost value.

use std::cmp::Ordering;
use std::cmp::Ordering::{Greater, Less, Equal};

/// A reference packed with a cost value.
pub struct RefWithCost<'a, N, T: 'a> {
    /// The reference to an object.
    pub object: &'a T,
    /// The cost of the object.
    pub cost:   N
}

impl<'a, N, T> RefWithCost<'a, N, T> {
    /// Creates a new reference packed with a cost value.
    #[inline]
    pub fn new(object: &'a T, cost: N) -> RefWithCost<'a, N, T> {
        RefWithCost {
            object: object,
            cost:   cost
        }
    }
}

impl<'a, N: PartialEq, T> PartialEq for RefWithCost<'a, N, T> {
    #[inline]
    fn eq(&self, other: &RefWithCost<'a, N, T>) -> bool {
        self.cost.eq(&other.cost)
    }
}

impl<'a, N: PartialEq, T> Eq for RefWithCost<'a, N, T> {
}

impl<'a, N: PartialOrd, T> PartialOrd for RefWithCost<'a, N, T> {
    #[inline]
    fn partial_cmp(&self, other: &RefWithCost<'a, N, T>) -> Option<Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl<'a, N: PartialOrd, T> Ord for RefWithCost<'a, N, T> {
    #[inline]
    fn cmp(&self, other: &RefWithCost<'a, N, T>) -> Ordering {
        if self.cost < other.cost {
            Less
        }
        else if self.cost > other.cost {
            Greater
        }
        else {
            Equal
        }
    }
}
