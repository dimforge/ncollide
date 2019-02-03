use std::cmp::PartialOrd;
use std::mem;
use std::ops::Deref;

/// A pair of elements sorted in increasing order.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SortedPair<T: PartialOrd>([T; 2]);

impl<T: PartialOrd> SortedPair<T> {
    /// Sorts two elements in increasing order into a new pair.
    pub fn new(element1: T, element2: T) -> Self {
        if element1 > element2 {
            SortedPair([element2, element1])
        } else {
            SortedPair([element1, element2])
        }
    }
}

impl<T: PartialOrd> Deref for SortedPair<T> {
    type Target = (T, T);

    fn deref(&self) -> &(T, T) {
        unsafe { mem::transmute(self) }
    }
}
