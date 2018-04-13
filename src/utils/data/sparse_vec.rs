use std::ops::{Index, IndexMut};
use std::mem;

/// A sparse vector data structure.
pub struct SparseVec<T> {
    data: Vec<Option<T>>,
    free: Vec<usize>,
}

impl<T> Index<usize> for SparseVec<T> {
    type Output = T;

    #[inline]
    fn index(&self, i: usize) -> &T {
        self.data[i].as_ref().expect("Element not found.")
    }
}

impl<T> IndexMut<usize> for SparseVec<T> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut T {
        self.data[i].as_mut().expect("Element not found.")
    }
}

impl<T> SparseVec<T> {
    /// Creates a new sparse vector data structure.
    pub fn new() -> SparseVec<T> {
        SparseVec {
            data: Vec::new(),
            free: Vec::new(),
        }
    }

    /// Adds an element to this vector and returns its index.
    #[inline]
    pub fn push(&mut self, data: T) -> usize {
        if let Some(i) = self.free.pop() {
            self.data[i] = Some(data);
            i
        } else {
            self.data.push(Some(data));
            self.data.len() - 1
        }
    }

    /// Attempts to remove an element from this vector and returns it.
    #[inline]
    pub fn remove(&mut self, id: usize) -> Option<T> {
        if id < self.data.len() {
            mem::replace(&mut self.data[id], None)
        } else {
            None
        }
    }

    /// Removes all elements from this vector.
    #[inline]
    pub fn clear(&mut self) {
        self.data.clear();
        self.free.clear();
    }

    /// Indicates whether this sparse vector is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.len() - self.free.len() == 0
    }
}
