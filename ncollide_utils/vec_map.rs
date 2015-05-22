// Copyright 2012-2014 The Rust Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution and at
// http://rust-lang.org/COPYRIGHT.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

//! A simple map based on a vector for small integer keys. Space requirements
//! are O(highest integer key).

use std::fmt;
use std::hash::{Hash, Hasher};
use std::iter::{Enumerate, FilterMap, Map};
use std::mem::{replace};
use std::ops::{Index, IndexMut, RangeFull, Range, RangeTo, RangeFrom};

use std::{vec, slice};
use std::vec::Vec;

use std::option::Option::{self, None, Some};

/// A map optimized for small integer keys.
pub struct VecMap<V> {
    v: Vec<Option<V>>,
}

impl<V> Default for VecMap<V> {
    #[inline]
    fn default() -> VecMap<V> { VecMap::new() }
}


impl<V:Clone> Clone for VecMap<V> {
    #[inline]
    fn clone(&self) -> VecMap<V> {
        VecMap { v: self.v.clone() }
    }

    #[inline]
    fn clone_from(&mut self, source: &VecMap<V>) {
        self.v.clone_from(&source.v);
    }
}


impl<V: Hash> Hash for VecMap<V> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // In order to not traverse the `VecMap` twice, count the elements
        // during iteration.
        let mut count: usize = 0;
        for elt in self {
            elt.hash(state);
            count += 1;
        }
        count.hash(state);
    }
}

impl<V> VecMap<V> {
    /// Creates an empty `VecMap`.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut map: VecMap<&str> = VecMap::new();
    /// ```
    
    pub fn new() -> VecMap<V> { VecMap { v: vec![] } }

    /// Creates an empty `VecMap` with space for at least `capacity`
    /// elements before resizing.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    /// let mut map: VecMap<&str> = VecMap::with_capacity(10);
    /// ```
    
    pub fn with_capacity(capacity: usize) -> VecMap<V> {
        VecMap { v: Vec::with_capacity(capacity) }
    }

    /// Returns the number of elements the `VecMap` can hold without
    /// reallocating.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    /// let map: VecMap<String> = VecMap::with_capacity(10);
    /// assert!(map.capacity() >= 10);
    /// ```
    #[inline]
    
    pub fn capacity(&self) -> usize {
        self.v.capacity()
    }

    /// Reserves capacity for the given `VecMap` to contain `len` distinct keys.
    /// In the case of `VecMap` this means reallocations will not occur as long
    /// as all inserted keys are less than `len`.
    ///
    /// The collection may reserve more space to avoid frequent reallocations.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    /// let mut map: VecMap<&str> = VecMap::new();
    /// map.reserve_len(10);
    /// assert!(map.capacity() >= 10);
    /// ```
    
    pub fn reserve_len(&mut self, len: usize) {
        let cur_len = self.v.len();
        if len >= cur_len {
            self.v.reserve(len - cur_len);
        }
    }

    /// Reserves the minimum capacity for the given `VecMap` to contain `len` distinct keys.
    /// In the case of `VecMap` this means reallocations will not occur as long as all inserted
    /// keys are less than `len`.
    ///
    /// Note that the allocator may give the collection more space than it requests.
    /// Therefore capacity cannot be relied upon to be precisely minimal.  Prefer
    /// `reserve_len` if future insertions are expected.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    /// let mut map: VecMap<&str> = VecMap::new();
    /// map.reserve_len_exact(10);
    /// assert!(map.capacity() >= 10);
    /// ```
    
    pub fn reserve_len_exact(&mut self, len: usize) {
        let cur_len = self.v.len();
        if len >= cur_len {
            self.v.reserve_exact(len - cur_len);
        }
    }

    /// Returns an iterator visiting all keys in ascending order of the keys.
    /// The iterator's element type is `usize`.
    
    pub fn keys<'r>(&'r self) -> Keys<'r, V> {
        fn first<A, B>((a, _): (A, B)) -> A { a }
        let first: fn((usize, &'r V)) -> usize = first; // coerce to fn pointer

        Keys { iter: self.iter().map(first) }
    }

    /// Returns an iterator visiting all values in ascending order of the keys.
    /// The iterator's element type is `&'r V`.
    
    pub fn values<'r>(&'r self) -> Values<'r, V> {
        fn second<A, B>((_, b): (A, B)) -> B { b }
        let second: fn((usize, &'r V)) -> &'r V = second; // coerce to fn pointer

        Values { iter: self.iter().map(second) }
    }

    /// Returns an iterator visiting all key-value pairs in ascending order of the keys.
    /// The iterator's element type is `(usize, &'r V)`.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// map.insert(3, "c");
    /// map.insert(2, "b");
    ///
    /// // Print `1: a` then `2: b` then `3: c`
    /// for (key, value) in map.iter() {
    ///     println!("{}: {}", key, value);
    /// }
    /// ```
    
    pub fn iter<'r>(&'r self) -> Iter<'r, V> {
        Iter {
            front: 0,
            back: self.v.len(),
            iter: self.v.iter()
        }
    }

    /// Returns an iterator visiting all key-value pairs in ascending order of the keys,
    /// with mutable references to the values.
    /// The iterator's element type is `(usize, &'r mut V)`.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// map.insert(2, "b");
    /// map.insert(3, "c");
    ///
    /// for (key, value) in map.iter_mut() {
    ///     *value = "x";
    /// }
    ///
    /// for (key, value) in map.iter() {
    ///     assert_eq!(value, &"x");
    /// }
    /// ```
    
    pub fn iter_mut<'r>(&'r mut self) -> IterMut<'r, V> {
        IterMut {
            front: 0,
            back: self.v.len(),
            iter: self.v.iter_mut()
        }
    }

    /// Returns the number of elements in the map.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut a = VecMap::new();
    /// assert_eq!(a.len(), 0);
    /// a.insert(1, "a");
    /// assert_eq!(a.len(), 1);
    /// ```
    
    pub fn len(&self) -> usize {
        self.v.iter().filter(|elt| elt.is_some()).count()
    }

    /// Returns true if the map contains no elements.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut a = VecMap::new();
    /// assert!(a.is_empty());
    /// a.insert(1, "a");
    /// assert!(!a.is_empty());
    /// ```
    
    pub fn is_empty(&self) -> bool {
        self.v.iter().all(|elt| elt.is_none())
    }

    /// Clears the map, removing all key-value pairs.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut a = VecMap::new();
    /// a.insert(1, "a");
    /// a.clear();
    /// assert!(a.is_empty());
    /// ```
    
    pub fn clear(&mut self) { self.v.clear() }

    /// Returns a reference to the value corresponding to the key.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// assert_eq!(map.get(&1), Some(&"a"));
    /// assert_eq!(map.get(&2), None);
    /// ```
    
    pub fn get(&self, key: &usize) -> Option<&V> {
        if *key < self.v.len() {
            match self.v[*key] {
              Some(ref value) => Some(value),
              None => None
            }
        } else {
            None
        }
    }

    /// Returns true if the map contains a value for the specified key.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// assert_eq!(map.contains_key(&1), true);
    /// assert_eq!(map.contains_key(&2), false);
    /// ```
    #[inline]
    
    pub fn contains_key(&self, key: &usize) -> bool {
        self.get(key).is_some()
    }

    /// Returns a mutable reference to the value corresponding to the key.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// if let Some(x) = map.get_mut(&1) {
    ///     *x = "b";
    /// }
    /// assert_eq!(map[1], "b");
    /// ```
    
    pub fn get_mut(&mut self, key: &usize) -> Option<&mut V> {
        if *key < self.v.len() {
            match *(&mut self.v[*key]) {
              Some(ref mut value) => Some(value),
              None => None
            }
        } else {
            None
        }
    }

    /// Inserts a key-value pair into the map. If the key already had a value
    /// present in the map, that value is returned. Otherwise, `None` is returned.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// assert_eq!(map.insert(37, "a"), None);
    /// assert_eq!(map.is_empty(), false);
    ///
    /// map.insert(37, "b");
    /// assert_eq!(map.insert(37, "c"), Some("b"));
    /// assert_eq!(map[37], "c");
    /// ```
    
    pub fn insert(&mut self, key: usize, value: V) -> Option<V> {
        let len = self.v.len();
        if len <= key {
            self.v.extend((0..key - len + 1).map(|_| None));
        }
        replace(&mut self.v[key], Some(value))
    }

    /// Removes a key from the map, returning the value at the key if the key
    /// was previously in the map.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// assert_eq!(map.remove(&1), Some("a"));
    /// assert_eq!(map.remove(&1), None);
    /// ```
    
    pub fn remove(&mut self, key: &usize) -> Option<V> {
        if *key >= self.v.len() {
            return None;
        }
        let result = &mut self.v[*key];
        result.take()
    }
}

impl<V: fmt::Debug> fmt::Debug for VecMap<V> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "{{"));

        for (i, (k, v)) in self.iter().enumerate() {
            if i != 0 { try!(write!(f, ", ")); }
            try!(write!(f, "{}: {:?}", k, *v));
        }

        write!(f, "}}")
    }
}

impl<T> IntoIterator for VecMap<T> {
    type Item = (usize, T);
    type IntoIter = IntoIter<T>;

    /// Returns an iterator visiting all key-value pairs in ascending order of
    /// the keys, consuming the original `VecMap`.
    /// The iterator's element type is `(usize, &'r V)`.
    ///
    /// # Examples
    ///
    /// ```
    /// # #![feature(collections)]
    /// use std::collections::VecMap;
    ///
    /// let mut map = VecMap::new();
    /// map.insert(1, "a");
    /// map.insert(3, "c");
    /// map.insert(2, "b");
    ///
    /// let vec: Vec<(usize, &str)> = map.into_iter().collect();
    ///
    /// assert_eq!(vec, [(1, "a"), (2, "b"), (3, "c")]);
    /// ```
    fn into_iter(self) -> IntoIter<T> {
        fn filter<A>((i, v): (usize, Option<A>)) -> Option<(usize, A)> {
            v.map(|v| (i, v))
        }
        let filter: fn((usize, Option<T>)) -> Option<(usize, T)> = filter; // coerce to fn ptr

        IntoIter { iter: self.v.into_iter().enumerate().filter_map(filter) }
    }
}


impl<'a, T> IntoIterator for &'a VecMap<T> {
    type Item = (usize, &'a T);
    type IntoIter = Iter<'a, T>;

    fn into_iter(self) -> Iter<'a, T> {
        self.iter()
    }
}


impl<'a, T> IntoIterator for &'a mut VecMap<T> {
    type Item = (usize, &'a mut T);
    type IntoIter = IterMut<'a, T>;

    fn into_iter(mut self) -> IterMut<'a, T> {
        self.iter_mut()
    }
}

impl<V> Index<usize> for VecMap<V> {
    type Output = V;

    #[inline]
    fn index<'a>(&'a self, i: usize) -> &'a V {
        self.get(&i).expect("key not present")
    }
}

impl<'a,V> Index<&'a usize> for VecMap<V> {
    type Output = V;

    #[inline]
    fn index(&self, i: &usize) -> &V {
        self.get(i).expect("key not present")
    }
}


impl<V> IndexMut<usize> for VecMap<V> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut V {
        self.get_mut(&i).expect("key not present")
    }
}


impl<'a, V> IndexMut<&'a usize> for VecMap<V> {
    #[inline]
    fn index_mut(&mut self, i: &usize) -> &mut V {
        self.get_mut(i).expect("key not present")
    }
}

macro_rules! iterator {
    (impl $name:ident -> $elem:ty, $($getter:ident),+) => {
        
        impl<'a, V> Iterator for $name<'a, V> {
            type Item = $elem;

            #[inline]
            fn next(&mut self) -> Option<$elem> {
                while self.front < self.back {
                    match self.iter.next() {
                        Some(elem) => {
                            match elem$(. $getter ())+ {
                                Some(x) => {
                                    let index = self.front;
                                    self.front += 1;
                                    return Some((index, x));
                                },
                                None => {},
                            }
                        }
                        _ => ()
                    }
                    self.front += 1;
                }
                None
            }

            #[inline]
            fn size_hint(&self) -> (usize, Option<usize>) {
                (0, Some(self.back - self.front))
            }
        }
    }
}

macro_rules! double_ended_iterator {
    (impl $name:ident -> $elem:ty, $($getter:ident),+) => {
        
        impl<'a, V> DoubleEndedIterator for $name<'a, V> {
            #[inline]
            fn next_back(&mut self) -> Option<$elem> {
                while self.front < self.back {
                    match self.iter.next_back() {
                        Some(elem) => {
                            match elem$(. $getter ())+ {
                                Some(x) => {
                                    self.back -= 1;
                                    return Some((self.back, x));
                                },
                                None => {},
                            }
                        }
                        _ => ()
                    }
                    self.back -= 1;
                }
                None
            }
        }
    }
}

/// An iterator over the key-value pairs of a map.

pub struct Iter<'a, V:'a> {
    front: usize,
    back: usize,
    iter: slice::Iter<'a, Option<V>>
}

// FIXME(#19839) Remove in favor of `#[derive(Clone)]`
impl<'a, V> Clone for Iter<'a, V> {
    fn clone(&self) -> Iter<'a, V> {
        Iter {
            front: self.front,
            back: self.back,
            iter: self.iter.clone()
        }
    }
}

iterator! { impl Iter -> (usize, &'a V), as_ref }
double_ended_iterator! { impl Iter -> (usize, &'a V), as_ref }

/// An iterator over the key-value pairs of a map, with the
/// values being mutable.

pub struct IterMut<'a, V:'a> {
    front: usize,
    back: usize,
    iter: slice::IterMut<'a, Option<V>>
}

iterator! { impl IterMut -> (usize, &'a mut V), as_mut }
double_ended_iterator! { impl IterMut -> (usize, &'a mut V), as_mut }

/// An iterator over the keys of a map.

pub struct Keys<'a, V: 'a> {
    iter: Map<Iter<'a, V>, fn((usize, &'a V)) -> usize>
}

// FIXME(#19839) Remove in favor of `#[derive(Clone)]`
impl<'a, V> Clone for Keys<'a, V> {
    fn clone(&self) -> Keys<'a, V> {
        Keys {
            iter: self.iter.clone()
        }
    }
}

/// An iterator over the values of a map.

pub struct Values<'a, V: 'a> {
    iter: Map<Iter<'a, V>, fn((usize, &'a V)) -> &'a V>
}

// FIXME(#19839) Remove in favor of `#[derive(Clone)]`
impl<'a, V> Clone for Values<'a, V> {
    fn clone(&self) -> Values<'a, V> {
        Values {
            iter: self.iter.clone()
        }
    }
}

/// A consuming iterator over the key-value pairs of a map.

pub struct IntoIter<V> {
    iter: FilterMap<
    Enumerate<vec::IntoIter<Option<V>>>,
    fn((usize, Option<V>)) -> Option<(usize, V)>>
}

impl<'a, V> Iterator for Keys<'a, V> {
    type Item = usize;

    fn next(&mut self) -> Option<usize> { self.iter.next() }
    fn size_hint(&self) -> (usize, Option<usize>) { self.iter.size_hint() }
}

impl<'a, V> DoubleEndedIterator for Keys<'a, V> {
    fn next_back(&mut self) -> Option<usize> { self.iter.next_back() }
}


impl<'a, V> Iterator for Values<'a, V> {
    type Item = &'a V;

    fn next(&mut self) -> Option<(&'a V)> { self.iter.next() }
    fn size_hint(&self) -> (usize, Option<usize>) { self.iter.size_hint() }
}

impl<'a, V> DoubleEndedIterator for Values<'a, V> {
    fn next_back(&mut self) -> Option<(&'a V)> { self.iter.next_back() }
}


impl<V> Iterator for IntoIter<V> {
    type Item = (usize, V);

    fn next(&mut self) -> Option<(usize, V)> { self.iter.next() }
    fn size_hint(&self) -> (usize, Option<usize>) { self.iter.size_hint() }
}

impl<V> DoubleEndedIterator for IntoIter<V> {
    fn next_back(&mut self) -> Option<(usize, V)> { self.iter.next_back() }
}

/// **RangeArgument** is implemented by Rust's built-in range types, produced
/// by range syntax like `..`, `a..`, `..b` or `c..d`.
pub trait RangeArgument<T> {
    /// Start index (inclusive)
    ///
    /// Return start value if present, else `None`.
    fn start(&self) -> Option<&T> { None }

    /// End index (exclusive)
    ///
    /// Return end value if present, else `None`.
    fn end(&self) -> Option<&T> { None }
}


impl<T> RangeArgument<T> for RangeFull {}

impl<T> RangeArgument<T> for RangeFrom<T> {
    fn start(&self) -> Option<&T> { Some(&self.start) }
}

impl<T> RangeArgument<T> for RangeTo<T> {
    fn end(&self) -> Option<&T> { Some(&self.end) }
}

impl<T> RangeArgument<T> for Range<T> {
    fn start(&self) -> Option<&T> { Some(&self.start) }
    fn end(&self) -> Option<&T> { Some(&self.end) }
}