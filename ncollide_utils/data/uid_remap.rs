//! A map allowing a slow lookup for arbitrary `usize` and fast lookup for small ones.

use std::iter::{FromIterator, IntoIterator};
use std::ops::{Index, IndexMut};
use std::default::Default;
use std::collections::hash_map::Entry;
use std::collections::{HashMap};
use num::Bounded;
use data::hash::{HashFun, UintTWHash};
use data::vec_map::{VecMap, Iter, IterMut, Values, Keys};

/// A special type of key used by `UidRemap` to perform faster lookups than with the user-defined
/// id of type `usize`.
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, RustcEncodable, RustcDecodable)]
pub struct FastKey {
    uid: usize
}

impl FastKey {
    /// Creates a new invalid key that won't be used by the `UidRemap` structure, ever.
    #[inline]
    pub fn new_invalid() -> FastKey {
        FastKey {
            uid: Bounded::max_value()
        }
    }

    /// The small uid contained by this key.
    #[inline]
    pub fn uid(&self) -> usize {
        self.uid
    }
}

#[derive(Debug, Clone, RustcEncodable, RustcDecodable)]
struct LookupData {
    uid2key:   HashMap<usize, FastKey>,
    free_keys: Vec<FastKey>,
}

/// A set of values having large usize key.
#[derive(Debug, Clone)]
pub struct UidRemap<O> { // FIXME: find a better name.
    values: VecMap<O>,
    lookup: Option<LookupData>
}

impl<O> Default for UidRemap<O> {
    #[inline]
    fn default() -> UidRemap<O> {
        UidRemap::new(false)
    }
}

impl<O> UidRemap<O> {
    /// Creates an empty `UidRemap`.
    pub fn new(small_uids: bool) -> UidRemap<O> {
        if small_uids {
            UidRemap {
                values: VecMap::new(),
                lookup: None
            }
        }
        else {
            UidRemap {
                values: VecMap::new(),
                lookup: Some(LookupData { uid2key: HashMap::new(), free_keys: Vec::new() })
            }
        }
    }

    /// Gets the fast key associated to the given key.
    #[inline]
    pub fn get_fast_key(&self, uid: usize) -> Option<FastKey> {
        match self.lookup {
            None => Some(FastKey { uid: uid }),
            Some(ref data) => data.uid2key.get(&uid).cloned()
        }
    }

    /// Return the number of elements in the map.
    #[inline]
    pub fn len(&self) -> usize {
        match self.lookup {
            None => self.values.len(),
            Some(ref data) => data.uid2key.len()
        }
    }

    /// Return true if the map contains no elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        match self.lookup {
            None => self.values.is_empty(),
            Some(ref data) => data.uid2key.is_empty()
        }
    }

    /// Clears the map, removing all key-value pairs.
    #[inline]
    pub fn clear(&mut self) {
        self.values.clear();

        if let Some(ref mut data) = self.lookup {
            data.uid2key.clear();
            data.free_keys.clear();
        }
    }

    /// Returns a reference to the value corresponding to the key.
    #[inline]
    pub fn get(&self, key: usize) -> Option<&O> {
        match self.get_fast_key(key) {
            Some(fast_key) => self.get_fast(&fast_key),
            None => None
        }
    }

    /// Returns a reference to the value corresponding to the fast key.
    #[inline]
    pub fn get_fast(&self, key: &FastKey) -> Option<&O> {
        self.values.get(&key.uid())
    }

    /// Returns true if the map contains a value for the specified key.
    #[inline]
    pub fn contains_key(&self, key: usize) -> bool {
        self.get(key).is_some()
    }

    /// Returns true if the map contains a value for the specified fast key.
    #[inline]
    pub fn contains_fast_key(&self, key: &FastKey) -> bool {
        self.get_fast(key).is_some()
    }

    /// Returns a mutable reference to the value corresponding to the key.
    #[inline]
    pub fn get_mut(&mut self, key: usize) -> Option<&mut O> {
        match self.get_fast_key(key) {
            Some(fast_key) => self.get_fast_mut(&fast_key),
            None => None
        }
    }

    /// Returns a mutable reference to the value corresponding to the fast key.
    #[inline]
    pub fn get_fast_mut<'a>(&'a mut self, key: &FastKey) -> Option<&'a mut O> {
        // We have to help the compiler deduce the lifetime here.
        self.values.get_mut(&key.uid())
    }

    /// Inserts a key-value pair to the map. If the key already had a value
    /// present in the map, that value and its fast key are returned. Otherwise, `None` and a new
    /// fast key is returned.
    #[inline]
    pub fn insert(&mut self, uid: usize, value: O) -> (FastKey, Option<O>) {
        self.insert_or_replace(uid, value, true)
    }

    /// Inserts a key-value pair to the map and possibly replace it if it already exists.
    ///
    /// If the element already exists, then the old value is replaced by `value` if `replace` is
    /// set to `true`. If `replace` is `false` then any old value is left unchanged.
    ///
    /// # Return
    /// Returns None if the entry was not found.  If the entry is found return Some(value) if
    /// replace is false, or the old value if replace is true.
    #[inline]
    pub fn insert_or_replace(&mut self, uid: usize, value: O, replace: bool) -> (FastKey, Option<O>) {
        match self.lookup {
            None => {
                let fast_key = FastKey { uid: uid };
                (fast_key, self.values.insert(uid, value))
            },
            Some(ref mut data) => {
                // We have `uid2key == values.len()`, so we don't use `values.len()` because it is
                // not a constant-time operation.
                let len = data.uid2key.len();

                match data.uid2key.entry(uid) {
                    Entry::Occupied(entry) => {
                        let fast_key = *entry.get();
                        if replace {
                            (fast_key, self.values.insert(fast_key.uid, value))
                        }
                        else {
                            (fast_key, Some(value))
                        }
                    },
                    Entry::Vacant(entry) => {
                        // The key does not exist yet.
                        let fast_key = match data.free_keys.pop() {
                            None      => FastKey { uid: len },
                            Some(key) => key
                        };

                        let _ = entry.insert(fast_key);

                        let old_value = self.values.insert(fast_key.uid(), value);
                        assert!(old_value.is_none());

                        (fast_key, None)
                    }
                }
            },
        }
    }

    /// Removes a key from the map, returning the value at the key if the key exists.
    #[inline]
    pub fn remove(&mut self, uid: usize) -> Option<(FastKey, O)> {
        match self.lookup {
            None => self.values.remove(&uid).map(|o| (FastKey { uid: uid }, o)),
            Some(ref mut data) => {
                match data.uid2key.remove(&uid) {
                    None => None,
                    Some(fast_key) => {
                        let res = self.values.remove(&fast_key.uid());
                        data.free_keys.push(fast_key);

                        res.map(|o| (fast_key, o))
                    }
                }
            }
        }
    }

    /// Returns an iterator visiting all keys.
    #[inline]
    pub fn keys<'a>(&'a self) -> FastKeys<'a, O> {
        FastKeys { raw_keys: self.values.keys() }
    }

    /// Returns an iterator visiting all values.
    /// The iterator's element type is `&'r O`.
    #[inline]
    pub fn values<'a>(&'a self) -> Values<'a, O> {
        self.values.values()
    }

    /// Returns an iterator visiting all key-value pairs.
    #[inline]
    pub fn iter<'a>(&'a self) -> FastKeysAndValues<'a, O> {
        FastKeysAndValues { iter: self.values.iter() }
    }

    /// Returns an iterator visiting all key-value pairs with mutable references to the values.
    #[inline]
    pub fn iter_mut<'a>(&'a mut self) -> FastKeysAndValuesMut<'a, O> {
        FastKeysAndValuesMut { iter_mut: self.values.iter_mut() }
    }
}

/// An iterator through a `UidRemap` fast keys in use.
pub struct FastKeys<'a, O: 'a> {
    raw_keys: Keys<'a, O>
}

impl<'a, O> Iterator for FastKeys<'a, O> {
    type Item = FastKey;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.raw_keys.next() {
            Some(key) => {
                Some(FastKey { uid: key })
            }
            None => None
        }
    }
}

/// An iterator through a `UidRemap` fast keys and values.
pub struct FastKeysAndValues<'a, O: 'a> {
    iter: Iter<'a, O>
}

impl<'a, O> Iterator for FastKeysAndValues<'a, O> {
    type Item = (FastKey, &'a O);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.iter.next() {
            Some(key_val) => {
                Some((FastKey { uid: key_val.0 }, key_val.1))
            }
            None => None
        }
    }
}

/// An iterator through a `UidRemap` fast keys and values.
pub struct FastKeysAndValuesMut<'a, O: 'a> {
    iter_mut: IterMut<'a, O>
}

impl<'a, O> Iterator for FastKeysAndValuesMut<'a, O> {
    type Item = (FastKey, &'a mut O);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.iter_mut.next() {
            Some(key_val) => {
                Some((FastKey { uid: key_val.0 }, key_val.1))
            }
            None => None
        }
    }
}

impl<O: Clone> UidRemap<O> {
    /// Updates a value in the map. If the key already exists in the map,
    /// modifies the value with `ff` taking `oldval, newval`.
    /// Otherwise, sets the value to `newval`.
    /// Returns `true` if the key did not already exist in the map.
    #[inline]
    pub fn update<F: Fn(O, O) -> O>(&mut self, key: usize, newval: O, ff: F) -> bool {
        self.update_with_key(key, newval, |_k, v, v1| ff(v,v1))
    }

    /// Updates a value in the map. If the key already exists in the map,
    /// modifies the value with `ff` taking `key, oldval, newval`.
    /// Otherwise, sets the value to `newval`.
    /// Returns `true` if the key did not already exist in the map.
    #[inline]
    pub fn update_with_key<F: Fn(&usize, O, O) -> O>(&mut self, key: usize, val: O, ff: F) -> bool {
        let new_val = match self.get(key) {
            None => val,
            Some(origin) => ff(&key, (*origin).clone(), val)
        };
        self.insert(key, new_val).1.is_none()
    }
}

impl<O> FromIterator<(usize, O)> for UidRemap<O> {
    #[inline]
    fn from_iter<I: IntoIterator<Item = (usize, O)>>(iter: I) -> UidRemap<O> {
        let mut map = UidRemap::new(false);
        map.extend(iter.into_iter());
        map
    }
}

impl<O> Extend<(usize, O)> for UidRemap<O> {
    #[inline]
    fn extend<Iter: IntoIterator<Item = (usize, O)>>(&mut self, iter: Iter) {
        for (k, v) in iter.into_iter() {
            let _ = self.insert(k, v);
        }
    }
}

impl<O> Index<FastKey> for UidRemap<O> {
    type Output = O;

    #[inline]
    fn index(&self, key: FastKey) -> &O {
        self.get_fast(&key).expect("key not present")
    }
}

impl<O> IndexMut<FastKey> for UidRemap<O> {
    #[inline]
    fn index_mut(&mut self, key: FastKey) -> &mut O {
        self.get_fast_mut(&key).expect("key not present")
    }
}

impl<O> Index<usize> for UidRemap<O> {
    type Output = O;

    #[inline]
    fn index(&self, key: usize) -> &O {
        self.get(key).expect("key not present")
    }
}

impl<O> IndexMut<usize> for UidRemap<O> {
    #[inline]
    fn index_mut(&mut self, key: usize) -> &mut O {
        self.get_mut(key).expect("key not present")
    }
}

/// A fast hasher for FastKey objects.
pub struct FastKeyTWHash;

impl HashFun<FastKey> for FastKeyTWHash {
    #[inline]
    fn hash(&self, key: &FastKey) -> usize {
        let hasher = UintTWHash;
        hasher.hash(&key.uid)
    }
}
