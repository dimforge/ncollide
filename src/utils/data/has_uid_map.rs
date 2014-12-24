//! A map using object implementing `HasUid` as keys.

use std::num::Int;
use std::iter;
use std::default::Default;
use std::intrinsics::TypeId;
use std::collections::hash_map::Entry;
use std::collections::vec_map;
use std::collections::{VecMap, HashMap};
use utils::data::has_uid::HasUid;

/// A special type of key used by `HasUidMap` to perform faster lookups than with the user-defined
/// key `K`.
#[deriving(Show, Clone, Hash, PartialEq, Eq, PartialOrd, Ord, Encodable, Decodable, Copy)]
pub struct FastKey {
    uid: uint
}

impl FastKey {
    /// Creates a new invalid key that won't be used by the `HasUidMap` structure, ever.
    #[inline]
    pub fn new_invalid() -> FastKey {
        FastKey {
            uid: Int::max_value()
        }
    }
}

impl HasUid for FastKey {
    #[inline]
    fn uid(&self) -> uint {
        self.uid
    }
}

#[deriving(Show, Clone, Encodable, Decodable)]
struct LookupData<K, O> {
    uid2key:   HashMap<uint, FastKey>,
    free_keys: Vec<FastKey>
}

/// A set of values having large uint key.
#[deriving(Show, Clone, Encodable, Decodable)]
pub struct HasUidMap<K, O> {
    values: VecMap<(K, O)>,
    lookup: Option<LookupData<K, O>>
}

impl<K: 'static + HasUid, O> Default for HasUidMap<K, O> {
    #[inline]
    fn default() -> HasUidMap<K, O> {
        HasUidMap::new()
    }
}

impl<K: 'static + HasUid, O> HasUidMap<K, O> {
    /// Creates an empty `HasUidMap`.
    pub fn new() -> HasUidMap<K, O> {
        if TypeId::of::<K>() == TypeId::of::<FastKey>() {
            HasUidMap {
                values: VecMap::new(),
                lookup: None
            }
        }
        else {
            HasUidMap {
                values: VecMap::new(),
                lookup: Some(LookupData { uid2key: HashMap::new(), free_keys: Vec::new() })
            }
        }
    }

    /// Gets the fast key associated to the given key.
    #[inline]
    pub fn get_fast_key(&self, key: &K) -> Option<FastKey> {
        self.get_fast_key_with_uid(key.uid())
    }

    /// Gets the fast key associated to the given key.
    #[inline]
    pub fn get_fast_key_with_uid(&self, uid: uint) -> Option<FastKey> {
        match self.lookup {
            None => Some(FastKey { uid: uid }),
            Some(ref data) => data.uid2key.get(&uid).cloned()
        }
    }

    /// Return the number of elements in the map.
    #[inline]
    pub fn len(&self) -> uint {
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
    pub fn get(&self, key: &K) -> Option<&O> {
        match self.get_fast_key(key) {
            Some(fast_key) => self.get_fast(&fast_key).map(|res| &res.1),
            None => None
        }
    }

    /// Returns a reference to the value corresponding to the fast key.
    #[inline]
    pub fn get_fast(&self, key: &FastKey) -> Option<&(K, O)> {
        self.values.get(&key.uid())
    }

    /// Returns true if the map contains a value for the specified key.
    #[inline]
    pub fn contains_key(&self, key: &K) -> bool {
        self.get(key).is_some()
    }

    /// Returns true if the map contains a value for the specified fast key.
    #[inline]
    pub fn contains_fast_key(&self, key: &FastKey) -> bool {
        self.get_fast(key).is_some()
    }

    /// Returns a mutable reference to the value corresponding to the key.
    #[inline]
    pub fn get_mut(&mut self, key: &K) -> Option<&mut O> {
        match self.get_fast_key(key) {
            Some(fast_key) => self.get_fast_mut(&fast_key).map(|res| res.1),
            None => None
        }
    }

    /// Returns a mutable reference to the value corresponding to the fast key.
    #[inline]
    pub fn get_fast_mut<'a>(&'a mut self, key: &FastKey) -> Option<(&'a K, &'a mut O)> {
        // We have to help the compiler deduce the lifetime here.
        let res: Option<&'a mut (K, O)> = self.values.get_mut(&key.uid());

        res.map(|&(ref k, ref mut o)| (k, o))
    }

    /// Inserts a key-value pair to the map. If the key already had a value
    /// present in the map, that value and its fast key are returned. Otherwise, `None` is
    /// returned.
    #[inline]
    pub fn insert(&mut self, key: K, value: O) -> (FastKey, Option<O>) {
        let uid = key.uid();

        match self.lookup {
            None => {
                let fast_key = FastKey { uid: uid };
                (fast_key, self.values.insert(uid, (key, value)).map(|val| val.1))
            },
            Some(ref mut data) => {
                // We have `uid2key == values.len()`, so we don't use `values.len()` because it is
                // not a constant-time operation.
                let len = data.uid2key.len();

                match data.uid2key.entry(uid) {
                    Entry::Occupied(entry) => {
                        let fast_key = entry.take();

                        let old_value = self.values.insert(fast_key.uid, (key, value));

                        // The key exists already in `uid2key` so `ord_value` should not be None.
                        (fast_key, Some(old_value.unwrap().1))
                    },
                    Entry::Vacant(entry) => {
                        // The key does not exist yet.
                        let fast_key = match data.free_keys.pop() {
                            None      => FastKey { uid: len },
                            Some(key) => key
                        };

                        let _ = entry.set(fast_key);

                        let old_value = self.values.insert(fast_key.uid(), (key, value));
                        assert!(old_value.is_none());

                        (fast_key, None)
                    }
                }
            },
        }
    }

    /// Removes a key from the map, returning the value at the key if the key exists.
    #[inline]
    pub fn remove(&mut self, key: &K) -> Option<O> {
        self.remove_with_uid(key.uid())
    }

    /// Removes a key from the map, returning the value at the key if the key exists.
    #[inline]
    pub fn remove_with_uid(&mut self, uid: uint) -> Option<O> {
        match self.lookup {
            None => self.values.remove(&uid).map(|val| val.1),
            Some(ref mut data) => {
                match data.uid2key.remove(&uid) {
                    None => None,
                    Some(fast_key) => {
                        let res = self.values.remove(&fast_key.uid());
                        data.free_keys.push(fast_key);

                        res.map(|(_, val)| val)
                    }
                }
            }
        }
    }

    /// Returns an iterator visiting all keys.
    /// The iterator's element type is `&'r K`.
    #[inline]
    pub fn keys<'a>(&'a self) -> Keys<'a, K, O> {
        fn fst<A, B>((a, _): (A, B)) -> A { a }
        // coerce to fn ptr
        let fst: fn((&'a K, &'a O)) -> &'a K = fst;

        self.iter().map(fst)
    }

    /// Returns an iterator visiting all values.
    /// The iterator's element type is `&'r O`.
    #[inline]
    pub fn values<'a>(&'a self) -> Values<'a, K, O> {
        fn snd<A, B>((_, b): (A, B)) -> B { b }
        // coerce to fn ptr
        let snd: fn((&'a K, &'a O)) -> &'a O = snd;

        self.iter().map(snd)
    }

    /// Returns an iterator visiting all key-value pairs.
    #[inline]
    pub fn iter<'a>(&'a self) -> Entries<'a, K, O> {
        fn snd_ref<'a, A, B, C>((_, &(ref b, ref c)): (A, &'a (B, C))) -> (&'a B, &'a C) { (b, c) }

        // Looks like we have to help the compiler deduce the lifetime here.
        let snd_ref_fn: fn((uint, &'a (K, O))) -> (&'a K, &'a O) = snd_ref;

        self.values.iter().map(snd_ref_fn)
    }

    /// Returns an iterator visiting all key-value pairs with mutable references to the values.
    #[inline]
    pub fn iter_mut<'a>(&'a mut self) -> MutEntries<'a, K, O> {
        fn snd_ref_mut<'a, A, B, C>((_, &(ref b, ref mut c)): (A, &'a mut (B, C))) -> (&'a B, &'a mut C) { (b, c) }

        // Looks like we have to help the compiler deduce the lifetime here.
        let snd_ref_mut_fn: fn((uint, &'a mut (K, O))) -> (&'a K, &'a mut O) = snd_ref_mut;

        self.values.iter_mut().map(snd_ref_mut_fn)
    }
}

/// Key iterator through a `HashUidMap`.
pub type Keys<'a, K, O> =
    iter::Map<(&'a K, &'a O), &'a K, Entries<'a, K, O>, fn((&'a K, &'a O)) -> &'a K>;

/// Values iterator through a `HashUidMap`.
pub type Values<'a, K, O> =
    iter::Map<(&'a K, &'a O), &'a O, Entries<'a, K, O>, fn((&'a K, &'a O)) -> &'a O>;

/// Entries iterator through a `HashUidMap`.
pub type Entries<'a, K, O> =
    iter::Map<(uint, &'a (K, O)), (&'a K, &'a O), vec_map::Iter<'a, (K, O)>,
              fn((uint, &'a (K, O))) -> (&'a K, &'a O)>;

/// Mutable entries iterator through a `HashUidMap`.
pub type MutEntries<'a, K, O> =
    iter::Map<(uint, &'a mut (K, O)), (&'a K, &'a mut O), vec_map::IterMut<'a, (K, O)>,
              fn((uint, &'a mut (K, O))) -> (&'a K, &'a mut O)>;

impl<K: 'static + HasUid, O: Clone> HasUidMap<K, O> {
    /// Updates a value in the map. If the key already exists in the map,
    /// modifies the value with `ff` taking `oldval, newval`.
    /// Otherwise, sets the value to `newval`.
    /// Returns `true` if the key did not already exist in the map.
    #[inline]
    pub fn update(&mut self, key: K, newval: O, ff: |O, O| -> O) -> bool {
        self.update_with_key(key, newval, |_k, v, v1| ff(v,v1))
    }

    /// Updates a value in the map. If the key already exists in the map,
    /// modifies the value with `ff` taking `key, oldval, newval`.
    /// Otherwise, sets the value to `newval`.
    /// Returns `true` if the key did not already exist in the map.
    #[inline]
    pub fn update_with_key(&mut self,
                           key: K,
                           val: O,
                           ff: |&K, O, O| -> O)
                           -> bool {
        let new_val = match self.get(&key) {
            None => val,
            Some(orig) => ff(&key, (*orig).clone(), val)
        };
        self.insert(key, new_val).1.is_none()
    }
}

impl<K: 'static + HasUid, O> FromIterator<(K, O)> for HasUidMap<K, O> {
    #[inline]
    fn from_iter<Iter: Iterator<(K, O)>>(iter: Iter) -> HasUidMap<K, O> {
        let mut map = HasUidMap::new();
        map.extend(iter);
        map
    }
}

impl<K: 'static + HasUid, O> Extend<(K, O)> for HasUidMap<K, O> {
    #[inline]
    fn extend<Iter: Iterator<(K, O)>>(&mut self, mut iter: Iter) {
        for (k, v) in iter {
            let _ = self.insert(k, v);
        }
    }
}

impl<K: 'static + HasUid, O> Index<FastKey, (K, O)> for HasUidMap<K, O> {
    #[inline]
    fn index(&self, key: &FastKey) -> &(K, O) {
        self.get_fast(key).expect("key not present")
    }
}
