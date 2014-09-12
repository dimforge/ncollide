//! An hash map with a customizable hash function.

use std::num;
use std::mem;
use utils::data::hash::HashFun;

/// Entry of an `HashMap`.
#[deriving(Clone, Encodable, Decodable)]
pub struct Entry<K, V> {
    /// The key of the entry.
    pub key:   K,
    /// The value of the entry.
    pub value: V
}

impl<K, V> Entry<K, V> {
    fn new(key: K, value: V) -> Entry<K, V> {
        Entry {
            key:   key,
            value: value
        }
    }
}

/// Alternative implementation of `HashMap`.
///
/// It is different from `std::hashmap::HashMap` because:
///
/// * the hash function can be personalized
/// * the hash table is separate from the data. Thus, the vector of entries is tight (no holes
///     due to sparse hashing).
#[deriving(Clone, Encodable, Decodable)]
pub struct HashMap<K, V, H> {
    hash:          H,
    table:         Vec<Entry<K, V>>,
    mask:          uint,
    htable:        Vec<int>,
    next:          Vec<int>,
    num_elem:      uint, // FIXME: redundent with self.table.len() ?
    max_elem:      uint,
    real_max_elem: uint
}

static HASH_CHARGE_FACTOR: f32 = 0.7;

impl<K, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Creates a new hash map.
    pub fn new(h: H) -> HashMap<K, V, H> {
        HashMap::new_with_capacity(31, h)
    }

    /// Creates a new hash map with a given capacity.
    pub fn new_with_capacity(capacity: uint, h: H) -> HashMap<K, V, H> {
        let pow2 = num::next_power_of_two(capacity);

        HashMap {
            hash:   h,
            table:  Vec::with_capacity(pow2),
            mask:   pow2 - 1,
            htable: Vec::from_elem(pow2, -1i),
            next:   Vec::from_elem(pow2, -1i),
            num_elem: 0,
            max_elem: pow2,
            real_max_elem: ((pow2 as f32) * 0.7) as uint
        }
    }

    /// The elements added to this hash map.
    ///
    /// This is a simple, contiguous array.
    #[inline]
    pub fn elements<'r>(&'r self) -> &'r [Entry<K, V>] {
        self.table.as_slice()
    }

    /// The elements added to this hash map.
    ///
    /// This is a simple, contiguous array.
    #[inline]
    pub fn elements_mut<'r>(&'r mut self) -> &'r mut [Entry<K, V>] {
        self.table.as_mut_slice()
    }
}


impl<K: PartialEq + Clone, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Removes the element at the specified position of the element array.
    ///
    /// If the index is greater than the table length, it returns `false`.
    pub fn remove_elem_at(&mut self, at: uint) -> bool {
        if at > self.table.len() {
            false
        }
        else {
            let key = self.table[at].key.clone();
            self.remove(&key)
        }
    }
}


impl<K: PartialEq, V, H: HashFun<K>> HashMap<K, V, H> {
    fn find_entry_id(&self, key: &K) -> int {
        let h = self.hash.hash(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos as uint].key != *key {
            while self.next[pos as uint] != -1 &&
                  self.table[self.next[pos as uint] as uint].key != *key {
                pos = self.next[pos as uint]
            }

            pos = self.next[pos as uint]
        }

        pos
    }

    fn may_grow(&mut self) {
        if self.num_elem >= self.real_max_elem {
            self.max_elem = self.max_elem * 2;

            self.real_max_elem = ((self.max_elem as f32)* HASH_CHARGE_FACTOR) as uint;

            self.mask = self.max_elem - 1;

            let mut newhash  = Vec::from_elem(self.max_elem, -1i);
            let mut newnext  = Vec::from_elem(self.max_elem, -1i);

            for i in range(0u, self.num_elem) {
                let h = self.hash.hash(&self.table[i].key) & self.mask;

                *newnext.get_mut(i) = newhash[h];
                *newhash.get_mut(h) = i as int;
            }

            mem::swap(&mut newhash, &mut self.htable);
            mem::swap(&mut newnext, &mut self.next);
        }
    }

    fn do_insert_or_replace(&mut self, key: K, value: V, replace: bool) -> (bool, uint) {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            self.may_grow();

            let h = self.hash.hash(&key) & self.mask;

            *self.next.get_mut(self.num_elem) = self.htable[h];
            *self.htable.get_mut(h) = self.num_elem as int;
            self.table.push(Entry::new(key, value));
            self.num_elem = self.num_elem + 1;

            (true, self.num_elem - 1)
        }
        else {
            if replace {
                self.table.get_mut(entry as uint).value = value
            }

            (false, entry as uint)
        }
    }

    /// Removes an element and returns its value if it existed.
    pub fn get_and_remove(&mut self, key: &K) -> Option<Entry<K, V>> {
        let h = self.hash.hash(key) & self.mask;

        let mut obji;
        let mut o = self.htable[h];

        if o != -1 {
            if self.table[o as uint].key != *key {
                while self.next[o as uint] != -1 && self.table[self.next[o as uint] as uint].key != *key {
                    o = self.next[o as uint]
                }

                if self.next[o as uint] == -1 {
                    return None
                }

                obji                             = self.next[o as uint];
                *self.next.get_mut(o as uint)    = self.next[obji as uint];
                *self.next.get_mut(obji as uint) = -1;
            }
            else {
                obji = o;
                *self.htable.get_mut(h)       = self.next[o as uint];
                *self.next.get_mut(o as uint) = -1;
            }

            self.num_elem = self.num_elem - 1;

            let removed = self.table.swap_remove(obji as uint);

            if obji != self.num_elem as int {
                let nh = self.hash.hash(&self.table[obji as uint].key) & self.mask;

                if self.htable[nh] == self.num_elem as int {
                    *self.htable.get_mut(nh) = obji
                }
                else {
                    let mut no = self.htable[nh];

                    while self.next[no as uint] != self.num_elem as int {
                        no = self.next[no as uint]
                    }

                    *self.next.get_mut(no as uint) = obji;
                }

                *self.next.get_mut(obji as uint)  = self.next[self.num_elem];
                *self.next.get_mut(self.num_elem) = -1;
            }

            removed
        }
        else {
            None
        }
    }

    /// Same as `self.insert_or_replace(key, value, false)` but with `value` a function which is
    /// called iff. the value does not exist yet. If the functions returns `None`, nothing is
    /// inserted.
    pub fn find_or_insert_lazy<'a>(&'a mut self, key: K, value: || -> Option<V>) -> Option<&'a mut V> {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            match value() {
                Some(v) => {
                    self.may_grow();

                    let h = self.hash.hash(&key) & self.mask;

                    *self.next.get_mut(self.num_elem) = self.htable[h];
                    *self.htable.get_mut(h) = self.num_elem as int;
                    self.table.push(Entry::new(key, v));
                    self.num_elem = self.num_elem + 1;

                    Some(&mut self.table.get_mut(self.num_elem - 1).value)
                }
                None => None

            }
        }
        else {
            Some(&mut self.table.get_mut(entry as uint).value)
        }
    }

    /// Inserts or replace an element.
    ///
    /// # Arguments.
    /// * `key` - key of the element to add.
    /// * `value` - value to add.
    /// * `replace` - if true the new value will replace the existing value. If false, the old
    ///               value is kept if it exists.
    pub fn insert_or_replace<'a>(&'a mut self, key: K, value: V, replace: bool) -> &'a mut V {
        let (_, res) = self.do_insert_or_replace(key, value, replace);

        &mut self.table.get_mut(res).value
    }
}

impl<K, V, H: HashFun<K>> Collection for HashMap<K, V, H> {
    #[inline]
    fn len(&self) -> uint {
        self.num_elem
    }
}

impl<K, V, H: HashFun<K>> Mutable for HashMap<K, V, H> {
    fn clear(&mut self) {
        self.table.clear();
        self.num_elem = 0;

        for i in self.htable.mut_iter() {
            *i = -1
        }

        for i in self.next.mut_iter() {
            *i = -1
        }
    }
}


impl<K: PartialEq, V, H: HashFun<K>> Map<K, V> for HashMap<K, V, H> {
    fn contains_key(&self, key: &K) -> bool {
        self.find(key).is_some()
    }

    fn find<'a>(&'a self, key: &K) -> Option<&'a V> {
        let h = self.hash.hash(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos as uint].key != *key {
            while self.next[pos as uint] != -1 &&
                  self.table[self.next[pos as uint] as uint].key != *key {
                pos = self.next[pos as uint]
            }

            pos = self.next[pos as uint]
        }

        if pos == -1 {
            None
        }
        else {
            Some(&self.table[pos as uint].value)
        }
    }
}

impl<K: PartialEq, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Inserts an element on the hash map.
    pub fn insert(&mut self, key: K, value: V) -> bool {
        let (res, _) = self.do_insert_or_replace(key, value, true);

        res
    }

    /// Remove an element from the hash map.
    pub fn remove(&mut self, key: &K) -> bool {
        self.get_and_remove(key).is_some()
    }

    // pub fn swap(&mut self, _: K, _: V) -> Option<V> {
    //     fail!("Not yet implemented.")
    // }

    // pub fn pop(&mut self, _: &K) -> Option<V> {
    //     fail!("Not yet implemented.")
    // }

    /// Gets a mutable reference to an element of the hashmap.
    pub fn find_mut<'a>(&'a mut self, key: &K) -> Option<&'a mut V> {
        let entry = self.find_entry_id(key);

        if entry == -1 {
            None
        }
        else {
            Some(&mut self.table.get_mut(entry as uint).value)
        }
    }
}

#[cfg(test)]
mod test {
    use super::HashMap;
    use std::collections::hashmap;
    use test::Bencher;
    use utils::data::hash::{UintTWHash, UintPairTWHash};

    // NOTE: some tests are simply copy-pasted from std::hashmap tests.
    #[test]
    fn test_find() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.find(&1).is_none());
        m.insert(1, 2);
        match m.find(&1) {
            None    => fail!(),
            Some(v) => assert!(*v == 2)
        }
    }

    #[test]
    fn test_is_empty() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(!m.is_empty());
        assert!(m.remove(&1));
        assert!(m.is_empty());
    }

    #[test]
    fn test_insert() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(2, 4));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert_eq!(*m.find(&2).unwrap(), 4);
    }

    #[test]
    fn test_find_mut() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 12));
        assert!(m.insert(2, 8));
        assert!(m.insert(5, 14));
        let new = 100;
        match m.find_mut(&5) {
            None => fail!(), Some(x) => *x = new
        }
        assert_eq!(m.find(&5), Some(&new));
    }

    #[test]
    fn test_insert_overwrite() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert!(!m.insert(1, 3));
        assert_eq!(*m.find(&1).unwrap(), 3);
    }

    #[test]
    fn test_insert_conflicts() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
        assert_eq!(*m.find(&1).unwrap(), 2);
    }

    #[test]
    fn test_conflict_remove() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert!(m.remove(&1));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
    }

    #[bench]
    fn bench_insert_this(bh: &mut Bencher) {
        let mut m: HashMap<(uint, uint), uint, UintPairTWHash> = HashMap::new(UintPairTWHash::new());

        bh.iter(|| {
            for i in range(0u, 500) {
                m.insert((i, i), i);
            }
        })
    }

    #[bench]
    fn bench_insert_std(bh: &mut Bencher) {
        let mut m = hashmap::HashMap::with_capacity(32);

        bh.iter(|| {
            for i in range(0u, 500) {
                m.insert((i, i), i);
            }
        })
    }

    #[bench]
    fn bench_insert_find_remove_this(bh: &mut Bencher) {
        let mut m: HashMap<(uint, uint), uint, UintPairTWHash> = HashMap::new(UintPairTWHash::new());

        bh.iter(|| {
            for i in range(0u, 200) {
                m.insert((i, i), i);
            }

            for i in range(0u, 200) {
                assert!(*m.find(&(i, i)).unwrap() == i)
            }

            for i in range(100u, 200) {
                m.remove(&(i, i));
            }

            for i in range(100u, 200) {
                assert!(m.find(&(i, i)).is_none())
            }

            for i in range(0u, 100) {
                m.insert((i, i), i * 2);
            }

            for i in range(0u, 100) {
                assert!(*m.find(&(i, i)).unwrap() == i * 2)
            }

            for i in range(0u, 100) {
                m.remove(&(i, i));
            }

            for i in range(0u, 100) {
                assert!(m.find(&(i, i)).is_none())
            }
        })
    }

    #[bench]
    fn bench_insert_find_remove_std(bh: &mut Bencher) {
        let mut m = hashmap::HashMap::with_capacity(32);

        bh.iter(|| {
            for i in range(0u, 200) {
                m.insert((i, i), i);
            }

            for i in range(0u, 200) {
                assert!(*m.find(&(i, i)).unwrap() == i)
            }

            for i in range(100u, 200) {
                m.remove(&(i, i));
            }

            for i in range(100u, 200) {
                assert!(m.find(&(i, i)).is_none())
            }

            for i in range(0u, 100) {
                m.insert((i, i), i * 2);
            }

            for i in range(0u, 100) {
                assert!(*m.find(&(i, i)).unwrap() == i * 2)
            }

            for i in range(0u, 100) {
                m.remove(&(i, i));
            }

            for i in range(0u, 100) {
                assert!(m.find(&(i, i)).is_none())
            }
        })
    }
}
