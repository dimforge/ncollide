//! An hash map with a customizable hash function.

use std::mem;
use std::iter;
use data::hash::HashFun;

/// Entry of an `HashMap`.
#[derive(Clone, RustcEncodable, RustcDecodable)]
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
/// It is different from `std::hash_map::HashMap` because:
///
/// * the hash function can be personalized
/// * the hash table is separate from the data. Thus, the vector of entries is tight (no holes
///     due to sparse hashing).
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub struct HashMap<K, V, H> {
    hash:          H,
    table:         Vec<Entry<K, V>>,
    mask:          usize,
    htable:        Vec<isize>,
    next:          Vec<isize>,
    num_elem:      usize, // FIXME: redundent with self.table.len() ?
    max_elem:      usize,
    real_max_elem: usize
}

static HASH_CHARGE_FACTOR: f32 = 0.7;

impl<K, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Creates a new hash map.
    pub fn new(h: H) -> HashMap<K, V, H> {
        HashMap::new_with_capacity(31, h)
    }

    /// Creates a new hash map with a given capacity.
    pub fn new_with_capacity(capacity: usize, h: H) -> HashMap<K, V, H> {
        let pow2 = capacity.next_power_of_two();

        HashMap {
            hash:   h,
            table:  Vec::with_capacity(pow2),
            mask:   pow2 - 1,
            htable: iter::repeat(-1isize).take(pow2).collect(),
            next:   iter::repeat(-1isize).take(pow2).collect(),
            num_elem: 0,
            max_elem: pow2,
            real_max_elem: ((pow2 as f32) * 0.7) as usize
        }
    }

    /// The elements added to this hash map.
    ///
    /// This is a simple, contiguous array.
    #[inline]
    pub fn elements(&self) -> &[Entry<K, V>] {
        &self.table[..]
    }

    /// The elements added to this hash map.
    ///
    /// This is a simple, contiguous array.
    #[inline]
    pub fn elements_mut(&mut self) -> &mut [Entry<K, V>] {
        &mut self.table[..]
    }

    /// The number of elements contained by this hashmap.
    #[inline]
    pub fn len(&self) -> usize {
        self.num_elem
    }

    /// Whether or not this hashmap is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.num_elem == 0
    }

    /// Removes everything from this hashmap.
    #[inline]
    pub fn clear(&mut self) {
        self.table.clear();
        self.num_elem = 0;

        for i in self.htable.iter_mut() {
            *i = -1
        }

        for i in self.next.iter_mut() {
            *i = -1
        }
    }
}


impl<K: PartialEq + Clone, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Removes the element at the specified position of the element array.
    ///
    /// If the index is greater than the table length, it returns `false`.
    pub fn remove_elem_at(&mut self, at: usize) -> bool {
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
    fn find_entry_id(&self, key: &K) -> isize {
        let h = self.hash.hash(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos as usize].key != *key {
            while self.next[pos as usize] != -1 &&
                  self.table[self.next[pos as usize] as usize].key != *key {
                pos = self.next[pos as usize]
            }

            pos = self.next[pos as usize]
        }

        pos
    }

    fn may_grow(&mut self) {
        if self.num_elem >= self.real_max_elem {
            self.max_elem = self.max_elem * 2;

            self.real_max_elem = ((self.max_elem as f32)* HASH_CHARGE_FACTOR) as usize;

            self.mask = self.max_elem - 1;

            let mut newhash: Vec<isize> = iter::repeat(-1isize).take(self.max_elem).collect();
            let mut newnext: Vec<isize> = iter::repeat(-1isize).take(self.max_elem).collect();

            for i in 0usize .. self.num_elem {
                let h = self.hash.hash(&self.table[i].key) & self.mask;

                newnext[i] = newhash[h];
                newhash[h] = i as isize;
            }

            mem::swap(&mut newhash, &mut self.htable);
            mem::swap(&mut newnext, &mut self.next);
        }
    }

    fn do_insert_or_replace(&mut self, key: K, value: V, replace: bool) -> (bool, usize) {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            self.may_grow();

            let h = self.hash.hash(&key) & self.mask;

            self.next[self.num_elem] = self.htable[h];
            self.htable[h] = self.num_elem as isize;
            self.table.push(Entry::new(key, value));
            self.num_elem = self.num_elem + 1;

            (true, self.num_elem - 1)
        }
        else {
            if replace {
                self.table[entry as usize].value = value
            }

            (false, entry as usize)
        }
    }

    /// Removes an element and returns its value if it existed.
    pub fn get_and_remove(&mut self, key: &K) -> Option<Entry<K, V>> {
        let h = self.hash.hash(key) & self.mask;

        let mut o = self.htable[h];

        if o != -1 {
            let obji;

            if self.table[o as usize].key != *key {
                while self.next[o as usize] != -1 && self.table[self.next[o as usize] as usize].key != *key {
                    o = self.next[o as usize]
                }

                if self.next[o as usize] == -1 {
                    return None
                }

                obji                             = self.next[o as usize];
                self.next[o as usize]    = self.next[obji as usize];
                self.next[obji as usize] = -1;
            }
            else {
                obji = o;
                self.htable[h]       = self.next[o as usize];
                self.next[o as usize] = -1;
            }

            self.num_elem = self.num_elem - 1;

            let removed = self.table.swap_remove(obji as usize);

            if obji != self.num_elem as isize {
                let nh = self.hash.hash(&self.table[obji as usize].key) & self.mask;

                if self.htable[nh] == self.num_elem as isize {
                    self.htable[nh] = obji
                }
                else {
                    let mut no = self.htable[nh];

                    while self.next[no as usize] != self.num_elem as isize {
                        no = self.next[no as usize]
                    }

                    self.next[no as usize] = obji;
                }

                self.next[obji as usize]  = self.next[self.num_elem];
                self.next[self.num_elem] = -1;
            }

            Some(removed)
        }
        else {
            None
        }
    }

    /// Same as `self.insert_or_replace(key, value, false)` but with `value` a function which is
    /// called iff. the value does not exist yet. If the functions returns `None`, nothing is
    /// inserted.
    pub fn find_or_insert_lazy<'a, F>(&'a mut self, key: K, mut value: F) -> Option<&'a mut V>
          where F: FnMut() -> Option<V> {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            match value() {
                Some(v) => {
                    self.may_grow();

                    let h = self.hash.hash(&key) & self.mask;

                    self.next[self.num_elem] = self.htable[h];
                    self.htable[h] = self.num_elem as isize;
                    self.table.push(Entry::new(key, v));
                    self.num_elem = self.num_elem + 1;

                    Some(&mut self.table[self.num_elem - 1].value)
                }
                None => None

            }
        }
        else {
            Some(&mut self.table[entry as usize].value)
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

        &mut self.table[res].value
    }

    /// Checks whether this hashmap contains a specific key.
    pub fn contains_key(&self, key: &K) -> bool {
        self.find(key).is_some()
    }

    /// Finds a reference to the element with a given key.
    pub fn find<'a>(&'a self, key: &K) -> Option<&'a V> {
        let h = self.hash.hash(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos as usize].key != *key {
            while self.next[pos as usize] != -1 &&
                  self.table[self.next[pos as usize] as usize].key != *key {
                pos = self.next[pos as usize]
            }

            pos = self.next[pos as usize]
        }

        if pos == -1 {
            None
        }
        else {
            Some(&self.table[pos as usize].value)
        }
    }

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
    //     panic!("Not yet implemented.")
    // }

    // pub fn pop(&mut self, _: &K) -> Option<V> {
    //     panic!("Not yet implemented.")
    // }

    /// Gets a mutable reference to an element of the hashmap.
    pub fn find_mut<'a>(&'a mut self, key: &K) -> Option<&'a mut V> {
        let entry = self.find_entry_id(key);

        if entry == -1 {
            None
        }
        else {
            Some(&mut self.table[entry as usize].value)
        }
    }
}

#[cfg(test)]
mod test {
    use super::HashMap;
    use data::hash::UintTWHash;

    // NOTE: some tests are simply copy-pasted from std::hash_map tests.
    #[test]
    fn test_find() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.find(&1).is_none());
        m.insert(1, 2);
        match m.find(&1) {
            None    => panic!(),
            Some(v) => assert!(*v == 2)
        }
    }

    #[test]
    fn test_is_empty() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(!m.is_empty());
        assert!(m.remove(&1));
        assert!(m.is_empty());
    }

    #[test]
    fn test_insert() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(2, 4));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert_eq!(*m.find(&2).unwrap(), 4);
    }

    #[test]
    fn test_find_mut() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 12));
        assert!(m.insert(2, 8));
        assert!(m.insert(5, 14));
        let new = 100;
        match m.find_mut(&5) {
            None => panic!(), Some(x) => *x = new
        }
        assert_eq!(m.find(&5), Some(&new));
    }

    #[test]
    fn test_insert_overwrite() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert!(!m.insert(1, 3));
        assert_eq!(*m.find(&1).unwrap(), 3);
    }

    #[test]
    fn test_insert_conflicts() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
        assert_eq!(*m.find(&1).unwrap(), 2);
    }

    #[test]
    fn test_conflict_remove() {
        let mut m: HashMap<usize, usize, UintTWHash> = HashMap::new(UintTWHash::new());
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert!(m.remove(&1));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
    }
}
