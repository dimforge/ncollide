use std::uint;
use std::util;
use std::vec;
use util::hash::HashFun;

/// Entry of an `HashMap`.
pub struct Entry<K, V> {
    /// The key of the entry.
    key:   K,
    /// The value of the entry.
    value: V
}

impl<K, V> Entry<K, V> {
    fn new(key: K, value: V) -> Entry<K, V> {
        Entry {
            key:   key,
            value: value
        }
    }
}

/// Alternative implementation of HashMap. It is different from std::hashmap::HashMap because:
///     * the hash function can be personalized
///     * the hash table are speratate from the datas. Thus, the vector of entries is tight (no
///     holes due to sparce hashing).
pub struct HashMap<K, V, H> {
    priv table:         ~[Entry<K, V>],
    priv mask:          uint,
    priv htable:        ~[int],
    priv next:          ~[int],
    priv num_elem:      uint, // FIXME: redundent with self.table.len() ?
    priv max_elem:      uint,
    priv real_max_elem: uint
}

static HASH_CHARGE_FACTOR: float = 0.7;

impl<K, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Creates a new hash map.
    pub fn new(h: H) -> HashMap<K, V, H> {
        HashMap::new_with_capacity(31, h)
    }

    /// Creates a new hash map with a given capacity.
    pub fn new_with_capacity(capacity: uint, _: H) -> HashMap<K, V, H> {
        let pow2 = uint::next_power_of_two(capacity);

        HashMap {
            table:  vec::with_capacity(pow2),
            mask:   pow2 - 1,
            htable: vec::from_elem(pow2, -1),
            next:   vec::from_elem(pow2, -1),
            num_elem: 0,
            max_elem: pow2,
            real_max_elem: ((pow2 as float) * 0.7) as uint
        }
    }

    /// The elements added to this hash map. This is a simple, contiguous array.
    #[inline]
    pub fn elements<'r>(&'r self) -> &'r [Entry<K, V>] {
        let table: &'r [Entry<K, V>] = self.table;

        table
    }

    /// The elements added to this hash map. This is a simple, contiguous array.
    #[inline]
    pub fn elements_mut<'r>(&'r mut self) -> &'r mut [Entry<K, V>] {
        let table: &'r mut [Entry<K, V>] = self.table;

        table
    }
}


impl<K: Eq + Clone, V, H: HashFun<K>> HashMap<K, V, H> {
    /// Removes the element at the specified position of the element array.
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


impl<K: Eq, V, H: HashFun<K>> HashMap<K, V, H> {
    fn find_entry_id(&self, key: &K) -> int {
        let h = HashFun::hash::<K, H>(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos].key != *key {
            while self.next[pos] != -1 && self.table[self.next[pos]].key != *key {
                pos = self.next[pos]
            }

            pos = self.next[pos]
        }

        pos
    }

    fn may_grow(&mut self) {
        if self.num_elem >= self.real_max_elem {
            self.max_elem = self.max_elem * 2;

            self.real_max_elem = ((self.max_elem as float)* HASH_CHARGE_FACTOR) as uint;

            self.mask = self.max_elem - 1;

            let mut newhash  = vec::from_elem(self.max_elem, -1);
            let mut newnext  = vec::from_elem(self.max_elem, -1);

            for i in range(0u, self.num_elem) {
                let h = HashFun::hash::<K, H>(&self.table[i].key) & self.mask;

                newnext[i] = newhash[h];
                newhash[h] = i as int;
            }

            util::swap(&mut newhash, &mut self.htable);
            util::swap(&mut newnext, &mut self.next);
        }
    }

    fn do_insert_or_replace<'a>(&'a mut self, key: K, value: V, replace: bool) -> (bool, &'a mut V) {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            self.may_grow();

            let h = HashFun::hash::<K, H>(&key) & self.mask;

            self.next[self.num_elem] = self.htable[h];
            self.htable[h] = self.num_elem as int;
            self.table.push(Entry::new(key, value));
            self.num_elem = self.num_elem + 1;

            (true, &'a mut self.table[self.num_elem - 1].value)
        }
        else {
            if replace {
                self.table[entry].value = value
            }

            (false, &'a mut self.table[entry].value)
        }
    }

    /// Same as `self.insert_or_replace(key, value, false)` but with `value` a function which is
    /// called iff. the value does not exist yet.
    pub fn find_or_insert_lazy<'a>(&'a mut self, key: K, value: &fn() -> V) -> &'a mut V {
        let entry = self.find_entry_id(&key);

        if entry == -1 {
            self.may_grow();

            let h = HashFun::hash::<K, H>(&key) & self.mask;

            self.next[self.num_elem] = self.htable[h];
            self.htable[h] = self.num_elem as int;
            self.table.push(Entry::new(key, value()));
            self.num_elem = self.num_elem + 1;

            &'a mut self.table[self.num_elem - 1].value
        }
        else {
            &'a mut self.table[entry].value
        }
    }

    /// Inserts or replace an element.
    ///
    /// # Arguments.
    ///     * `key` - key of the element to add.
    ///     * `value` - value to add.
    ///     * `replace` - if true the new value will replace the existing value. If false, the old
    ///     value is keeped if it exists.
    pub fn insert_or_replace<'a>(&'a mut self, key: K, value: V, replace: bool) -> &'a mut V {
        let (_, res) = self.do_insert_or_replace(key, value, replace);

        res
    }
}

impl<K, V, H: HashFun<K>> Container for HashMap<K, V, H> {
    #[inline]
    fn len(&self) -> uint {
        self.num_elem
    }
}

impl<K, V, H: HashFun<K>> Mutable for HashMap<K, V, H> {
    fn clear(&mut self) {
        self.table.clear();
        self.num_elem = 0;
        // FIXME ??? self.htable.clear()
        // FIXME ??? self.next.clear()
        fail!("Not yet implemented.")
    }
}


impl<K: Eq, V, H: HashFun<K>> Map<K, V> for HashMap<K, V, H> {
    fn contains_key(&self, key: &K) -> bool {
        self.find(key).is_some()
    }

    fn find<'a>(&'a self, key: &K) -> Option<&'a V> {
        let h = HashFun::hash::<K, H>(key) & self.mask;

        let mut pos = self.htable[h];

        if pos != -1 && self.table[pos].key != *key {
            while self.next[pos] != -1 && self.table[self.next[pos]].key != *key {
                pos = self.next[pos]
            }

            pos = self.next[pos]
        }

        if pos == -1 {
            None
        }
        else {
            Some(&'a self.table[pos].value)
        }
    }
}

impl<K: Eq, V, H: HashFun<K>> MutableMap<K, V> for HashMap<K, V, H> {
    fn insert(&mut self, key: K, value: V) -> bool {
        let (res, _) = self.do_insert_or_replace(key, value, true);

        res
    }

    fn remove(&mut self, key: &K) -> bool {
        let h = HashFun::hash::<K, H>(key) & self.mask;

        let mut obji;
        let mut o    = self.htable[h];

        if o != -1 {
            if self.table[o].key != *key {
                while self.next[o] != -1 && self.table[self.next[o]].key != *key {
                    o = self.next[o]
                }

                if self.next[o] == -1 {
                    return false
                }

                obji            = self.next[o];
                self.next[o]    = self.next[obji];
                self.next[obji] = -1;
            }
            else {
                obji = o;
                self.htable[h] = self.next[o];
                self.next[o]   = -1;
            }

            self.num_elem = self.num_elem - 1;
            let p = self.table.pop();

            if obji != self.num_elem as int {
                let nh = HashFun::hash::<K, H>(&p.key) & self.mask;

                self.table[obji] = p;


                if self.htable[nh] == self.num_elem as int {
                    self.htable[nh] = obji
                }
                else {
                    let mut no = self.htable[nh];

                    while self.next[no]  != self.num_elem as int {
                        no = self.next[no]
                    }

                    self.next[no] = obji;
                }

                self.next[obji] = self.next[self.num_elem];
                self.next[self.num_elem] = -1;
            }

            true
        }
        else {
            false
        }
    }

    fn swap(&mut self, _: K, _: V) -> Option<V> {
        fail!("Not yet implemented.")
    }

    fn pop(&mut self, _: &K) -> Option<V> {
        fail!("Not yet implemented.")
    }

    fn find_mut<'a>(&'a mut self, key: &K) -> Option<&'a mut V> {
        let entry = self.find_entry_id(key);

        if entry == -1 {
            None
        }
        else {
            Some(&'a mut self.table[entry].value)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::hashmap;
    use extra::test::BenchHarness;
    use util::hash::{UintTWHash, UintPairTWHash};
    use extra::time;

    // NOTE: some tests are simply copy-pasted from std::hashmap tests.
    #[test]
    fn test_find() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.find(&1).is_none());
        m.insert(1, 2);
        match m.find(&1) {
            None => fail!(),
            Some(v) => assert!(*v == 2)
        }
    }

    #[test]
    fn test_is_empty() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.insert(1, 2));
        assert!(!m.is_empty());
        assert!(m.remove(&1));
        assert!(m.is_empty());
    }

    #[test]
    fn test_insert() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.insert(1, 2));
        assert!(m.insert(2, 4));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert_eq!(*m.find(&2).unwrap(), 4);
    }

    #[test]
    fn test_find_mut() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
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
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.insert(1, 2));
        assert_eq!(*m.find(&1).unwrap(), 2);
        assert!(!m.insert(1, 3));
        assert_eq!(*m.find(&1).unwrap(), 3);
    }

    #[test]
    fn test_insert_conflicts() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
        assert_eq!(*m.find(&1).unwrap(), 2);
    }

    #[test]
    fn test_conflict_remove() {
        let mut m: HashMap<uint, uint, UintTWHash> = HashMap::new(UintTWHash);
        assert!(m.insert(1, 2));
        assert!(m.insert(5, 3));
        assert!(m.insert(9, 4));
        assert!(m.remove(&1));
        assert_eq!(*m.find(&9).unwrap(), 4);
        assert_eq!(*m.find(&5).unwrap(), 3);
    }

    #[bench]
    fn bench_insert_this(bh: &mut BenchHarness) {
        let mut m: HashMap<(uint, uint), uint, UintPairTWHash> = HashMap::new(UintPairTWHash);

        do bh.iter {
            for i in range(0u, 500) {
                m.insert((i, i), i);
            }
        }
    }

    #[bench]
    fn bench_insert_std(bh: &mut BenchHarness) {
        let mut m = hashmap::HashMap::with_capacity(32);

        do bh.iter {
            for i in range(0u, 500) {
                m.insert((i, i), i);
            }
        }
    }

    #[bench]
    fn bench_insert_find_remove_this(bh: &mut BenchHarness) {
        let mut m: HashMap<(uint, uint), uint, UintPairTWHash> = HashMap::new(UintPairTWHash);

        do bh.iter {
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
        }
    }

    #[bench]
    fn bench_insert_find_remove_std(bh: &mut BenchHarness) {
        let mut m = hashmap::HashMap::with_capacity(32);

        do bh.iter {
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
        }
    }

    #[test]
    fn abench_insert_find_this() {
        let before = time::precise_time_ns();

        let mut m: HashMap<(uint, uint), uint, UintPairTWHash> = HashMap::new_with_capacity(20000, UintPairTWHash);

        for i in range(0u, 20000) {
            m.insert((i, i), i);
        }

        for i in range(0u, 20000) {
            assert!(*m.find(&(i, i)).unwrap() == i)
        }

        println(((time::precise_time_ns() - before) as f64 / 1000000.0).to_str());

    }
}
