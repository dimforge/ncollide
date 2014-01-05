use util::hash_map::HashMap;
use util::pair::{Pair, PairTWHash};
use util::has_uid::HasUid;
use broad::Dispatcher;


/**
 * The broad phase which does nothing.
 *
 * It always returns false positives since it assumes that all object is in collision with all
 * objects. Do not use this but for benchmarking the narrow phase.
 */
pub struct BruteForceBroadPhase<B, D, DV> {
    priv dispatcher: D,
    priv pairs:      HashMap<Pair<B>, DV, PairTWHash>,
    priv objects:    ~[B]
}

impl<B: HasUid + Clone, D: Dispatcher<B, B, DV>, DV> BruteForceBroadPhase<B, D, DV> {
    /// Builds a new brute force broad phase.
    pub fn new(dispatcher: D) -> BruteForceBroadPhase<B, D, DV> {
        BruteForceBroadPhase {
            dispatcher: dispatcher,
            pairs:      HashMap::new(PairTWHash::new()),
            objects:    ~[],
        }
    }

    /// The pair manager of this broad phase.
    pub fn pairs<'r>(&'r self) -> &'r HashMap<Pair<B>, DV, PairTWHash> {
        &'r self.pairs
    }

    /// Adds an element to this broad phase.
    pub fn add(&mut self, b: B) {
        for o in self.objects.iter() {
            if self.dispatcher.is_valid(o, &b) {
                self.pairs.insert(Pair::new(o.clone(), b.clone()), self.dispatcher.dispatch(o, &b));
            }
        }

        self.objects.push(b)
    }

    /// Removes an element from this broad phase.
    pub fn remove(&mut self, _: &B) {
        fail!("Not yet implemented.");
    }
}

#[cfg(test)]
mod test {
    use super::BruteForceBroadPhase;
    use broad::NoIdDispatcher;
    use util::pair::Pair;

    #[test]
    fn test_bf() {
        let dispatcher: NoIdDispatcher<int> = NoIdDispatcher;
        let mut bf = BruteForceBroadPhase::new(dispatcher);

        let a = 10;
        let b = 20;
        let c = 30;

        bf.add(a);
        bf.add(b);
        bf.add(c);

        let pairs = bf.pairs();

        assert!(pairs.elements().len() == 3);
        assert!(pairs.contains_key(&Pair::new(a, b)));
        assert!(pairs.contains_key(&Pair::new(b, c)));
        assert!(pairs.contains_key(&Pair::new(a, c)));

        assert!(pairs.contains_key(&Pair::new(b, a)));
        assert!(pairs.contains_key(&Pair::new(c, b)));
        assert!(pairs.contains_key(&Pair::new(c, a)));

        assert!(!pairs.contains_key(&Pair::new(a, a)));
        assert!(!pairs.contains_key(&Pair::new(b, b)));
        assert!(!pairs.contains_key(&Pair::new(c, c)));
    }
}
