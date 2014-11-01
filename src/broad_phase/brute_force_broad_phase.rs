use utils::data::hash_map::HashMap;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::has_uid::HasUid;
use broad_phase::Dispatcher;


/**
 * Broad phase with quadratic complexity.
 *
 * It always returns false positives since it assumes that all object is in collision with all
 * objects. Do not use this but for benchmarking the narrow phase.
 */
pub struct BruteForceBroadPhase<B, D, DV> {
    dispatcher: D,
    pairs:      HashMap<Pair<B>, DV, PairTWHash>,
    objects:    Vec<B>
}

impl<B: HasUid + Clone, D: Dispatcher<B, B, DV>, DV> BruteForceBroadPhase<B, D, DV> {
    /// Builds a new brute force broad phase.
    pub fn new(dispatcher: D) -> BruteForceBroadPhase<B, D, DV> {
        BruteForceBroadPhase {
            dispatcher: dispatcher,
            pairs:      HashMap::new(PairTWHash::new()),
            objects:    Vec::new(),
        }
    }

    /// The pair manager of this broad phase.
    #[inline]
    pub fn pairs<'r>(&'r self) -> &'r HashMap<Pair<B>, DV, PairTWHash> {
        &self.pairs
    }

    /// Adds an element to this broad phase.
    pub fn add(&mut self, b: B) {
        for o in self.objects.iter() {
            if self.dispatcher.is_valid(o, &b) {
                match self.dispatcher.dispatch(o, &b) {
                    Some(nf) => { let _ = self.pairs.insert(Pair::new(o.clone(), b.clone()), nf); },
                    None     => { }
                }
            }
        }

        self.objects.push(b)
    }

    /// Removes an element from this broad phase.
    pub fn remove(&mut self, _: &B) {
        panic!("Not yet implemented.");
    }
}
