/// A signal handler for contact detection.
pub trait BroadPhasePairFilter<B> {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, b1: &B, b2: &B) -> bool;
}

impl<B> BroadPhasePairFilter<B> for fn(&B, &B) -> bool {
    #[inline]
    fn is_pair_valid(&self, b1: &B, b2: &B) -> bool {
        self(b1, b2)
    }
}

impl<B> BroadPhasePairFilter<B> for Fn(&B, &B) -> bool {
    #[inline]
    fn is_pair_valid(&self, b1: &B, b2: &B) -> bool {
        self(b1, b2)
    }
}

/// Filters deciding whether a proximity is to be further investigated by the narrow phase or not.
///
/// All filters have have to return `true` in order to allow a proximity to be further handled.
pub struct BroadPhasePairFilters<B> {
    filters: Vec<(String, Box<BroadPhasePairFilter<B> + 'static>)>,
}

impl<B> BroadPhasePairFilters<B> {
    /// Creates a new set of collision filters.
    pub fn new() -> BroadPhasePairFilters<B> {
        BroadPhasePairFilters {
            filters: Vec::new(),
        }
    }

    /// Registers a collision filter.
    pub fn register_collision_filter(&mut self, name: &str, callback: Box<BroadPhasePairFilter<B> + 'static>) {
        for &mut (ref mut n, ref mut f) in self.filters.iter_mut() {
            if name == &n[..] {
                *f = callback;
                return;
            }
        }

        self.filters.push((name.to_string(), callback))
    }

    /// Unregisters a collision filter.
    ///
    /// Returns `true` if the filter was found.
    pub fn unregister_collision_filter(&mut self, name: &str) -> bool {
        let mut to_remove = self.filters.len();

        for (i, &mut (ref n, _)) in self.filters.iter_mut().enumerate() {
            if name == &n[..] {
                to_remove = i;
            }
        }

        if to_remove != self.filters.len() {
            let _ = self.filters.remove(to_remove);
            true
        }
        else {
            false
        }
    }

    /// Tells if the collision between `b1` and `b2` is to be handled by the narrow-phase.
    pub fn is_pair_valid(&self, b1: &B, b2: &B) -> bool {
        self.filters.iter().all(|&(_, ref f)| f.is_pair_valid(b1, b2))
    }
}
