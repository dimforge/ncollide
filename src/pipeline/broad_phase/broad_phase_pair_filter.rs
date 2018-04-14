use std::any::Any;
use na::Real;

use pipeline::world::CollisionObject;
use math::{Point, Isometry};

/// A signal handler for contact detection.
pub trait BroadPhasePairFilter<N: Real, T>: Any + Send + Sync {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, b1: &CollisionObject<N, T>, b2: &CollisionObject<N, T>) -> bool;
}

/// Filters deciding whether a proximity is to be further investigated by the narrow phase or not.
///
/// All filters have have to return `true` in order to allow a proximity to be further handled.
pub struct BroadPhasePairFilters<N: Real, T> {
    filters: Vec<(String, Box<BroadPhasePairFilter<N, T>>)>,
}

impl<N: Real, T> BroadPhasePairFilters<N, T> {
    /// Creates a new set of collision filters.
    pub fn new() -> BroadPhasePairFilters<N, T> {
        BroadPhasePairFilters {
            filters: Vec::new(),
        }
    }

    /// Registers a collision filter.
    pub fn register_collision_filter(
        &mut self,
        name: &str,
        callback: Box<BroadPhasePairFilter<N, T>>,
    ) {
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
        } else {
            false
        }
    }

    /// Tells if the collision between `b1` and `b2` is to be handled by the narrow-phase.
    pub fn is_pair_valid(
        &self,
        b1: &CollisionObject<N, T>,
        b2: &CollisionObject<N, T>,
    ) -> bool {
        self.filters
            .iter()
            .all(|&(_, ref f)| f.is_pair_valid(b1, b2))
    }
}
