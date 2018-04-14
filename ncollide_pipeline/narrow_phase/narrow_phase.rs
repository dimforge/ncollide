use std::any::Any;
use std::collections::hash_map::Iter;

use utils::data::SortedPair;
use geometry::query::ContactManifold;
use narrow_phase::{ContactAlgorithm, ContactManifoldGenerator, ProximityAlgorithm, ProximityDetector};
use events::{ContactEvents, ProximityEvents};
use world::{CollisionObject, CollisionObjectHandle, CollisionObjectSlab};
use math::{Point, Isometry};

/// Trait implemented by the narrow phase manager.
///
/// The narrow phase manager is responsible for creating, updating and generating contact pairs
/// between objects identified by the broad phase.
pub trait NarrowPhase<N: Real, T>: Any + Send + Sync {
    /// Updates this narrow phase.
    fn update(
        &mut self,
        objects: &CollisionObjectSlab<P, M, T>,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        timestamp: usize,
    );

    /// Called when the broad phase detects that two objects are, or stop to be, in close proximity.
    fn handle_interaction(
        &mut self,
        contact_signal: &mut ContactEvents,
        proximity_signal: &mut ProximityEvents,
        objects: &CollisionObjectSlab<P, M, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
        started: bool,
    );

    /// Called when the interactions between two objects have to be removed because at least one of the objects is being removed.
    ///
    /// While either `objects[handle1]` or `objects[handle2]` is being removed, the `handle_removal` is assumed to be called before the removal from the list `objects` is done.
    fn handle_removal(
        &mut self,
        objects: &CollisionObjectSlab<P, M, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    );

    // FIXME: the fact that the return type is imposed is not as generic as it could be.
    /// Returns all the potential contact pairs found during the broad phase, and validated by the
    /// narrow phase.
    fn contact_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<P, M, T>,
    ) -> ContactPairs<'a, P, M, T>;

    /// Returns all the potential proximity pairs found during the broad phase, and validated by
    /// the narrow phase.
    fn proximity_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<P, M, T>,
    ) -> ProximityPairs<'a, P, M, T>;
}

/// Iterator through contact pairs.
pub struct ContactPairs<'a, N: Real + 'a, M: 'a + Isometry<P>, T: 'a> {
    objects: &'a CollisionObjectSlab<P, M, T>,
    pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ContactManifoldGenerator<P, M>>>,
}

impl<'a, P: 'a + Point, M: 'a + Isometry<P>, T: 'a> ContactPairs<'a, P, M, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(
        objects: &'a CollisionObjectSlab<P, M, T>,
        pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ContactManifoldGenerator<P, M>>>,
    ) -> ContactPairs<'a, P, M, T> {
        ContactPairs {
            objects: objects,
            pairs: pairs,
        }
    }

    /// Transforms contact-pairs iterator to an iterator through each individual contact manifold.
    #[inline]
    pub fn contact_manifolds(self) -> ContactManifolds<'a, P, M, T> {
        ContactManifolds {
            objects: self.objects,
            co1: None,
            co2: None,
            pairs: self.pairs,
            collector: Vec::new(), // FIXME: avoid allocations.
            curr_contact: 0,
        }
    }
}

impl<'a, N: Real, T> Iterator for ContactPairs<'a, P, M, T> {
    type Item = (
        &'a CollisionObject<P, M, T>,
        &'a CollisionObject<P, M, T>,
        &'a ContactAlgorithm<P, M>,
    );

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.pairs.next() {
            Some((key, value)) => {
                let co1 = &self.objects[key.0];
                let co2 = &self.objects[key.1];

                Some((&co1, &co2, value))
            }
            None => None,
        }
    }
}

/// An iterator through contact manifolds.
pub struct ContactManifolds<'a, P: 'a + Point, M: 'a + Isometry<P>, T: 'a> {
    objects: &'a CollisionObjectSlab<P, M, T>,
    co1: Option<&'a CollisionObject<P, M, T>>,
    co2: Option<&'a CollisionObject<P, M, T>>,
    pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ContactManifoldGenerator<P, M>>>,
    collector: Vec<&'a ContactManifold<P>>,
    curr_contact: usize,
}

impl<'a, N: Real, T> Iterator for ContactManifolds<'a, P, M, T> {
    type Item = (
        &'a CollisionObject<P, M, T>,
        &'a CollisionObject<P, M, T>,
        &'a ContactManifold<P>,
    );

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        // FIXME: is there a more efficient way to do this (i-e. avoid using an index)?
        if self.curr_contact < self.collector.len() {
            self.curr_contact = self.curr_contact + 1;

            // FIXME: would be nice to avoid the `clone` and return a reference
            // instead (but what would be its lifetime?).
            Some((
                self.co1.unwrap(),
                self.co2.unwrap(),
                self.collector[self.curr_contact - 1],
            ))
        } else {
            self.collector.clear();

            while let Some((key, value)) = self.pairs.next() {
                value.contacts(&mut self.collector);

                if !self.collector.is_empty() {
                    self.co1 = Some(&self.objects[key.0]);
                    self.co2 = Some(&self.objects[key.1]);
                    self.curr_contact = 1; // Start at 1 instead of 0 because we will return the first one here.

                    // FIXME: would be nice to avoid the `clone` and return a reference
                    // instead (but what would be its lifetime?).
                    return Some((self.co1.unwrap(), self.co2.unwrap(), self.collector[0]));
                }
            }

            None
        }
    }
}

/// Iterator through proximity pairs.
pub struct ProximityPairs<'a, N: Real + 'a, M: 'a + Isometry<P>, T: 'a> {
    objects: &'a CollisionObjectSlab<P, M, T>,
    pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ProximityDetector<P, M>>>,
}

impl<'a, P: 'a + Point, M: 'a + Isometry<P>, T: 'a> ProximityPairs<'a, P, M, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(
        objects: &'a CollisionObjectSlab<P, M, T>,
        pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ProximityDetector<P, M>>>,
    ) -> ProximityPairs<'a, P, M, T> {
        ProximityPairs {
            objects: objects,
            pairs: pairs,
        }
    }
}

impl<'a, N: Real, T> Iterator for ProximityPairs<'a, P, M, T> {
    type Item = (
        &'a CollisionObject<P, M, T>,
        &'a CollisionObject<P, M, T>,
        &'a ProximityAlgorithm<P, M>,
    );

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.pairs.next() {
            Some((key, value)) => {
                let co1 = &self.objects[key.0];
                let co2 = &self.objects[key.1];

                Some((&co1, &co2, value))
            }
            None => None,
        }
    }
}
