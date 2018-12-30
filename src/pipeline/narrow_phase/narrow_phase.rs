use na::Real;
use crate::pipeline::events::{ContactEvents, ProximityEvents};
use crate::pipeline::narrow_phase::{
    ContactAlgorithm, ContactManifoldGenerator, ProximityAlgorithm, ProximityDetector,
};
use crate::pipeline::world::{CollisionObject, CollisionObjectHandle, CollisionObjectSlab};
use crate::query::ContactManifold;
use std::any::Any;
use std::collections::hash_map::Iter;
use crate::utils::SortedPair;

/// Trait implemented by the narrow phase manager.
///
/// The narrow phase manager is responsible for creating, updating and generating contact pairs
/// between objects identified by the broad phase.
pub trait NarrowPhase<N: Real, T>: Any + Send + Sync {
    /// Updates this narrow phase.
    fn update(
        &mut self,
        objects: &CollisionObjectSlab<N, T>,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        timestamp: usize,
    );

    /// Called when the broad phase detects that two objects are, or stop to be, in close proximity.
    fn handle_interaction(
        &mut self,
        contact_signal: &mut ContactEvents,
        proximity_signal: &mut ProximityEvents,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
        started: bool,
    );

    /// Called when the interactions between two objects have to be removed because at least one of the objects is being removed.
    ///
    /// While either `objects[handle1]` or `objects[handle2]` is being removed, the `handle_removal` is assumed to be called before the removal from the list `objects` is done.
    fn handle_removal(
        &mut self,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    );

    /// Retrieve the contact pair for the given two objects.
    ///
    /// Retruns `None` if no pairs is available for those two objects.
    fn contact_pair(
        &self,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) -> Option<(&ContactAlgorithm<N>, &ContactManifold<N>)>;

    // FIXME: the fact that the return type is imposed is not as generic as it could be.
    /// Returns all the potential contact pairs found during the broad phase, and validated by the
    /// narrow phase.
    fn contact_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<N, T>,
    ) -> ContactPairs<'a, N, T>;

    /// Returns all the potential proximity pairs found during the broad phase, and validated by
    /// the narrow phase.
    fn proximity_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<N, T>,
    ) -> ProximityPairs<'a, N, T>;
}

/// Iterator through contact pairs.
pub struct ContactPairs<'a, N: Real + 'a, T: 'a> {
    objects: &'a CollisionObjectSlab<N, T>,
    pairs: Iter<
        'a,
        SortedPair<CollisionObjectHandle>,
        (Box<ContactManifoldGenerator<N>>, ContactManifold<N>),
    >,
}

impl<'a, N: 'a + Real, T: 'a> ContactPairs<'a, N, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(
        objects: &'a CollisionObjectSlab<N, T>,
        pairs: Iter<
            'a,
            SortedPair<CollisionObjectHandle>,
            (Box<ContactManifoldGenerator<N>>, ContactManifold<N>),
        >,
    ) -> ContactPairs<'a, N, T>
    {
        ContactPairs { objects, pairs }
    }
}

impl<'a, N: Real, T> Iterator for ContactPairs<'a, N, T> {
    type Item = (
        &'a CollisionObject<N, T>,
        &'a CollisionObject<N, T>,
        &'a ContactAlgorithm<N>,
        &'a ContactManifold<N>,
    );

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.pairs.next() {
            Some((key, value)) => {
                let co1 = &self.objects[key.0];
                let co2 = &self.objects[key.1];

                Some((&co1, &co2, &value.0, &value.1))
            }
            None => None,
        }
    }
}

/// Iterator through proximity pairs.
pub struct ProximityPairs<'a, N: Real + 'a, T: 'a> {
    objects: &'a CollisionObjectSlab<N, T>,
    pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ProximityDetector<N>>>,
}

impl<'a, N: 'a + Real, T: 'a> ProximityPairs<'a, N, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(
        objects: &'a CollisionObjectSlab<N, T>,
        pairs: Iter<'a, SortedPair<CollisionObjectHandle>, Box<ProximityDetector<N>>>,
    ) -> ProximityPairs<'a, N, T>
    {
        ProximityPairs {
            objects: objects,
            pairs: pairs,
        }
    }
}

impl<'a, N: Real, T> Iterator for ProximityPairs<'a, N, T> {
    type Item = (
        &'a CollisionObject<N, T>,
        &'a CollisionObject<N, T>,
        &'a ProximityAlgorithm<N>,
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
