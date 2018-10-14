use na::Real;
use pipeline::events::{ContactEvent, ContactEvents, ProximityEvent, ProximityEvents};
use pipeline::narrow_phase::{
    ContactAlgorithm, ContactDispatcher, ContactPairs, NarrowPhase, ProximityAlgorithm,
    ProximityDispatcher, ProximityPairs,
};
use pipeline::world::{CollisionObjectHandle, CollisionObjectSlab, GeometricQueryType};
use query::Proximity;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use utils::IdAllocator;
use utils::{DeterministicState, SortedPair};

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct DefaultNarrowPhase<N> {
    id_alloc: IdAllocator,
    contact_dispatcher: Box<ContactDispatcher<N>>,
    contact_generators:
        HashMap<SortedPair<CollisionObjectHandle>, ContactAlgorithm<N>, DeterministicState>,

    proximity_dispatcher: Box<ProximityDispatcher<N>>,
    proximity_detectors:
        HashMap<SortedPair<CollisionObjectHandle>, ProximityAlgorithm<N>, DeterministicState>,
}

impl<N: 'static> DefaultNarrowPhase<N> {
    /// Creates a new `DefaultNarrowPhase`.
    pub fn new(
        contact_dispatcher: Box<ContactDispatcher<N>>,
        proximity_dispatcher: Box<ProximityDispatcher<N>>,
    ) -> DefaultNarrowPhase<N> {
        DefaultNarrowPhase {
            id_alloc: IdAllocator::new(),
            contact_dispatcher: contact_dispatcher,
            contact_generators: HashMap::with_hasher(DeterministicState::new()),

            proximity_dispatcher: proximity_dispatcher,
            proximity_detectors: HashMap::with_hasher(DeterministicState::new()),
        }
    }
}

impl<N: Real, T> NarrowPhase<N, T> for DefaultNarrowPhase<N> {
    fn update(
        &mut self,
        objects: &CollisionObjectSlab<N, T>,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        timestamp: usize,
    ) {
        for (key, value) in self.contact_generators.iter_mut() {
            let co1 = &objects[key.0];
            let co2 = &objects[key.1];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let had_contacts = value.num_contacts() != 0;

                if let Some(prediction) = co1
                    .query_type()
                    .contact_queries_to_prediction(co2.query_type())
                {
                    let _ = value.update(
                        &*self.contact_dispatcher,
                        0,
                        &co1.position(),
                        co1.shape().as_ref(),
                        0,
                        &co2.position(),
                        co2.shape().as_ref(),
                        &prediction,
                        &mut self.id_alloc,
                    );
                } else {
                    panic!("Unable to compute contact between collision objects with query types different from `GeometricQueryType::Contacts(..)`.")
                }

                if value.num_contacts() == 0 {
                    if had_contacts {
                        contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
                    }
                } else {
                    if !had_contacts {
                        contact_events.push(ContactEvent::Started(co1.handle(), co2.handle()));
                    }
                }
            }
        }

        for (key, value) in self.proximity_detectors.iter_mut() {
            let co1 = &objects[key.0];
            let co2 = &objects[key.1];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let prev_prox = value.proximity();

                let _ = value.update(
                    &*self.proximity_dispatcher,
                    &co1.position(),
                    co1.shape().as_ref(),
                    &co2.position(),
                    co2.shape().as_ref(),
                    co1.query_type().query_limit() + co2.query_type().query_limit(),
                );

                let new_prox = value.proximity();

                if new_prox != prev_prox {
                    proximity_events.push(ProximityEvent::new(
                        co1.handle(),
                        co2.handle(),
                        prev_prox,
                        new_prox,
                    ));
                }
            }
        }
    }

    fn handle_interaction(
        &mut self,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
        started: bool,
    ) {
        let key = SortedPair::new(handle1, handle2);
        let co1 = &objects[key.0];
        let co2 = &objects[key.1];

        match (co1.query_type(), co2.query_type()) {
            (GeometricQueryType::Contacts(..), GeometricQueryType::Contacts(..)) => {
                if started {
                    let dispatcher = &self.contact_dispatcher;

                    if let Entry::Vacant(entry) = self.contact_generators.entry(key) {
                        if let Some(detector) = dispatcher
                            .get_contact_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                        {
                            let _ = entry.insert(detector);
                        }
                    }
                } else {
                    // Proximity stopped.
                    if let Some(detector) = self.contact_generators.remove(&key) {
                        // Register a collision lost event if there was a contact.
                        if detector.num_contacts() != 0 {
                            contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
                        }
                    }
                }
            }
            (_, GeometricQueryType::Proximity(_)) | (GeometricQueryType::Proximity(_), _) => {
                if started {
                    let dispatcher = &self.proximity_dispatcher;

                    if let Entry::Vacant(entry) = self.proximity_detectors.entry(key) {
                        if let Some(detector) = dispatcher
                            .get_proximity_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                        {
                            let _ = entry.insert(detector);
                        }
                    }
                } else {
                    // Proximity stopped.
                    if let Some(detector) = self.proximity_detectors.remove(&key) {
                        // Register a proximity lost signal if they were not disjoint.
                        let prev_prox = detector.proximity();

                        if prev_prox != Proximity::Disjoint {
                            let event = ProximityEvent::new(
                                co1.handle(),
                                co2.handle(),
                                prev_prox,
                                Proximity::Disjoint,
                            );
                            proximity_events.push(event);
                        }
                    }
                }
            }
        }
    }

    fn handle_removal(
        &mut self,
        _: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) {
        let key = SortedPair::new(handle1, handle2);
        let _ = self.proximity_detectors.remove(&key);
        let _ = self.contact_generators.remove(&key);
    }

    fn contact_pair(
        &self,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) -> Option<&ContactAlgorithm<N>> {
        let key = SortedPair::new(handle1, handle2);
        self.contact_generators.get(&key)
    }

    fn contact_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<N, T>,
    ) -> ContactPairs<'a, N, T> {
        ContactPairs::new(objects, self.contact_generators.iter())
    }

    fn proximity_pairs<'a>(
        &'a self,
        objects: &'a CollisionObjectSlab<N, T>,
    ) -> ProximityPairs<'a, N, T> {
        ProximityPairs::new(objects, self.proximity_detectors.iter())
    }
}
