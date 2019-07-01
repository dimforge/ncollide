use na::RealField;
use slotmap::{Key, SlotMap};

use crate::pipeline::narrow_phase::{
    ContactEvent, ContactEvents, ProximityEvent, ProximityEvents,
    ContactDispatcher, ProximityDispatcher, InteractionGraph, Interaction, CollisionObjectGraphIndex,
    ContactManifoldGenerator, ProximityDetector,
};
use crate::query::{Proximity, ContactManifold, ContactId};
use crate::pipeline::object::{CollisionObjectRef, CollisionObjectSet, GeometricQueryType, CollisionObjectHandle};


/// Collision detector dispatcher for collision objects.
pub struct NarrowPhase<N: RealField, Handle: CollisionObjectHandle> {
    contact_dispatcher: Box<ContactDispatcher<N>>,
    proximity_dispatcher: Box<ProximityDispatcher<N>>,
    contact_events: ContactEvents<Handle>,
    proximity_events: ProximityEvents<Handle>,
    id_allocator: SlotMap<ContactId, bool>,
}

impl<N: RealField, Handle: CollisionObjectHandle> NarrowPhase<N, Handle> {
    /// Creates a new `NarrowPhase`.
    pub fn new(
        contact_dispatcher: Box<ContactDispatcher<N>>,
        proximity_dispatcher: Box<ProximityDispatcher<N>>,
    ) -> NarrowPhase<N, Handle>
    {
        NarrowPhase {
            contact_dispatcher,
            proximity_dispatcher,
            contact_events: ContactEvents::new(),
            proximity_events: ProximityEvents::new(),
            id_allocator: SlotMap::with_key(),
        }
    }

    fn garbage_collect_ids(&mut self, interactions: &mut InteractionGraph<N, Handle>) {
        for interaction in interactions.0.edge_weights_mut() {
            match interaction {
                Interaction::Contact(_, manifold) => {
                    for contact in manifold.contacts() {
                        if !contact.id.is_null() {
                            self.id_allocator[contact.id] = true;
                        }
                    }
                },
                Interaction::Proximity(_) => {}
            }
        }

        self.id_allocator.retain(|_, is_valid| {
            std::mem::replace(is_valid, false)
        })
    }

    /// Update the specified contact manifold between two collision objects.
    pub fn update_contact<'a>(
        &mut self,
        co1: impl CollisionObjectRef<'a, N>,
        co2: impl CollisionObjectRef<'a, N>,
        handle1: Handle,
        handle2: Handle,
        detector: &mut ContactManifoldGenerator<N>,
        manifold: &mut ContactManifold<N>) {
        let had_contacts = manifold.len() != 0;

        if let Some(prediction) = co1
            .query_type()
            .contact_queries_to_prediction(co2.query_type())
        {
            manifold.save_cache_and_clear();
            let _ = detector.generate_contacts(
                &*self.contact_dispatcher,
                &co1.position(),
                co1.shape(),
                None,
                &co2.position(),
                co2.shape(),
                None,
                &prediction,
                manifold,
            );

            for contact in manifold.contacts_mut() {
                if contact.id.is_null() {
                    contact.id = self.id_allocator.insert(false)
                }
            }
        } else {
            panic!("Unable to compute contact between collision objects with query types different from `GeometricQueryType::Contacts(..)`.")
        }

        if manifold.len() == 0 {
            if had_contacts {
                self.contact_events.push(ContactEvent::Stopped(handle1, handle2));
            }
        } else {
            if !had_contacts {
                self.contact_events.push(ContactEvent::Started(handle1, handle2));
            }
        }
    }

    /// Update the specified proximity between two collision objects.
    pub fn update_proximity<'a>(
        &mut self,
        co1: impl CollisionObjectRef<'a, N>,
        co2: impl CollisionObjectRef<'a, N>,
        handle1: Handle,
        handle2: Handle,
        detector: &mut ProximityDetector<N>) {
        let prev_prox = detector.proximity();

        let _ = detector.update(
            &*self.proximity_dispatcher,
            &co1.position(),
            co1.shape(),
            &co2.position(),
            co2.shape(),
            co1.query_type().query_limit() + co2.query_type().query_limit(),
        );

        let new_prox = detector.proximity();

        if new_prox != prev_prox {
            self.proximity_events.push(ProximityEvent::new(
                handle1,
                handle2,
                prev_prox,
                new_prox,
            ));
        }
    }

    /// Update the specified interaction between two collision objects.
    pub fn update_interaction<'a>(
        &mut self,
        co1: impl CollisionObjectRef<'a, N>,
        co2: impl CollisionObjectRef<'a, N>,
        handle1: Handle,
        handle2: Handle,
        interaction: &mut Interaction<N>) {
        match interaction {
            Interaction::Contact(detector, manifold) => {
                self.update_contact(co1, co2, handle1, handle2, &mut **detector, manifold)
            }
            Interaction::Proximity(detector) => {
                self.update_proximity(co1, co2, handle1, handle2, &mut **detector)
            }
        }
    }

    /// Updates the narrow-phase by actually computing contact points and proximities between the
    /// interactions pairs reported by the broad-phase.
    ///
    /// This will push relevant events to `contact_events` and `proximity_events`.
    pub fn update<'a, Objects>(
        &mut self,
        interactions: &mut InteractionGraph<N, Objects::Handle>,
        objects: &'a Objects)
        where Objects: CollisionObjectSet<'a, N, Handle = Handle>
    {
        for eid in interactions.0.edge_indices() {
            let (id1, id2) = interactions.0.edge_endpoints(eid).unwrap();
            let handle1 = interactions.0[id1];
            let handle2 = interactions.0[id2];
            let co1 = objects.get(handle1).unwrap();
            let co2 = objects.get(handle2).unwrap();
            let flags1 = co1.update_flags();
            let flags2 = co2.update_flags();

            if flags1.needs_narrow_phase_update() || flags2.needs_narrow_phase_update() {
                self.update_interaction(co1, co2, handle1, handle2, interactions.0.edge_weight_mut(eid).unwrap())
            }
        }

        // FIXME: don't do this at each update?
        self.garbage_collect_ids(interactions)
    }

    /// Handles a pair of collision objects detected as either started or stopped interacting.
    pub fn handle_interaction<'a, Objects>(
        &mut self,
        interactions: &mut InteractionGraph<N, Objects::Handle>,
        objects: &'a Objects,
        mut handle1: Objects::Handle,
        mut handle2: Objects::Handle,
        started: bool,
    )
        where Objects: CollisionObjectSet<'a, N, Handle = Handle>
    {
        let mut co1 = objects.get(handle1).unwrap();
        let mut co2 = objects.get(handle2).unwrap();
        let mut id1 = co1.graph_index();
        let mut id2 = co2.graph_index();

        if id1 > id2 {
            std::mem::swap(&mut co1, &mut co2);
            std::mem::swap(&mut id1, &mut id2);
            std::mem::swap(&mut handle1, &mut handle2);
        }

        if started {
            if !interactions.0.contains_edge(id1, id2) {
                match (co1.query_type(), co2.query_type()) {
                    (GeometricQueryType::Contacts(..), GeometricQueryType::Contacts(..)) => {
                        let dispatcher = &self.contact_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_contact_algorithm(co1.shape(), co2.shape())
                            {
                                let manifold = detector.init_manifold();
                                let _ = interactions.0.add_edge(id1, id2, Interaction::Contact(detector, manifold));
                            }
                    }
                    (_, GeometricQueryType::Proximity(_)) | (GeometricQueryType::Proximity(_), _) => {
                        let dispatcher = &self.proximity_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_proximity_algorithm(co1.shape(), co2.shape())
                            {
                                let _ = interactions.0.add_edge(id1, id2, Interaction::Proximity(detector));
                            }
                    }
                }
            }
        } else {
            if let Some(eid) = interactions.0.find_edge(id1, id2) {
                if let Some(detector) = interactions.0.remove_edge(eid) {
                    match detector {
                        Interaction::Contact(_, mut manifold) => {
                            // Register a collision lost event if there was a contact.
                            if manifold.len() != 0 {
                                self.contact_events.push(ContactEvent::Stopped(handle1, handle2));
                            }

                            manifold.clear();
                        }
                        Interaction::Proximity(detector) => {
                            // Register a proximity lost signal if they were not disjoint.
                            let prev_prox = detector.proximity();

                            if prev_prox != Proximity::Disjoint {
                                let event = ProximityEvent::new(
                                    handle1,
                                    handle2,
                                    prev_prox,
                                    Proximity::Disjoint,
                                );
                                self.proximity_events.push(event);
                            }
                        }
                    }
                }
            }
        }
    }

    /// The set of contact events generated by this narrow-phase.
    pub fn contact_events(&self) -> &ContactEvents<Handle> {
        &self.contact_events
    }

    /// The set of proximity events generated by this narrow-phase.
    pub fn proximity_events(&self) -> &ProximityEvents<Handle> {
        &self.proximity_events
    }

    /// Clear the events generated by this narrow-phase.
    pub fn clear_events(&mut self) {
        self.contact_events.clear();
        self.proximity_events.clear();
    }
}
