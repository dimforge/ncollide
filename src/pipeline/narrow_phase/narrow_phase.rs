use na::RealField;
use slotmap::{Key, SlotMap};

use crate::pipeline::narrow_phase::{
    ContactDispatcher, ContactEvent, ContactEvents, ContactManifoldGenerator, Interaction,
    InteractionGraph, ProximityDetector, ProximityDispatcher, ProximityEvent, ProximityEvents,
};
use crate::pipeline::object::{
    CollisionObjectHandle, CollisionObjectRef, CollisionObjectSet, GeometricQueryType,
};
use crate::query::{ContactId, ContactManifold, Proximity};

/// Collision detector dispatcher for collision objects.
pub struct NarrowPhase<N: RealField + Copy, Handle: CollisionObjectHandle> {
    contact_dispatcher: Box<dyn ContactDispatcher<N>>,
    proximity_dispatcher: Box<dyn ProximityDispatcher<N>>,
    contact_events: ContactEvents<Handle>,
    proximity_events: ProximityEvents<Handle>,
    id_allocator: SlotMap<ContactId, bool>,
}

impl<N: RealField + Copy, Handle: CollisionObjectHandle> NarrowPhase<N, Handle> {
    /// Creates a new `NarrowPhase`.
    pub fn new(
        contact_dispatcher: Box<dyn ContactDispatcher<N>>,
        proximity_dispatcher: Box<dyn ProximityDispatcher<N>>,
    ) -> NarrowPhase<N, Handle> {
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
                }
                Interaction::Proximity(..) => {}
            }
        }

        self.id_allocator
            .retain(|_, is_valid| std::mem::replace(is_valid, false))
    }

    /// Update the specified contact manifold between two collision objects.
    pub fn update_contact(
        &mut self,
        co1: &impl CollisionObjectRef<N>,
        co2: &impl CollisionObjectRef<N>,
        handle1: Handle,
        handle2: Handle,
        detector: &mut dyn ContactManifoldGenerator<N>,
        manifold: &mut ContactManifold<N>,
    ) {
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
                self.contact_events
                    .push(ContactEvent::Stopped(handle1, handle2));
            }
        } else {
            if !had_contacts {
                self.contact_events
                    .push(ContactEvent::Started(handle1, handle2));
            }
        }
    }

    // FIXME: the fact this is public is only useful for nphysics.
    // Perhaps the event pools should not be owned by the NarrowPhase struct?
    #[doc(hidden)]
    pub fn emit_proximity_event(
        &mut self,
        handle1: Handle,
        handle2: Handle,
        prev_prox: Proximity,
        new_prox: Proximity,
    ) {
        if prev_prox != new_prox {
            self.proximity_events
                .push(ProximityEvent::new(handle1, handle2, prev_prox, new_prox));
        }
    }

    /// Update the specified proximity between two collision objects.
    pub fn update_proximity(
        &mut self,
        co1: &impl CollisionObjectRef<N>,
        co2: &impl CollisionObjectRef<N>,
        handle1: Handle,
        handle2: Handle,
        detector: &mut dyn ProximityDetector<N>,
        curr_proximity: &mut Proximity,
    ) {
        if let Some(new_proximity) = detector.update(
            &*self.proximity_dispatcher,
            &co1.position(),
            co1.shape(),
            &co2.position(),
            co2.shape(),
            co1.query_type().query_limit() + co2.query_type().query_limit(),
        ) {
            self.emit_proximity_event(handle1, handle2, *curr_proximity, new_proximity);
            *curr_proximity = new_proximity;
        }
    }

    /// Update the specified interaction between two collision objects.
    pub fn update_interaction(
        &mut self,
        co1: &impl CollisionObjectRef<N>,
        co2: &impl CollisionObjectRef<N>,
        handle1: Handle,
        handle2: Handle,
        interaction: &mut Interaction<N>,
    ) {
        match interaction {
            Interaction::Contact(detector, manifold) => {
                self.update_contact(co1, co2, handle1, handle2, &mut **detector, manifold)
            }
            Interaction::Proximity(detector, prox) => {
                self.update_proximity(co1, co2, handle1, handle2, &mut **detector, prox)
            }
        }
    }

    /// Updates the narrow-phase by actually computing contact points and proximities between the
    /// interactions pairs reported by the broad-phase.
    ///
    /// This will push relevant events to `contact_events` and `proximity_events`.
    pub fn update<Objects>(
        &mut self,
        interactions: &mut InteractionGraph<N, Objects::CollisionObjectHandle>,
        objects: &Objects,
    ) where
        Objects: CollisionObjectSet<N, CollisionObjectHandle = Handle>,
    {
        for eid in interactions.0.edge_indices() {
            let (id1, id2) = interactions.0.edge_endpoints(eid).unwrap();
            let handle1 = interactions.0[id1];
            let handle2 = interactions.0[id2];
            let co1 = objects.collision_object(handle1).unwrap();
            let co2 = objects.collision_object(handle2).unwrap();
            let flags1 = co1.update_flags();
            let flags2 = co2.update_flags();

            if flags1.needs_narrow_phase_update() || flags2.needs_narrow_phase_update() {
                self.update_interaction(
                    co1,
                    co2,
                    handle1,
                    handle2,
                    interactions.0.edge_weight_mut(eid).unwrap(),
                )
            }
        }

        // FIXME: don't do this at each update?
        self.garbage_collect_ids(interactions)
    }

    /// Handles a pair of collision objects detected as either started or stopped interacting.
    pub fn handle_interaction<Objects>(
        &mut self,
        interactions: &mut InteractionGraph<N, Objects::CollisionObjectHandle>,
        objects: &Objects,
        handle1: Objects::CollisionObjectHandle,
        handle2: Objects::CollisionObjectHandle,
        started: bool,
    ) where
        Objects: CollisionObjectSet<N, CollisionObjectHandle = Handle>,
    {
        let co1 = objects.collision_object(handle1).unwrap();
        let co2 = objects.collision_object(handle2).unwrap();
        let id1 = co1.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = co2.graph_index().expect(crate::NOT_REGISTERED_ERROR);

        if started {
            if !interactions.0.contains_edge(id1, id2) {
                match (co1.query_type(), co2.query_type()) {
                    (GeometricQueryType::Contacts(..), GeometricQueryType::Contacts(..)) => {
                        let dispatcher = &self.contact_dispatcher;

                        if let Some(detector) =
                            dispatcher.get_contact_algorithm(co1.shape(), co2.shape())
                        {
                            let manifold = detector.init_manifold();
                            let _ = interactions.0.add_edge(
                                id1,
                                id2,
                                Interaction::Contact(detector, manifold),
                            );
                        }
                    }
                    (_, GeometricQueryType::Proximity(_))
                    | (GeometricQueryType::Proximity(_), _) => {
                        let dispatcher = &self.proximity_dispatcher;

                        if let Some(detector) =
                            dispatcher.get_proximity_algorithm(co1.shape(), co2.shape())
                        {
                            let _ = interactions.0.add_edge(
                                id1,
                                id2,
                                Interaction::Proximity(detector, Proximity::Disjoint),
                            );
                        }
                    }
                }
            }
        } else {
            if let Some(eid) = interactions.0.find_edge(id1, id2) {
                let endpoints = interactions.0.edge_endpoints(eid).unwrap();
                let handle1 = *interactions.0.node_weight(endpoints.0).unwrap();
                let handle2 = *interactions.0.node_weight(endpoints.1).unwrap();

                if let Some(detector) = interactions.0.remove_edge(eid) {
                    match detector {
                        Interaction::Contact(_, mut manifold) => {
                            // Register a collision lost event if there was a contact.
                            if manifold.len() != 0 {
                                self.contact_events
                                    .push(ContactEvent::Stopped(handle1, handle2));
                            }

                            manifold.clear();
                        }
                        Interaction::Proximity(_, prev_prox) => {
                            // Register a proximity lost signal if they were not disjoint.
                            self.emit_proximity_event(
                                handle1,
                                handle2,
                                prev_prox,
                                Proximity::Disjoint,
                            );
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
