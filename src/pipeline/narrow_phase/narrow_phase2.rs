use na::RealField;
use slotmap::{Key, SlotMap};

use crate::pipeline::events::{ContactEvent, ContactEvents2, ProximityEvent, ProximityEvents2};
use crate::pipeline::narrow_phase::{
    ContactDispatcher, ProximityDispatcher, InteractionGraph2, Interaction2, CollisionObjectGraphIndex2,
    ContactManifoldGenerator, ProximityDetector,
};
use crate::pipeline::world::{CollisionObjectHandle, CollisionObjectSlab, CollisionObject, GeometricQueryType};
use crate::query::{Proximity, ContactManifold, ContactId};
use crate::utils::SortedPair;
use crate::pipeline::world2::{CollisionObjectRef, CollisionObjectSet};

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct NarrowPhase<N: RealField, Handle: Copy> {
    contact_dispatcher: Box<ContactDispatcher<N>>,
    proximity_dispatcher: Box<ProximityDispatcher<N>>,
    contact_events: ContactEvents2<Handle>,
    proximity_events: ProximityEvents2<Handle>,
    id_allocator: SlotMap<ContactId, bool>,
}

impl<N: RealField, Handle: Copy> NarrowPhase<N, Handle> {
    /// Creates a new `NarrowPhase`.
    pub fn new(
        contact_dispatcher: Box<ContactDispatcher<N>>,
        proximity_dispatcher: Box<ProximityDispatcher<N>>,
    ) -> NarrowPhase<N, Handle>
    {
        NarrowPhase {
            contact_dispatcher,
            proximity_dispatcher,
            contact_events: ContactEvents2::new(),
            proximity_events: ProximityEvents2::new(),
            id_allocator: SlotMap::with_key(),
        }
    }

    fn garbage_collect_ids(&mut self, interactions: &mut InteractionGraph2<N, Handle>) {
        for interaction in interactions.0.edge_weights_mut() {
            match interaction {
                Interaction2::Contact(_, manifold) => {
                    for contact in manifold.contacts() {
                        if !contact.id.is_null() {
                            self.id_allocator[contact.id] = true;
                        }
                    }
                },
                Interaction2::Proximity(_) => {}
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
                co1.shape().as_ref(),
                None,
                &co2.position(),
                co2.shape().as_ref(),
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
                unimplemented!()
//                self.contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
            }
        } else {
            if !had_contacts {
                unimplemented!()
//                self.contact_events.push(ContactEvent::Started(co1.handle(), co2.handle()));
            }
        }
    }

    /// Update the specified proximity between two collision objects.
    pub fn update_proximity<'a>(
        &mut self,
        co1: impl CollisionObjectRef<'a, N>,
        co2: impl CollisionObjectRef<'a, N>,
        detector: &mut ProximityDetector<N>) {
        let prev_prox = detector.proximity();

        let _ = detector.update(
            &*self.proximity_dispatcher,
            &co1.position(),
            co1.shape().as_ref(),
            &co2.position(),
            co2.shape().as_ref(),
            co1.query_type().query_limit() + co2.query_type().query_limit(),
        );

        let new_prox = detector.proximity();

        if new_prox != prev_prox {
            unimplemented!()
//            self.proximity_events.push(ProximityEvent::new(
//                co1.handle(),
//                co2.handle(),
//                prev_prox,
//                new_prox,
//            ));
        }
    }

    /// Update the specified interaction between two collision objects.
    pub fn update_interaction<'a>(
        &mut self,
        co1: impl CollisionObjectRef<'a, N>,
        co2: impl CollisionObjectRef<'a, N>,
        interaction: &mut Interaction2<N>) {
        match interaction {
            Interaction2::Contact(detector, manifold) => {
                self.update_contact(co1, co2, &mut **detector, manifold)
            }
            Interaction2::Proximity(detector) => {
                self.update_proximity(co1, co2, &mut **detector)
            }
        }
    }

    /// Updates the narrow-phase by actually computing contact points and proximities between the
    /// interactions pairs reported by the broad-phase.
    ///
    /// This will push relevant events to `contact_events` and `proximity_events`.
    pub fn update<'a, Objects>(
        &mut self,
        interactions: &mut InteractionGraph2<N, Objects::Handle>,
        objects: &'a Objects)
        where Objects: CollisionObjectSet<'a, N, Handle = Handle>
    {
        for eid in interactions.0.edge_indices() {
            let (id1, id2) = interactions.0.edge_endpoints(eid).unwrap();
            let co1 = objects.get(interactions.0[id1]).unwrap();
            let co2 = objects.get(interactions.0[id2]).unwrap();
            let flags1 = co1.update_flags();
            let flags2 = co2.update_flags();

            if flags1.needs_narrow_phase_update() || flags2.needs_narrow_phase_update() {
                self.update_interaction(co1, co2, interactions.0.edge_weight_mut(eid).unwrap())
            }
        }

        // FIXME: don't do this at each update?
        self.garbage_collect_ids(interactions)
    }

    /// Handles a pair of collision objects detected as either started or stopped interacting.
    pub fn handle_interaction<'a, Objects>(
        &mut self,
        interactions: &mut InteractionGraph2<N, Objects::Handle>,
        objects: &'a Objects,
        handle1: Objects::Handle,
        handle2: Objects::Handle,
        started: bool,
    )
        where Objects: CollisionObjectSet<'a, N>
    {
        let mut co1 = objects.get(handle1).unwrap();
        let mut co2 = objects.get(handle2).unwrap();
        let mut id1 = co1.graph_index();
        let mut id2 = co2.graph_index();

        if id1 > id2 {
            std::mem::swap(&mut co1, &mut co2);
            std::mem::swap(&mut id1, &mut id2);
        }

        if started {
            if !interactions.0.contains_edge(id1, id2) {
                match (co1.query_type(), co2.query_type()) {
                    (GeometricQueryType::Contacts(..), GeometricQueryType::Contacts(..)) => {
                        let dispatcher = &self.contact_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_contact_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                            {
                                let manifold = detector.init_manifold();
                                let _ = interactions.0.add_edge(id1, id2, Interaction2::Contact(detector, manifold));
                            }
                    }
                    (_, GeometricQueryType::Proximity(_)) | (GeometricQueryType::Proximity(_), _) => {
                        let dispatcher = &self.proximity_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_proximity_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                            {
                                let _ = interactions.0.add_edge(id1, id2, Interaction2::Proximity(detector));
                            }
                    }
                }
            }
        } else {
            if let Some(eid) = interactions.0.find_edge(id1, id2) {
                if let Some(detector) = interactions.0.remove_edge(eid) {
                    match detector {
                        Interaction2::Contact(_, mut manifold) => {
                            // Register a collision lost event if there was a contact.
                            if manifold.len() != 0 {
                                unimplemented!()
//                                self.contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
                            }

                            manifold.clear();
                        }
                        Interaction2::Proximity(detector) => {
                            // Register a proximity lost signal if they were not disjoint.
                            let prev_prox = detector.proximity();

                            if prev_prox != Proximity::Disjoint {
                                unimplemented!()
//                                let event = ProximityEvent::new(
//                                    co1.handle(),
//                                    co2.handle(),
//                                    prev_prox,
//                                    Proximity::Disjoint,
//                                );
//                                self.proximity_events.push(event);
                            }
                        }
                    }
                }
            }
        }
    }

    /// The set of contact events generated by this narrow-phase.
    pub fn contact_events(&self) -> &ContactEvents2<Handle> {
        &self.contact_events
    }

    /// The set of proximity events generated by this narrow-phase.
    pub fn proximity_events(&self) -> &ProximityEvents2<Handle> {
        &self.proximity_events
    }


    /// Clear the events generated by this narrow-phase.
    pub fn clear_events(&mut self) {
        self.contact_events.clear();
        self.proximity_events.clear();
    }
}
