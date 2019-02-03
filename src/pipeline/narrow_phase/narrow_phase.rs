use na::Real;

use crate::pipeline::events::{ContactEvent, ContactEvents, ProximityEvent, ProximityEvents};
use crate::pipeline::narrow_phase::{
    ContactDispatcher, ProximityDispatcher, InteractionGraph, Interaction, InteractionGraphIndex
};
use crate::pipeline::world::{CollisionObjectHandle, CollisionObjectSlab, CollisionObject, GeometricQueryType};
use crate::query::Proximity;
use crate::utils::IdAllocator;
use crate::utils::SortedPair;

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct NarrowPhase<N: Real> {
    id_alloc: IdAllocator,
    contact_dispatcher: Box<ContactDispatcher<N>>,
    proximity_dispatcher: Box<ProximityDispatcher<N>>,
    interactions: InteractionGraph<N>
}

impl<N: Real> NarrowPhase<N> {
    /// Creates a new `NarrowPhase`.
    pub fn new(
        contact_dispatcher: Box<ContactDispatcher<N>>,
        proximity_dispatcher: Box<ProximityDispatcher<N>>,
    ) -> NarrowPhase<N>
    {
        NarrowPhase {
            id_alloc: IdAllocator::new(),
            contact_dispatcher,
            proximity_dispatcher,
            interactions: InteractionGraph::new()
        }
    }

    /// Updates the narrow-phase by actually computing contact points and proximities between the
    /// interactions pairs reported by the broad-phase.
    ///
    /// This will push relevant events to `contact_events` and `proximity_events`.
    pub fn update<T>(
        &mut self,
        objects: &CollisionObjectSlab<N, T>,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        timestamp: usize,
    )
    {
        for eid in self.interactions.graph.edge_indices() {
            let (id1, id2) = self.interactions.graph.edge_endpoints(eid).unwrap();
            let co1 = &objects[self.interactions.graph[id1]];
            let co2 = &objects[self.interactions.graph[id2]];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                match self.interactions.graph.edge_weight_mut(eid).unwrap() {
                    Interaction::Contact(detector, manifold) => {
                        let had_contacts = manifold.len() != 0;

                        if let Some(prediction) = co1
                            .query_type()
                            .contact_queries_to_prediction(co2.query_type())
                            {
                                manifold.save_cache_and_clear(&mut self.id_alloc);
                                let _ = detector.generate_contacts(
                                    &*self.contact_dispatcher,
                                    &co1.position(),
                                    co1.shape().as_ref(),
                                    None,
                                    &co2.position(),
                                    co2.shape().as_ref(),
                                    None,
                                    &prediction,
                                    &mut self.id_alloc,
                                    manifold,
                                );
                            } else {
                            panic!("Unable to compute contact between collision objects with query types different from `GeometricQueryType::Contacts(..)`.")
                        }

                        if manifold.len() == 0 {
                            if had_contacts {
                                contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
                            }
                        } else {
                            if !had_contacts {
                                contact_events.push(ContactEvent::Started(co1.handle(), co2.handle()));
                            }
                        }
                    }
                    Interaction::Proximity(detector) => {
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
        }
    }

    /// Handles a pair of collision objects detected as either started or stopped interacting.
    pub fn handle_interaction<T>(
        &mut self,
        contact_events: &mut ContactEvents,
        proximity_events: &mut ProximityEvents,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
        started: bool,
    )
    {
        let key = SortedPair::new(handle1, handle2);
        let co1 = &objects[key.0];
        let co2 = &objects[key.1];
        let id1 = co1.graph_index();
        let id2 = co2.graph_index();

        if started {
            if !self.interactions.graph.contains_edge(id1, id2) {
                match (co1.query_type(), co2.query_type()) {
                    (GeometricQueryType::Contacts(..), GeometricQueryType::Contacts(..)) => {
                        let dispatcher = &self.contact_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_contact_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                            {
                                let manifold = detector.init_manifold();
                                let _ = self.interactions.graph.add_edge(id1, id2, Interaction::Contact(detector, manifold));
                            }
                    }
                    (_, GeometricQueryType::Proximity(_)) | (GeometricQueryType::Proximity(_), _) => {
                        let dispatcher = &self.proximity_dispatcher;

                        if let Some(detector) = dispatcher
                            .get_proximity_algorithm(co1.shape().as_ref(), co2.shape().as_ref())
                            {
                                let _ = self.interactions.graph.add_edge(id1, id2, Interaction::Proximity(detector));
                            }
                    }
                }
            }
        } else {
            if let Some(eid) = self.interactions.graph.find_edge(id1, id2) {
                if let Some(detector) = self.interactions.graph.remove_edge(eid) {
                    match detector {
                        Interaction::Contact(_, mut manifold) => {
                            // Register a collision lost event if there was a contact.
                            if manifold.len() != 0 {
                                contact_events.push(ContactEvent::Stopped(co1.handle(), co2.handle()));
                            }

                            manifold.clear(&mut self.id_alloc);
                        }
                        Interaction::Proximity(detector) => {
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
    }

    /// Handles the removal of a collision object.
    pub fn handle_removal<T>(
        &mut self,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    )
    {
        let key = SortedPair::new(handle1, handle2);
        let id1 = objects[key.0].graph_index();
        let id2 = objects[key.1].graph_index();

        if let Some(edge) = self.interactions.graph.find_edge(id1, id2) {
            let interaction = self.interactions.graph.remove_edge(edge);
            if let Some(Interaction::Contact(_, mut manifold)) = interaction {
                manifold.clear(&mut self.id_alloc)
            }
        }
    }

    /// Handles the addition of a new collision object.
    pub fn handle_collision_object_added(&mut self, object: CollisionObjectHandle) -> InteractionGraphIndex {
        self.interactions.insert(object)
    }

    /// Handles the removal of a collision object.
    pub fn handle_collision_object_removed<T>(&mut self, object: &CollisionObject<N, T>) -> Option<CollisionObjectHandle> {
        let id = object.graph_index();
        let mut nbhs = self.interactions.graph.neighbors(id).detach();

        // Clear all the manifold to avoid leaking contact IDs.
        while let Some((eid, _)) = nbhs.next(&self.interactions.graph) {
            match self.interactions.graph.edge_weight_mut(eid).unwrap() {
                Interaction::Contact(_, manifold) => manifold.clear(&mut self.id_alloc),
                Interaction::Proximity(_) => {}
            }
        }

        self.interactions.remove(object.graph_index())
    }

    /// The graph where nodes are collision object handles and edges are their interactions
    /// (contact or proximity algorithms).
    pub fn interaction_graph(&self) -> &InteractionGraph<N> {
        &self.interactions
    }
}
