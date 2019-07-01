use petgraph::graph::{UnGraph, NodeIndex};
use petgraph::visit::EdgeRef;
use na::RealField;

use crate::query::{ContactManifold, Proximity};
use crate::pipeline::narrow_phase::{ContactAlgorithm, ProximityAlgorithm};
use crate::pipeline::object::CollisionObjectHandle;
use petgraph::prelude::EdgeIndex;
use petgraph::Direction;

/// Index of a node of the interaction graph.
pub type CollisionObjectGraphIndex = NodeIndex<usize>;
/// Temporary index to and edge of the interaction graph.
pub type TemporaryInteractionIndex = EdgeIndex<usize>;

/// An interaction between two collision objects.
pub enum Interaction<N: RealField> {
    /// A potential contact between two collision objects.
    ///
    /// Generated only for pairs of collision objects both configured
    /// with a `GeometricQueryType::Contact(..)`.
    Contact(ContactAlgorithm<N>, ContactManifold<N>),
    /// A proximity between two collision objects.
    ///
    /// Generated only for pairs of collision objects with at least one configured
    /// with a `GeometricQueryType::Contact(..)`.
    Proximity(ProximityAlgorithm<N>)
}

impl<N: RealField> Interaction<N> {
    /// Checks if this interaction is a potential contact interaction.
    pub fn is_contact(&self) -> bool {
        match self {
            Interaction::Contact(..) => true,
            _ => false
        }
    }

    /// Checks if this interaction is a potential proximity interaction.
    pub fn is_proximity(&self) -> bool {
        match self {
            Interaction::Proximity(_) => true,
            _ => false
        }
    }
}

/// A graph where nodes are collision objects and edges are contact or proximity algorithms.
pub struct InteractionGraph<N: RealField, Handle: CollisionObjectHandle>(pub(crate) UnGraph<Handle, Interaction<N>, usize>);

impl<N: RealField, Handle: CollisionObjectHandle> InteractionGraph<N, Handle> {
    /// Creates a new empty collection of collision objects.
    pub fn new() -> Self {
        InteractionGraph(UnGraph::with_capacity(10, 10))
    }

    /// Adds a handle to this graph.
    pub fn add_node(&mut self, handle: Handle) -> CollisionObjectGraphIndex {
        self.0.add_node(handle)
    }

    /// Removes a handle from this graph and returns a handle that must have its graph index changed to `id`.
    ///
    /// When a node is removed, another node of the graph takes it place. This means that the `CollisionObjectGraphIndex`
    /// of the collision object returned by this method will be equal to `id`. Thus if you maintain
    /// a map between `CollisionObjectSlabHandle` and `CollisionObjectGraphIndex`, then you should update this
    /// map to associate `id` to the handle returned by this method. For example:
    ///
    /// ```
    /// // Let `id` be the graph index of the collision object we want to remove.
    /// if let Some(other_handle) = graph.remove_node(id) {
    ///    // The graph index of `other_handle` changed to `id` due to the removal.
    ///    map.insert(other_handle, id) ;
    /// }
    /// ```
    #[must_use = "The graph index of the collision object returned by this method has been changed to `id`."]
    pub fn remove_node(&mut self, id: CollisionObjectGraphIndex) -> Option<Handle> {
        let _ = self.0.remove_node(id);
        self.0.node_weight(id).cloned()
    }

    /// All the interactions pairs on this graph.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        Handle,
        Handle,
        &Interaction<N>
    )> {
        self.0
            .edge_references()
            .filter_map(move |e| {
            let interaction = e.weight();

            if !effective_only || Self::is_interaction_effective(interaction) {
                Some((self.0[e.source()], self.0[e.target()], e.weight()))
            } else {
                None
            }
        })
    }

    /// All the contact pairs on this graph.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        Handle,
        Handle,
        &ContactAlgorithm<N>,
        &ContactManifold<N>,
    )> {
        self.interaction_pairs(effective_only)
            .filter_map(|(h1, h2, inter)| {
            match inter {
                Interaction::Contact(algo, manifold) => Some((h1, h2, algo, manifold)),
                _ => None
            }
        })
    }

    /// All the proximity pairs on this graph.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        Handle,
        Handle,
        &ProximityAlgorithm<N>,
    )> {
        self.interaction_pairs(effective_only)
            .filter_map(|(h1, h2, inter)| {
            match inter {
                Interaction::Proximity(algo) => Some((h1, h2, algo)),
                _ => None
            }
        })
    }

    /// The interaction between the two collision objects identified by their graph index.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pair(&self, id1: CollisionObjectGraphIndex, id2: CollisionObjectGraphIndex, effective_only: bool) -> Option<(Handle, Handle, &Interaction<N>)> {
        let inter = self.0.find_edge(id1, id2).and_then(|edge| {
            let endpoints = self.0.edge_endpoints(edge)?;
            let h1 = self.0.node_weight(endpoints.0)?;
            let h2 = self.0.node_weight(endpoints.1)?;
            Some((*h1, *h2, self.0.edge_weight(edge)?))
        });

        if effective_only {
            inter.filter(|inter| Self::is_interaction_effective(inter.2))
        } else {
            inter
        }
    }


    /// The contact pair between the two collision objects identified by their graph index.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pair(&self, id1: CollisionObjectGraphIndex, id2: CollisionObjectGraphIndex, effective_only: bool) -> Option<(Handle, Handle, &ContactAlgorithm<N>, &ContactManifold<N>)> {
        self.interaction_pair(id1, id2, effective_only).and_then(|inter| {
            match inter.2 {
                Interaction::Contact(algo, manifold) => Some((inter.0, inter.1, algo, manifold)),
                _ => None
            }
        })
    }

    /// The proximity pair between the two collision objects identified by their graph index.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pair(&self, id1: CollisionObjectGraphIndex, id2: CollisionObjectGraphIndex, effective_only: bool) -> Option<(Handle, Handle, &ProximityAlgorithm<N>)> {
        self.interaction_pair(id1, id2, effective_only).and_then(|inter| {
            match inter.2 {
                Interaction::Proximity(algo) => Some((inter.0, inter.1, algo)),
                _ => None
            }
        })
    }

    /// All the interaction involving the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with(&self, id: CollisionObjectGraphIndex, effective_only: bool) -> impl Iterator<Item = (Handle, Handle, &Interaction<N>)> {
        self.0.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if !effective_only || Self::is_interaction_effective(inter) {
                let endpoints = self.0.edge_endpoints(e.id()).unwrap();
                Some((self.0[endpoints.0], self.0[endpoints.1], e.weight()))
            } else {
                None
            }
        })
    }

    /// Gets the interaction with the given index.
    pub fn index_interaction(&self, id: TemporaryInteractionIndex) -> Option<(Handle, Handle, &Interaction<N>)> {
        if let (Some(e), Some(endpoints)) = (self.0.edge_weight(id), self.0.edge_endpoints(id)) {
            Some((self.0[endpoints.0], self.0[endpoints.1], e))
        } else {
            None
        }
    }

    /// All the mutable references to interactions involving the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with_mut(&mut self, id: CollisionObjectGraphIndex) -> impl Iterator<Item = (Handle, Handle, TemporaryInteractionIndex, &mut Interaction<N>)> {
        let incoming_edge = self.0.first_edge(id, Direction::Incoming);
        let outgoing_edge = self.0.first_edge(id, Direction::Outgoing);

        InteractionsWithMut {
            graph: self,
            incoming_edge,
            outgoing_edge,
        }
    }

    /// All the proximity pairs involving the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximities_with(&self, handle: CollisionObjectGraphIndex, effective_only: bool) -> impl Iterator<Item = (Handle, Handle, &ProximityAlgorithm<N>)> {
        self.interactions_with(handle, effective_only)
            .filter_map(|(h1, h2, inter)| {
                match inter {
                    Interaction::Proximity(algo) => Some((h1, h2, algo)),
                    _ => None
                }
            })
    }


    /// All the contact pairs involving the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contacts_with(&self, handle: CollisionObjectGraphIndex, effective_only: bool) -> impl Iterator<Item = (Handle, Handle, &ContactAlgorithm<N>, &ContactManifold<N>)> {
        self.interactions_with(handle, effective_only)
            .filter_map(|(h1, h2, inter)| {
                match inter {
                    Interaction::Contact(algo, manifold) => Some((h1, h2, algo, manifold)),
                    _ => None
                }
            })
    }


    /// All the collision object handles of collision objects interacting with the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_interacting_with<'a>(&'a self, id: CollisionObjectGraphIndex) -> impl Iterator<Item = Handle> + 'a {
        self.0.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if Self::is_interaction_effective(inter) {
                if e.source() == id {
                    Some(self.0[e.target()])
                } else {
                    Some(self.0[e.source()])
                }
            } else {
                None
            }
        })
    }

    /// All the collision object handles of collision objects in contact with the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_in_contact_with<'a>(&'a self, id: CollisionObjectGraphIndex) -> impl Iterator<Item = Handle> + 'a {
        self.0.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if inter.is_contact() && Self::is_interaction_effective(inter) {
                if e.source() == id {
                    Some(self.0[e.target()])
                } else {
                    Some(self.0[e.source()])
                }
            } else {
                None
            }
        })
    }

    /// All the collision object handles of collision objects in proximity of with the collision object with graph index `id`.
    ///
    /// Refer to the official [user guide](https://ncollide.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_in_proximity_of<'a>(&'a self, id: CollisionObjectGraphIndex) -> impl Iterator<Item = Handle> + 'a {
        self.0.edges(id).filter_map(move |e| {
            if let Interaction::Proximity(alg) = e.weight() {
                if alg.proximity() == Proximity::Intersecting {
                    if e.source() == id {
                        return Some(self.0[e.target()]);
                    } else {
                        return Some(self.0[e.source()]);
                    }
                }
            }

            None
        })
    }

    // NOTE: we don't make this method public because different
    // applications will have a different interpretation of when a
    // contact is considered effective (for example in nphysics).
    fn is_interaction_effective(interaction: &Interaction<N>) -> bool {
        match interaction {
            Interaction::Contact(_, manifold) => {
                if let Some(ctct) = manifold.deepest_contact() {
                    ctct.contact.depth >= N::zero()
                } else {
                    false
                }
            },
            Interaction::Proximity(alg) => alg.proximity() == Proximity::Intersecting,
        }
    }
}


pub struct InteractionsWithMut<'a, N: RealField, Handle: CollisionObjectHandle> {
    graph: &'a mut InteractionGraph<N, Handle>,
    incoming_edge: Option<EdgeIndex<usize>>,
    outgoing_edge: Option<EdgeIndex<usize>>,
}

impl<'a, N: RealField, Handle: CollisionObjectHandle> Iterator for InteractionsWithMut<'a, N, Handle> {
    type Item = (Handle, Handle, TemporaryInteractionIndex, &'a mut Interaction<N>);

    #[inline]
    fn next(&mut self) -> Option<(Handle, Handle, TemporaryInteractionIndex, &'a mut Interaction<N>)> {
        if let Some(edge) = self.incoming_edge {
            self.incoming_edge = self.graph.0.next_edge(edge, Direction::Incoming);
            let endpoints = self.graph.0.edge_endpoints(edge).unwrap();
            let (co1, co2) = (self.graph.0[endpoints.0], self.graph.0[endpoints.1]);
            let interaction = self.graph.0.edge_weight_mut(edge)?;
            return Some((co1, co2, edge, unsafe { std::mem::transmute(interaction) }));
        }

        let edge = self.outgoing_edge?;
        self.outgoing_edge = self.graph.0.next_edge(edge, Direction::Outgoing);
        let endpoints = self.graph.0.edge_endpoints(edge).unwrap();
        let (co1, co2) = (self.graph.0[endpoints.0], self.graph.0[endpoints.1]);
        let interaction = self.graph.0.edge_weight_mut(edge)?;
        Some((co1, co2, edge, unsafe { std::mem::transmute(interaction) }))
    }
}