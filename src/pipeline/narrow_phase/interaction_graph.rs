use petgraph::graph::{UnGraph, NodeIndex};
use petgraph::visit::EdgeRef;
use na::Real;

use crate::query::{ContactManifold, Proximity};
use crate::world::CollisionObjectHandle;
use crate::pipeline::narrow_phase::{ContactAlgorithm, ProximityAlgorithm};

pub type InteractionGraphIndex = NodeIndex<usize>;
pub enum Interaction<N: Real> {
    Contact(ContactAlgorithm<N>, ContactManifold<N>),
    Proximity(ProximityAlgorithm<N>)
}

impl<N: Real> Interaction<N> {
    pub fn is_contact(&self) -> bool {
        match self {
            Interaction::Contact(..) => true,
            _ => false
        }
    }

    pub fn is_proximity(&self) -> bool {
        match self {
            Interaction::Proximity(_) => true,
            _ => false
        }
    }
}

/// A graph where nodes are collision objects and edges are contact or proximity algorithms.
pub struct InteractionGraph<N: Real> {
    pub(crate) graph: UnGraph<CollisionObjectHandle, Interaction<N>, usize>
}

impl<N: Real> InteractionGraph<N> {
    /// Creates a new empty collection of collision objects.
    pub fn new() -> Self {
        InteractionGraph {
            graph: UnGraph::with_capacity(10, 10),
        }
    }

    pub fn raw_graph(&self) -> &UnGraph<CollisionObjectHandle, Interaction<N>, usize> {
        &self.graph
    }

    pub fn into_inner(self) -> UnGraph<CollisionObjectHandle, Interaction<N>, usize> {
        self.graph
    }

    /// Inserts a new collision object into this collection and returns the corresponding handle.
    #[inline]
    pub fn insert(&mut self, handle: CollisionObjectHandle) -> InteractionGraphIndex {
        self.graph.add_node(handle)
    }

    /// Removes from this collection the collision object identified by the given handle.
    ///
    /// The removed collision object structure is returned.
    #[inline]
    pub fn remove(&mut self, id: InteractionGraphIndex) -> Option<CollisionObjectHandle> {
        let _ = self.graph.remove_node(id);
        self.graph.node_weight(id).map(|h| *h)
    }

    pub fn interaction_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        CollisionObjectHandle,
        CollisionObjectHandle,
        &Interaction<N>
    )> {
        self.graph
            .edge_references()
            .filter_map(move |e| {
            let interaction = e.weight();

            if !effective_only || Self::is_interaction_effective(interaction) {
                Some((self.graph[e.source()], self.graph[e.target()], e.weight()))
            } else {
                None
            }
        })
    }

    pub fn contact_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        CollisionObjectHandle,
        CollisionObjectHandle,
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

    pub fn proximity_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        CollisionObjectHandle,
        CollisionObjectHandle,
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

    pub fn interaction_pair(&self, id1: InteractionGraphIndex, id2: InteractionGraphIndex, effective_only: bool) -> Option<(CollisionObjectHandle, CollisionObjectHandle, &Interaction<N>)> {
        let inter = self.graph.find_edge(id1, id2).and_then(|edge| {
            let endpoints = self.graph.edge_endpoints(edge)?;
            let h1 = self.graph.node_weight(endpoints.0)?;
            let h2 = self.graph.node_weight(endpoints.1)?;
            Some((*h1, *h2, self.graph.edge_weight(edge)?))
        });

        if effective_only {
            inter.filter(|inter| Self::is_interaction_effective(inter.2))
        } else {
            inter
        }
    }

    pub fn contact_pair(&self, id1: InteractionGraphIndex, id2: InteractionGraphIndex, effective_only: bool) -> Option<(CollisionObjectHandle, CollisionObjectHandle, &ContactAlgorithm<N>, &ContactManifold<N>)> {
        self.interaction_pair(id1, id2, effective_only).and_then(|inter| {
            match inter.2 {
                Interaction::Contact(algo, manifold) => Some((inter.0, inter.1, algo, manifold)),
                _ => None
            }
        })
    }

    pub fn proximity_pair(&self, id1: InteractionGraphIndex, id2: InteractionGraphIndex, effective_only: bool) -> Option<(CollisionObjectHandle, CollisionObjectHandle, &ProximityAlgorithm<N>)> {
        self.interaction_pair(id1, id2, effective_only).and_then(|inter| {
            match inter.2 {
                Interaction::Proximity(algo) => Some((inter.0, inter.1, algo)),
                _ => None
            }
        })
    }


    pub fn interactions_with(&self, id: InteractionGraphIndex, effective_only: bool) -> impl Iterator<Item = (CollisionObjectHandle, CollisionObjectHandle, &Interaction<N>)> {
        self.graph.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if !effective_only || Self::is_interaction_effective(inter) {
                Some((self.graph[e.source()], self.graph[e.target()], e.weight()))
            } else {
                None
            }
        })
    }

    pub fn proximities_with(&self, handle: InteractionGraphIndex, effective_only: bool) -> impl Iterator<Item = (CollisionObjectHandle, CollisionObjectHandle, &ProximityAlgorithm<N>)> {
        self.interactions_with(handle, effective_only)
            .filter_map(|(h1, h2, inter)| {
                match inter {
                    Interaction::Proximity(algo) => Some((h1, h2, algo)),
                    _ => None
                }
            })
    }

    pub fn contacts_with(&self, handle: InteractionGraphIndex, effective_only: bool) -> impl Iterator<Item = (CollisionObjectHandle, CollisionObjectHandle, &ContactAlgorithm<N>, &ContactManifold<N>)> {
        self.interactions_with(handle, effective_only)
            .filter_map(|(h1, h2, inter)| {
                match inter {
                    Interaction::Contact(algo, manifold) => Some((h1, h2, algo, manifold)),
                    _ => None
                }
            })
    }


    pub fn collision_objects_interacting_with<'a>(&'a self, id: InteractionGraphIndex) -> impl Iterator<Item = CollisionObjectHandle> + 'a {
        self.graph.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if Self::is_interaction_effective(inter) {
                if e.source() == id {
                    Some(self.graph[e.target()])
                } else {
                    Some(self.graph[e.source()])
                }
            } else {
                None
            }
        })
    }

    pub fn collision_objects_in_contact_with<'a>(&'a self, id: InteractionGraphIndex) -> impl Iterator<Item = CollisionObjectHandle> + 'a {
        self.graph.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if inter.is_contact() && Self::is_interaction_effective(inter) {
                if e.source() == id {
                    Some(self.graph[e.target()])
                } else {
                    Some(self.graph[e.source()])
                }
            } else {
                None
            }
        })
    }

    pub fn collision_objects_in_proximity_of<'a>(&'a self, id: InteractionGraphIndex) -> impl Iterator<Item = CollisionObjectHandle> + 'a {
        self.graph.edges(id).filter_map(move |e| {
            let inter = e.weight();

            if let Interaction::Proximity(alg) = e.weight() {
                if alg.proximity() == Proximity::Intersecting {
                    if e.source() == id {
                        return Some(self.graph[e.target()]);
                    } else {
                        return Some(self.graph[e.source()]);
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