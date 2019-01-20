use petgraph::graph::{DiGraph, NodeIndex};
use slab::Slab;
use na::Real;

use crate::query::ContactManifold;
use crate::world::{CollisionObject, CollisionObjectHandle};
use crate::pipeline::narrow_phase::{ContactAlgorithm, ProximityAlgorithm};

pub type InteractionGraphIndex = NodeIndex<usize>;
pub enum Interaction<N: Real> {
    Contact(ContactAlgorithm<N>, ContactManifold<N>),
    Proximity(ProximityAlgorithm<N>)
}

/// A graph where nodes are collision objects and edges are contact or proximity algorithms.
pub struct InteractionGraph<N: Real> {
    pub graph: DiGraph<CollisionObjectHandle, Interaction<N>, usize>
}

impl<N: Real> InteractionGraph<N> {
    /// Creates a new empty collection of collision objects.
    pub fn new() -> Self {
        InteractionGraph {
            graph: DiGraph::with_capacity(10, 10),
        }
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
}