use petgraph::stable_graphStableDiGraph;
use petgraph::graph::NodeIndex;
use na::Real;

pub struct CollisionObjectHandle(NodeIndex<usize>);
pub struct CollisionObjectGraphIndex(usize);
pub enum ContactOrProximity<N: Real> {
    Contact(ContactAlgorithm<N>, ContactManifold<N>),
    Proximity(ContactAlgorithm<N>)
}

pub struct CollisionObjectGraph<N: Real, T>(StableDiGraph<CollisionObject<N, T>, ContactOrProximity<N>, usize>);

impl<N: Real, T> CollisionObjectGraph<N, T> {
    /// Creates a new empty collection of collision objects.
    pub fn new() -> CollisionObjectGraph<N, T> {
        Self(StableDiGraph::new())
    }

    /// Inserts a new collision object into this collection and returns the corresponding handle.
    #[inline]
    pub fn insert(&mut self, co: CollisionObject<N, T>) -> CollisionObjectHandle {
        CollisionObjectHandle(self.0.add_node(co))
    }

    /// Removes from this collection the collision object identified by the given handle.
    ///
    /// The removed collision object structure is returned.
    #[inline]
    pub fn remove(&mut self, handle: CollisionObjectHandle) -> Option<CollisionObject<N, T>> {
        self.0.remove_node(handle.0)
    }

    /// If it exists, retrieves a reference to the collision object identified by the given handle.
    #[inline]
    pub fn get(&self, handle: CollisionObjectHandle) -> Option<&CollisionObject<N, T>> {
        self.0.node_weight(handle.0)
    }

    /// If it exists, retrieves a mutable reference to the collision object identified by the given handle.
    #[inline]
    pub fn get_mut(&mut self, handle: CollisionObjectHandle) -> Option<&mut CollisionObject<N, T>> {
        self.objects.get_mut(handle.0)
    }

    /// Returns `true` if the specified handle identifies a collision object stored in this collection.
    #[inline]
    pub fn contains(&self, handle: CollisionObjectHandle) -> bool {
        self.objects.contains(handle.0)
    }

    /// Retrieves an iterator yielding references to each collision object.
    #[inline]
    pub fn iter(&self) -> CollisionObjects<N, T> {
        CollisionObjects {
            iter: self.objects.iter(),
        }
    }

    /// The number of collision objects on this slab.
    #[inline]
    pub fn len(&self) -> usize {
        self.objects.len()
    }
}