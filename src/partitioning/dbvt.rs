use crate::bounding_volume::BoundingVolume;
use crate::math::Point;
use na::{self, RealField};
use crate::partitioning::BVH;
use slab::Slab;
use std::ops::Index;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
/// The unique identifier of a DBVT leaf.
pub struct DBVTLeafId(usize);

impl DBVTLeafId {
    /// Creates an invalid identifier.
    #[inline]
    pub fn new_invalid() -> Self {
        DBVTLeafId(usize::max_value())
    }

    /// Checkis if this identifier is invalid.
    #[inline]
    pub fn is_invalid(&self) -> bool {
        let DBVTLeafId(val) = *self;
        val == usize::max_value()
    }
}

#[derive(Copy, Clone, Debug)]
enum UpdateStatus {
    NeedsShrink,
    UpToDate,
}

#[derive(Copy, Clone, Debug, Hash)]
enum DBVTInternalId {
    RightChildOf(usize),
    LeftChildOf(usize),
    Root,
}

/// The identifier of a node of the DBVT.
#[derive(Copy, Clone, Debug, Hash)]
pub enum DBVTNodeId {
    Leaf(usize),
    Internal(usize),
}

/// A bounding volume hierarchy on which objects can be added or removed after construction.
#[derive(Clone)]
pub struct DBVT<N: RealField, T, BV> {
    root: DBVTNodeId,
    leaves: Slab<DBVTLeaf<N, T, BV>>,
    internals: Slab<DBVTInternal<N, BV>>,
}

/// Leaf of a Dynamic Bounding Volume Tree.
#[derive(Clone)]
pub struct DBVTLeaf<N: RealField, T, BV> {
    /// The bounding volume of this node.
    pub bounding_volume: BV,
    /// The center of this node bounding volume.
    pub center: Point<N>,
    /// An user-defined data.
    pub data: T,
    /// This node parent.
    parent: DBVTInternalId,
}

/// Internal node of a DBVT. An internal node always has two children.
#[derive(Clone)]
struct DBVTInternal<N: RealField, BV> {
    /// The bounding volume of this node. It always encloses both its children bounding volumes.
    bounding_volume: BV,
    /// The center of this node bounding volume.
    center: Point<N>,
    /// This node left child.
    left: DBVTNodeId,
    /// This node right child.
    right: DBVTNodeId,
    /// This node parent.
    parent: DBVTInternalId,

    state: UpdateStatus,
}

impl<N: RealField, T, BV: BoundingVolume<N>> DBVTLeaf<N, T, BV> {
    /// Creates a new DBVT leaf from its bounding volume and contained data.
    pub fn new(bounding_volume: BV, data: T) -> DBVTLeaf<N, T, BV> {
        DBVTLeaf {
            center: bounding_volume.center(),
            bounding_volume: bounding_volume,
            data: data,
            parent: DBVTInternalId::Root,
        }
    }

    /// Returns `true` if this leaf is the root of the tree, or if it detached from any tree.
    pub fn is_root(&self) -> bool {
        match self.parent {
            DBVTInternalId::Root => true,
            _ => false,
        }
    }
}

impl<N: RealField, BV: BoundingVolume<N>> DBVTInternal<N, BV> {
    /// Creates a new internal node.
    fn new(
        bounding_volume: BV,
        parent: DBVTInternalId,
        left: DBVTNodeId,
        right: DBVTNodeId,
    ) -> DBVTInternal<N, BV>
    {
        DBVTInternal {
            center: bounding_volume.center(),
            bounding_volume: bounding_volume,
            left: left,
            right: right,
            parent: parent,
            state: UpdateStatus::UpToDate,
        }
    }
}

impl<N: RealField, T, BV: BoundingVolume<N>> DBVT<N, T, BV> {
    /// Creates a new empty dynamic bonding volume hierarchy.
    pub fn new() -> DBVT<N, T, BV> {
        DBVT {
            root: DBVTNodeId::Leaf(0),
            leaves: Slab::new(),
            internals: Slab::new(),
        }
    }

    /// The bounding volume of the root of this DBVT.
    ///
    /// Returns `None` if the DBVT is empty.
    #[inline]
    pub fn root_bounding_volume(&self) -> Option<&BV> {
        if self.leaves.len() == 0 {
            return None;
        }

        match self.root {
            DBVTNodeId::Leaf(i) => Some(&self.leaves[i].bounding_volume),
            DBVTNodeId::Internal(i) => Some(&self.internals[i].bounding_volume),
        }
    }

    /// Indicates whether this DBVT empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.leaves.is_empty()
    }

    /// Inserts a leaf into this DBVT.
    pub fn insert(&mut self, leaf: DBVTLeaf<N, T, BV>) -> DBVTLeafId {
        if self.is_empty() {
            let new_id = self.leaves.insert(leaf);
            self.leaves[new_id].parent = DBVTInternalId::Root;
            self.root = DBVTNodeId::Leaf(new_id);

            return DBVTLeafId(new_id);
        }

        match self.root {
            DBVTNodeId::Internal(_) => {
                let mut curr = self.root;

                loop {
                    match curr {
                        DBVTNodeId::Internal(id) => {
                            // FIXME: we could avoid the systematic merge
                            let (left, right) = {
                                let node = &mut self.internals[id];
                                node.bounding_volume.merge(&leaf.bounding_volume);
                                (node.left, node.right)
                            };

                            let dist1 = match left {
                                DBVTNodeId::Leaf(l) => {
                                    na::distance_squared(&self.leaves[l].center, &leaf.center)
                                }
                                DBVTNodeId::Internal(i) => {
                                    na::distance_squared(&self.internals[i].center, &leaf.center)
                                }
                            };

                            let dist2 = match right {
                                DBVTNodeId::Leaf(l) => {
                                    na::distance_squared(&self.leaves[l].center, &leaf.center)
                                }
                                DBVTNodeId::Internal(i) => {
                                    na::distance_squared(&self.internals[i].center, &leaf.center)
                                }
                            };

                            curr = if dist1 < dist2 { left } else { right };
                        }
                        DBVTNodeId::Leaf(id) => {
                            let parent_bv = self.leaves[id]
                                .bounding_volume
                                .merged(&leaf.bounding_volume);
                            let grand_parent = self.leaves[id].parent;

                            let new_id = self.leaves.insert(leaf);
                            let parent = DBVTInternal::new(
                                parent_bv,
                                grand_parent,
                                curr,
                                DBVTNodeId::Leaf(new_id),
                            );
                            let parent_id = self.internals.insert(parent);
                            self.leaves[id].parent = DBVTInternalId::LeftChildOf(parent_id);
                            self.leaves[new_id].parent = DBVTInternalId::RightChildOf(parent_id);

                            match grand_parent {
                                DBVTInternalId::LeftChildOf(pp) => {
                                    self.internals[pp].left = DBVTNodeId::Internal(parent_id)
                                }
                                DBVTInternalId::RightChildOf(pp) => {
                                    self.internals[pp].right = DBVTNodeId::Internal(parent_id)
                                }
                                _ => unreachable!(),
                            }

                            break DBVTLeafId(new_id);
                        }
                    }
                }
            }
            DBVTNodeId::Leaf(id) => {
                let new_id = self.leaves.insert(leaf);

                // Create a common parent which is the new root.
                let root_bv = self.leaves[id]
                    .bounding_volume
                    .merged(&self.leaves[new_id].bounding_volume);
                let root = DBVTInternal::new(
                    root_bv,
                    DBVTInternalId::Root,
                    DBVTNodeId::Leaf(id),
                    DBVTNodeId::Leaf(new_id),
                );

                let root_id = self.internals.insert(root);
                self.leaves[id].parent = DBVTInternalId::LeftChildOf(root_id);
                self.leaves[new_id].parent = DBVTInternalId::RightChildOf(root_id);
                self.root = DBVTNodeId::Internal(root_id);

                DBVTLeafId(new_id)
            }
        }
    }

    /// Removes a leaf from this DBVT.
    ///
    /// Panics if the provided leaf is not attached to this DBVT.
    pub fn remove(&mut self, leaf_id: DBVTLeafId) -> DBVTLeaf<N, T, BV> {
        let DBVTLeafId(leaf_id) = leaf_id;
        let leaf = self.leaves.remove(leaf_id);

        if !leaf.is_root() {
            let p;
            let other;

            match leaf.parent {
                DBVTInternalId::RightChildOf(parent) => {
                    other = self.internals[parent].left;
                    p = parent;
                }
                DBVTInternalId::LeftChildOf(parent) => {
                    other = self.internals[parent].right;
                    p = parent;
                }
                DBVTInternalId::Root => unreachable!(),
            }

            match self.internals[p].parent {
                DBVTInternalId::RightChildOf(pp) => {
                    match other {
                        DBVTNodeId::Internal(id) => {
                            self.internals[id].parent = DBVTInternalId::RightChildOf(pp)
                        }
                        DBVTNodeId::Leaf(id) => {
                            self.leaves[id].parent = DBVTInternalId::RightChildOf(pp)
                        }
                    }

                    self.internals[pp].right = other;
                    self.internals[pp].state = UpdateStatus::NeedsShrink;
                }
                DBVTInternalId::LeftChildOf(pp) => {
                    match other {
                        DBVTNodeId::Internal(id) => {
                            self.internals[id].parent = DBVTInternalId::LeftChildOf(pp)
                        }
                        DBVTNodeId::Leaf(id) => {
                            self.leaves[id].parent = DBVTInternalId::LeftChildOf(pp)
                        }
                    }

                    self.internals[pp].left = other;
                    self.internals[pp].state = UpdateStatus::NeedsShrink;
                }
                DBVTInternalId::Root => {
                    // The root changes to the other child.
                    match other {
                        DBVTNodeId::Leaf(id) => self.leaves[id].parent = DBVTInternalId::Root,
                        DBVTNodeId::Internal(id) => {
                            self.internals[id].parent = DBVTInternalId::Root
                        }
                    }

                    self.root = other;
                }
            }

            let _ = self.internals.remove(p);
        } else {
            // The tree is now empty.
            self.leaves.clear();
            self.internals.clear();
        }

        leaf
    }

    /// Gets the given leaf if it exists.
    #[inline]
    pub fn get(&self, DBVTLeafId(id): DBVTLeafId) -> Option<&DBVTLeaf<N, T, BV>> {
        self.leaves.get(id)
    }
}

impl<N: RealField, T, BV> Index<DBVTLeafId> for DBVT<N, T, BV> {
    type Output = DBVTLeaf<N, T, BV>;

    #[inline]
    fn index(&self, DBVTLeafId(id): DBVTLeafId) -> &Self::Output {
        &self.leaves[id]
    }
}

impl<'a, N: RealField, T, BV> BVH<T, BV> for DBVT<N, T, BV> {
    type Node = DBVTNodeId;

    fn root(&self) -> Option<Self::Node> {
        if self.leaves.len() != 0 {
            Some(self.root)
        } else {
            None
        }
    }

    fn num_children(&self, node: Self::Node) -> usize {
        match node {
            DBVTNodeId::Internal(_) => 2,
            DBVTNodeId::Leaf(_) => 0,
        }
    }

    fn child(&self, i: usize, node: Self::Node) -> Self::Node {
        match node {
            DBVTNodeId::Internal(node_id) => {
                if i == 0 {
                    self.internals[node_id].left
                } else {
                    self.internals[node_id].right
                }
            }
            DBVTNodeId::Leaf(_) => panic!("DBVT child index out of bounds."),
        }
    }

    fn content(&self, node: Self::Node) -> (&BV, Option<&T>) {
        match node {
            DBVTNodeId::Internal(i) => {
                let node = &self.internals[i];
                (&node.bounding_volume, None)
            }
            DBVTNodeId::Leaf(i) => {
                let node = &self.leaves[i];
                (&node.bounding_volume, Some(&node.data))
            }
        }
    }
}
