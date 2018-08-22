use slab::Slab;
use std::ops::Index;

use na::{self, Real};

use bounding_volume::BoundingVolume;
use math::Point;
use partitioning::BVTVisitor;

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

#[derive(Copy, Clone, Debug, Hash)]
enum DBVTNodeId {
    Leaf(usize),
    Internal(usize),
}

/// A boundin volume hierarchy on which objects can be added or removed after construction.
pub struct DBVT<N: Real, B, BV> {
    root: DBVTNodeId,
    leaves: Slab<DBVTLeaf<N, B, BV>>,
    internals: Slab<DBVTInternal<N, BV>>,
}

/// Leaf of a Dynamic Bounding Volume Tree.
#[derive(Clone)]
pub struct DBVTLeaf<N: Real, B, BV> {
    /// The bounding volume of this node.
    pub bounding_volume: BV,
    /// The center of this node bounding volume.
    pub center: Point<N>,
    /// An user-defined data.
    pub data: B,
    /// This node parent.
    parent: DBVTInternalId,
}

/// Internal node of a DBVT. An internal node always has two children.
struct DBVTInternal<N: Real, BV> {
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

impl<N: Real, B, BV: BoundingVolume<N>> DBVTLeaf<N, B, BV> {
    /// Creates a new DBVT leaf from its bounding volume and contained data.
    pub fn new(bounding_volume: BV, data: B) -> DBVTLeaf<N, B, BV> {
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

impl<N: Real, BV: BoundingVolume<N>> DBVTInternal<N, BV> {
    /// Creates a new internal node.
    fn new(
        bounding_volume: BV,
        parent: DBVTInternalId,
        left: DBVTNodeId,
        right: DBVTNodeId,
    ) -> DBVTInternal<N, BV> {
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

impl<N: Real, B, BV: BoundingVolume<N>> DBVT<N, B, BV> {
    /// Creates a new empty dynamic bonding volume hierarchy.
    pub fn new() -> DBVT<N, B, BV> {
        DBVT {
            root: DBVTNodeId::Leaf(0),
            leaves: Slab::new(),
            internals: Slab::new(),
        }
    }

    /// Indicates whether this DBVT empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.leaves.is_empty()
    }

    /// Inserts a leaf into this DBVT.
    pub fn insert(&mut self, leaf: DBVTLeaf<N, B, BV>) -> DBVTLeafId {
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

    /// Returns an interference iterator for the given bounding volume.
    pub fn interferences_with_bounding_volume(&self, bv: BV) -> DBVTBVIterator<N, B, BV> {
        DBVTBVIterator{
            bv: bv,
            node: vec![self.root],
            tree: &self,
        }
    }

    /// Removes a leaf from this DBVT.
    ///
    /// Panics if the provided leaf is not attached to this DBVT.
    pub fn remove(&mut self, leaf_id: DBVTLeafId) -> DBVTLeaf<N, B, BV> {
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

    /// Traverses this tree using an object implementing the `BVTVisitor`trait.
    ///
    /// This will traverse the whole tree and call the visitor `.visit_internal(...)` (resp.
    /// `.visit_leaf(...)`) method on each internal (resp. leaf) node.
    pub fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        if !self.is_empty() {
            self.visit_node(visitor, self.root);
        }
    }

    fn visit_node<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis, node: DBVTNodeId) {
        match node {
            DBVTNodeId::Internal(i) => {
                let internal = &self.internals[i];
                if visitor.visit_internal(&internal.bounding_volume) {
                    self.visit_node(visitor, internal.left);
                    self.visit_node(visitor, internal.right);
                }
            }
            DBVTNodeId::Leaf(i) => {
                let leaf = &self.leaves[i];
                visitor.visit_leaf(&leaf.data, &leaf.bounding_volume);
            }
        }
    }
}

impl<N: Real, B, BV> Index<DBVTLeafId> for DBVT<N, B, BV> {
    type Output = DBVTLeaf<N, B, BV>;

    #[inline]
    fn index(&self, DBVTLeafId(id): DBVTLeafId) -> &Self::Output {
        &self.leaves[id]
    }
}

/// Bounding Volume iterator over a dynamic bounding volume tree.
pub struct DBVTBVIterator<'a, N: Real, B: 'a, BV: 'a> {
    bv: BV,
    node: Vec<DBVTNodeId>,
    tree: &'a DBVT<N, B, BV>,
}

impl <'a, N: Real, B: Copy, BV: BoundingVolume<N>> Iterator for DBVTBVIterator<'a, N, B, BV> {
    type Item = B;

    fn next(&mut self) -> Option<B> {
        while let Some(node) = self.node.pop() {
            match node {
                DBVTNodeId::Internal(i) => {
                    let internal = &self.tree.internals[i];
                    if self.bv.intersects(&internal.bounding_volume) {
                        self.node.push(internal.left);
                        self.node.push(internal.right);
                    }
                }
                DBVTNodeId::Leaf(i) => {
                    let leaf = &self.tree.leaves[i];
                    return Some(leaf.data);
                }
            }
        }

        None
    }
}
