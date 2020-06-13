//! A read-only Bounding Volume Tree.

use crate::bounding_volume::BoundingVolume;
use crate::math::{Point, DIM};
use crate::partitioning::BVH;
use crate::utils;
use simba::scalar::RealField;
use std::collections::VecDeque;
use std::iter;
use std::usize;

/// A Bounding Volume Tree.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BVT<T, BV> {
    root: BVTNodeId,
    internals: Vec<BVTInternal<BV>>,
    leaves: Vec<BVTLeaf<T, BV>>,
    // This will be filled only when at least one
    // deformation occurred to avoid memory usage
    // that are not needed in the general case.
    deformation_timestamp: usize,
    deformation_infos: Vec<BVTDeformationInfo>,
    // Infos for leaves are stored starting at the index self.internals.len().
    parents_to_update: VecDeque<usize>,
}

/// The identifier of a BVT node.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub enum BVTNodeId {
    /// Identifier of an internal node.
    Internal(usize),
    /// Identifier of a leaf node.
    Leaf(usize),
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
struct BVTInternal<BV> {
    bounding_volume: BV,
    right: BVTNodeId,
    left: BVTNodeId,
}

/// A leaf of the BVT.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BVTLeaf<T, BV> {
    bounding_volume: BV,
    data: T,
}

impl<T, BV> BVTLeaf<T, BV> {
    /// The bounding volume stored on this leaf.
    #[inline]
    pub fn bounding_volume(&self) -> &BV {
        &self.bounding_volume
    }

    /// The user-data stored on this leaf.
    #[inline]
    pub fn data(&self) -> &T {
        &self.data
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
struct BVTDeformationInfo {
    parent: usize,
    timestamp: usize,
}

/// Result of a binary partition.
pub enum BinaryPartition<T, BV> {
    /// Result of the partitioning of one element.
    Part(T),
    /// Result of the partitioning of several elements.
    Parts(Vec<(T, BV)>, Vec<(T, BV)>),
}

impl<T, BV> BVT<T, BV> {
    /// Builds a bounding volume tree using the specified partitioning function.
    #[deprecated(note = "please use `from_partitioning` instead")]
    pub fn new_with_partitioning<F: FnMut(usize, Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>)>(
        elements: Vec<(T, BV)>,
        partitioning: &mut F,
    ) -> BVT<T, BV> {
        Self::from_partitioning(elements, partitioning)
    }

    // FIXME: add higher level constructors ?
    /// Builds a bounding volume tree using the specified partitioning function.
    pub fn from_partitioning(
        elements: Vec<(T, BV)>,
        partitioning: &mut impl FnMut(usize, Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>),
    ) -> BVT<T, BV> {
        if elements.len() == 0 {
            BVT {
                root: BVTNodeId::Leaf(0),
                internals: Vec::new(),
                leaves: Vec::new(),
                deformation_timestamp: 1,
                deformation_infos: Vec::new(),
                parents_to_update: VecDeque::new(),
            }
        } else {
            let mut internals = Vec::new();
            let mut leaves = Vec::new();
            let root =
                Self::_from_partitioning(0, elements, &mut internals, &mut leaves, partitioning);
            internals.shrink_to_fit();
            leaves.shrink_to_fit();

            BVT {
                root,
                internals,
                leaves,
                deformation_timestamp: 1,
                deformation_infos: Vec::new(),
                parents_to_update: VecDeque::new(),
            }
        }
    }

    /// The set of leaves on this BVT.
    #[inline]
    pub fn leaves(&self) -> &[BVTLeaf<T, BV>] {
        &self.leaves
    }

    /// Referenceto the i-th leaf of this BVT.
    #[inline]
    pub fn leaf(&self, i: usize) -> &BVTLeaf<T, BV> {
        &self.leaves[i]
    }

    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume(&self) -> Option<&BV> {
        if self.leaves.is_empty() {
            return None;
        }

        match self.root {
            BVTNodeId::Leaf(i) => Some(&self.leaves[i].bounding_volume),
            BVTNodeId::Internal(i) => Some(&self.internals[i].bounding_volume),
        }
    }

    /// Set the bounding volume of the i-th leaf.
    ///
    /// If `refit_now` is `true`, the bounding volumes of all the ancestors of the
    /// modifiad leaf will be updated as well to enclose the new leaf bounding volume.
    /// If `refit_now` is `false`, no ancestor update will be performed until the
    /// `.refit()` method is called. This is useful to refit the tree only once after
    /// several leaf bounding volume modifications.
    pub fn set_leaf_bounding_volume<N: RealField>(&mut self, i: usize, bv: BV, refit_now: bool)
    where
        BV: BoundingVolume<N>,
    {
        self.init_deformation_infos();
        self.leaves[i].bounding_volume = bv;

        if refit_now {
            let mut curr = self.deformation_infos[self.internals.len() + i].parent;

            while curr != usize::max_value() {
                let new_bv = match (self.internals[curr].left, self.internals[curr].right) {
                    (BVTNodeId::Internal(i), BVTNodeId::Internal(j)) => self.internals[i]
                        .bounding_volume
                        .merged(&self.internals[j].bounding_volume),
                    (BVTNodeId::Internal(i), BVTNodeId::Leaf(j)) => self.internals[i]
                        .bounding_volume
                        .merged(&self.leaves[j].bounding_volume),
                    (BVTNodeId::Leaf(i), BVTNodeId::Internal(j)) => self.leaves[i]
                        .bounding_volume
                        .merged(&self.internals[j].bounding_volume),
                    (BVTNodeId::Leaf(i), BVTNodeId::Leaf(j)) => self.leaves[i]
                        .bounding_volume
                        .merged(&self.leaves[j].bounding_volume),
                };
                self.internals[curr].bounding_volume = new_bv;
                curr = self.deformation_infos[curr].parent;
            }
        } else {
            if self.leaves.len() != 1 {
                self.parents_to_update
                    .push_back(self.deformation_infos[self.internals.len() + i].parent)
            }
        }
    }

    /// Refits the bounding volumes so that all node of the BVT have boundin volumes that enclose their children.
    ///
    /// This must be called to ensure the BVT is in a valid state after several calls to
    /// `.set_leaf_bounding_volume(_, _, false)`.
    /// Every bounding volume created during this update will be enlarged by a margin of `margin`.
    /// The larger this margin here, the looser will the resulting AABB will be, but the less frequent
    /// future updates will be necessary.
    /// Setting a margin equal to 0.0 is allowed.
    pub fn refit<N: RealField>(&mut self, margin: N)
    where
        BV: BoundingVolume<N>,
    {
        assert!(margin >= N::zero(), "Cannot set a negative margin.");

        self.deformation_timestamp += 1;

        while let Some(curr) = self.parents_to_update.pop_front() {
            let infos = &mut self.deformation_infos[curr];
            if infos.timestamp < self.deformation_timestamp {
                // This node has not been updated yet.
                infos.timestamp = self.deformation_timestamp;

                let mut new_bv = match (self.internals[curr].left, self.internals[curr].right) {
                    (BVTNodeId::Internal(i), BVTNodeId::Internal(j)) => self.internals[i]
                        .bounding_volume
                        .merged(&self.internals[j].bounding_volume),
                    (BVTNodeId::Internal(i), BVTNodeId::Leaf(j)) => self.internals[i]
                        .bounding_volume
                        .merged(&self.leaves[j].bounding_volume),
                    (BVTNodeId::Leaf(i), BVTNodeId::Internal(j)) => self.leaves[i]
                        .bounding_volume
                        .merged(&self.internals[j].bounding_volume),
                    (BVTNodeId::Leaf(i), BVTNodeId::Leaf(j)) => self.leaves[i]
                        .bounding_volume
                        .merged(&self.leaves[j].bounding_volume),
                };

                if !self.internals[curr].bounding_volume.contains(&new_bv) {
                    if !margin.is_zero() {
                        new_bv.loosen(margin)
                    }

                    self.internals[curr].bounding_volume = new_bv;

                    if infos.parent != usize::max_value() {
                        // Push the parent if it is not the root.
                        self.parents_to_update.push_back(infos.parent);
                    }
                }
            }
        }
    }

    fn init_deformation_infos(&mut self) {
        if self.deformation_infos.is_empty() {
            self.deformation_infos = iter::repeat(BVTDeformationInfo {
                parent: usize::max_value(),
                timestamp: 0,
            })
            .take(self.internals.len() + self.leaves.len())
            .collect();

            for (i, internal) in self.internals.iter().enumerate() {
                match internal.left {
                    BVTNodeId::Internal(j) => self.deformation_infos[j].parent = i,
                    BVTNodeId::Leaf(j) => {
                        self.deformation_infos[self.internals.len() + j].parent = i
                    }
                }

                match internal.right {
                    BVTNodeId::Internal(j) => self.deformation_infos[j].parent = i,
                    BVTNodeId::Leaf(j) => {
                        self.deformation_infos[self.internals.len() + j].parent = i
                    }
                }
            }
        }
    }
}

impl<T, BV> BVT<T, BV> {
    /// Creates a balanced `BVT`.
    pub fn new_balanced<N>(leaves: Vec<(T, BV)>) -> BVT<T, BV>
    where
        N: RealField,
        BV: BoundingVolume<N> + Clone,
    {
        BVT::from_partitioning(leaves, &mut Self::median_partitioning)
    }

    /// Construction function for a kdree to be used with `BVT::from_partitioning`.
    pub fn median_partitioning_with_centers<N, F: FnMut(&T, &BV) -> Point<N>>(
        depth: usize,
        leaves: Vec<(T, BV)>,
        center: &mut F,
    ) -> (BV, BinaryPartition<T, BV>)
    where
        N: RealField,
        BV: BoundingVolume<N> + Clone,
    {
        if leaves.len() == 0 {
            panic!("Cannot build a tree without leaves.");
        } else if leaves.len() == 1 {
            let (b, bv) = leaves.into_iter().next().unwrap();
            (bv, BinaryPartition::Part(b))
        } else {
            let sep_axis = depth % DIM;

            // compute the median along sep_axis
            let mut median = Vec::new();

            for l in leaves.iter() {
                let c = (*center)(&l.0, &l.1);
                median.push(c[sep_axis]);
            }

            let median = utils::median(&mut median[..]);

            // build the partitions
            let mut right = Vec::new();
            let mut left = Vec::new();
            let mut bounding_bounding_volume = leaves[0].1.clone();

            let mut insert_left = false;

            for (b, bv) in leaves.into_iter() {
                bounding_bounding_volume.merge(&bv);

                let pos = (*center)(&b, &bv)[sep_axis];

                if pos < median || (pos == median && insert_left) {
                    left.push((b, bv));
                    insert_left = false;
                } else {
                    right.push((b, bv));
                    insert_left = true;
                }
            }

            // XXX: hack to avoid degeneracies.
            if left.len() == 0 {
                left.push(right.pop().unwrap());
            } else if right.len() == 0 {
                right.push(left.pop().unwrap());
            }

            (
                bounding_bounding_volume,
                BinaryPartition::Parts(left, right),
            )
        }
    }

    /// Construction function for a kdree to be used with `BVT::from_partitioning`.
    pub fn median_partitioning<N>(
        depth: usize,
        leaves: Vec<(T, BV)>,
    ) -> (BV, BinaryPartition<T, BV>)
    where
        N: RealField,
        BV: BoundingVolume<N> + Clone,
    {
        Self::median_partitioning_with_centers(depth, leaves, &mut |_, bv| bv.center())
    }

    fn _from_partitioning<F: FnMut(usize, Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>)>(
        depth: usize,
        leaves: Vec<(T, BV)>,
        out_internals: &mut Vec<BVTInternal<BV>>,
        out_leaves: &mut Vec<BVTLeaf<T, BV>>,
        partitioning: &mut F,
    ) -> BVTNodeId {
        let (bv, partitions) = partitioning(depth, leaves);

        match partitions {
            BinaryPartition::Part(b) => {
                out_leaves.push(BVTLeaf {
                    bounding_volume: bv,
                    data: b,
                });
                BVTNodeId::Leaf(out_leaves.len() - 1)
            }
            BinaryPartition::Parts(left, right) => {
                let left = Self::_from_partitioning(
                    depth + 1,
                    left,
                    out_internals,
                    out_leaves,
                    partitioning,
                );
                let right = Self::_from_partitioning(
                    depth + 1,
                    right,
                    out_internals,
                    out_leaves,
                    partitioning,
                );
                out_internals.push(BVTInternal {
                    bounding_volume: bv,
                    left,
                    right,
                });
                BVTNodeId::Internal(out_internals.len() - 1)
            }
        }
    }
}

impl<'a, T, BV> BVH<T, BV> for BVT<T, BV> {
    type Node = BVTNodeId;

    fn root(&self) -> Option<Self::Node> {
        if self.leaves.len() != 0 {
            Some(self.root)
        } else {
            None
        }
    }

    fn num_children(&self, node: Self::Node) -> usize {
        match node {
            BVTNodeId::Internal(_) => 2,
            BVTNodeId::Leaf(_) => 0,
        }
    }

    fn child(&self, i: usize, node: Self::Node) -> Self::Node {
        match node {
            BVTNodeId::Internal(node_id) => {
                if i == 0 {
                    self.internals[node_id].left
                } else {
                    self.internals[node_id].right
                }
            }
            BVTNodeId::Leaf(_) => panic!("DBVT child index out of bounds."),
        }
    }

    fn content(&self, node: Self::Node) -> (&BV, Option<&T>) {
        match node {
            BVTNodeId::Internal(i) => {
                let node = &self.internals[i];
                (&node.bounding_volume, None)
            }
            BVTNodeId::Leaf(i) => {
                let node = &self.leaves[i];
                (&node.bounding_volume, Some(&node.data))
            }
        }
    }
}
