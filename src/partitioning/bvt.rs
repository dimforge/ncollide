//! A read-only Bounding Volume Tree.

use alga::general::Real;
use bounding_volume::BoundingVolume;
use math::{DIM, Point};
use partitioning::BVH;
use std::collections::VecDeque;
use utils;

/// A Bounding Volume Tree.
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
    deformed_nodes: VecDeque<BVTNodeId>,
}

/// The identifier of a BVT node.
#[derive(Copy, Clone, Hash, PartialEq, Eq)]
pub enum BVTNodeId {
    /// Identifier of an internal node.
    Internal(usize),
    /// Identifier of a leaf node.
    Leaf(usize),
}

#[derive(Clone)]
struct BVTInternal<BV> {
    bounding_volume: BV,
    right: BVTNodeId,
    left: BVTNodeId,
}

#[derive(Clone)]
struct BVTLeaf<T, BV> {
    bounding_volume: BV,
    data: T,
}

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
    // FIXME: add higher level constructors ?
    /// Builds a bounding volume tree using an user-defined construction function.
    pub fn new_with_partitioner<F: FnMut(usize, Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>)>(
        elements: Vec<(T, BV)>,
        partitioner: &mut F,
    ) -> BVT<T, BV> {
        if elements.len() == 0 {
            BVT {
                root: BVTNodeId::Leaf(0),
                internals: Vec::new(),
                leaves: Vec::new(),
                deformation_timestamp: 0,
                deformation_infos: Vec::new(),
                deformed_nodes: VecDeque::new(),
            }
        } else {
            let mut internals = Vec::new();
            let mut leaves = Vec::new();
            let root = Self::_new_with_partitioner(0, elements, &mut internals, &mut leaves, partitioner);
            internals.shrink_to_fit();
            leaves.shrink_to_fit();

            BVT {
                root,
                internals,
                leaves,
                deformation_timestamp: 0,
                deformation_infos: Vec::new(),
                deformed_nodes: VecDeque::new(),
            }
        }
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
}

impl<T, BV> BVT<T, BV> {
    /// Creates a balanced `BVT`.
    pub fn new_balanced<N>(leaves: Vec<(T, BV)>) -> BVT<T, BV>
        where
            N: Real,
            BV: BoundingVolume<N> + Clone,
    {
        BVT::new_with_partitioner(leaves, &mut Self::median_partitioner)
    }

    /// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
    pub fn median_partitioner_with_centers<N, F: FnMut(&T, &BV) -> Point<N>>(
        depth: usize,
        leaves: Vec<(T, BV)>,
        center: &mut F,
    ) -> (BV, BinaryPartition<T, BV>)
        where
            N: Real,
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

    /// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
    pub fn median_partitioner<N>(depth: usize, leaves: Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>)
        where
            N: Real,
            BV: BoundingVolume<N> + Clone,
    {
        Self::median_partitioner_with_centers(depth, leaves, &mut |_, bv| bv.center())
    }

    fn _new_with_partitioner<F: FnMut(usize, Vec<(T, BV)>) -> (BV, BinaryPartition<T, BV>)>(
        depth: usize,
        leaves: Vec<(T, BV)>,
        out_internals: &mut Vec<BVTInternal<BV>>,
        out_leaves: &mut Vec<BVTLeaf<T, BV>>,
        partitioner: &mut F,
    ) -> BVTNodeId {
        let (bv, partitions) = partitioner(depth, leaves);

        match partitions {
            BinaryPartition::Part(b) => {
                out_leaves.push(BVTLeaf { bounding_volume: bv, data: b });
                BVTNodeId::Leaf(out_leaves.len() - 1)
            }
            BinaryPartition::Parts(left, right) => {
                let left = Self::_new_with_partitioner(depth + 1, left, out_internals, out_leaves, partitioner);
                let right = Self::_new_with_partitioner(depth + 1, right, out_internals, out_leaves, partitioner);
                out_internals.push(BVTInternal { bounding_volume: bv, left, right });
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
            BVTNodeId::Leaf(_) => 0
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
            BVTNodeId::Leaf(_) => panic!("DBVT child index out of bounds.")
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