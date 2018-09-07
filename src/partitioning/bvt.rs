//! A read-only Bounding Volume Tree.

use alga::general::Real;
use bounding_volume::BoundingVolume;
use math::{DIM, Point};
use na;
use partitioning::{self, BestFirstVisitor, BVH, SimultaneousVisitor, Visitor};
use utils;

/// A Bounding Volume Tree.
#[derive(Clone)]
pub struct BVT<T, BV> {
    tree: Option<BVTNode<T, BV>>,
}

/// A node of the bounding volume tree.
#[derive(Clone)]
pub enum BVTNode<T, BV> {
    // XXX: give a faster access to the BV
    /// An internal node.
    Internal(BV, Box<BVTNode<T, BV>>, Box<BVTNode<T, BV>>),
    /// A leaf.
    Leaf(BV, T),
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
        leaves: Vec<(T, BV)>,
        partitioner: &mut F,
    ) -> BVT<T, BV> {
        if leaves.len() == 0 {
            BVT { tree: None }
        } else {
            BVT {
                tree: Some(Self::_new_with_partitioner(0, leaves, partitioner)),
            }
        }
    }

    /// Traverses this tree using a visitor.
    pub fn visit<Vis: Visitor<T, BV>>(&self, visitor: &mut Vis) {
        // FIXME: preallocate the vec?
        partitioning::visit(&self, visitor, &mut Vec::new())
    }

    /// Visits the bounding volume traversal tree implicitly formed with `other`.
    pub fn visit_bvtt<Vis: SimultaneousVisitor<T, BV>>(&self, other: &BVT<T, BV>, visitor: &mut Vis) {
        // FIXME: preallocate the vec?
        partitioning::simultaneous_visit(&self, &other, visitor, &mut Vec::new())
    }

    // FIXME: really return a ref to T ?
    /// Performs a best-fist-search on the tree.
    ///
    /// Returns the content of the leaf with the smallest associated cost, and a result of
    /// user-defined type.
    pub fn best_first_search<'a, N, BFS>(&'a self, algorithm: &mut BFS) -> Option<BFS::Result>
        where
            N: Real,
            BFS: BestFirstVisitor<N, T, BV>,
    {
        partitioning::best_first_visit(&self, algorithm)
    }

    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume<'r>(&'r self) -> Option<&'r BV> {
        match self.tree {
            Some(ref n) => match *n {
                BVTNode::Internal(ref bv, _, _) => Some(bv),
                BVTNode::Leaf(ref bv, _) => Some(bv),
            },
            None => None,
        }
    }

    /// Computes the depth of this tree.
    pub fn depth(&self) -> usize {
        match self.tree {
            Some(ref n) => n.depth(),
            None => 0,
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
        partitioner: &mut F,
    ) -> BVTNode<T, BV> {
        let (bv, partitions) = partitioner(depth, leaves);

        match partitions {
            BinaryPartition::Part(b) => BVTNode::Leaf(bv, b),
            BinaryPartition::Parts(left, right) => {
                let left = Self::_new_with_partitioner(depth + 1, left, partitioner);
                let right = Self::_new_with_partitioner(depth + 1, right, partitioner);
                BVTNode::Internal(bv, Box::new(left), Box::new(right))
            }
        }
    }
}

impl<T, BV> BVTNode<T, BV> {
    /// The bounding volume of this node.
    #[inline]
    pub fn bounding_volume<'a>(&'a self) -> &'a BV {
        match *self {
            BVTNode::Internal(ref bv, _, _) => bv,
            BVTNode::Leaf(ref bv, _) => bv,
        }
    }

    fn depth(&self) -> usize {
        match *self {
            BVTNode::Internal(_, ref left, ref right) => 1 + na::max(left.depth(), right.depth()),
            BVTNode::Leaf(_, _) => 1,
        }
    }
}

impl<'a, T, BV> BVH<T, BV> for &'a BVT<T, BV> {
    type Node = &'a BVTNode<T, BV>;

    fn root(&self) -> Option<Self::Node> {
        self.tree.as_ref()
    }

    fn num_children(&self, node: Self::Node) -> usize {
        match node {
            BVTNode::Internal(..) => 2,
            BVTNode::Leaf(..) => 0
        }
    }

    fn child(&self, i: usize, node: Self::Node) -> Self::Node {
        match node {
            BVTNode::Internal(_, l, r) => [l, r][i],
            BVTNode::Leaf(..) => panic!("Child index out of bounds.")
        }
    }

    fn content(&self, node: Self::Node) -> (&BV, Option<&T>) {
        match node {
            BVTNode::Leaf(bv, t) => (bv, Some(t)),
            BVTNode::Internal(bv, ..) => (bv, None)
        }
    }
}