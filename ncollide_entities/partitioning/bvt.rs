//! A read-only Bounding Volume Tree.

use std::collections::BinaryHeap;
use na::{Translation, Bounded};
use na;
use partitioning::{BVTVisitor, BVTTVisitor, BVTCostFn};
use bounding_volume::BoundingVolume;
use utils::data::ref_with_cost::RefWithCost;
use utils;
use math::{Scalar, Vect};


/// A Boundig Volume Tree.
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub struct BVT<B, BV> {
    tree: Option<BVTNode<B, BV>>
}

/// A node of the bounding volume tree.
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub enum BVTNode<B, BV> {
    // XXX: give a faster access to the BV
    /// An internal node.
    Internal(BV, Box<BVTNode<B, BV>>, Box<BVTNode<B, BV>>),
    /// A leaf.
    Leaf(BV, B)
}

/// Result of a binary partition.
pub enum BinaryPartition<B, BV> {
    /// Result of the partitioning of one element.
    Part(B),
    /// Result of the partitioning of several elements.
    Parts(Vec<(B, BV)>, Vec<(B, BV)>)
}

impl<B, BV> BVT<B, BV> {
    // FIXME: add higher level constructors ?
    /// Builds a bounding volume tree using an user-defined construction function.
    pub fn new_with_partitioner<F: FnMut(usize, Vec<(B, BV)>) -> (BV, BinaryPartition<B, BV>)>
      (leaves:      Vec<(B, BV)>,
       partitioner: &mut F)
          -> BVT<B, BV> {
        if leaves.len() == 0 {
            BVT {
                tree: None
            }
        }
        else {
            BVT {
                tree: Some(_new_with_partitioner(0, leaves, partitioner))
            }
        }
    }

    /// Visit this tree using… a visitor!
    pub fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match self.tree {
            Some(ref t) => t.visit(visitor),
            None        => { }
        }
    }

    /// Visits the bounding volume traversal tree implicitely formed with `other`.
    pub fn visit_bvtt<Vis: BVTTVisitor<B, BV>>(&self, other: &BVT<B, BV>, visitor: &mut Vis) {
        match (&self.tree, &other.tree) {
            (&Some(ref ta), &Some(ref tb)) => ta.visit_bvtt(tb, visitor),
            _ => { }
        }
    }

    // FIXME: internalize the type parameter R using associated types.
    // FIXME: really return a ref to B ?
    /// Performs a best-fist-search on the tree.
    ///
    /// Returns the content of the best leaf nound, and a result of user-defined type.
    pub fn best_first_search<'a, N, BFS, R>(&'a self, algorithm: &mut BFS) -> Option<(&'a B, R)>
        where N:   Scalar,
              BFS: BVTCostFn<N, B, BV, R> {
        match self.tree {
            Some(ref t) => t.best_first_search(algorithm),
            None        => None
        }
    }

    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume<'r>(&'r self) -> Option<&'r BV> {
        match self.tree {
            Some(ref n) => {
                match *n {
                    BVTNode::Internal(ref bv, _, _) => Some(bv),
                    BVTNode::Leaf(ref bv, _) => Some(bv)
                }
            },
            None => None
        }
    }

    /// Computes the depth of this tree.
    pub fn depth(&self) -> usize {
        match self.tree {
            Some(ref n) => n.depth(),
            None        => 0
        }
    }
}

impl<B, BV> BVT<B, BV> {
    /// Creates a balanced `BVT`.
    pub fn new_balanced<V>(leaves: Vec<(B, BV)>) -> BVT<B, BV>
        where V: Vect,
              BV: Translation<V> + BoundingVolume<V::Scalar> + Clone {
        BVT::new_with_partitioner(leaves, &mut median_partitioner)
    }
}

impl<B, BV> BVTNode<B, BV> {
    /// The bounding volume of this node.
    #[inline]
    pub fn bounding_volume<'a>(&'a self) -> &'a BV {
        match *self {
            BVTNode::Internal(ref bv, _, _) => bv,
            BVTNode::Leaf(ref bv, _)        => bv
        }
    }

    fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match *self {
            BVTNode::Internal(ref bv, ref left, ref right) => {
                if visitor.visit_internal(bv) {
                    left.visit(visitor);
                    right.visit(visitor);
                }
            },
            BVTNode::Leaf(ref bv, ref b) => {
                visitor.visit_leaf(b, bv);
            }
        }
    }

    fn visit_bvtt<Vis: BVTTVisitor<B, BV>>(&self, other: &BVTNode<B, BV>, visitor: &mut Vis) {
        match (self, other) {
            (&BVTNode::Internal(ref bva, ref la, ref ra), &BVTNode::Internal(ref bvb, ref lb, ref rb)) => {
                if visitor.visit_internal_internal(bva, bvb) {
                    la.visit_bvtt(&**lb, visitor);
                    la.visit_bvtt(&**rb, visitor);
                    ra.visit_bvtt(&**lb, visitor);
                    ra.visit_bvtt(&**rb, visitor);
                }
            },
            (&BVTNode::Internal(ref bva, ref la, ref ra), &BVTNode::Leaf(ref bvb, ref bb)) => {
                if visitor.visit_internal_leaf(bva, bb, bvb) {
                    la.visit_bvtt(other, visitor);
                    ra.visit_bvtt(other, visitor);
                }
            },
            (&BVTNode::Leaf(ref bva, ref ba), &BVTNode::Internal(ref bvb, ref lb, ref rb)) => {
                if visitor.visit_leaf_internal(ba, bva, bvb) {
                    self.visit_bvtt(&**lb, visitor);
                    self.visit_bvtt(&**rb, visitor);
                }
            },
            (&BVTNode::Leaf(ref bva, ref ba), &BVTNode::Leaf(ref bvb, ref bb)) => {
                visitor.visit_leaf_leaf(ba, bva, bb, bvb)
            }
        }
    }

    fn best_first_search<'a, N, BFS, R>(&'a self, algorithm: &mut BFS) -> Option<(&'a B, R)>
        where N:   Scalar,
              BFS: BVTCostFn<N, B, BV, R> {
        let mut queue: BinaryHeap<RefWithCost<'a, N, BVTNode<B, BV>>> = BinaryHeap::new();
        let mut best_cost = Bounded::max_value();
        let mut result    = None;

        match algorithm.compute_bv_cost(self.bounding_volume()) {
            Some(cost) => queue.push(RefWithCost::new(self, cost)),
            None       => return None
        }

        loop {
            match queue.pop() {
                Some(node) => {
                    if -node.cost >= best_cost {
                        break; // solution found.
                    }

                    match *node.object {
                        BVTNode::Internal(_, ref left, ref right) => {
                            match algorithm.compute_bv_cost(left.bounding_volume()) {
                                Some(lcost) => {
                                    if lcost < best_cost {
                                        queue.push(RefWithCost::new(&**left, -lcost))
                                    }
                                },
                                None => { }
                            }

                            match algorithm.compute_bv_cost(right.bounding_volume()) {
                                Some(rcost) => {
                                    if rcost < best_cost {
                                        queue.push(RefWithCost::new(&**right, -rcost))
                                    }
                                },
                                None => { }
                            }
                        },
                        BVTNode::Leaf(_, ref b) => {
                            match algorithm.compute_b_cost(b) {
                                Some((candidate_cost, candidate_result)) => {
                                    if candidate_cost < best_cost {
                                        best_cost = candidate_cost;
                                        result    = Some((b, candidate_result));
                                    }
                                }
                                None => { }
                            }
                        }
                    }
                }
                None => break,
            }
        }

        result
    }

    fn depth(&self) -> usize {
        match *self {
            BVTNode::Internal(_, ref left, ref right) => 1 + na::max(left.depth(), right.depth()),
            BVTNode::Leaf(_, _) => 1
        }
    }
}

/// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
pub fn median_partitioner_with_centers<V, B, BV, F: FnMut(&B, &BV) -> V>
        (depth: usize, leaves: Vec<(B, BV)>, center: &mut F) -> (BV, BinaryPartition<B, BV>)
    where V:  Vect,
          BV: BoundingVolume<V::Scalar> + Clone {
    if leaves.len() == 0 {
        panic!("Cannot build a tree without leaves.");
    }
    else if leaves.len() == 1 {
        let (b, bv) = leaves.into_iter().next().unwrap();
        (bv, BinaryPartition::Part(b))
    }
    else {
        let sep_axis = depth % na::dim::<V>();

        // compute the median along sep_axis
        let mut median = Vec::new();

        for l in leaves.iter() {
            let c = (*center)(&l.0, &l.1);
            median.push(c[sep_axis]);
        }

        let median = utils::median(&mut median[..]);

        // build the partitions
        let mut right = Vec::new();
        let mut left  = Vec::new();
        let mut bounding_bounding_volume = leaves[0].1.clone();

        let mut insert_left = false;

        for (b, bv) in leaves.into_iter() {
            bounding_bounding_volume.merge(&bv);

            let pos = (*center)(&b, &bv)[sep_axis];

            if pos < median || (pos == median && insert_left) {
                left.push((b, bv));
                insert_left = false;
            }
            else {
                right.push((b, bv));
                insert_left = true;
            }
        }

        // XXX: hack to avoid degeneracies.
        if left.len() == 0 {
            left.push(right.pop().unwrap());
        }
        else if right.len() == 0 {
            right.push(left.pop().unwrap());
        }

        (bounding_bounding_volume, BinaryPartition::Parts(left, right))
    }
}

/// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
pub fn median_partitioner<V, B, BV>(depth: usize, leaves: Vec<(B, BV)>) -> (BV, BinaryPartition<B, BV>)
    where V:  Vect,
          BV: Translation<V> + BoundingVolume<V::Scalar> + Clone {
    median_partitioner_with_centers(depth, leaves, &mut |_, bv| bv.translation())
}

fn _new_with_partitioner<B, BV, F: FnMut(usize, Vec<(B, BV)>) -> (BV, BinaryPartition<B, BV>)>
                         (depth: usize, leaves: Vec<(B, BV)>, partitioner: &mut F) -> BVTNode<B, BV> {
    __new_with_partitioner(depth, leaves, partitioner)
}

fn __new_with_partitioner<B, BV, F: FnMut(usize, Vec<(B, BV)>) -> (BV, BinaryPartition<B, BV>)>
                          (depth: usize, leaves: Vec<(B, BV)>, partitioner: &mut F) -> BVTNode<B, BV> {
    let (bv, partitions) = partitioner(depth, leaves);

    match partitions {
        BinaryPartition::Part(b)            => BVTNode::Leaf(bv, b),
        BinaryPartition::Parts(left, right) => {
            let left  = __new_with_partitioner(depth + 1, left, partitioner);
            let right = __new_with_partitioner(depth + 1, right, partitioner);
            BVTNode::Internal(bv, Box::new(left), Box::new(right))
        }
    }
}
