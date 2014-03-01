//! A read-only Bounding Volume Tree.

use std::num::Bounded;
use extra::stats::Stats;
use nalgebra::na::{Translation, Indexable};
use nalgebra::na;
use ray::{Ray, RayCast};
use partitioning::bvt_visitor::BVTVisitor;
use bounding_volume::{BoundingVolume, AABB};
use math::{N, V};

/// A Boundig Volume Tree.
#[deriving(Clone, Encodable, Decodable)]
pub struct BVT<B, BV> {
    priv tree: Option<BVTNode<B, BV>>
}

#[deriving(Clone, Encodable, Decodable)]
enum BVTNode<B, BV> {
    Internal(BV, ~BVTNode<B, BV>, ~BVTNode<B, BV>),
    Leaf(BV, B)
}

/// Result of a binary partitioning.
pub enum BinaryPartition<B, BV> {
    /// Result of the partitioning of one element.
    Part(B),
    /// Result of the partitioning of several elements.
    Parts(~[(B, BV)], ~[(B, BV)])
}

impl<B, BV> BVT<B, BV> {
    // FIXME: add higher level constructors ?
    /// Builds a bounding volume tree using an user-defined construction function.
    pub fn new_with_partitioner(leaves:      ~[(B, BV)],
                                partitioner: |uint, ~[(B, BV)]| -> (BV, BinaryPartition<B, BV>))
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

    /// Visit this tree using… a visitor! Visitor arguments are mutable.
    pub fn visit_mut<Vis: BVTVisitor<B, BV>>(&mut self, visitor: &mut Vis) {
        match self.tree {
            Some(ref mut t) => t.visit_mut(visitor),
            None            => { }
        }
    }

    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume<'r>(&'r self) -> Option<&'r BV> {
        match self.tree {
            Some(ref n) => {
                match *n {
                    Internal(ref bv, _, _) => Some(bv),
                    Leaf(ref bv, _)     => Some(bv)
                }
            },
            None => None
        }
    }

    /// Computes the depth of this tree.
    pub fn depth(&self) -> uint {
        match self.tree {
            Some(ref n) => n.depth(),
            None        => 0
        }
    }
}

impl<B> BVT<B, AABB> {
    /// Creates a new kdtree.
    pub fn new_kdtree(leaves: ~[(B, AABB)]) -> BVT<B, AABB> {
        BVT::new_with_partitioner(leaves, kdtree_partitioner)
    }
}

impl<B, BV> BVTNode<B, BV> {
    pub fn bounding_volume<'a>(&'a self) -> &'a BV {
        match *self {
            Internal(ref bv, _, _) => bv,
            Leaf(ref bv, _)        => bv
        }
    }

    fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match *self {
            Internal(ref bv, ref left, ref right) => {
                if visitor.visit_internal(bv) {
                    left.visit(visitor);
                    right.visit(visitor);
                }
            },
            Leaf(ref bv, ref b) => {
                visitor.visit_leaf(b, bv);
            }
        }
    }

    fn visit_mut<Vis: BVTVisitor<B, BV>>(&mut self, visitor: &mut Vis) {
        match *self {
            Internal(ref mut bv, ref mut left, ref mut right) => {
                if visitor.visit_internal_mut(bv) {
                    left.visit_mut(visitor);
                    right.visit_mut(visitor);
                }
            },
            Leaf(ref mut bv, ref mut b) => {
                visitor.visit_leaf_mut(b, bv);
            }
        }
    }

    fn depth(&self) -> uint {
        match *self {
            Internal(_, ref left, ref right) => 1 + na::max(left.depth(), right.depth()),
            Leaf(_, _) => 1
        }
    }
}

impl<B, BV: RayCast> BVT<B, BV> {
    /// Computes the closest intersection between the objects stored on this tree and a given ray.
    pub fn cast_ray<'a, T>(&'a self,
                           ray:     &Ray,
                           cast_fn: &|&B, &Ray| -> Option<(N, T)>) -> Option<(N, T, &'a B)> {
        match self.tree {
            None        => None,
            Some(ref n) => {
                if n.bounding_volume().toi_with_ray(ray, true).is_some() {
                    n.cast_ray(ray, Bounded::max_value(), cast_fn)
                }
                else {
                    None
                }
            }
        }
    }
}

impl<B, BV: RayCast> BVTNode<B, BV> {
    fn cast_ray<'a, T>(&'a self,
                       ray:         &Ray,
                       upper_bound: N,
                       cast_fn:     &|&B, &Ray| -> Option<(N, T)>) -> Option<(N, T, &'a B)> {
        match *self {
            Internal(_, ref left, ref right) => {
                let left_toi  = left.bounding_volume().toi_with_ray(ray, true);
                let right_toi = right.bounding_volume().toi_with_ray(ray, true);

                match (left_toi, right_toi) {
                    (Some(t1), Some(t2)) => {
                        let best;
                        let other;
                        let closest;
                        let farthest;

                        if t1 < t2 {
                            farthest = t2;
                            closest  = t1;
                            best     = left;
                            other    = right;
                        }
                        else {
                            farthest = t1;
                            closest  = t2;
                            best     = right;
                            other    = left;
                        }

                        if closest > upper_bound {
                            // a better solution has already been found
                            return None;
                        }

                        match best.cast_ray(ray, upper_bound, cast_fn) {
                            None    => other.cast_ray(ray, upper_bound, cast_fn),
                            Some(t) => {
                                if farthest < *t.ref0() {
                                    match other.cast_ray(ray, *t.ref0(), cast_fn) {
                                        None         => Some(t),
                                        Some(tother) => {
                                            if *t.ref0() < *tother.ref0() {
                                                Some(t)
                                            }
                                            else {
                                                Some(tother)
                                            }
                                        }
                                    }
                                }
                                else {
                                    Some(t)
                                }
                            }
                        }
                    },
                    (Some(_), None) => left.cast_ray(ray, upper_bound, cast_fn),
                    (None, Some(_)) => right.cast_ray(ray, upper_bound, cast_fn),
                    (None, None)    => None,
                }
            },
            Leaf(_, ref b) => {
                // do not test the bounding volume: this has been done by the parent node.
                match (*cast_fn)(b, ray) {
                    None         => None,
                    Some((t, d)) => {
                        if t < upper_bound {
                            Some((t, d, b))
                        }
                        else {
                            None
                        }
                    }
                }
            }
        }
    }
}

/// Construction function for a kdree.
///
/// Use this as a parameter of `new_with_partitioner`.
#[allow(unnecessary_typecast)]
pub fn kdtree_partitioner<B>(depth: uint, leaves: ~[(B, AABB)]) -> (AABB, BinaryPartition<B, AABB>) {
    if leaves.len() == 0 {
        fail!("Cannot build a tree without leaves.");
    }
    else if leaves.len() == 1 {
        let (b, aabb) = leaves[0];
        (aabb, Part(b))
    }
    else {
        let sep_axis = depth % na::dim::<V>();

        // compute the median along sep_axis
        let mut median = ~[];

        for l in leaves.iter() {
            let center = l.ref1().translation();
            median.push(center.at(sep_axis) as f64);
        }

        let median = na::cast(median.median());

        // build the partitions
        let mut right = ~[];
        let mut left  = ~[];
        let mut bounding_bounding_box = AABB::new_invalid();

        let mut insert_left = false;

        for (b, aabb) in leaves.move_rev_iter() {
            bounding_bounding_box.merge(&aabb);

            let pos = aabb.translation().at(sep_axis);

            if pos < median || (pos == median && insert_left) {
                left.push((b, aabb));
                insert_left = false;
            }
            else {
                right.push((b, aabb));
                insert_left = true;
            }
        }

        (bounding_bounding_box, Parts(left, right))
    }
}

fn _new_with_partitioner<B, BV>(depth:       uint,
                                leaves:      ~[(B, BV)],
                                partitioner: |uint, ~[(B, BV)]| -> (BV, BinaryPartition<B, BV>))
                               -> BVTNode<B, BV> {
    __new_with_partitioner(depth, leaves, partitioner)
}

fn __new_with_partitioner<B, BV>(depth:       uint,
                                 leaves:      ~[(B, BV)],
                                 partitioner: |uint, ~[(B, BV)]| -> (BV, BinaryPartition<B, BV>))
                                 -> BVTNode<B, BV> {
    let (bv, partitions) = partitioner(depth, leaves);

    match partitions {
        Part(b)      => Leaf(bv, b),
        Parts(left, right) => {
            let left  = __new_with_partitioner(depth + 1, left, |i, p| partitioner(i, p));
            let right = __new_with_partitioner(depth + 1, right, |i, p| partitioner(i, p));
            Internal(bv, ~left, ~right)
        }
    }
}
