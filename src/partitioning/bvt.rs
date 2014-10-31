//! A read-only Bounding Volume Tree.

use std::num::Bounded;
use test::stats::Stats;
use na::Translation;
use na;
use ray::{Ray, LocalRayCast};
use partitioning::{BVTVisitor, BVTTVisitor};
use bounding_volume::BoundingVolume;
use math::{Scalar, Vect};


/// A Boundig Volume Tree.
#[deriving(Clone, Encodable, Decodable)]
pub struct BVT<B, BV> {
    tree: Option<BVTNode<B, BV>>
}

/// A node of the bounding volume tree.
#[deriving(Clone, Encodable, Decodable)]
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
    pub fn new_with_partitioner(leaves:      Vec<(B, BV)>,
                                partitioner: |uint, Vec<(B, BV)>| -> (BV, BinaryPartition<B, BV>))
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

    /// Visits the bounding volume traversal tree implicitely formed with `other`.
    pub fn visit_bvtt<Vis: BVTTVisitor<B, BV>>(&self, other: &BVT<B, BV>, visitor: &mut Vis) {
        match (&self.tree, &other.tree) {
            (&Some(ref ta), &Some(ref tb)) => ta.visit_bvtt(tb, visitor),
            _ => { }
        }
    }

    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume<'r>(&'r self) -> Option<&'r BV> {
        match self.tree {
            Some(ref n) => {
                match *n {
                    Internal(ref bv, _, _) => Some(bv),
                    Leaf(ref bv, _) => Some(bv)
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

impl<N, V, B, BV> BVT<B, BV>
    where N:  Scalar,
          V:  Vect<N>,
          BV: Translation<V> + BoundingVolume<N> + Clone {
    /// Creates a balanced `BVT`.
    pub fn new_balanced(leaves: Vec<(B, BV)>) -> BVT<B, BV> {
        BVT::new_with_partitioner(leaves, median_partitioner)
    }
}

impl<B, BV> BVTNode<B, BV> {
    /// The bounding volume of this node.
    #[inline]
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

    fn visit_bvtt<Vis: BVTTVisitor<B, BV>>(&self, other: &BVTNode<B, BV>, visitor: &mut Vis) {
        match (self, other) {
            (&Internal(ref bva, ref la, ref ra), &Internal(ref bvb, ref lb, ref rb)) => {
                if visitor.visit_internal_internal(bva, bvb) {
                    la.visit_bvtt(&**lb, visitor);
                    la.visit_bvtt(&**rb, visitor);
                    ra.visit_bvtt(&**lb, visitor);
                    ra.visit_bvtt(&**rb, visitor);
                }
            },
            (&Internal(ref bva, ref la, ref ra), &Leaf(ref bvb, ref bb)) => {
                if visitor.visit_internal_leaf(bva, bb, bvb) {
                    la.visit_bvtt(other, visitor);
                    ra.visit_bvtt(other, visitor);
                }
            },
            (&Leaf(ref bva, ref ba), &Internal(ref bvb, ref lb, ref rb)) => {
                if visitor.visit_leaf_internal(ba, bva, bvb) {
                    self.visit_bvtt(&**lb, visitor);
                    self.visit_bvtt(&**rb, visitor);
                }
            },
            (&Leaf(ref bva, ref ba), &Leaf(ref bvb, ref bb)) => {
                visitor.visit_leaf_leaf(ba, bva, bb, bvb)
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

impl<N, P, V, B, BV> BVT<B, BV>
    where N: Scalar,
          BV: LocalRayCast<N, P, V> {
    /// Computes the closest intersection between the objects stored on this tree and a given ray.
    pub fn cast_ray<'a, T>(&'a self,
                           ray:     &Ray<P, V>,
                           cast_fn: &mut |&B, &Ray<P, V>| -> Option<(N, T)>) -> Option<(N, T, &'a B)> {
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

impl<N, P, V, B, BV> BVTNode<B, BV>
    where N: Scalar,
          BV: LocalRayCast<N, P, V> {
    fn cast_ray<'a, T>(&'a self,
                       ray:         &Ray<P, V>,
                       upper_bound: N,
                       cast_fn:     &mut |&B, &Ray<P, V>| -> Option<(N, T)>) -> Option<(N, T, &'a B)> {
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
                            // A better solution has already been found.
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
                // Do not test the bounding volume: this has been done by the parent node.
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

/// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
pub fn median_partitioner_with_centers<N, V, B, BV>(depth:  uint,
                                                    leaves: Vec<(B, BV)>,
                                                    center: &mut |&B, &BV| -> V)
                                                    -> (BV, BinaryPartition<B, BV>)
    where N: Scalar,
          V:  Vect<N>,
          BV: BoundingVolume<N> + Clone{
    if leaves.len() == 0 {
        panic!("Cannot build a tree without leaves.");
    }
    else if leaves.len() == 1 {
        let (b, bv) = leaves.into_iter().next().unwrap();
        (bv, Part(b))
    }
    else {
        let sep_axis = depth % na::dim::<V>();

        // compute the median along sep_axis
        let mut median = Vec::new();

        for l in leaves.iter() {
            let c = (*center)(l.ref0(), l.ref1());
            median.push(c[sep_axis]);
        }

        let median = median.as_slice().median();

        // build the partitions
        let mut right = Vec::new();
        let mut left  = Vec::new();
        let mut bounding_bounding_volume = leaves[0].ref1().clone();

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

        (bounding_bounding_volume, Parts(left, right))
    }
}

/// Construction function for a kdree to be used with `BVT::new_with_partitioner`.
pub fn median_partitioner<N, V, B, BV>(depth:  uint,
                                       leaves: Vec<(B, BV)>)
                                       -> (BV, BinaryPartition<B, BV>)
    where N:  Scalar,
          V:  Vect<N>,
          BV: Translation<V> + BoundingVolume<N> + Clone {
    median_partitioner_with_centers(depth, leaves, &mut |_, bv| bv.translation())
}

fn _new_with_partitioner<B, BV>(depth:       uint,
                                leaves:      Vec<(B, BV)>,
                                partitioner: |uint, Vec<(B, BV)>| -> (BV, BinaryPartition<B, BV>))
                               -> BVTNode<B, BV> {
    __new_with_partitioner(depth, leaves, partitioner)
}

fn __new_with_partitioner<B, BV>(depth:       uint,
                                 leaves:      Vec<(B, BV)>,
                                 partitioner: |uint, Vec<(B, BV)>| -> (BV, BinaryPartition<B, BV>))
                                 -> BVTNode<B, BV> {
    let (bv, partitions) = partitioner(depth, leaves);

    match partitions {
        Part(b)            => Leaf(bv, b),
        Parts(left, right) => {
            let left  = __new_with_partitioner(depth + 1, left, |i, p| partitioner(i, p));
            let right = __new_with_partitioner(depth + 1, right, |i, p| partitioner(i, p));
            Internal(bv, box left, box right)
        }
    }
}
