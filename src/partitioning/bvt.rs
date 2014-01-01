//! A read-only Bounding Volume Tree.

use std::num::Zero;
use std::vec;
use std::rand;
use std::rand::Rng;
use nalgebra::na::{Translation, Indexable};
use nalgebra::na;
use partitioning::bvt_visitor::BVTVisitor;
use bounding_volume::{BoundingVolume, AABB};
use math::V;

/// A Boundig Volume Tree.
#[deriving(Clone, Encodable, Decodable)]
pub struct BVT<B, BV> {
    priv tree: Option<BVTNode<B, BV>>
}

#[deriving(Clone, Encodable, Decodable)]
enum BVTNode<B, BV> {
    Internal(BV, ~[BVTNode<B, BV>]),
    Leaf(BV, B)
}

type PartFnResult<B, BV> = (BV, Either<B, ~[~[(B, BV)]]>);

impl<B, BV> BVT<B, BV> {
    // FIXME: add higher level constructors ?
    /// Builds a bounding volume tree using an user-defined construction function.
    pub fn new_with_partitioner(leaves:      ~[(B, BV)],
                                partitioner: |&mut rand::StdRng, ~[(B, BV)]| -> PartFnResult<B, BV>)
                                -> BVT<B, BV> {
        if leaves.len() == 0 {
            BVT {
                tree: None
            }
        }
        else {
            BVT {
                tree: Some(new_with_partitioner(leaves, partitioner))
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
}

impl<B, BV> BVT<B, BV> {
    /// Reference to the bounding volume of the tree root.
    pub fn root_bounding_volume<'r>(&'r self) -> Option<&'r BV> {
        match self.tree {
            Some(ref n) => {
                match *n {
                    Internal(ref bv, _) => Some(bv),
                    Leaf(ref bv, _)     => Some(bv)
                }
            },
            None => None
        }
    }
}

impl<B, BV> BVTNode<B, BV> {
    fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match *self {
            Internal(ref bv, ref children) => {
                if visitor.visit_internal(bv) {
                    for child in children.iter() {
                        child.visit(visitor)
                    }
                }
            },
            Leaf(ref bv, ref b) => {
                visitor.visit_leaf(b, bv);
            }
        }
    }

    fn visit_mut<Vis: BVTVisitor<B, BV>>(&mut self, visitor: &mut Vis) {
        match *self {
            Internal(ref mut bv, ref mut children) => {
                if visitor.visit_internal_mut(bv) {
                    for child in children.mut_iter() {
                        child.visit_mut(visitor)
                    }
                }
            },
            Leaf(ref mut bv, ref mut b) => {
                visitor.visit_leaf_mut(b, bv);
            }
        }
    }
}

/// Construction function for quadtree in 2d, an octree in 4d, and a 2^n tree in n-d.
///
/// Use this as a parameter of `new_with_partitioner`.
pub fn kdtree_partitioner<B>(rng: &mut rand::StdRng, leaves: ~[(B, AABB)])
                             -> PartFnResult<B, AABB> {
    if leaves.len() == 0 {
        fail!("Cannot build a tree without leaves.");
    }
    else if leaves.len() == 1 {
        let (b, aabb) = leaves[0];
        (aabb, Left(b))
    }
    else {
        // merge all bounding boxes
        let bounding_bounding_box =
            leaves.iter().fold(AABB::new_invalid(), |curr_aabb, &(_, ref other_aabb)| {
                curr_aabb.merged(other_aabb)
            });

        let center = bounding_bounding_box.translation();

        // build the partitions
        let mut partitions = vec::from_fn(1u << na::dim::<V>(), |_| ~[]);
        for (b, aabb) in leaves.move_rev_iter() {
            let dpos    = aabb.translation() - center;
            let mut key = 0u;

            for i in range(0u, na::dim::<V>()) {
                if dpos.at(i).is_negative() {
                    key = key | (1u << i);
                }
                else if dpos.at(i).is_zero() {
                    if rng.gen() {
                        key = key | (1u << i);
                    }
                }
            }

            partitions[key].push((b, aabb))
        }

        (bounding_bounding_box, Right(partitions))
    }
}

fn new_with_partitioner<B, BV>(leaves:      ~[(B, BV)],
                               partitioner: |&mut rand::StdRng, ~[(B, BV)]| -> PartFnResult<B, BV>)
                               -> BVTNode<B, BV> {
    let mut rng: rand::StdRng = rand::SeedableRng::from_seed(&[1, 2, 3, 4]);
    _new_with_partitioner(&mut rng, leaves, partitioner)
}

fn _new_with_partitioner<B, BV>(rng:         &mut rand::StdRng,
                                leaves:      ~[(B, BV)],
                                partitioner: |&mut rand::StdRng, ~[(B, BV)]| -> PartFnResult<B, BV>)
                                -> BVTNode<B, BV> {
    let (bv, partitions) = partitioner(rng, leaves);

    match partitions {
        Left(b)      => Leaf(bv, b),
        Right(parts) => {
            let mut children = ~[];

            for part in parts.move_rev_iter() {
                if part.len() != 0 {
                    children.push(_new_with_partitioner(rng, part, |r, x| partitioner(r, x)))
                }
            }

            // FIXME. if children.len() == 1 && children[0].len() > 1,
            // ignore this node!
            Internal(bv, children)
        }
    }
}
