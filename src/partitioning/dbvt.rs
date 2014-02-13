//! A Dynamic Bounding Volume Tree.

use std::gc::Gc;
use std::cell::RefCell;
use std::ptr;
use std::mem;
use util::owned_allocation_cache::OwnedAllocationCache;
use nalgebra::na::Translation;
use nalgebra::na;
use bounding_volume::BoundingVolume;
use partitioning::bvt_visitor::{BVTVisitor, BoundingVolumeInterferencesCollector};
use math::{N, V};

#[deriving(Encodable, Decodable)]
enum UpdateState {
    NeedsShrink,
    UpToDate
}

type Cache<B, BV> = OwnedAllocationCache<DBVTInternal<B, BV>>;

/// A Dynamic Bounding Volume Tree.
pub struct DBVT<B, BV> {
    priv cache: Cache<B, BV>,
    priv tree:  Option<DBVTNode<B, BV>>,
    priv len:   uint
}

impl<B, BV> DBVT<B, BV> {
    /// Creates a new Dynamic Bounding Volume Tree.
    pub fn new() -> DBVT<B, BV> {
        DBVT {
            cache: OwnedAllocationCache::new(),
            tree:  None,
            len:   0
        }
    }
}

impl<BV: 'static + BoundingVolume + Translation<V> + Clone,
     B:  'static + Clone>
DBVT<B, BV> {
    /// Removes a leaf from the tree. Fails if the tree is empty.
    pub fn remove(&mut self, leaf: &mut Gc<RefCell<DBVTLeaf<B, BV>>>) {
        let self_tree = self.tree.take_unwrap();

        let mut bleaf = leaf.borrow().borrow_mut();
        self.tree = bleaf.get().unlink(&mut self.cache, self_tree);
        self.len  = self.len - 1;
    }

    /// Inserts a leaf to the tree.
    pub fn insert(&mut self, leaf: Gc<RefCell<DBVTLeaf<B, BV>>>) {
        let mut self_tree = None;
        mem::swap(&mut self_tree, &mut self.tree);

        self.tree = match self_tree {
            None    => Some(Leaf(leaf)),
            Some(t) => Some(Internal(t.insert(&mut self.cache, leaf)))
        };

        self.len = self.len + 1;
    }

    /// Visit this tree using… a visitor!
    pub fn visit<Vis: BVTVisitor<Gc<RefCell<DBVTLeaf<B, BV>>>, BV>>(&self, visitor: &mut Vis) {
        match self.tree {
            Some(ref t) => t.visit(visitor),
            None        => { }
        }
    }

//    /// Visit this tree using… a visitor! Visitor arguments are mutable.
//    pub fn visit_mut<Vis: BVTVisitor<GcMut<DBVTLeaf<B, BV>>, BV>>(&mut self, visitor: &mut Vis) {
//        match self.tree {
//            Some(ref mut t) => t.visit_mut(visitor),
//            None            => { }
//        }
//    }

    /// Finds all leaves which have their bounding boxes intersecting a specific leave's bounding
    /// volume.
    ///
    /// # Arguments:
    /// * `to_test` - the leaf to check interferences with.
    /// * `out` - will be filled with all leaves intersecting `to_test`. Note that `to_test`
    ///           is not considered intersecting itself.
    pub fn interferences_with_leaf(&self,
                                   leaf: &DBVTLeaf<B, BV>,
                                   out:  &mut ~[Gc<RefCell<DBVTLeaf<B, BV>>>]) {
        match self.tree {
            Some(ref tree) => tree.interferences_with_leaf(leaf, out),
            None           => { }
        }
    }

    /// Finds all interferences between this tree and another one.
    pub fn interferences_with_tree(&self,
                                   leaf: &DBVT<B, BV>,
                                   out:  &mut ~[Gc<RefCell<DBVTLeaf<B, BV>>>]) {
        match (&self.tree, &leaf.tree) {
            (&Some(ref a), &Some(ref b)) => a.interferences_with_tree(b, out),
            (&None, _) => { },
            (_, &None) => { }
        }
    }
}

/// Node of the Dynamic Bounding Volume Tree.
enum DBVTNode<B, BV> {
    Internal(~DBVTInternal<B, BV>),
    Leaf(Gc<RefCell<DBVTLeaf<B, BV>>>),
    Invalid
}

/// Internal node of a DBVT. An internal node always has two children.
struct DBVTInternal<B, BV> {
    /// The bounding volume of this node. It always encloses both its children bounding volumes.
    bounding_volume: BV,
    /// The center of this node bounding volume.
    center:          V,
    /// This node left child.
    left:            DBVTNode<B, BV>,
    /// This node right child.
    right:           DBVTNode<B, BV>,
    /// This node parent.
    parent:          *mut DBVTInternal<B, BV>,

    state:      UpdateState
}

impl<BV: Translation<V>, B> DBVTInternal<B, BV> {
    /// Creates a new internal node.
    fn new(bounding_volume: BV,
           parent:          *mut DBVTInternal<B, BV>,
           left:            DBVTNode<B, BV>,
           right:           DBVTNode<B, BV>)
           -> DBVTInternal<B, BV> {
        DBVTInternal {
            center:          bounding_volume.translation(),
            bounding_volume: bounding_volume,
            left:            left,
            right:           right,
            parent:          parent,
            state:           UpToDate
        }
    }
}

#[deriving(Clone)]
enum LeafState<B, BV> {
    RightChildOf(*mut DBVTInternal<B, BV>),
    LeftChildOf(*mut DBVTInternal<B, BV>),
    Detached
}

impl<B, BV> LeafState<B, BV> {
    pub fn is_detached(&self) -> bool {
        match *self {
            Detached => true,
            _        => false
        }
    }

    pub fn unwrap(self) -> (bool, *mut DBVTInternal<B, BV>) {
        match self {
            RightChildOf(p) => (false, p),
            LeftChildOf(p)  => (true, p),
            _               => fail!("Attempting to unwrap a detached node.")
        }
    }
}

/// Leaf of a Dynamic Bounding Volume Tree.
#[deriving(Clone)]
pub struct DBVTLeaf<B, BV> {
    /// The bounding volume of this node.
    bounding_volume: BV,
    /// The center of this node bounding volume.
    center:          V,
    /// An user-defined object.
    object:          B,
    /// This node parent.
    parent:          LeafState<B, BV>
}

impl<B, BV> DBVTNode<B, BV> {
    fn take_internal(self) -> ~DBVTInternal<B, BV> {
        match self {
            Internal(i) => i,
            _ => fail!("DBVT internal error: this is not an internal node.")
        }
    }

    fn invalidate(&mut self) -> DBVTNode<B, BV> {
        let mut res = Invalid;

        mem::swap(&mut res, self);

        res
    }
}

impl<B, BV> DBVTInternal<B, BV> {
    fn is_right_internal_node(&self, r: &mut DBVTInternal<B, BV>) -> bool
    {
        match self.right {
            Internal(ref i) => &**i as *DBVTInternal<B, BV> == &*r as *DBVTInternal<B, BV>,
            _ => false
        }
    }
}

impl<B: 'static, BV: Translation<V> + 'static> DBVTLeaf<B, BV> {
    /// Creates a new leaf.
    pub fn new(bounding_volume: BV, object: B) -> DBVTLeaf<B, BV> {
        DBVTLeaf {
            center:          bounding_volume.translation(),
            bounding_volume: bounding_volume,
            object:          object,
            parent:          Detached
        }
    }

    /// Removes this leaf from the tree.
    ///
    /// Returns the new root of the tree.
    ///
    /// # Arguments:
    /// * `curr_root`: current root of the tree.
    fn unlink(&mut self,
              cache:     &mut Cache<B, BV>,
              curr_root: DBVTNode<B, BV>) -> Option<DBVTNode<B, BV>> {
        if !self.parent.is_detached() {
            let (is_left, p) = mem::replace(&mut self.parent, Detached).unwrap();

            let pp           = unsafe { (*p).parent };
            let parent_left  = unsafe { (*p).left.invalidate() };
            let parent_right = unsafe { (*p).right.invalidate() };

            let mut other = if is_left { parent_right } else { parent_left };

            if pp.is_not_null() {
                let is_p_right_to_pp = unsafe { (*pp).is_right_internal_node(&mut *p) };
                // we are far away from the root
                unsafe {
                    match other {
                        Internal(ref mut i) => i.parent = pp,
                        Leaf(ref mut l)     => {
                            let mut bl = l.borrow().borrow_mut();
                            bl.get().parent = if is_p_right_to_pp { RightChildOf(pp) } else { LeftChildOf(pp) }
                        },
                        Invalid             => unreachable!()
                    }

                    if is_p_right_to_pp {
                        mem::swap(&mut (*pp).right, &mut other);
                        // NOTE: the children have already been invalidated before
                        cache.retain(other.take_internal())
                    }
                    else {
                        mem::swap(&mut (*pp).left, &mut other);
                        // NOTE: the children have already been invalidated before
                        cache.retain(other.take_internal())
                    }

                    (*pp).state = NeedsShrink;
                }

                Some(curr_root)
            }
            else {
                // the root changes to the other child
                match other {
                    Internal(ref mut i) => i.parent = ptr::mut_null(),
                    Leaf(ref l)         => {
                        let mut bl = l.borrow().borrow_mut();
                        bl.get().parent = Detached
                    },
                    Invalid             => unreachable!()
                }

                Some(other)
            }
        }
        else {
            self.parent = Detached;

            // the tree becomes empty
            None
        }
    }
}

impl<BV: 'static + BoundingVolume, B: 'static> DBVTNode<B, BV> {
    fn sqdist_to(&self, to: &V) -> N {
        match *self {
            Internal(ref i) => na::sqnorm(&(i.center - *to)),
            Leaf(ref l)     => {
                let bl = l.borrow().borrow();
                na::sqnorm(&(bl.get().center - *to))
            },
            Invalid         => unreachable!()
        }
    }

    /*
    fn enclosing_volume(&self, other: &DBVTNode<B, BV>) -> BV {
        match (self, other) {
            (&Internal(ref a), &Internal(ref b)) => a.bounding_volume.merged(&b.bounding_volume),
            (&Leaf(ref a)    , &Internal(ref b)) => a.bounding_volume.merged(&b.bounding_volume),
            (&Internal(ref a), &Leaf(ref b))     => a.bounding_volume.merged(&b.bounding_volume),
            (&Leaf(ref a)    , &Leaf(ref b))     => a.bounding_volume.merged(&b.bounding_volume),
            _ => unreachable!() // combination including invalide nodes
        }
    }
    */
}

impl<BV: 'static + Translation<V> + BoundingVolume, B: 'static> DBVTInternal<B, BV> {
    fn is_closest_to_left(&self, pt: &V) -> bool {
        self.right.sqdist_to(pt) > self.left.sqdist_to(pt)
    }

    /*
    fn is_closest_to_right(&self, pt: &V) -> bool {
        !self.is_closest_to_left(pt)
    }

    fn partial_optimise(&mut self) {
        match self.state {
            NeedsShrink => {
                self.bounding_volume = self.right.enclosing_volume(&self.left);
                self.state = UpToDate;
            }
            _ => { }
        }
    }
    */
}

impl<BV: 'static + BoundingVolume + Translation<V> + Clone, B: 'static + Clone> DBVTNode<B, BV> {
    /// Inserts a new leaf on this tree.
    fn insert(self,
              cache:     &mut Cache<B, BV>,
              to_insert: Gc<RefCell<DBVTLeaf<B, BV>>>)
              -> ~DBVTInternal<B, BV> {

        let mut bto_insert = to_insert.borrow().borrow_mut();
        let pto_insert     = bto_insert.get();

        match self {
            Internal(i) => {
                /*
                 * NOTE: the insersion is done with unsafe pointers.
                 * This is so because using &mut references dont seem to be possible since we have
                 * to take successive references to nodes contents.
                 */

                let mut mut_internal = i;
                let mut parent       = ptr::to_mut_unsafe_ptr(mut_internal);

                unsafe {
                    (*parent).bounding_volume.merge(&pto_insert.bounding_volume);

                    // iteratively go to the leaves
                    let mut curr;
                    let mut left;

                    if (*parent).is_closest_to_left(&pto_insert.center) {
                        curr = ptr::to_mut_unsafe_ptr(&mut (*parent).left);
                        left = true;
                    }
                    else {
                        curr = ptr::to_mut_unsafe_ptr(&mut (*parent).right);
                        left = false;
                    }

                    loop {
                        match *curr {
                            Internal(ref mut ci) => {
                                // FIXME: we could avoid the systematic merge
                                ci.bounding_volume.merge(&pto_insert.bounding_volume);

                                if ci.is_closest_to_left(&pto_insert.center) { // FIXME
                                    curr = ptr::to_mut_unsafe_ptr(&mut ci.left);
                                    left = true;
                                }
                                else {
                                    curr = ptr::to_mut_unsafe_ptr(&mut ci.right);
                                    left = false;
                                }

                                parent = ptr::to_mut_unsafe_ptr(*ci);
                            },
                            Leaf(ref l) => {
                                let mut bl       = l.borrow().borrow_mut();
                                let     pl       = bl.get();
                                let mut internal = cache.alloc(DBVTInternal::new(
                                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                                    parent,
                                    Leaf(l.clone()),
                                    Leaf(to_insert.clone())));

                                pl.parent         = LeftChildOf(ptr::to_mut_unsafe_ptr(internal));
                                pto_insert.parent = RightChildOf(ptr::to_mut_unsafe_ptr(internal));

                                if left {
                                    (*parent).left = Internal(internal)
                                }
                                else {
                                    (*parent).right = Internal(internal)
                                }

                                break;
                            },
                            Invalid => unreachable!()
                        }
                    }
                }

                mut_internal
            },
            Leaf(l) => {
                let mut bl = l.borrow().borrow_mut();
                let     pl = bl.get();

                // create the root
                let mut root = cache.alloc(DBVTInternal::new(
                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                    ptr::mut_null(),
                    Leaf(l),
                    Leaf(to_insert.clone())));

                pl.parent         = LeftChildOf(ptr::to_mut_unsafe_ptr(root));
                pto_insert.parent = RightChildOf(ptr::to_mut_unsafe_ptr(root));

                root
            },
            Invalid => unreachable!()
        }
    }

    /// Finds all leaves which have their bounding boxes intersecting a specific leave's bounding
    /// volume.
    ///
    /// # Arguments:
    /// * `to_test` - the leaf to check interference with.
    /// * `out` - will be filled with all leaves intersecting `to_test`. Note that `to_test`
    /// is not considered intersecting itself.
    fn interferences_with_leaf(&self,
                               to_test: &DBVTLeaf<B, BV>,
                               out:     &mut ~[Gc<RefCell<DBVTLeaf<B, BV>>>]) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&to_test.bounding_volume, out);
        self.visit(&mut visitor)
    }

    fn visit<Vis: BVTVisitor<Gc<RefCell<DBVTLeaf<B, BV>>>, BV>>(&self, visitor: &mut Vis) {
        match *self {
            Internal(ref i) => {
                if visitor.visit_internal(&i.bounding_volume) {
                    i.left.visit(visitor);
                    i.right.visit(visitor);
                }
            },
            Leaf(ref l) => {
                let bl = l.borrow().borrow();
                visitor.visit_leaf(l, &bl.get().bounding_volume)
            },
            Invalid => unreachable!()
        }
    }

//    fn visit_mut<Vis: BVTVisitor<GcMut<DBVTLeaf<B, BV>>, BV>>(&mut self, visitor: &mut Vis) {
//        match *self {
//            Internal(ref mut i) => {
//                i.partial_optimise();
//
//                if visitor.visit_internal(&mut i.bounding_volume) {
//                    i.left.visit(visitor);
//                    i.right.visit(visitor);
//                }
//            },
//            Leaf(ref mut l) => {
//                l.with_mut_borrow(|l_| visitor.visit_leaf_mut(l, &mut l_.bounding_volume))
//            },
//            Invalid => util::unreachable()
//        }
//    }

    /// Finds all interferences between this tree and another one.
    fn interferences_with_tree(&self,
                               to_test: &DBVTNode<B, BV>,
                               out:     &mut ~[Gc<RefCell<DBVTLeaf<B, BV>>>]) {
        match (self, to_test) {
            (&Leaf(_), &Leaf(ref lb)) => {
                let blb = lb.borrow().borrow();
                self.interferences_with_leaf(blb.get(), out)
            },
            (&Leaf(ref la), &Internal(_)) => {
                let bla = la.borrow().borrow();
                to_test.interferences_with_leaf(bla.get(), out)
            },
            (&Internal(_), &Leaf(ref lb)) => {
                let blb = lb.borrow().borrow();
                self.interferences_with_leaf(blb.get(), out)
            },
            (&Internal(ref la), &Internal(ref lb)) => {
                // FIXME: la.partial_optimise();
                // FIXME: lb.partial_optimise();

                if (&**la as *DBVTInternal<B, BV> != &**lb as *DBVTInternal<B, BV>) &&
                    la.bounding_volume.intersects(&lb.bounding_volume)
                {
                    la.right.interferences_with_tree(&lb.right, out);
                    la.left.interferences_with_tree(&lb.left, out);
                    la.right.interferences_with_tree(&lb.left, out);
                    la.left.interferences_with_tree(&lb.right, out);
                }
            },
            _ => unreachable!() // combinations with Invalid
        }
    }
}

/*
impl<BV: 'static + BoundingVolume + RayCast<N, V> + Translation<V>,
     B:  'static,
     V:  'static + AlgebraicVec<N>,
     N:  Algebraic + Ord>
DBVTNode<V, B, BV> {
    fn interferences_with_ray(&self, ray: &Ray<V>, out: &mut ~[Gc<RefCell<f<V, B, BV>>]) {
        match (*self) {
            Internal(ref i) => {
                if i.bounding_volume.intersects_ray(ray) {
                    i.left.interferences_with_ray(ray, out);
                    i.right.interferences_with_ray(ray, out)
                }
            },
            Leaf(ref l) => {
                if l.bounding_volume.intersects_ray(ray) {
                    out.push(l.clone())
                }
            },
            Invalid => unreachable!()
        }
    }
}
*/

// XXX: Drop should be implemented to invalidate the leaves parents when the tree is dropped.
