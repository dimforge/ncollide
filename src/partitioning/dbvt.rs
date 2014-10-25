//! A Dynamic Bounding Volume Tree.

use std::cell::RefCell;
use std::rc::Rc;
use std::ptr;
use std::mem;
use utils::data::owned_allocation_cache::OwnedAllocationCache;
use na::{FloatVec, Translation};
use na;
use bounding_volume::BoundingVolume;
use partitioning::bvt_visitor::{BVTVisitor, BoundingVolumeInterferencesCollector};
use math::{Scalar, Point};


#[deriving(Encodable, Decodable)]
enum UpdateState {
    NeedsShrink,
    UpToDate
}

type Cache<P, B, BV> = OwnedAllocationCache<DBVTInternal<P, B, BV>>;

/// A Dynamic Bounding Volume Tree.
pub struct DBVT<P, B, BV> {
    cache: Cache<P, B, BV>,
    tree:  Option<DBVTNode<P, B, BV>>,
    len:   uint
}

impl<P, B, BV> DBVT<P, B, BV> {
    /// Creates a new Dynamic Bounding Volume Tree.
    pub fn new() -> DBVT<P, B, BV> {
        DBVT {
            cache: OwnedAllocationCache::new(),
            tree:  None,
            len:   0
        }
    }
}

impl<N, P, V, B, BV> DBVT<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + BoundingVolume<N> + Translation<V> + Clone,
          B:  'static + Clone {
    /// Removes a leaf from the tree. Fails if the tree is empty.
    pub fn remove(&mut self, leaf: &mut Rc<RefCell<DBVTLeaf<P, B, BV>>>) {
        let self_tree = self.tree.take().unwrap();

        let mut bleaf = (*leaf).borrow_mut();
        self.tree = bleaf.unlink(&mut self.cache, self_tree);
        self.len  = self.len - 1;
    }

    // FIXME: it feels strange that this method takes (B, BV) in this order while the leaves
    // constructor takes (BV, B)…
    /// Creates, inserts, and returns a new leaf with the given content.
    pub fn insert_new(&mut self, b: B, bv: BV) -> Rc<RefCell<DBVTLeaf<P, B, BV>>> {
        let leaf = Rc::new(RefCell::new(DBVTLeaf::new(bv, b)));

        self.insert(leaf.clone());

        leaf
    }

    /// Inserts a leaf to the tree.
    pub fn insert(&mut self, leaf: Rc<RefCell<DBVTLeaf<P, B, BV>>>) {
        let mut self_tree = None;
        mem::swap(&mut self_tree, &mut self.tree);

        self.tree = match self_tree {
            None    => Some(Leaf(leaf)),
            Some(t) => Some(Internal(t.insert(&mut self.cache, leaf)))
        };

        self.len = self.len + 1;
    }

    /// Visit this tree using… a visitor!
    pub fn visit<Vis: BVTVisitor<Rc<RefCell<DBVTLeaf<P, B, BV>>>, BV>>(&self, visitor: &mut Vis) {
        match self.tree {
            Some(ref t) => t.visit(visitor),
            None        => { }
        }
    }

//    /// Visit this tree using… a visitor! Visitor arguments are mutable.
//    pub fn visit_mut<Vis: BVTVisitor<GcMut<DBVTLeaf<P, B, BV>>, BV>>(&mut self, visitor: &mut Vis) {
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
                                   leaf: &DBVTLeaf<P, B, BV>,
                                   out:  &mut Vec<Rc<RefCell<DBVTLeaf<P, B, BV>>>>) {
        match self.tree {
            Some(ref tree) => tree.interferences_with_leaf(leaf, out),
            None           => { }
        }
    }

    /// Finds all interferences between this tree and another one.
    pub fn interferences_with_tree(&self,
                                   leaf: &DBVT<P, B, BV>,
                                   out:  &mut Vec<Rc<RefCell<DBVTLeaf<P, B, BV>>>>) {
        match (&self.tree, &leaf.tree) {
            (&Some(ref a), &Some(ref b)) => a.interferences_with_tree(b, out),
            (&None, _) => { },
            (_, &None) => { }
        }
    }
}

/// Node of the Dynamic Bounding Volume Tree.
enum DBVTNode<P, B, BV> {
    Internal(Box<DBVTInternal<P, B, BV>>),
    Leaf(Rc<RefCell<DBVTLeaf<P, B, BV>>>),
    Invalid
}

/// Internal node of a DBVT. An internal node always has two children.
struct DBVTInternal<P, B, BV> {
    /// The bounding volume of this node. It always encloses both its children bounding volumes.
    bounding_volume: BV,
    /// The center of this node bounding volume.
    center:          P,
    /// This node left child.
    left:            DBVTNode<P, B, BV>,
    /// This node right child.
    right:           DBVTNode<P, B, BV>,
    /// This node parent.
    parent:          *mut DBVTInternal<P, B, BV>,

    state:           UpdateState
}

impl<N, P: Point<N, V>, V, BV: Translation<V>, B> DBVTInternal<P, B, BV> {
    /// Creates a new internal node.
    fn new(bounding_volume: BV,
           parent:          *mut DBVTInternal<P, B, BV>,
           left:            DBVTNode<P, B, BV>,
           right:           DBVTNode<P, B, BV>)
           -> DBVTInternal<P, B, BV> {
        DBVTInternal {
            center:          na::orig::<P>() + bounding_volume.translation(),
            bounding_volume: bounding_volume,
            left:            left,
            right:           right,
            parent:          parent,
            state:           UpToDate
        }
    }
}

#[allow(raw_pointer_deriving)]
#[deriving(Clone)]
/// State of a leaf.
enum DBVTLeafState<P, B, BV> {
    /// This leaf is the right child of another node.
    RightChildOf(*mut DBVTInternal<P, B, BV>),
    /// This leaf is the left child of another node.
    LeftChildOf(*mut DBVTInternal<P, B, BV>),
    /// This leaf is detached from any tree.
    Detached
}

impl<P, B, BV> DBVTLeafState<P, B, BV> {
    /// Indicates whether this leaf is detached.
    #[inline]
    pub fn is_detached(&self) -> bool {
        match *self {
            Detached => true,
            _        => false
        }
    }

    /// Returns a pointer to this leaf parent and `true` if it is the left child.
    #[inline]
    fn unwrap(self) -> (bool, *mut DBVTInternal<P, B, BV>) {
        match self {
            RightChildOf(p) => (false, p),
            LeftChildOf(p)  => (true, p),
            _               => panic!("Attempting to unwrap a detached node.")
        }
    }
}

/// Leaf of a Dynamic Bounding Volume Tree.
#[deriving(Clone)]
pub struct DBVTLeaf<P, B, BV> {
    /// The bounding volume of this node.
    pub bounding_volume: BV,
    /// The center of this node bounding volume.
    pub center:          P,
    /// An user-defined object.
    pub object:          B,
    /// This node parent.
    parent:              DBVTLeafState<P, B, BV>
}

impl<P, B, BV> DBVTNode<P, B, BV> {
    fn take_internal(self) -> Box<DBVTInternal<P, B, BV>> {
        match self {
            Internal(i) => i,
            _ => panic!("DBVT internal error: this is not an internal node.")
        }
    }

    fn invalidate(&mut self) -> DBVTNode<P, B, BV> {
        let mut res = Invalid;

        mem::swap(&mut res, self);

        res
    }
}

impl<P, B, BV> DBVTInternal<P, B, BV> {
    fn is_right_internal_node(&self, r: &mut DBVTInternal<P, B, BV>) -> bool
    {
        match self.right {
            Internal(ref i) => &**i as *const DBVTInternal<P, B, BV> == &*r as *const DBVTInternal<P, B, BV>,
            _ => false
        }
    }
}

impl<N, P: Point<N, V>, V, B: 'static, BV: Translation<V> + 'static> DBVTLeaf<P, B, BV> {
    /// Creates a new leaf.
    pub fn new(bounding_volume: BV, object: B) -> DBVTLeaf<P, B, BV> {
        DBVTLeaf {
            center:          na::orig::<P>() + bounding_volume.translation(),
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
              cache:     &mut Cache<P, B, BV>,
              curr_root: DBVTNode<P, B, BV>) -> Option<DBVTNode<P, B, BV>> {
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
                            (**l).borrow_mut().parent =
                                if is_p_right_to_pp { RightChildOf(pp) } else { LeftChildOf(pp) }
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
                    Internal(ref mut i) => i.parent = ptr::null_mut(),
                    Leaf(ref l)         => (**l).borrow_mut().parent = Detached,
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

impl<N, P, V, BV, B> DBVTNode<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + BoundingVolume<N>,
          B: 'static {
    fn sqdist_to(&self, to: &P) -> N {
        match *self {
            Internal(ref i) => na::sqdist(&i.center, to),
            Leaf(ref l)     => {
                let bl = l.borrow();
                na::sqdist(&bl.center, to)
            },
            Invalid         => unreachable!()
        }
    }
}

impl<N, P, V, B, BV> DBVTInternal<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + Translation<V> + BoundingVolume<N>,
          B:  'static {
    fn is_closest_to_left(&self, pt: &P) -> bool {
        self.right.sqdist_to(pt) > self.left.sqdist_to(pt)
    }
}

impl<N, P, V, BV, B> DBVTNode<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + Translation<V> + BoundingVolume<N>,
          B:  'static {
    /// Inserts a new leaf on this tree.
    fn insert(self,
              cache:     &mut Cache<P, B, BV>,
              to_insert: Rc<RefCell<DBVTLeaf<P, B, BV>>>)
              -> Box<DBVTInternal<P, B, BV>> {

        let mut bto_insert = (*to_insert).borrow_mut();
        let pto_insert     = bto_insert.deref_mut();

        match self {
            Internal(i) => {
                /*
                 * NOTE: the insersion is done with unsafe pointers.
                 * This is so because using &mut references dont seem to be possible since we have
                 * to take successive references to nodes contents.
                 */

                let mut mut_internal = i;
                let mut parent       = &mut *mut_internal as *mut DBVTInternal<P, B, BV>;

                unsafe {
                    (*parent).bounding_volume.merge(&pto_insert.bounding_volume);

                    // iteratively go to the leaves
                    let mut curr;
                    let mut left;

                    if (*parent).is_closest_to_left(&pto_insert.center) {
                        curr = &mut (*parent).left as *mut DBVTNode<P, B, BV>;
                        left = true;
                    }
                    else {
                        curr = &mut (*parent).right as *mut DBVTNode<P, B, BV>;
                        left = false;
                    }

                    loop {
                        match *curr {
                            Internal(ref mut ci) => {
                                // FIXME: we could avoid the systematic merge
                                ci.bounding_volume.merge(&pto_insert.bounding_volume);

                                if ci.is_closest_to_left(&pto_insert.center) { // FIXME
                                    curr = &mut ci.left as *mut DBVTNode<P, B, BV>;
                                    left = true;
                                }
                                else {
                                    curr = &mut ci.right as *mut DBVTNode<P, B, BV>;
                                    left = false;
                                }

                                parent = &mut **ci as *mut DBVTInternal<P, B, BV>;
                            },
                            Leaf(ref l) => {
                                let mut bl       = (**l).borrow_mut();
                                let     pl       = bl.deref_mut();
                                let mut internal = cache.alloc(DBVTInternal::new(
                                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                                    parent,
                                    Leaf(l.clone()),
                                    Leaf(to_insert.clone())));

                                pl.parent         = LeftChildOf(&mut *internal as *mut DBVTInternal<P, B, BV>);
                                pto_insert.parent = RightChildOf(&mut *internal as *mut DBVTInternal<P, B, BV>);

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
                let     cl = l.clone();
                let mut bl = (*cl).borrow_mut();
                let     pl = bl.deref_mut();

                // create the root
                let mut root = cache.alloc(DBVTInternal::new(
                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                    ptr::null_mut(),
                    Leaf(l),
                    Leaf(to_insert.clone())));

                pl.parent         = LeftChildOf(&mut *root as *mut DBVTInternal<P, B, BV>);
                pto_insert.parent = RightChildOf(&mut *root as *mut DBVTInternal<P, B, BV>);

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
                               to_test: &DBVTLeaf<P, B, BV>,
                               out:     &mut Vec<Rc<RefCell<DBVTLeaf<P, B, BV>>>>) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&to_test.bounding_volume, out);
        self.visit(&mut visitor)
    }

    fn visit<Vis: BVTVisitor<Rc<RefCell<DBVTLeaf<P, B, BV>>>, BV>>(&self, visitor: &mut Vis) {
        match *self {
            Internal(ref i) => {
                if visitor.visit_internal(&i.bounding_volume) {
                    i.left.visit(visitor);
                    i.right.visit(visitor);
                }
            },
            Leaf(ref l) => {
                let bl = l.borrow();
                visitor.visit_leaf(l, &bl.bounding_volume)
            },
            Invalid => unreachable!()
        }
    }

//    fn visit_mut<Vis: BVTVisitor<GcMut<DBVTLeaf<P, B, BV>>, BV>>(&mut self, visitor: &mut Vis) {
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
//            Invalid => data::unreachable()
//        }
//    }

    /// Finds all interferences between this tree and another one.
    fn interferences_with_tree(&self,
                               to_test: &DBVTNode<P, B, BV>,
                               out:     &mut Vec<Rc<RefCell<DBVTLeaf<P, B, BV>>>>) {
        match (self, to_test) {
            (&Leaf(_), &Leaf(ref lb)) => {
                let blb = lb.borrow();
                self.interferences_with_leaf(blb.deref(), out)
            },
            (&Leaf(ref la), &Internal(_)) => {
                let bla = la.borrow();
                to_test.interferences_with_leaf(bla.deref(), out)
            },
            (&Internal(_), &Leaf(ref lb)) => {
                let blb = lb.borrow();
                self.interferences_with_leaf(blb.deref(), out)
            },
            (&Internal(ref la), &Internal(ref lb)) => {
                // FIXME: la.partial_optimise();
                // FIXME: lb.partial_optimise();

                if (&**la as *const DBVTInternal<P, B, BV> != &**lb as *const DBVTInternal<P, B, BV>) &&
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

// XXX: Drop should be implemented to invalidate the leaves parents when the tree is dropped.
