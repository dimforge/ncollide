//! A Dynamic Bounding Volume Tree.

use std::cell::RefCell;
use std::rc::Rc;
use std::ptr;
use std::mem;
use utils::data::owned_allocation_cache::OwnedAllocationCache;
use na::{FloatVec, Translation};
use na;
use bounding_volume::BoundingVolume;
use partitioning::bvt_visitor::BVTVisitor;
use math::{Scalar, Point};


#[derive(RustcEncodable, RustcDecodable)]
enum UpdateState {
    NeedsShrink,
    UpToDate
}

type Cache<P, B, BV> = OwnedAllocationCache<DBVTInternal<P, B, BV>>;

/// A Dynamic Bounding Volume Tree.
pub struct DBVT<P, B, BV> {
    cache: Cache<P, B, BV>,
    tree:  Option<DBVTNode<P, B, BV>>,
    len:   usize
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

#[old_impl_check]
impl<N, P, V, B, BV> DBVT<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + BoundingVolume<N> + Translation<V> + Clone,
          B:  'static + Clone {
    /// Removes a leaf from the tree. Fails if the tree is empty.
    pub fn remove(&mut self, leaf: &mut Rc<RefCell<DBVTLeaf<P, B, BV>>>) {
        let self_tree = self.tree.take().expect("This tree was empty.");

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
            None    => {
                leaf.borrow_mut().parent = DBVTLeafState::Root;
                Some(DBVTNode::Leaf(leaf))
            },
            Some(t) => Some(DBVTNode::Internal(t.insert(&mut self.cache, leaf)))
        };

        self.len = self.len + 1;
    }

    /// Visit this tree using… a visitor!
    pub fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match self.tree {
            Some(ref t) => t.visit(visitor),
            None        => { }
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

#[old_impl_check]
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
            state:           UpdateState::UpToDate
        }
    }
}

#[allow(raw_pointer_derive)]
#[derive(Clone)]
/// State of a leaf.
enum DBVTLeafState<P, B, BV> {
    /// This leaf is the root of a tree.
    Root,
    /// This leaf is the right child of another node.
    RightChildOf(*mut DBVTInternal<P, B, BV>),
    /// This leaf is the left child of another node.
    LeftChildOf(*mut DBVTInternal<P, B, BV>),
    /// This leaf is detached from any tree.
    Detached
}

impl<P, B, BV> DBVTLeafState<P, B, BV> {
    /// Indicates whether this leaf is the root.
    #[inline]
    pub fn is_root(&self) -> bool {
        match *self {
            DBVTLeafState::Root => true,
            _ => false
        }
    }

    /// Indicates whether this leaf is detached.
    #[inline]
    pub fn is_detached(&self) -> bool {
        match *self {
            DBVTLeafState::Detached => true,
            _ => false
        }
    }

    /// Returns a pointer to this leaf parent and `true` if it is the left child.
    #[inline]
    fn unwrap(self) -> (bool, *mut DBVTInternal<P, B, BV>) {
        match self {
            DBVTLeafState::RightChildOf(p) => (false, p),
            DBVTLeafState::LeftChildOf(p)  => (true, p),
            _ => panic!("Attempting to unwrap a root or detached node.")
        }
    }
}

/// Leaf of a Dynamic Bounding Volume Tree.
#[derive(Clone)]
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
            DBVTNode::Internal(i) => i,
            _ => panic!("DBVT internal error: this is not an internal node.")
        }
    }

    fn invalidate(&mut self) -> DBVTNode<P, B, BV> {
        let mut res = DBVTNode::Invalid;

        mem::swap(&mut res, self);

        res
    }
}

impl<P, B, BV> DBVTInternal<P, B, BV> {
    fn is_right_internal_node(&self, r: &mut DBVTInternal<P, B, BV>) -> bool
    {
        match self.right {
            DBVTNode::Internal(ref i) => &**i as *const DBVTInternal<P, B, BV> == &*r as *const DBVTInternal<P, B, BV>,
            _ => false
        }
    }
}

#[old_impl_check]
impl<N, P: Point<N, V>, V, B: 'static, BV: Translation<V> + 'static> DBVTLeaf<P, B, BV> {
    /// Creates a new leaf.
    pub fn new(bounding_volume: BV, object: B) -> DBVTLeaf<P, B, BV> {
        DBVTLeaf {
            center:          na::orig::<P>() + bounding_volume.translation(),
            bounding_volume: bounding_volume,
            object:          object,
            parent:          DBVTLeafState::Detached
        }
    }

    /// Tests if this node is the root.
    pub fn is_root(&self) -> bool {
        self.parent.is_root()
    }

    /// Tests if this node has no parent.
    pub fn is_detached(&self) -> bool {
        self.parent.is_detached()
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
        if !self.is_root() {
            let (is_left, p) = mem::replace(&mut self.parent, DBVTLeafState::Detached).unwrap();

            let pp           = unsafe { (*p).parent };
            let parent_left  = unsafe { (*p).left.invalidate() };
            let parent_right = unsafe { (*p).right.invalidate() };

            let mut other = if is_left { parent_right } else { parent_left };

            if !pp.is_null() {
                let is_p_right_to_pp = unsafe { (*pp).is_right_internal_node(&mut *p) };
                // we are far away from the root
                unsafe {
                    match other {
                        DBVTNode::Internal(ref mut i) => i.parent = pp,
                        DBVTNode::Leaf(ref mut l)     => {
                            (**l).borrow_mut().parent =
                                if is_p_right_to_pp { DBVTLeafState::RightChildOf(pp) }
                                else { DBVTLeafState::LeftChildOf(pp) }
                        },
                        DBVTNode::Invalid => unreachable!()
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

                    (*pp).state = UpdateState::NeedsShrink;
                }

                Some(curr_root)
            }
            else {
                // the root changes to the other child
                match other {
                    DBVTNode::Internal(ref mut i) => i.parent = ptr::null_mut(),
                    DBVTNode::Leaf(ref l)         => (**l).borrow_mut().parent = DBVTLeafState::Root,
                    DBVTNode::Invalid             => unreachable!()
                }

                Some(other)
            }
        }
        else {
            self.parent = DBVTLeafState::Detached;

            // the tree becomes empty
            None
        }
    }
}

#[old_impl_check]
impl<N, P, V, BV, B> DBVTNode<P, B, BV>
    where N: Scalar,
          P:  Point<N, V>,
          V:  FloatVec<N>,
          BV: 'static + BoundingVolume<N>,
          B: 'static {
    fn sqdist_to(&self, to: &P) -> N {
        match *self {
            DBVTNode::Internal(ref i) => na::sqdist(&i.center, to),
            DBVTNode::Leaf(ref l)     => {
                let bl = l.borrow();
                na::sqdist(&bl.center, to)
            },
            DBVTNode::Invalid => unreachable!()
        }
    }
}

#[old_impl_check]
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

#[old_impl_check]
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
        assert!(to_insert.borrow().is_detached(), "Cannot insert the same node twice.");

        let mut bto_insert = (*to_insert).borrow_mut();
        let pto_insert     = &mut *bto_insert;

        match self {
            DBVTNode::Internal(i) => {
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
                            DBVTNode::Internal(ref mut ci) => {
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
                            DBVTNode::Leaf(ref l) => {
                                let mut bl       = (**l).borrow_mut();
                                let     pl       = &mut *bl;
                                let mut internal = cache.alloc(DBVTInternal::new(
                                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                                    parent,
                                    DBVTNode::Leaf(l.clone()),
                                    DBVTNode::Leaf(to_insert.clone())));

                                pl.parent = DBVTLeafState::LeftChildOf(&mut *internal as *mut DBVTInternal<P, B, BV>);
                                pto_insert.parent =
                                    DBVTLeafState::RightChildOf(&mut *internal as *mut DBVTInternal<P, B, BV>);

                                if left {
                                    (*parent).left = DBVTNode::Internal(internal)
                                }
                                else {
                                    (*parent).right = DBVTNode::Internal(internal)
                                }

                                break;
                            },
                            DBVTNode::Invalid => unreachable!()
                        }
                    }
                }

                mut_internal
            },
            DBVTNode::Leaf(l) => {
                let     cl = l.clone();
                let mut bl = (*cl).borrow_mut();
                let     pl = &mut *bl;

                // create the root
                let mut root = cache.alloc(DBVTInternal::new(
                    pl.bounding_volume.merged(&pto_insert.bounding_volume),
                    ptr::null_mut(),
                    DBVTNode::Leaf(l),
                    DBVTNode::Leaf(to_insert.clone())));

                pl.parent = DBVTLeafState::LeftChildOf(&mut *root as *mut DBVTInternal<P, B, BV>);
                pto_insert.parent = DBVTLeafState::RightChildOf(&mut *root as *mut DBVTInternal<P, B, BV>);

                root
            },
            DBVTNode::Invalid => unreachable!()
        }
    }

    fn visit<Vis: BVTVisitor<B, BV>>(&self, visitor: &mut Vis) {
        match *self {
            DBVTNode::Internal(ref i) => {
                if visitor.visit_internal(&i.bounding_volume) {
                    i.left.visit(visitor);
                    i.right.visit(visitor);
                }
            },
            DBVTNode::Leaf(ref l) => {
                let bl = l.borrow();
                visitor.visit_leaf(&bl.object, &bl.bounding_volume)
            },
            DBVTNode::Invalid => unreachable!()
        }
    }
}

// XXX: Drop should be implemented to invalidate the leaves parents when the tree is dropped.
