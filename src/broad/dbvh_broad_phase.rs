use std::cast;
use std::managed;
use nalgebra::traits::translation::Translation;
use utils::empty::Empty;
use broad::broad_phase::BroadPhase;
use bounding_volume::bounding_volume::{BoundingVolume, LooseBoundingVolume};
use bounding_volume::has_bounding_volume::HasBoundingVolume;

pub trait HasDBVHProxy<BV, V>
{
    fn proxy<'r>(&'r self)         -> &'r DBVHProxy<BV, V>;
    fn proxy_mut<'r>(&'r mut self) -> &'r mut DBVHProxy<BV, V>;
}

// Note: this type cannot depend on RB since this would create a recursive
// type.
pub struct DBVHProxy<BV, V>
{ priv node: @mut Empty }

impl<BV, V> DBVHProxy<BV, V>
{
    fn new<RB>(node: @mut DBVHNode<BV, RB, V>) -> DBVHProxy<BV, V>
    {
        DBVHProxy {
            node: unsafe { cast::transmute::<@mut DBVHNode<BV, RB, V>, @mut Empty>(node) }
        }
    }

    fn node<RB>(&self) -> @mut DBVHNode<BV, RB, V>
    { unsafe { cast::transmute::<@mut Empty, @mut DBVHNode<BV, RB, V>>(self.node) } }
}

enum UpdateState
{
    NeedsShrink,
    NeedsEnlarge,
    UpToDate
}

pub struct DBVHNode<BV, RB, V>
{
    bounding_volume: BV,
    center:          V,
    body:            Option<@mut RB>,
    left:            Option<@mut DBVHNode<BV, RB, V>>,
    right:           Option<@mut DBVHNode<BV, RB, V>>,
    parent:          Option<@mut DBVHNode<BV, RB, V>>,
    state:           UpdateState
}

impl<BV: Translation<V> + BoundingVolume, RB, V> DBVHNode<BV, RB, V>
{
    pub fn unlink(@mut self, root: @mut DBVHNode<BV, RB, V>)
        -> Option<@mut DBVHNode<BV, RB, V>>
        {
            assert!(self.left.is_none());
            assert!(self.right.is_none());

            let mut current_root = root;

            match self.parent
            {
                Some(parent) => {
                    let other = if managed::mut_ptr_eq(self, parent.right.unwrap())
                    { parent.left.unwrap() }
                    else
                    { parent.right.unwrap() };

                    other.parent = parent.parent;

                    match parent.parent
                    {
                        Some(pp) => {
                            if managed::mut_ptr_eq(pp.right.unwrap(), parent)
                            { pp.right = Some(other) }
                            else
                            {
                                assert!(managed::mut_ptr_eq(pp.left.unwrap(), parent));
                                pp.left = Some(other)
                            }

                            other.parent.unwrap().state = NeedsShrink;
                        }

                        None => current_root = other
                    }

                    other.invariant();

                    if managed::mut_ptr_eq(parent, current_root)
                    { Some(other) }
                    else
                    { Some(current_root) }
                }
                None => None
            }
        }

    fn invariant(@mut self)
    {
        assert!(self.parent.is_none() ||
                managed::mut_ptr_eq(self.parent.unwrap().right.unwrap(), self) ||
                managed::mut_ptr_eq(self.parent.unwrap().left.unwrap(), self));
        assert!((self.body.is_none() && self.right.is_some() && self.left.is_some()) ||
                (self.body.is_some() && self.right.is_none() && self.left.is_none()));
    }

    pub fn depth(&self) -> uint
    {
        match self.right
        {
            Some(r) => (1 + r.depth()).max(&(1 + self.left.unwrap().depth())),
            None    => 1
        }
    }

    pub fn num_leaves(&self) -> uint
    {
        match self.right
        {
            Some(r) => r.num_leaves() + self.left.unwrap().num_leaves(),
            None    => 1
        }
    }

    pub fn update(&mut self)
    {
        if self.body.is_none()
        {
            self.bounding_volume =
                self.left.unwrap().bounding_volume.merged(&self.right.unwrap().bounding_volume);
        }

        self.center = self.bounding_volume.translation();
        self.state  = UpToDate;
    }

    pub fn update_shrink(&mut self)
    {
        match self.state
        {
            NeedsShrink => {
                self.update();
                match self.parent
                {
                    Some(p) => p.state = NeedsShrink,
                    None => { }
                }
            }
            _ => { }
        }
    }

    pub fn collect_interferences(&self,
                                 other:  &mut DBVHNode<BV, RB, V>,
                                 pairs: &mut ~[(@mut RB, @mut RB)])
    {
        match other.body
        {
            // We assume that we already tested the intersection with the other
            Some(b) => pairs.push((self.body.unwrap(), b)),
            None => {
                if self.bounding_volume.intersects(&other.left.unwrap().bounding_volume)
                { self.collect_interferences(other.left.unwrap(), pairs) }

                if self.bounding_volume.intersects(&other.right.unwrap().bounding_volume)
                { self.collect_interferences(other.right.unwrap(), pairs) }

                other.update_shrink();
            }
        }
    }
}

pub struct DBVHBroadPhase<BV, RB, V, N>
{
    panding: ~[@mut DBVHNode<BV, RB, V>],
    root:    Option<@mut DBVHNode<BV, RB, V>>,
    margin:  N
}

impl<BV, RB, V, N> DBVHBroadPhase<BV, RB, V, N>
{
    pub fn new(margin: N) -> DBVHBroadPhase<BV, RB, V, N>
    {
        DBVHBroadPhase {
            panding: ~[],
            root:    None,
            margin:  margin
        }
    }
}

impl<BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     RB: 'static + HasBoundingVolume<BV> + HasDBVHProxy<BV, V>,
     V:  'static,
     N:  'static + Clone> BroadPhase<RB> for DBVHBroadPhase<BV, RB, V, N>
{
    fn add(&mut self, rb: @mut RB)
    {
        // FIXME: test that the body has not been added already?
        let bv = rb.bounding_volume().loosened(self.margin.clone());

        let new_node = @mut DBVHNode {
            center:          bv.translation(),
            bounding_volume: bv,
            body:            Some(rb),
            left:            None,
            right:           None,
            parent:          None,
            state:           UpToDate
        };

        let proxy = rb.proxy_mut();
        *proxy = DBVHProxy::new(new_node);

        self.panding.push(new_node);
    }

    fn remove(&mut self, _: @mut RB)
    {
        fail!("Not yet implemented.");
    }

    fn collision_pairs(&mut self, _: &[@mut RB]) -> ~[(@mut RB, @mut RB)]
    {
        fail!("Not yet implemented.");
    }
}
