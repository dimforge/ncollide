use std::mem;
use std::gc::{GC, Gc};
use std::cell::RefCell;
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::has_uid::HasUid;
use na;
use broad::Dispatcher;
use bounding_volume::{HasBoundingVolume, LooseBoundingVolume};
use math::Scalar;

/// Association of an object with its loose bounding volume.
#[deriving(Clone)]
pub struct BoundingVolumeProxy<B, BV> {
    /// The objects loose bounding volume.
    bounding_volume: BV,
    /// The object.
    body:            B
}

impl<BV: LooseBoundingVolume, B: HasBoundingVolume<BV>> BoundingVolumeProxy<B, BV> {
    /// Builds a new brute force broad phase based on loose bounding volumes.
    ///
    /// # Arguments:
    /// * `b` - collision dispatcher.
    /// * `margin` - loosening margin.
    pub fn new(b: B, margin: Scalar) -> BoundingVolumeProxy<B, BV> {
        BoundingVolumeProxy {
            bounding_volume: b.bounding_volume().loosened(margin),
            body:            b
        }
    }

    /// Updates this proxy.
    ///
    /// Returns `true` if the stored bounding volume has been changed.
    pub fn update(&mut self, margin: &Scalar) -> bool {
        let mut new_bv = self.body.bounding_volume();

        if !self.bounding_volume.contains(&new_bv) {
            new_bv.loosen(margin.clone());
            self.bounding_volume = new_bv;

            true
        }
        else {
            false
        }
    }
}

/// Broad phase with quadratic complexity but sped up using loose bounding volumes.
///
/// Interference detection is executed only for objects which have their bounding volumes updated.
///
/// Do not use this broad phase.
pub struct BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
    objects:    Vec<Gc<RefCell<BoundingVolumeProxy<B, BV>>>>, // active   objects
    sobjects:   Vec<Gc<RefCell<BoundingVolumeProxy<B, BV>>>>, // inactive objects
    rb2bv:      HashMap<uint, uint, UintTWHash>,
    pairs:      HashMap<Pair<Gc<RefCell<BoundingVolumeProxy<B, BV>>>>, DV, PairTWHash>, // pair manager
    dispatcher: D,
    margin:     Scalar,
    to_update:  Vec<Gc<RefCell<BoundingVolumeProxy<B, BV>>>>,
    update_off: uint // incremental pairs removal index
}

impl<B:  'static + HasBoundingVolume<BV> + HasUid + Clone,
     BV: 'static + LooseBoundingVolume + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
    /// Creates a new bounding volume based brute force broad phase.
    pub fn new(dispatcher: D, margin: Scalar) -> BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
        BruteForceBoundingVolumeBroadPhase {
            objects:    Vec::new(),
            sobjects:   Vec::new(),
            to_update:  Vec::new(),
            rb2bv:      HashMap::new(UintTWHash::new()),
            pairs:      HashMap::new(PairTWHash::new()),
            dispatcher: dispatcher,
            update_off: 0,
            margin:     margin
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> uint {
        self.pairs.len()
    }

    /// Adds an element to this broad phase.
    #[inline]
    pub fn add(&mut self, rb: B) {
        let proxy = box(GC) RefCell::new(BoundingVolumeProxy::new(rb, self.margin.clone()));
        self.objects.push(proxy);
        self.to_update.push(proxy);
    }

    /// Removes an element from this broad phase.
    #[inline]
    pub fn remove(&mut self, _: &B) {
        fail!("Not yet implemented.");
    }

    /// Marks and object as active or inactive. The bounding volume of an inactive object is never
    /// updated. Activating/deactivating an already active/inactive objects leads to undefined
    /// behaviour.
    pub fn set_active(&mut self, b: &B, active: bool) {
        let (key, at) =
            match self.rb2bv.find_mut(&b.uid()) {
                None    => fail!("Unable to change the active state of an unknown object."),
                Some(i) => {
                    if active {
                        // remove from sobjects…
                        let proxy  = self.sobjects[*i];
                        let lproxy = self.sobjects.pop().unwrap();
                        *self.sobjects.get_mut(*i) = lproxy;

                        // … then add to objects
                        self.objects.push(proxy);

                        let mut at = self.objects.len() - 1;

                        mem::swap(&mut at, i);

                        (lproxy, at)
                    }
                    else {
                        // remove from objects…
                        let proxy  = self.objects[*i];
                        let lproxy = self.objects.pop().unwrap();
                        *self.objects.get_mut(*i) = lproxy;

                        // … then add to sobjects
                        self.sobjects.push(proxy);

                        let mut at = self.sobjects.len() - 1;

                        mem::swap(&mut at, i);

                        (lproxy, at)
                    }
                }
            };

        self.rb2bv.insert(key.uid(), at);
    }

    /// Updates the collision pairs based on the objects bounding volumes.
    pub fn update(&mut self) {
        let mut new_colls = 0u;

        for b in self.objects.iter_mut() {
            let margin = self.margin;
            if b.borrow_mut().update(&margin) {
                self.to_update.push(b.clone())
            }
        }

        for &b1 in self.to_update.iter() {
            for &b2 in self.objects.iter() {
                let bb1 = b1.borrow();
                let bb2 = b2.borrow();
                if self.dispatcher.is_valid(&bb1.body, &bb2.body) {
                    if bb2.bounding_volume.intersects(&bb1.bounding_volume) {
                        let dispatcher = &mut self.dispatcher;
                        let _ = self.pairs.find_or_insert_lazy(
                            Pair::new(b1, b2),
                            || dispatcher.dispatch(&bb1.body, &bb2.body)
                        );

                        new_colls = new_colls + 1;
                    }
                }
            }
        }

        if new_colls != 0 {
            let len          = self.pairs.len();
            let num_removals = na::clamp(new_colls, (len / 10), len);

            for i in range(self.update_off, self.update_off + num_removals) {
                let id = i % self.pairs.len();

                let remove = {
                    let elts  = self.pairs.elements();
                    let entry = &elts[id];

                    let bf = entry.key.first.borrow();
                    let bs = entry.key.second.borrow();
                    if !bf.bounding_volume.intersects(&bs.bounding_volume) {
                        true
                    }
                    else {
                        false
                    }
                };

                if remove {
                    self.pairs.remove_elem_at(id);
                }
            }

            self.update_off = (self.update_off + num_removals) % self.pairs.len();
        }

        self.to_update.clear()
    }
}

#[cfg(all(test, dim3, f64))]
mod test {
    use super::BruteForceBoundingVolumeBroadPhase;
    use std::rc::Rc;
    use std::cell::RefCell;
    use std::vec::Vec;
    use na::{Vec3, Iso3};
    use na;
    use geom::Ball;
    use bounding_volume::WithAABB;
    use broad::NoIdDispatcher;

    #[test]
    fn test_bfbv_empty() {
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Rc<Shape>> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 30.0, j as f64 * 30.0, 0.0);
                bf.add(Rc::new(WithAABB(Iso3::new(t, na::zero()), ball)));
            }
        }

        bf.update();

        assert_eq!(bf.num_interferences(), 0)
    }

    #[test]
    fn test_bfbv_nbh_collide() {
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Rc<Shape>> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
                bf.add(Rc::new(WithAABB(Iso3::new(t, na::zero()), ball)));
            }
        }

        bf.update();

        assert_eq!(
            bf.num_interferences(),
            (18 * 18 * 8 + // internal rectangles have 8 neighbors
             18 * 4 * 5  + // border (excluding corners) rectangles have 5 neighbors
             4 * 3)        // corners have 3 neighbors
             / 2            // remove all duplicates
        )
    }

    #[test]
    fn test_dbvt_nbh_move_collide() {
        type Shape = Rc<RefCell<WithAABB<Ball>>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        let mut to_move = Vec::new();

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
                let to_add = Rc::new(RefCell::new(WithAABB(Iso3::new(t, na::zero()), ball)));
                bf.add(to_add.clone());
                to_move.push(to_add);
            }
        }

        for e in to_move.iter_mut() {
            let mut pe = e.borrow_mut();
            let m      = pe.m().clone();
            let g      = pe.g().clone();
            *pe        = WithAABB(na::append_translation(&m, &Vec3::new(10.0, 10.0, 10.0)), g)
        }

        bf.update();

        assert_eq!(
            bf.num_interferences(),
            (18 * 18 * 8 + // internal rectangles have 8 neighbors
             18 * 4 * 5  + // border (excluding corners) rectangles have 5 neighbors
             4 * 3)        // corners have 3 neighbors
            / 2            // remove all duplicates
        )
    }

    #[test]
    fn test_bfbv_quadratic_collide() {
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Rc<Shape>> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        for _ in range(0, 400) {
            bf.add(Rc::new(WithAABB(Iso3::new(na::zero(), na::zero()), ball)))
        }

        bf.update();

        assert_eq!(bf.num_interferences(), (399 * (399 + 1)) / 2)
    }
}
