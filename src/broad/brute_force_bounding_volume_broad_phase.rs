use std::mem;
use std::cell::RefCell;
use std::rc::Rc;
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::has_uid::HasUid;
use na;
use broad::Dispatcher;
use bounding_volume::{HasBoundingVolume, BoundingVolume};
use math::Scalar;


/// Association of an object with its loose bounding volume.
#[deriving(Clone)]
pub struct BoundingVolumeProxy<B, BV> {
    /// The objects loose bounding volume.
    bounding_volume: BV,
    /// The object.
    body:            B
}

impl<N, B, BV> BoundingVolumeProxy<B, BV>
    where N: Scalar,
          BV: BoundingVolume<N>,
          B:  HasBoundingVolume<BV> {
    /// Builds a new brute force broad phase based on loose bounding volumes.
    ///
    /// # Arguments:
    /// * `b` - collision dispatcher.
    /// * `margin` - loosening margin.
    pub fn new(b: B, margin: N) -> BoundingVolumeProxy<B, BV> {
        BoundingVolumeProxy {
            bounding_volume: b.bounding_volume().loosened(margin),
            body:            b
        }
    }

    /// Updates this proxy.
    ///
    /// Returns `true` if the stored bounding volume has been changed.
    pub fn update(&mut self, margin: N) -> bool {
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
pub struct BruteForceBoundingVolumeBroadPhase<N, B, BV, D, DV> {
    objects:    Vec<Rc<RefCell<BoundingVolumeProxy<B, BV>>>>, // active   objects
    sobjects:   Vec<Rc<RefCell<BoundingVolumeProxy<B, BV>>>>, // inactive objects
    rb2bv:      HashMap<uint, uint, UintTWHash>,
    pairs:      HashMap<Pair<Rc<RefCell<BoundingVolumeProxy<B, BV>>>>, DV, PairTWHash>, // pair manager
    dispatcher: D,
    margin:     N,
    to_update:  Vec<Rc<RefCell<BoundingVolumeProxy<B, BV>>>>,
    update_off: uint // incremental pairs removal index
}

impl<N, B, BV, D, DV> BruteForceBoundingVolumeBroadPhase<N, B, BV, D, DV>
    where N: Scalar,
          B:  'static + HasBoundingVolume<BV> + HasUid + Clone,
          BV: 'static + BoundingVolume<N> + Clone,
          D:  Dispatcher<B, B, DV> {
    /// Creates a new bounding volume based brute force broad phase.
    pub fn new(dispatcher: D, margin: N) -> BruteForceBoundingVolumeBroadPhase<N, B, BV, D, DV> {
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
        let proxy = Rc::new(RefCell::new(BoundingVolumeProxy::new(rb, self.margin.clone())));
        self.objects.push(proxy.clone());
        self.to_update.push(proxy);
    }

    /// Removes an element from this broad phase.
    #[inline]
    pub fn remove(&mut self, _: &B) {
        panic!("Not yet implemented.");
    }

    /// Marks and object as active or inactive. The bounding volume of an inactive object is never
    /// updated. Activating/deactivating an already active/inactive objects leads to undefined
    /// behaviour.
    pub fn set_active(&mut self, b: &B, active: bool) {
        let (key, at) =
            match self.rb2bv.find_mut(&b.uid()) {
                None    => panic!("Unable to change the active state of an unknown object."),
                Some(i) => {
                    if active {
                        // remove from sobjects…
                        let proxy  = self.sobjects[*i].clone();
                        let lproxy = self.sobjects.pop().unwrap();
                        *self.sobjects.get_mut(*i) = lproxy.clone();

                        // … then add to objects
                        self.objects.push(proxy);

                        let mut at = self.objects.len() - 1;

                        mem::swap(&mut at, i);

                        (lproxy, at)
                    }
                    else {
                        // remove from objects…
                        let proxy  = self.objects[*i].clone();
                        let lproxy = self.objects.pop().unwrap();
                        *self.objects.get_mut(*i) = lproxy.clone();

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
            if b.borrow_mut().update(margin) {
                self.to_update.push(b.clone())
            }
        }

        for b1 in self.to_update.iter() {
            for b2 in self.objects.iter() {
                let bb1 = b1.borrow();
                let bb2 = b2.borrow();
                if self.dispatcher.is_valid(&bb1.body, &bb2.body) {
                    if bb2.bounding_volume.intersects(&bb1.bounding_volume) {
                        let dispatcher = &mut self.dispatcher;
                        let _ = self.pairs.find_or_insert_lazy(
                            Pair::new(b1.clone(), b2.clone()),
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
