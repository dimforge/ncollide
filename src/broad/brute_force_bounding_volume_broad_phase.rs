use std::ptr;
use std::util;
use util::hash_map::HashMap;
use util::hash::UintTWHash;
use util::pair::{Pair, PairTWHash};
use broad::Dispatcher;
use bounding_volume::{HasBoundingVolume, LooseBoundingVolume};
use math::N;

/// Association of an object with its loose bounding volume.
pub struct BoundingVolumeProxy<B, BV> {
    /// The objects loose bounding volume.
    bounding_volume: BV,
    /// The object.
    body:            @mut B
}

impl<BV: LooseBoundingVolume, B: HasBoundingVolume<BV>> BoundingVolumeProxy<B, BV> {
    /// Builds a new brute force broad phase based on loose bounding volumes.
    ///
    /// # Arguments:
    ///     * `b` - collision dispatcher.
    ///     * `margin` - loosening margin.
    pub fn new(b: @mut B, margin: N) -> BoundingVolumeProxy<B, BV> {
        BoundingVolumeProxy {
            bounding_volume: b.bounding_volume().loosened(margin),
            body:            b
        }
    }

    /// Updates this proxy.
    ///
    /// Returns `true` if the stored bounding volume has been changed.
    pub fn update(&mut self, margin: &N) -> bool {
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
/// Dont use this broad phase. It exists mainly as a transition broad phase between the Brute Force
/// one and the DBVH/SAP based broad phases.
pub struct BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
    priv objects:    ~[@mut BoundingVolumeProxy<B, BV>], // active   objects
    priv sobjects:   ~[@mut BoundingVolumeProxy<B, BV>], // inactive objects
    priv rb2bv:      HashMap<uint, uint, UintTWHash>,
    priv pairs:      HashMap<Pair<BoundingVolumeProxy<B, BV>>, DV, PairTWHash>, // pair manager
    priv dispatcher: D,
    priv margin:     N,
    priv to_update:  ~[@mut BoundingVolumeProxy<B, BV>],
    priv update_off: uint // incremental pairs removal index
}

impl<B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume,
     D:  Dispatcher<B, B, DV>,
     DV>
BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
    /// Creates a new bounding volume based brute force broad phase.
    pub fn new(dispatcher: D, margin: N) -> BruteForceBoundingVolumeBroadPhase<B, BV, D, DV> {
        BruteForceBoundingVolumeBroadPhase {
            objects:    ~[],
            sobjects:   ~[],
            to_update:  ~[],
            rb2bv:      HashMap::new(UintTWHash::new()),
            pairs:      HashMap::new(PairTWHash::new()),
            dispatcher: dispatcher,
            update_off: 0,
            margin:     margin
        }
    }

    /// Number of interferences detected by this broad phase.
    pub fn num_interferences(&self) -> uint {
        self.pairs.len()
    }

    /// The pair manager of this broad phase.
    pub fn pairs<'r>(&'r self) -> &'r HashMap<Pair<BoundingVolumeProxy<B, BV>>, DV, PairTWHash> {
        &'r self.pairs
    }

    /// The pair manager of this broad phase.
    pub fn pairs_mut<'r>(&'r mut self)
                         -> &'r mut HashMap<Pair<BoundingVolumeProxy<B, BV>>, DV, PairTWHash> {
        &'r mut self.pairs
    }

    /// Adds an element to this broad phase.
    pub fn add(&mut self, rb: @mut B) {
        let proxy = @mut BoundingVolumeProxy::new(rb, self.margin.clone());
        self.objects.push(proxy);
        self.to_update.push(proxy);
    }

    /// Removes an element from this broad phase.
    pub fn remove(&mut self, _: @mut B) {
        fail!("Not yet implemented.");
    }

    /// Marks and object as active or inactive. The bounding volume of an inactive object is never
    /// updated. Activating/deactivating an already active/inactive objecs leads to undefined
    /// behaviour.
    pub fn set_active(&mut self, b: @mut B, active: bool) {
        let (key, at) =
            match self.rb2bv.find_mut(&(ptr::to_mut_unsafe_ptr(b) as uint)) {
                None    => fail!("Unable to change the active state of an unknown object."),
                Some(i) => {
                    if active {
                        // remove from sobjects…
                        let proxy  = self.sobjects[*i];
                        let lproxy = self.sobjects.pop();
                        self.sobjects[*i] = lproxy;

                        // … then add to objects
                        self.objects.push(proxy);

                        let mut at = self.objects.len() - 1;

                        util::swap(&mut at, i);

                        (lproxy, at)
                    }
                    else {
                        // remove from objects…
                        let proxy  = self.objects[*i];
                        let lproxy = self.objects.pop();
                        self.objects[*i] = lproxy;

                        // … then add to sobjects
                        self.sobjects.push(proxy);

                        let mut at = self.sobjects.len() - 1;

                        util::swap(&mut at, i);

                        (lproxy, at)
                    }
                }
            };

        self.rb2bv.insert(ptr::to_mut_unsafe_ptr(key) as uint, at);
    }

    /// Updates the collision pairs based on the objects bounding volumes.
    pub fn update(&mut self) {
        let mut new_colls = 0u;

        for &b in self.objects.mut_iter() {
            if b.update(&self.margin) {
                self.to_update.push(b)
            }
        }

        for &b1 in self.to_update.iter() {
            for &b2 in self.objects.iter() {
                if self.dispatcher.is_valid(b1.body, b2.body) {
                    if b2.bounding_volume.intersects(&b1.bounding_volume) {
                        self.pairs.find_or_insert_lazy(
                            Pair::new(b1, b2),
                            || self.dispatcher.dispatch(b1.body, b2.body)
                        );

                        new_colls = new_colls + 1;
                    }
                }
            }
        }

        if new_colls != 0 {
            let len          = self.pairs.len();
            let num_removals = new_colls.clamp(&(len / 10), &len);

            for i in range(self.update_off, self.update_off + num_removals) {
                let id = i % self.pairs.len();

                let remove = {
                    let elts  = self.pairs.elements();
                    let entry = &elts[id];

                    if (!entry.key.first.bounding_volume.intersects(&entry.key.second.bounding_volume)) {
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

#[cfg(test, dim3, f64)]
mod test {
    use super::BruteForceBoundingVolumeBroadPhase;
    use nalgebra::na::{Vec3, Iso3};
    use nalgebra::na;
    use geom::Ball;
    use bounding_volume::WithAABB;
    use broad::NoIdDispatcher;

    #[test]
    fn test_bfbv_empty() {
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 30.0, j as f64 * 30.0, 0.0);
                bf.add(@mut WithAABB(Iso3::new(t, na::zero()), ball));
            }
        }

        bf.update();

        assert_eq!(bf.num_interferences(), 0)
    }

    #[test]
    fn test_bfbv_nbh_collide() {
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
                bf.add(@mut WithAABB(Iso3::new(t, na::zero()), ball));
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
        type Shape = WithAABB<Ball>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        let mut to_move = ~[];

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
                let to_add = @mut WithAABB(Iso3::new(t, na::zero()), ball);
                bf.add(to_add);
                to_move.push(to_add);
            }
        }

        for e in to_move.mut_iter() {
            let WithAABB(m, c) = **e;
            **e = WithAABB(na::append_translation(&m, &Vec3::new(10.0, 10.0, 10.0)), c)
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
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = BruteForceBoundingVolumeBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        400.times(|| {
            bf.add(@mut WithAABB(Iso3::new(na::zero(), na::zero()), ball))
        });

        bf.update();

        assert_eq!(bf.num_interferences(), (399 * (399 + 1)) / 2)
    }
}
