use alga::general::Id;
use utils::data::uid_remap::{UidRemap, FastKey};
use geometry::bounding_volume::BoundingVolume;
use geometry::query::{Ray, RayCast, PointQuery};
use broad_phase::BroadPhase;
use math::Point;

/// A broad phase testing explicitly all bounding volume pairs.
///
/// Use this for debugging purposes only.
pub struct BruteForceBroadPhase<N, BV, T> {
    proxies:   UidRemap<(BV, T)>,
    margin:    N, // The margin added to each bounding volume.
    to_remove: Vec<usize>,
    to_add:    Vec<(usize, BV, T)>,
    to_update: Vec<(FastKey, BV)>
}

impl<N, BV, T> BruteForceBroadPhase<N, BV, T> {
    /// Creates a new brute-force broad phase.
    pub fn new(margin: N, small_keys: bool) -> BruteForceBroadPhase<N, BV, T> {
        BruteForceBroadPhase {
            proxies:   UidRemap::new(small_keys),
            margin:    margin,
            to_remove: Vec::new(),
            to_add:    Vec::new(),
            to_update: Vec::new()
        }
    }
}

impl<P: Point, BV: Sync + Send, T: Sync + Send> BroadPhase<P, BV, T> for BruteForceBroadPhase<P::Real, BV, T> where
    BV: 'static + BoundingVolume<P> +
        RayCast<P, Id> + PointQuery<P, Id> + Clone {
    fn deferred_add(&mut self, uid: usize, bv: BV, data: T) {
        // XXX: should not be inserted now!
        let (key, _) = self.proxies.insert(uid, (bv.clone(), data));

        self.to_update.push((key, bv))
    }

    fn deferred_remove(&mut self, uid: usize) {
        if self.proxies.get_fast_key(uid).is_some() {
            self.to_remove.push(uid);
        }
    }

    fn deferred_set_bounding_volume(&mut self, uid: usize, bv: BV) {
        if let Some(key) = self.proxies.get_fast_key(uid) {
            if !self.proxies[key].0.contains(&bv) {
                self.to_update.push((key, bv.loosened(self.margin)));
            }
        }
    }

    fn deferred_recompute_all_proximities(&mut self) {
        for proxy in self.proxies.iter() {
            self.to_update.push((proxy.0, (proxy.1).0.clone()));
        }
    }

    fn update(&mut self, allow_proximity: &mut FnMut(&T, &T) -> bool, proximity_handler: &mut FnMut(&T, &T, bool)) {
        for (uid, bv, data) in self.to_add.drain(..) {
            let lbv = bv.loosened(self.margin.clone());
            let key = self.proxies.insert(uid, (lbv.clone(), data));
            self.to_update.push((key.0, lbv))
        }

        for rm in self.to_remove.drain(..) {
            let _ = self.proxies.remove(rm);
        }

        for new_proxy in self.to_update.iter() {
            if let Some(proxy) = self.proxies.get_fast_mut(&new_proxy.0) {
                proxy.0 = new_proxy.1.clone();
            }
        }

        for proxy1_key in self.to_update.drain(..) {
            if let Some(proxy1) = self.proxies.get_fast(&proxy1_key.0) {
                for proxy2 in self.proxies.values() {
                    if allow_proximity(&proxy1.1, &proxy2.1) {
                        proximity_handler(&proxy1.1, &proxy2.1, proxy1.0.intersects(&proxy2.0))
                    }
                }
            }
        }
    }

    fn interferences_with_bounding_volume<'a>(&'a self, bv: &BV, out: &mut Vec<&'a T>) {
        for proxy in self.proxies.values() {
            if proxy.0.intersects(bv) {
                out.push(&proxy.1)
            }
        }
    }

    fn interferences_with_ray<'a>(&'a self, ray: &Ray<P>, out: &mut Vec<&'a T>) {
        for proxy in self.proxies.values() {
            if proxy.0.intersects_ray(&Id::new(), ray) {
                out.push(&proxy.1)
            }
        }
    }

    fn interferences_with_point<'a>(&'a self, point: &P, out: &mut Vec<&'a T>) {
        for proxy in self.proxies.values() {
            if proxy.0.contains_point(&Id::new(), point) {
                out.push(&proxy.1)
            }
        }
    }
}
