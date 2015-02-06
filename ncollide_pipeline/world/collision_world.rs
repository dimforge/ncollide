use std::sync::Arc;
use na::{Translate, Cross, Rotation};
use math::{Scalar, Point, Vect, Isometry};
use utils::data::uid_remap::{UidRemap, FastKey};
use entities::inspection::Repr;
use entities::bounding_volume::{AABB, HasAABB};
use queries::geometry::Contact;
use queries::ray::{RayCast, Ray, RayIntersection};
use queries::point::PointQuery;
use narrow_phase::{BasicCollisionDispatcher, ContactSignalHandler, CollisionAlgorithm};
use broad_phase::{BroadPhase, DBVTBroadPhase};
use world::{CollisionObjectsDispatcher, CollisionObject, CollisionGroups};

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

// FIXME: be generic wrt the BV?
/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<P, V> = Box<BroadPhase<P, V, AABB<P>, FastKey> + 'static>;

/// A world that handles collision objects.
pub struct CollisionWorld<N, P, V, M, T> {
    objects:       UidRemap<CollisionObject<N, P, V, M, T>>,
    broad_phase:   BroadPhaseObject<P, V>,
    narrow_phase:  CollisionObjectsDispatcher<N, P, V, M, T>,
    pos_to_update: Vec<(FastKey, M)>,
    timestamp:     usize
    // FIXME: allow modification of the other properties too.
}

impl<N, P, V, AV, M, T> CollisionWorld<N, P, V, M, T>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<Output = AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and `prediction` and allow their modification by the
    // user ?
    pub fn new(margin: N, prediction: N, small_uids: bool) -> CollisionWorld<N, P, V, M, T> {
        let objects = UidRemap::new(small_uids);
        let sdispatcher = Box::new(BasicCollisionDispatcher::new(prediction));
        let broad_phase = Box::new(DBVTBroadPhase::new(margin, true));
        let narrow_phase = CollisionObjectsDispatcher::new(sdispatcher);

        CollisionWorld {
            objects:       objects,
            broad_phase:   broad_phase as BroadPhaseObject<P, V>,
            narrow_phase:  narrow_phase,
            pos_to_update: Vec::new(),
            timestamp:     0
        }
    }

    /// Adds a collision object to the world.
    pub fn add(&mut self,
               uid: usize,
               position: M,
               shape: Arc<Box<Repr<N, P, V, M>>>,
               collision_groups: CollisionGroups,
               data: T) {
        // FIXME: test that we did not add this object already ?

        let mut collision_object = CollisionObject::new(position, shape, collision_groups, data);
        collision_object.timestamp = self.timestamp;
        let aabb = collision_object.shape.aabb(&collision_object.position);
        let fk = self.objects.insert(uid, collision_object).0;
        self.broad_phase.defered_add(fk.uid(), aabb, fk)
    }

    /// Remove a collision object from the world.
    pub fn remove(&mut self, uid: usize) {
        if let Some((fk, _)) = self.objects.remove(uid) {
            self.broad_phase.defered_remove(fk.uid());
        }
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline: the broad phase first, then the
    /// narrow phase.
    pub fn update(&mut self) {
        self.perform_position_update();
        self.perform_broad_phase();
        self.perform_narrow_phase();
    }

    /// Sets the position the collision object attached to the specified object will have during
    /// the next update.
    pub fn defered_set_position(&mut self, uid: usize, pos: M) {
        if let Some(fk) = self.objects.get_fast_key(uid) {
            self.pos_to_update.push((fk, pos))
        }
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactSignalHandler<T> + 'static {
        self.narrow_phase.register_contact_signal_handler(name, Box::new(handler))
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.narrow_phase.unregister_contact_signal_handler(name)
    }

    /// Executes the position updates.
    pub fn perform_position_update(&mut self) {
        for &(ref fk, ref pos) in self.pos_to_update.iter() {
            if let Some(co) = self.objects.get_fast_mut(fk) {
                co.position = pos.clone();
                co.timestamp = self.timestamp;
                self.broad_phase.defered_set_bounding_volume(fk.uid(), co.shape.aabb(pos));
            }
        }

        self.pos_to_update.clear();
    }

    /// Executes the broad phase of the collision detection pipeline.
    ///
    /// Not that this does not take in account the changes made to the collision updates after the
    /// last `.perform_position_update()` call.
    pub fn perform_broad_phase(&mut self) {
        let bf = &mut self.broad_phase;
        let nf = &mut self.narrow_phase;
        let objs = &self.objects;

        bf.update(
            &mut |b1, b2| CollisionObjectsDispatcher::is_proximity_allowed(objs, b1, b2),
            &mut |b1, b2, started| nf.handle_proximity(objs, b1, b2, started)
        );
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        self.narrow_phase.update(&self.objects, self.timestamp);
        self.timestamp = self.timestamp + 1;
    }

    /// Iterats through all the contact pairs.
    #[inline(always)]
    pub fn contact_pairs<F>(&self, f: F)
          where F: FnMut(&T, &T, &CollisionAlgorithm<N, P, V, M>) {
        self.narrow_phase.contact_pairs(&self.objects, f)
    }

    /// Collects every contact detected since the last update.
    #[inline(always)]
    pub fn contacts<F>(&self, f: F)
          where F: FnMut(&T, &T, &Contact<N, P, V>) {
        self.narrow_phase.contacts(&self.objects, f)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a ray.
    #[inline(always)]
    pub fn interferences_with_ray<'a, F>(&'a mut self, ray: &Ray<P, V>, mut f: F)
          where F: FnMut(&T, RayIntersection<N, V>) {
        let mut bodies = Vec::new();

        self.broad_phase.interferences_with_ray(ray, &mut bodies);

        for b in bodies.into_iter() {
            let co = &self.objects[*b];

            let inter = co.shape.toi_and_normal_with_transform_and_ray(&co.position, ray, true);

            if let Some(inter) = inter {
                f(&co.data, inter)
            }
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline(always)]
    pub fn interferences_with_point<F>(&self, point: &P, mut f: F)
          where F: FnMut(&T) {
        let mut bodies = Vec::new();

        self.broad_phase.interferences_with_point(point, &mut bodies);

        for b in bodies.into_iter() {
            let co = &self.objects[*b];

            if co.shape.contains_point_with_transform(&co.position, point) {
                f(&co.data)
            }
        }
    }

    // FIXME: replace by iterators.
    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline(always)]
    pub fn interferences_with_aabb<F>(&self, aabb: &AABB<P>, mut f: F)
          where F: FnMut(&T) {
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_bounding_volume(aabb, &mut fks);

        for fk in fks.into_iter() {
            f(&self.objects[*fk].data)
        }
    }
}

/// 2D collision world containing objects of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<N, Pnt2<N>, Vec2<N>, Iso2<N>, T>;
/// 3D collision world containing objects of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<N, Pnt3<N>, Vec3<N>, Iso3<N>, T>;
