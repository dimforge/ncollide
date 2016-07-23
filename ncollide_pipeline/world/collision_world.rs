use std::mem;
use std::ops::Mul;
use std::vec::IntoIter;
use na::{Translate, Cross, Translation, Rotation};
use math::{Point, Vector, Isometry};
use utils::data::uid_remap::{UidRemap, FastKey};
use utils::data::vec_map::Values;
use geometry::bounding_volume::{self, BoundingVolume, AABB};
use geometry::shape::ShapeHandle;
use geometry::query::{RayCast, Ray, RayIntersection, PointQuery};
use narrow_phase::{NarrowPhase, DefaultNarrowPhase, DefaultContactDispatcher, DefaultProximityDispatcher,
                   ContactHandler, ContactPairs, Contacts, ContactSignal, ProximityHandler,
                   ProximitySignal, ProximityPairs};
use broad_phase::{BroadPhase, DBVTBroadPhase, BroadPhasePairFilter, BroadPhasePairFilters};
use world::{CollisionObject, GeometricQueryType, CollisionGroups, CollisionGroupsPairFilter};

use na::{Point2, Point3, Isometry2, Isometry3};

/// Type of the narrow phase trait-object used by the collision world.
pub type NarrowPhaseObject<P, M, T> = Box<NarrowPhase<P, M, T> + 'static>;
/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<P> = Box<BroadPhase<P, AABB<P>, FastKey> + 'static>;

/// A world that handles collision objects.
pub struct CollisionWorld<P: Point, M, T> {
    objects:           UidRemap<CollisionObject<P, M, T>>,
    broad_phase:       BroadPhaseObject<P>,
    narrow_phase:      NarrowPhaseObject<P, M, T>,
    contact_signal:    ContactSignal<P, M, T>,
    proximity_signal:  ProximitySignal<P, M, T>,
    pair_filters:      BroadPhasePairFilters<P, M, T>,
    pos_to_update:     Vec<(FastKey, M)>,
    objects_to_remove: Vec<usize>,
    objects_to_add:    Vec<CollisionObject<P, M, T>>,
    timestamp:         usize
    // FIXME: allow modification of the other properties too.
}

impl<P, M, T> CollisionWorld<P, M, T>
    where P: Point,
          P::Vect: Translate<P> + Cross,
          <P::Vect as Cross>::CrossProductType: Vector<Scalar = <P::Vect as Vector>::Scalar> +
                                                Mul<<P::Vect as Vector>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M:  Isometry<P> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: <P::Vect as Vector>::Scalar, small_uids: bool) -> CollisionWorld<P, M, T> {
        let objects          = UidRemap::new(small_uids);
        let coll_dispatcher  = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher  = Box::new(DefaultProximityDispatcher::new());
        let broad_phase      = Box::new(DBVTBroadPhase::<P, AABB<P>, FastKey>::new(margin, true));
        let narrow_phase     = DefaultNarrowPhase::new(coll_dispatcher, prox_dispatcher);
        let contact_signal   = ContactSignal::new();
        let proximity_signal = ProximitySignal::new();

        CollisionWorld {
            contact_signal:    contact_signal,
            proximity_signal:  proximity_signal,
            objects:           objects,
            broad_phase:       broad_phase,
            narrow_phase:      Box::new(narrow_phase),
            pair_filters:      BroadPhasePairFilters::new(),
            pos_to_update:     Vec::new(),
            objects_to_remove: Vec::new(),
            objects_to_add:    Vec::new(),
            timestamp:         0
        }
    }

    /// Adds a collision object to the world.
    pub fn deferred_add(&mut self,
               uid:              usize,
               position:         M,
               shape:            ShapeHandle<P, M>,
               collision_groups: CollisionGroups,
               query_type:       GeometricQueryType<<P::Vect as Vector>::Scalar>,
               data:             T) {
        assert!(!self.objects.contains_key(uid), "Unable to add a collision object with the same uid twice.");

        let mut collision_object = CollisionObject::new(uid, position, shape, collision_groups, query_type, data);
        collision_object.timestamp = self.timestamp;

        self.objects_to_add.push((collision_object));
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline: the broad phase first, then the
    /// narrow phase.
    pub fn update(&mut self) {
        self.perform_position_update();
        self.perform_additions_removals_and_broad_phase(); // this will perform the Broad Phase as well.
        self.perform_narrow_phase();
    }

    /// Marks a collision object for removal from the world during the next update.
    pub fn deferred_remove(&mut self, uid: usize) {
        // Mark the object to be removed.
        if self.objects.contains_key(uid) {
            self.objects_to_remove.push(uid);
        }
        else {
            panic!("Attempting to remove an unknown object. \
                    Did you forgot to call `.update()` after `.deferred_add()`-ing your objects?");
        }
    }

    /// Sets the position the collision object attached to the specified object will have during
    /// the next update.
    pub fn deferred_set_position(&mut self, uid: usize, pos: M) {
        if let Some(fk) = self.objects.get_fast_key(uid) {
            self.pos_to_update.push((fk, pos))
        }
        else {
            panic!("Attempting to set the position of an unknown object. \
                    Did you forgot to call `.update()` after `.deferred_add()`-ing your objects?");
        }
    }

    /// Adds a filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of collision objects if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn register_broad_phase_pair_filter<F>(&mut self, name: &str, filter: F)
        where F: BroadPhasePairFilter<P, M, T> + 'static {
        self.pair_filters.register_collision_filter(name, Box::new(filter));
        self.broad_phase.deferred_recompute_all_proximities();
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        if self.pair_filters.unregister_collision_filter(name) {
            self.broad_phase.deferred_recompute_all_proximities();
        }
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactHandler<P, M, T> + 'static {
        self.contact_signal.register_contact_handler(name, Box::new(handler));
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_handler(&mut self, name: &str) {
        self.contact_signal.unregister_contact_handler(name);
    }

    /// Registers a handler for proximity status change events.
    pub fn register_proximity_handler<H>(&mut self, name: &str, handler: H)
        where H: ProximityHandler<P, M, T> + 'static {
        self.proximity_signal.register_proximity_handler(name, Box::new(handler));
    }

    /// Unregisters a handler for proximity status change events.
    pub fn unregister_proximity_handler(&mut self, name: &str) {
        self.proximity_signal.unregister_proximity_handler(name);
    }

    /// Executes the position updates.
    pub fn perform_position_update(&mut self) {
        for &(ref fk, ref pos) in self.pos_to_update.iter() {
            if let Some(co) = self.objects.get_fast_mut(fk) {
                co.position = pos.clone();
                co.timestamp = self.timestamp;
                let mut aabb = bounding_volume::aabb(co.shape.as_ref(), pos);
                aabb.loosen(co.query_type.query_limit());
                self.broad_phase.deferred_set_bounding_volume(fk.uid(), aabb);
            }
        }

        self.pos_to_update.clear();
    }


    /// Actually adds or removes all the objects marked by `.deferred_add(...)` or
    /// `.deferred_remove(...)` and updates the broad phase.
    pub fn perform_additions_removals_and_broad_phase(&mut self) {
        // Add objects.
        for co in self.objects_to_add.drain(..) {
            let mut aabb = bounding_volume::aabb(co.shape.as_ref(), &co.position);
            aabb.loosen(co.query_type.query_limit());
            let fk = self.objects.insert(co.uid, co).0;
            self.broad_phase.deferred_add(fk.uid(), aabb, fk)
        }

        self.objects_to_add.shrink_to_fit();


        // Clean up objects that have been marked as removed
        for uid in self.objects_to_remove.iter() {
            if let Some(fk) = self.objects.get_fast_key(*uid) {
                self.broad_phase.deferred_remove(fk.uid());
            }
        }

        self.perform_broad_phase();

        // clean up objects that have been marked as removed
        while let Some(uid) = self.objects_to_remove.pop() {
            let _ = self.objects.remove(uid);
        }
    }

    /// Executes the broad phase of the collision detection pipeline.
    ///
    /// Note that this does not take in account the changes made to the collision updates after the
    /// last `.perform_position_update()` call.
    pub fn perform_broad_phase(&mut self) {
        let bf    = &mut self.broad_phase;
        let nf    = &mut self.narrow_phase;
        let sig   = &mut self.contact_signal;
        let prox  = &mut self.proximity_signal;
        let filts = &self.pair_filters;
        let objs  = &self.objects;

        bf.update(
            // Filter:
            &mut |b1, b2| CollisionWorld::filter_collision(filts, objs, b1, b2),
            // Handler:
            &mut |b1, b2, started| nf.handle_interaction(sig, prox, objs, b1, b2, started));
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        self.narrow_phase.update(
            &self.objects,
            &mut self.contact_signal,
            &mut self.proximity_signal,
            self.timestamp);
        self.timestamp = self.timestamp + 1;
    }

    /// Sets a new narrow phase and returns the previous one.
    ///
    /// Keep in mind that modifying the narrow-pase will have a non-trivial overhead during the
    /// next update as it will force re-detection of all collision pairs and their associated
    /// contacts.
    pub fn set_narrow_phase(&mut self, narrow_phase: Box<NarrowPhase<P, M, T>>) -> Box<NarrowPhase<P, M, T>> {
        let old = mem::replace(&mut self.narrow_phase, narrow_phase);
        self.broad_phase.deferred_recompute_all_proximities();

        old
    }

    /// Iterates through all the contact pairs detected since the last update.
    #[inline]
    pub fn contact_pairs(&self) -> ContactPairs<P, M, T> {
        self.narrow_phase.contact_pairs(&self.objects)
    }

    /// Iterates through all the proximity pairs detected since the last update.
    #[inline]
    pub fn proximity_pairs(&self) -> ProximityPairs<P, M, T> {
        self.narrow_phase.proximity_pairs(&self.objects)
    }

    /// Iterates through every contact detected since the last update.
    #[inline]
    pub fn contacts(&self) -> Contacts<P, M, T> {
        self.narrow_phase.contact_pairs(&self.objects).contacts()
    }

    /// Iterates through all collision objects.
    #[inline]
    pub fn collision_objects(&self) -> Values<CollisionObject<P, M, T>> {
        self.objects.values()
    }

    /// Returns a reference to the collision object identified by `uid`.
    #[inline]
    pub fn collision_object(&self, uid: usize) -> Option<&CollisionObject<P, M, T>> {
        self.objects.get(uid)
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a>(&'a self, ray: &'a Ray<P>, groups: &'a CollisionGroups)
        -> InterferencesWithRay<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_ray(ray, &mut fks);

        InterferencesWithRay {
            ray:     ray,
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a>(&'a self, point: &'a P, groups: &'a CollisionGroups)
        -> InterferencesWithPoint<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_point(point, &mut fks);

        InterferencesWithPoint {
            point:   point,
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a>(&'a self, aabb: &'a AABB<P>, groups: &'a CollisionGroups)
        -> InterferencesWithAABB<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_bounding_volume(aabb, &mut fks);

        InterferencesWithAABB {
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }

    // Filters by group and by the user-provided callback.
    #[inline]
    fn filter_collision(filters: &BroadPhasePairFilters<P, M, T>,
                        objects: &UidRemap<CollisionObject<P, M, T>>,
                        fk1:     &FastKey,
                        fk2:     &FastKey) -> bool {
        let o1 = &objects[*fk1];
        let o2 = &objects[*fk2];
        let filter_by_groups = CollisionGroupsPairFilter;

        filter_by_groups.is_pair_valid(o1, o2) && filters.is_pair_valid(o1, o2)
    }
}


/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, P: 'a + Point, M: 'a, T: 'a> {
    ray:     &'a Ray<P>,
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P, M, T> Iterator for InterferencesWithRay<'a, P, M, T>
    where P: Point,
          M: Isometry<P> + Translation<P::Vect> {
    type Item = (&'a CollisionObject<P, M, T>, RayIntersection<P::Vect>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_interact_with_groups(self.groups) {
                let inter = co.shape.toi_and_normal_with_ray(&co.position, self.ray, true);

                if let Some(inter) = inter {
                    return Some((co, inter))
                }
            }
        }

        None
    }
}

/// Iterator through all the objects on the world that intersect a specific point.
pub struct InterferencesWithPoint<'a, P: 'a + Point, M: 'a, T: 'a> {
    point:   &'a P,
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P, M, T> Iterator for InterferencesWithPoint<'a, P, M, T>
    where P: Point,
          M: Isometry<P> + Translation<P::Vect> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_interact_with_groups(self.groups) &&
               co.shape.contains_point(&co.position, self.point) {
                return Some(co)
            }
        }

        None
    }
}

/// Iterator through all the objects on the world which bounding volume intersects a specific AABB.
pub struct InterferencesWithAABB<'a, P: 'a + Point, M: 'a, T: 'a> {
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P: Point, M, T> Iterator for InterferencesWithAABB<'a, P, M, T> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_interact_with_groups(self.groups) {
                return Some(co)
            }
        }

        None
    }
}


/// 2D collision world containing objects of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<Point2<N>, Isometry2<N>, T>;
/// 3D collision world containing objects of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<Point3<N>, Isometry3<N>, T>;
