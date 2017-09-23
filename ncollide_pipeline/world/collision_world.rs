use std::mem;
use std::vec::IntoIter;
use std::slice::Iter;
use math::{Point, Isometry};
use utils::data::uid_remap::{UidRemap, FastKey};
use utils::data::vec_map::Values;
use geometry::bounding_volume::{self, BoundingVolume, AABB};
use geometry::shape::ShapeHandle;
use geometry::query::{RayCast, Ray, RayIntersection, PointQuery};
use narrow_phase::{NarrowPhase, DefaultNarrowPhase, DefaultContactDispatcher, DefaultProximityDispatcher,
                   ContactPairs, Contacts, ProximityPairs};
use broad_phase::{BroadPhase, DBVTBroadPhase, BroadPhasePairFilter, BroadPhasePairFilters};
use world::{CollisionObject, GeometricQueryType, CollisionGroups, CollisionGroupsPairFilter};
use events::{ContactEvents, ProximityEvents, ContactEvent, ProximityEvent};

/// Type of the narrow phase trait-object used by the collision world.
pub type NarrowPhaseObject<P, M, T> = Box<NarrowPhase<P, M, T>>;
/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<P> = Box<BroadPhase<P, AABB<P>, FastKey>>;

/// A world that handles collision objects.
pub struct CollisionWorld<P: Point, M, T> {
    objects:           UidRemap<CollisionObject<P, M, T>>,
    broad_phase:       BroadPhaseObject<P>,
    narrow_phase:      Box<NarrowPhase<P, M, T>>,
    contact_events:    ContactEvents,
    proximity_events:  ProximityEvents,
    pair_filters:      BroadPhasePairFilters<P, M, T>,
    pos_to_update:     Vec<(FastKey, M)>,
    objects_to_remove: Vec<usize>,
    objects_to_add:    Vec<CollisionObject<P, M, T>>,
    timestamp:         usize
    // FIXME: allow modification of the other properties too.
}

impl<P: Point, M: Isometry<P>, T> CollisionWorld<P, M, T> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: P::Real, small_uids: bool) -> CollisionWorld<P, M, T> {
        let objects          = UidRemap::new(small_uids);
        let coll_dispatcher  = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher  = Box::new(DefaultProximityDispatcher::new());
        let broad_phase      = Box::new(DBVTBroadPhase::<P, AABB<P>, FastKey>::new(margin, true));
        let narrow_phase     = DefaultNarrowPhase::new(coll_dispatcher, prox_dispatcher);

        CollisionWorld {
            contact_events:    ContactEvents::new(),
            proximity_events:  ProximityEvents::new(),
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
               query_type:       GeometricQueryType<P::Real>,
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
        where F: BroadPhasePairFilter<P, M, T> {
        self.pair_filters.register_collision_filter(name, Box::new(filter));
        self.broad_phase.deferred_recompute_all_proximities();
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        if self.pair_filters.unregister_collision_filter(name) {
            self.broad_phase.deferred_recompute_all_proximities();
        }
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
        let sig   = &mut self.contact_events;
        let prox  = &mut self.proximity_events;
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
            &mut self.contact_events,
            &mut self.proximity_events,
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

    /// Iterates through every contact events since the last update.
    #[inline]
    pub fn contact_events(&self) -> Iter<ContactEvent> {
        self.contact_events.iter()
    }

    /// Iterates through every proximity events since the last update.
    #[inline]
    pub fn proximity_events(&self) -> Iter<ProximityEvent> {
        self.proximity_events.iter()
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

impl<'a, P: Point, M: Isometry<P>, T> Iterator for InterferencesWithRay<'a, P, M, T> {
    type Item = (&'a CollisionObject<P, M, T>, RayIntersection<P::Vector>);

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

impl<'a, P: Point, M: Isometry<P>, T> Iterator for InterferencesWithPoint<'a, P, M, T> {
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
