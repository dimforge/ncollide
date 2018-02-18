use std::mem;
use std::vec::IntoIter;

use math::{Isometry, Point};
use geometry::bounding_volume::{self, BoundingVolume, AABB};
use geometry::shape::ShapeHandle;
use geometry::query::{PointQuery, Ray, RayCast, RayIntersection};
use narrow_phase::{ContactManifolds, ContactPairs, DefaultContactDispatcher, DefaultNarrowPhase,
                   DefaultProximityDispatcher, NarrowPhase, ProximityPairs, NAVOID};
use broad_phase::{BroadPhase, BroadPhasePairFilter, BroadPhasePairFilters, DBVTBroadPhase,
                  ProxyHandle};
use world::{CollisionGroups, CollisionGroupsPairFilter, CollisionObject, CollisionObjectHandle,
            CollisionObjectSlab, CollisionObjects, GeometricQueryType};
use events::{ContactEvent, ContactEvents, ProximityEvents};

/// Type of the narrow phase trait-object used by the collision world.
pub type NarrowPhaseObject<P, M, T> = Box<NarrowPhase<P, M, T>>;
/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<P> = Box<BroadPhase<P, AABB<P>, CollisionObjectHandle>>;

/// A world that handles collision objects.
pub struct CollisionWorld<P: Point, M, T> {
    objects: CollisionObjectSlab<P, M, T>,
    broad_phase: BroadPhaseObject<P>,
    narrow_phase: Box<NarrowPhase<P, M, T>>,
    contact_events: ContactEvents,
    proximity_events: ProximityEvents,
    pair_filters: BroadPhasePairFilters<P, M, T>,
    timestamp: usize, // FIXME: allow modification of the other properties too.
}

impl<P: Point, M: Isometry<P>, T> CollisionWorld<P, M, T> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: P::Real) -> CollisionWorld<P, M, T> {
        let objects = CollisionObjectSlab::new();
        let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
        let broad_phase = Box::new(DBVTBroadPhase::<P, AABB<P>, CollisionObjectHandle>::new(
            margin,
        ));
        let narrow_phase = DefaultNarrowPhase::new(coll_dispatcher, prox_dispatcher);

        CollisionWorld {
            contact_events: ContactEvents::new(),
            proximity_events: ProximityEvents::new(),
            objects: objects,
            broad_phase: broad_phase,
            narrow_phase: Box::new(narrow_phase),
            pair_filters: BroadPhasePairFilters::new(),
            timestamp: 0,
        }
    }

    /// Adds a collision object to the world.
    pub fn add(
        &mut self,
        position: M,
        shape: ShapeHandle<P, M>,
        collision_groups: CollisionGroups,
        query_type: GeometricQueryType<P::Real>,
        data: T,
    ) -> CollisionObjectHandle {
        let mut co = CollisionObject::new(
            CollisionObjectHandle::invalid(),
            ProxyHandle::invalid(),
            position,
            shape,
            collision_groups,
            query_type,
            data,
        );
        co.timestamp = self.timestamp;
        let handle = self.objects.insert(co);

        // Add objects.
        let co = &mut self.objects[handle];
        let mut aabb = bounding_volume::aabb(co.shape().as_ref(), co.position());
        aabb.loosen(co.query_type().query_limit());
        let proxy_handle = self.broad_phase.create_proxy(aabb, handle);

        co.set_handle(handle);
        co.set_proxy_handle(proxy_handle);

        handle
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline:
    /// 1. Clears the event pools.
    /// 2. Executes the broad phase first.
    /// 3. Executes the narrow phase.
    pub fn update(&mut self) {
        self.clear_events();
        self.perform_broad_phase();
        self.perform_narrow_phase();
    }

    /// Empty the contact and proximity event pools.
    pub fn clear_events(&mut self) {
        self.contact_events.clear();
        self.proximity_events.clear();
    }

    /// Removed the specified set of collision objects from the world.
    ///
    /// Panics of any handle is invalid, or if the list contains duplicates.
    pub fn remove(&mut self, handles: &[CollisionObjectHandle]) {
        {
            let mut proxy_handles = Vec::new();

            for handle in handles {
                let co = self.objects
                    .get(*handle)
                    .expect("Removal: collision object not found.");
                proxy_handles.push(co.proxy_handle());
            }

            let nf = &mut self.narrow_phase;
            let objects = &mut self.objects;
            self.broad_phase.remove(&proxy_handles, &mut |b1, b2| {
                nf.handle_removal(objects, *b1, *b2)
            });
        }

        for handle in handles {
            let _ = self.objects.remove(*handle);
        }

        let objects = &self.objects;
        self.proximity_events
            .retain(|e| objects.contains(e.collider1) && objects.contains(e.collider2));
        self.contact_events.retain(|e| match *e {
            ContactEvent::Started(co1, co2) | ContactEvent::Stopped(co1, co2) => {
                objects.contains(co1) && objects.contains(co2)
            }
        })
    }

    /// Sets the position the collision object attached to the specified object.
    pub fn set_position(&mut self, handle: CollisionObjectHandle, pos: M) {
        let co = self.objects
            .get_mut(handle)
            .expect("Set position: collision object not found.");
        co.set_position(pos.clone());
        co.timestamp = self.timestamp;
        let mut aabb = bounding_volume::aabb(co.shape().as_ref(), &pos);
        aabb.loosen(co.query_type().query_limit());
        self.broad_phase
            .deferred_set_bounding_volume(co.proxy_handle(), aabb);
    }

    /// Adds a filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of collision objects if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn register_broad_phase_pair_filter<F>(&mut self, name: &str, filter: F)
    where
        F: BroadPhasePairFilter<P, M, T>,
    {
        self.pair_filters
            .register_collision_filter(name, Box::new(filter));
        self.broad_phase.deferred_recompute_all_proximities();
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        if self.pair_filters.unregister_collision_filter(name) {
            self.broad_phase.deferred_recompute_all_proximities();
        }
    }

    /// Executes the broad phase of the collision detection pipeline.
    pub fn perform_broad_phase(&mut self) {
        let bf = &mut self.broad_phase;
        let nf = &mut self.narrow_phase;
        let sig = &mut self.contact_events;
        let prox = &mut self.proximity_events;
        let filts = &self.pair_filters;
        let objs = &self.objects;

        bf.update(
            // Filter:
            &mut |b1, b2| CollisionWorld::filter_collision(filts, objs, *b1, *b2),
            // Handler:
            &mut |b1, b2, started| nf.handle_interaction(sig, prox, objs, *b1, *b2, started),
        );
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        NAVOID.with(|e| *e.borrow_mut() = 0);
        self.narrow_phase.update(
            &self.objects,
            &mut self.contact_events,
            &mut self.proximity_events,
            self.timestamp,
        );
        self.timestamp = self.timestamp + 1;
        NAVOID.with(|e| println!("Avoidable GJK/EPA: {}", *e.borrow()));
    }

    /// Sets a new narrow phase and returns the previous one.
    ///
    /// Keep in mind that modifying the narrow-pase will have a non-trivial overhead during the
    /// next update as it will force re-detection of all collision pairs and their associated
    /// contacts.
    pub fn set_narrow_phase(
        &mut self,
        narrow_phase: Box<NarrowPhase<P, M, T>>,
    ) -> Box<NarrowPhase<P, M, T>> {
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
    pub fn contact_manifolds(&self) -> ContactManifolds<P, M, T> {
        self.narrow_phase
            .contact_pairs(&self.objects)
            .contact_manifolds()
    }

    /// Iterates through all collision objects.
    #[inline]
    pub fn collision_objects(&self) -> CollisionObjects<P, M, T> {
        self.objects.iter()
    }

    /// Returns a reference to the collision object identified by its handle.
    #[inline]
    pub fn collision_object(
        &self,
        handle: CollisionObjectHandle,
    ) -> Option<&CollisionObject<P, M, T>> {
        self.objects.get(handle)
    }

    /// Returns a mutable reference to the collision object identified by its handle.
    #[inline]
    pub fn collision_object_mut(
        &mut self,
        handle: CollisionObjectHandle,
    ) -> Option<&mut CollisionObject<P, M, T>> {
        self.objects.get_mut(handle)
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a, 'b>(
        &'a self,
        ray: &'b Ray<P>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithRay<'a, 'b, P, M, T> {
        // FIXME: avoid allocation.
        let mut handles = Vec::new();
        self.broad_phase.interferences_with_ray(ray, &mut handles);

        InterferencesWithRay {
            ray: ray,
            groups: groups,
            objects: &self.objects,
            handles: handles.into_iter(),
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a, 'b>(
        &'a self,
        point: &'b P,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithPoint<'a, 'b, P, M, T> {
        // FIXME: avoid allocation.
        let mut handles = Vec::new();
        self.broad_phase
            .interferences_with_point(point, &mut handles);

        InterferencesWithPoint {
            point: point,
            groups: groups,
            objects: &self.objects,
            handles: handles.into_iter(),
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a, 'b>(
        &'a self,
        aabb: &'b AABB<P>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithAABB<'a, 'b, P, M, T> {
        // FIXME: avoid allocation.
        let mut handles = Vec::new();
        self.broad_phase
            .interferences_with_bounding_volume(aabb, &mut handles);

        InterferencesWithAABB {
            groups: groups,
            objects: &self.objects,
            handles: handles.into_iter(),
        }
    }

    /// The contact events pool.
    pub fn contact_events(&self) -> &ContactEvents {
        &self.contact_events
    }

    /// The proximity events pool.
    pub fn proximity_events(&self) -> &ProximityEvents {
        &self.proximity_events
    }

    // Filters by group and by the user-provided callback.
    #[inline]
    fn filter_collision(
        filters: &BroadPhasePairFilters<P, M, T>,
        objects: &CollisionObjectSlab<P, M, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) -> bool {
        let o1 = &objects[handle1];
        let o2 = &objects[handle2];
        let filter_by_groups = CollisionGroupsPairFilter;

        filter_by_groups.is_pair_valid(o1, o2) && filters.is_pair_valid(o1, o2)
    }
}

/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, 'b, P: 'a + Point, M: 'a, T: 'a> {
    ray: &'b Ray<P>,
    objects: &'a CollisionObjectSlab<P, M, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, P: Point, M: Isometry<P>, T> Iterator for InterferencesWithRay<'a, 'b, P, M, T> {
    type Item = (&'a CollisionObject<P, M, T>, RayIntersection<P::Vector>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            let co = &self.objects[*handle];

            if co.collision_groups().can_interact_with_groups(self.groups) {
                let inter = co.shape()
                    .toi_and_normal_with_ray(&co.position(), self.ray, true);

                if let Some(inter) = inter {
                    return Some((co, inter));
                }
            }
        }

        None
    }
}

/// Iterator through all the objects on the world that intersect a specific point.
pub struct InterferencesWithPoint<'a, 'b, P: 'a + Point, M: 'a, T: 'a> {
    point: &'b P,
    objects: &'a CollisionObjectSlab<P, M, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, P: Point, M: Isometry<P>, T> Iterator for InterferencesWithPoint<'a, 'b, P, M, T> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            let co = &self.objects[*handle];

            if co.collision_groups().can_interact_with_groups(self.groups)
                && co.shape().contains_point(&co.position(), self.point)
            {
                return Some(co);
            }
        }

        None
    }
}

/// Iterator through all the objects on the world which bounding volume intersects a specific AABB.
pub struct InterferencesWithAABB<'a, 'b, P: 'a + Point, M: 'a, T: 'a> {
    objects: &'a CollisionObjectSlab<P, M, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, P: Point, M, T> Iterator for InterferencesWithAABB<'a, 'b, P, M, T> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            let co = &self.objects[*handle];

            if co.collision_groups().can_interact_with_groups(self.groups) {
                return Some(co);
            }
        }

        None
    }
}
