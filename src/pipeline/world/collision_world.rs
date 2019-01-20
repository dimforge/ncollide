use crate::bounding_volume::{self, BoundingVolume, AABB};
use crate::math::{Isometry, Point};
use na::Real;
use crate::pipeline::broad_phase::{
    BroadPhase, BroadPhasePairFilter, BroadPhasePairFilters, DBVTBroadPhase, ProxyHandle,
};
use crate::pipeline::events::{ContactEvent, ContactEvents, ProximityEvents};
use crate::pipeline::narrow_phase::{
    ContactAlgorithm, DefaultContactDispatcher, NarrowPhase, DefaultProximityDispatcher,
    ProximityAlgorithm, InteractionGraphIndex, Interaction
};
use crate::pipeline::world::{
    CollisionGroups, CollisionGroupsPairFilter, CollisionObject, CollisionObjectHandle,
    CollisionObjectSlab, CollisionObjects, GeometricQueryType,
};
use crate::query::{ContactManifold, PointQuery, Ray, RayCast, RayIntersection};
use crate::shape::ShapeHandle;
use std::mem;
use std::vec::IntoIter;

/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<N> = Box<BroadPhase<N, AABB<N>, CollisionObjectHandle>>;

/// A world that handles collision objects.
pub struct CollisionWorld<N: Real, T> {
    objects: CollisionObjectSlab<N, T>,
    broad_phase: BroadPhaseObject<N>,
    narrow_phase: NarrowPhase<N>,
    contact_events: ContactEvents,
    proximity_events: ProximityEvents,
    pair_filters: BroadPhasePairFilters<N, T>,
    timestamp: usize, // FIXME: allow modification of the other properties too.
}

impl<N: Real, T> CollisionWorld<N, T> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: N) -> CollisionWorld<N, T> {
        let objects = CollisionObjectSlab::new();
        let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
        let broad_phase = Box::new(DBVTBroadPhase::<N, AABB<N>, CollisionObjectHandle>::new(
            margin,
        ));
        let narrow_phase = NarrowPhase::new(coll_dispatcher, prox_dispatcher);

        CollisionWorld {
            contact_events: ContactEvents::new(),
            proximity_events: ProximityEvents::new(),
            objects,
            broad_phase,
            narrow_phase,
            pair_filters: BroadPhasePairFilters::new(),
            timestamp: 0,
        }
    }

    /// Adds a collision object to the world.
    pub fn add(
        &mut self,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        collision_groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: T,
    ) -> &mut CollisionObject<N, T>
    {
        let mut co = CollisionObject::new(
            CollisionObjectHandle::invalid(),
            ProxyHandle::invalid(),
            InteractionGraphIndex::new(0),
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
        let graph_index = self.narrow_phase.handle_collision_object_added(handle);

        co.set_handle(handle);
        co.set_proxy_handle(proxy_handle);
        co.set_graph_index(graph_index);
        co
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline:
    /// 1. Clears the event pools.
    /// 2. Executes the broad phase first.
    /// 3. Executes the narrow phase.
    pub fn update(&mut self) {
        println!(
            "Structures lens: {}, {}, {}",
            self.objects.len(),
            self.contact_events.len(),
            self.proximity_events.len()
        );
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
                let co = self
                    .objects
                    .get(*handle)
                    .expect("Removal: collision object not found.");
                let graph_index = co.graph_index();
                proxy_handles.push(co.proxy_handle());

                if let Some(handle2) = self.narrow_phase.handle_collision_object_removed(co) {
                    // Properly transfer the graph index.
                    self.objects[handle2].set_graph_index(graph_index)
                }
            }

            // NOTE: no need to notify the narrow phase in the callback because
            // the nodes have already been removed in the loop above.
            self.broad_phase.remove(&proxy_handles, &mut |_, _| {});
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
    pub fn set_position(&mut self, handle: CollisionObjectHandle, pos: Isometry<N>) {
        let co = self
            .objects
            .get_mut(handle)
            .expect("Set position: collision object not found.");
        co.set_position(pos.clone());
        co.timestamp = self.timestamp;
        let mut aabb = bounding_volume::aabb(co.shape().as_ref(), &pos);
        aabb.loosen(co.query_type().query_limit());
        self.broad_phase
            .deferred_set_bounding_volume(co.proxy_handle(), aabb);
    }

    /// Apply the given deformations to the specified object.
    pub fn set_deformations(
        &mut self,
        handle: CollisionObjectHandle,
        coords: &[N],
    )
    {
        let co = self
            .objects
            .get_mut(handle)
            .expect("Set deformations: collision object not found.");
        co.set_deformations(coords);
        co.timestamp = self.timestamp;
        let mut aabb = bounding_volume::aabb(co.shape().as_ref(), co.position());
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
    where F: BroadPhasePairFilter<N, T> {
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
        self.narrow_phase.update(
            &self.objects,
            &mut self.contact_events,
            &mut self.proximity_events,
            self.timestamp,
        );
        self.timestamp = self.timestamp + 1;
    }

    /// The interaction pair, if any, between the given collision objects.
    #[inline]
    pub fn interaction_pair(
        &self,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) -> Option<(&CollisionObject<N, T>, &CollisionObject<N, T>, &Interaction<N>)>
    {
        self.narrow_phase.interaction_pair(&self.objects, handle1, handle2)
    }

    /// Iterates through all the interaction pairs detected since the last update.
    #[inline]
    pub fn interaction_pairs(&self) -> impl Iterator<Item = (
        &CollisionObject<N, T>,
        &CollisionObject<N, T>,
        &Interaction<N>,
    )> {
        self.narrow_phase.interaction_pairs(&self.objects)
    }

    /// Iterates through all collision objects.
    #[inline]
    pub fn collision_objects(&self) -> CollisionObjects<N, T> {
        self.objects.iter()
    }

    /// Returns a reference to the collision object identified by its handle.
    #[inline]
    pub fn collision_object(
        &self,
        handle: CollisionObjectHandle,
    ) -> Option<&CollisionObject<N, T>>
    {
        self.objects.get(handle)
    }

    /// Returns a mutable reference to the collision object identified by its handle.
    #[inline]
    pub fn collision_object_mut(
        &mut self,
        handle: CollisionObjectHandle,
    ) -> Option<&mut CollisionObject<N, T>>
    {
        self.objects.get_mut(handle)
    }

    /// Sets the collision groups of the given collision object.
    #[inline]
    pub fn set_collision_groups(&mut self, handle: CollisionObjectHandle, groups: CollisionGroups) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_collision_groups(groups);
            self.broad_phase
                .deferred_recompute_all_proximities_with(co.proxy_handle());
        }
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a, 'b>(
        &'a self,
        ray: &'b Ray<N>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithRay<'a, 'b, N, T>
    {
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
        point: &'b Point<N>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithPoint<'a, 'b, N, T>
    {
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
        aabb: &'b AABB<N>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithAABB<'a, 'b, N, T>
    {
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
        filters: &BroadPhasePairFilters<N, T>,
        objects: &CollisionObjectSlab<N, T>,
        handle1: CollisionObjectHandle,
        handle2: CollisionObjectHandle,
    ) -> bool
    {
        let o1 = &objects[handle1];
        let o2 = &objects[handle2];
        let filter_by_groups = CollisionGroupsPairFilter;

        filter_by_groups.is_pair_valid(o1, o2) && filters.is_pair_valid(o1, o2)
    }
}

/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, 'b, N: 'a + Real, T: 'a> {
    ray: &'b Ray<N>,
    objects: &'a CollisionObjectSlab<N, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, N: Real, T> Iterator for InterferencesWithRay<'a, 'b, N, T> {
    type Item = (&'a CollisionObject<N, T>, RayIntersection<N>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            let co = &self.objects[*handle];

            if co.collision_groups().can_interact_with_groups(self.groups) {
                let inter = co
                    .shape()
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
pub struct InterferencesWithPoint<'a, 'b, N: 'a + Real, T: 'a> {
    point: &'b Point<N>,
    objects: &'a CollisionObjectSlab<N, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, N: Real, T> Iterator for InterferencesWithPoint<'a, 'b, N, T> {
    type Item = &'a CollisionObject<N, T>;

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
pub struct InterferencesWithAABB<'a, 'b, N: 'a + Real, T: 'a> {
    objects: &'a CollisionObjectSlab<N, T>,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a CollisionObjectHandle>,
}

impl<'a, 'b, N: Real, T> Iterator for InterferencesWithAABB<'a, 'b, N, T> {
    type Item = &'a CollisionObject<N, T>;

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
