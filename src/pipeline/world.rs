//! High level API to detect collisions in large, complex scenes.

use na::{RealField, Unit};

use crate::bounding_volume::{BoundingVolume, AABB};
use crate::math::{Isometry, Point, Rotation, Translation, Vector};
use crate::pipeline::broad_phase::{BroadPhase, BroadPhasePairFilter, DBVTBroadPhase};
use crate::pipeline::glue::{
    self, FirstInterferenceWithRay, InterferencesWithAABB, InterferencesWithPoint,
    InterferencesWithRay,
};
use crate::pipeline::narrow_phase::{
    ContactAlgorithm, ContactEvents, DefaultContactDispatcher, DefaultProximityDispatcher,
    Interaction, InteractionGraph, NarrowPhase, ProximityDetector, ProximityEvents,
    TemporaryInteractionIndex,
};
use crate::pipeline::object::{
    CollisionGroups, CollisionObject, CollisionObjectSet, CollisionObjectSlab,
    CollisionObjectSlabHandle, CollisionObjects, GeometricQueryType,
};
use crate::query::{ContactManifold, DefaultTOIDispatcher, Proximity, Ray, TOIDispatcher, TOI};
use crate::shape::{Shape, ShapeHandle};

/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<N> = Box<dyn BroadPhase<N, AABB<N>, CollisionObjectSlabHandle>>;

/// A world that handles collision objects.
pub struct CollisionWorld<N: RealField, T> {
    /// The set of objects on this collision world.
    pub objects: CollisionObjectSlab<N, T>,
    /// The broad phase used by this collision world.
    pub broad_phase: BroadPhaseObject<N>,
    /// The narrow-phase used by this collision world.
    pub narrow_phase: NarrowPhase<N, CollisionObjectSlabHandle>,
    /// The Time of Impact dispatcher used.
    pub toi_dispatcher: Box<dyn TOIDispatcher<N>>,
    /// The graph of interactions detected so far.
    pub interactions: InteractionGraph<N, CollisionObjectSlabHandle>,
    /// A user-defined broad-phase pair filter.
    pub pair_filters: Option<Box<dyn BroadPhasePairFilter<N, CollisionObjectSlab<N, T>>>>,
}

impl<N: RealField, T> CollisionWorld<N, T> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: N) -> CollisionWorld<N, T> {
        let objects = CollisionObjectSlab::new();
        let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
        let toi_dispatcher = Box::new(DefaultTOIDispatcher);
        let broad_phase =
            Box::new(DBVTBroadPhase::<N, AABB<N>, CollisionObjectSlabHandle>::new(margin));
        let narrow_phase = NarrowPhase::new(coll_dispatcher, prox_dispatcher);

        CollisionWorld {
            interactions: InteractionGraph::new(),
            objects,
            broad_phase,
            narrow_phase,
            toi_dispatcher,
            pair_filters: None,
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
    ) -> (CollisionObjectSlabHandle, &mut CollisionObject<N, T>) {
        let entry = self.objects.objects.vacant_entry();
        let handle = CollisionObjectSlabHandle(entry.key());
        let (proxy_handle, graph_index) = glue::create_proxies(
            handle,
            &mut *self.broad_phase,
            &mut self.interactions,
            &position,
            shape.as_ref(),
            query_type,
        );

        let co = CollisionObject::new(
            Some(proxy_handle),
            Some(graph_index),
            position,
            shape,
            collision_groups,
            query_type,
            data,
        );

        (handle, entry.insert(co))
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline:
    /// 1. Clears the event pools.
    /// 2. Executes the broad phase first.
    /// 3. Executes the narrow phase.
    pub fn update(&mut self) {
        self.narrow_phase.clear_events();

        glue::perform_all_pipeline(
            &self.objects,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.interactions,
            self.pair_filters.as_ref().map(|f| &**f),
        );

        // Clear update flags.
        for (_, co) in self.objects.iter_mut() {
            co.clear_update_flags();
        }
    }

    /// Empty the contact and proximity event pools.
    pub fn clear_events(&mut self) {
        self.narrow_phase.clear_events();
    }

    /// Removed the specified set of collision objects from the world.
    ///
    /// Panics of any handle is invalid, or if the list contains duplicates.
    pub fn remove(&mut self, handles: &[CollisionObjectSlabHandle]) {
        for handle in handles {
            let co = self.objects.remove(*handle);
            let graph_index = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
            let proxy_handle = co.proxy_handle().expect(crate::NOT_REGISTERED_ERROR);

            if let Some((new_handle, new_index)) = glue::remove_proxies(
                &mut *self.broad_phase,
                &mut self.interactions,
                proxy_handle,
                graph_index,
            ) {
                self.objects[new_handle].set_graph_index(Some(new_index))
            }
        }
    }

    /// Sets the position of the collision object attached to the specified object.
    #[deprecated = "Call directly the method `.set_position` on the collision object."]
    pub fn set_position(&mut self, handle: CollisionObjectSlabHandle, pos: Isometry<N>) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_position(pos);
        }
    }

    /*
    /// Sets the position of the collision object attached to the specified object and update its bounding volume
    /// by taking into account its predicted next position.
    pub fn set_position_with_prediction(&mut self, handle: CollisionObjectSlabHandle, pos: Isometry<N>, predicted_pos: &Isometry<N>) {
        let co = self
            .objects
            .get_mut(handle)
            .expect("Set position: collision object not found.");
        co.set_position(pos.clone());
        co.timestamp = self.timestamp;
        let mut aabb1 = bounding_volume::aabb(co.shape().as_ref(), &pos);
        let mut aabb2 = bounding_volume::aabb(co.shape().as_ref(), predicted_pos);
        aabb1.loosen(co.query_type().query_limit());
        aabb2.loosen(co.query_type().query_limit());
        aabb1.merge(&aabb2);
        self.broad_phase
            .deferred_set_bounding_volume(co.proxy_handle(), aabb1);

    }*/

    /// Sets the `GeometricQueryType` of the collision object.
    #[inline]
    #[deprecated = "Call directly the method `.set_query_type` on the collision object."]
    pub fn set_query_type(
        &mut self,
        handle: CollisionObjectSlabHandle,
        query_type: GeometricQueryType<N>,
    ) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_query_type(query_type);
        }
    }

    /// Sets the shape of the given collision object.
    #[inline]
    #[deprecated = "Call directly the method `.set_shape` on the collision object."]
    pub fn set_shape(&mut self, handle: CollisionObjectSlabHandle, shape: ShapeHandle<N>) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_shape(shape);
        }
    }

    /// Apply the given deformations to the specified object.
    #[deprecated = "Call directly the method `.set_deformations` on the collision object."]
    pub fn set_deformations(&mut self, handle: CollisionObjectSlabHandle, coords: &[N]) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_deformations(coords);
        }
    }

    /// Sets the user-defined filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of collision objects if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn set_broad_phase_pair_filter<F>(&mut self, filter: Option<F>)
    where
        F: BroadPhasePairFilter<N, CollisionObjectSlab<N, T>> + 'static,
    {
        self.pair_filters = filter
            .map(|f| Box::new(f) as Box<dyn BroadPhasePairFilter<N, CollisionObjectSlab<N, T>>>);
        self.broad_phase.deferred_recompute_all_proximities();
    }

    /// Executes the broad phase of the collision detection pipeline.
    pub fn perform_broad_phase(&mut self) {
        glue::perform_broad_phase(
            &self.objects,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.interactions,
            self.pair_filters.as_ref().map(|f| &**f),
        )
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        glue::perform_narrow_phase(
            &self.objects,
            &mut self.narrow_phase,
            &mut self.interactions,
        )
    }

    /// The broad-phase aabb for the given collision object.
    pub fn broad_phase_aabb(&self, handle: CollisionObjectSlabHandle) -> Option<&AABB<N>> {
        let co = self.objects.collision_object(handle)?;
        let proxy_handle = co.proxy_handle().expect(crate::NOT_REGISTERED_ERROR);
        self.broad_phase.proxy(proxy_handle).map(|p| p.0)
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
        handle: CollisionObjectSlabHandle,
    ) -> Option<&CollisionObject<N, T>> {
        self.objects.collision_object(handle)
    }

    /// Returns a mutable reference to the collision object identified by its handle.
    #[inline]
    pub fn get_mut(
        &mut self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<&mut CollisionObject<N, T>> {
        self.objects.get_mut(handle)
    }

    /// Returns a mutable reference to a pair collision object identified by their handles.
    ///
    /// Panics if both handles are equal.
    #[inline]
    pub fn collision_object_pair_mut(
        &mut self,
        handle1: CollisionObjectSlabHandle,
        handle2: CollisionObjectSlabHandle,
    ) -> (
        Option<&mut CollisionObject<N, T>>,
        Option<&mut CollisionObject<N, T>>,
    ) {
        self.objects.get_pair_mut(handle1, handle2)
    }

    /// Sets the collision groups of the given collision object.
    #[inline]
    #[deprecated = "Call directly the method `.set_collision_groups` on the collision object."]
    pub fn set_collision_groups(
        &mut self,
        handle: CollisionObjectSlabHandle,
        groups: CollisionGroups,
    ) {
        if let Some(co) = self.objects.get_mut(handle) {
            co.set_collision_groups(groups);
        }
    }

    /// Returns all objects in the collision world that intersect with the shape
    /// transformed by `isometry` along `direction` until `maximum_distance` is
    /// reached. The objects are not returned in any particular order. You may
    /// use the `toi` returned for each object to determine the closest object.
    #[inline]
    pub fn sweep_test<'a>(
        &'a self,
        shape: &'a dyn Shape<N>,
        isometry: &'a Isometry<N>,
        direction: &'a Unit<Vector<N>>,
        maximum_distance: N,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = (CollisionObjectSlabHandle, TOI<N>)> + 'a {
        let a = shape.aabb(&isometry);
        let b = shape.aabb(&Isometry::from_parts(
            Translation::from(isometry.translation.vector + direction.as_ref() * maximum_distance),
            Rotation::identity(),
        ));
        let aabb = a.merged(&b);

        // FIXME: avoid allocation.
        let interferences: Vec<_> = self.interferences_with_aabb(&aabb, groups).collect();

        let dispatcher = &*self.toi_dispatcher;
        interferences.into_iter().filter_map(move |(handle, x)| {
            dispatcher
                .time_of_impact(
                    dispatcher,
                    &isometry,
                    &direction,
                    shape,
                    x.position(),
                    &Vector::zeros(),
                    x.shape().as_ref(),
                    N::max_value(),
                    N::zero(),
                )
                .unwrap_or(None)
                .map(|toi| (handle, toi))
        })
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a, 'b>(
        &'a self,
        ray: &'b Ray<N>,
        max_toi: N,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithRay<'a, 'b, N, CollisionObjectSlab<N, T>> {
        glue::interferences_with_ray(&self.objects, &*self.broad_phase, ray, max_toi, groups)
    }

    /// Computes the first interference with `ray` and
    #[inline]
    pub fn first_interference_with_ray<'a, 'b>(
        &'a self,
        ray: &'b Ray<N>,
        max_toi: N,
        groups: &'b CollisionGroups,
    ) -> Option<FirstInterferenceWithRay<'a, N, CollisionObjectSlab<N, T>>> {
        glue::first_interference_with_ray(&self.objects, &*self.broad_phase, ray, max_toi, groups)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a, 'b>(
        &'a self,
        point: &'b Point<N>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithPoint<'a, 'b, N, CollisionObjectSlab<N, T>> {
        glue::interferences_with_point(&self.objects, &*self.broad_phase, point, groups)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a, 'b>(
        &'a self,
        aabb: &'b AABB<N>,
        groups: &'b CollisionGroups,
    ) -> InterferencesWithAABB<'a, 'b, N, CollisionObjectSlab<N, T>> {
        glue::interferences_with_aabb(&self.objects, &*self.broad_phase, aabb, groups)
    }

    /// Customize the selection of narrowphase collision detection algorithms
    pub fn set_narrow_phase(&mut self, narrow_phase: NarrowPhase<N, CollisionObjectSlabHandle>) {
        self.narrow_phase = narrow_phase;
        self.broad_phase.deferred_recompute_all_proximities();
    }

    /*
     *
     * Operations on the interaction graph.
     *
     */

    /// All the potential interactions pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pairs(
        &self,
        effective_only: bool,
    ) -> impl Iterator<
        Item = (
            CollisionObjectSlabHandle,
            CollisionObjectSlabHandle,
            &Interaction<N>,
        ),
    > {
        self.interactions.interaction_pairs(effective_only)
    }

    /// All the potential contact pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pairs(
        &self,
        effective_only: bool,
    ) -> impl Iterator<
        Item = (
            CollisionObjectSlabHandle,
            CollisionObjectSlabHandle,
            &ContactAlgorithm<N>,
            &ContactManifold<N>,
        ),
    > {
        self.interactions.contact_pairs(effective_only)
    }

    /// All the potential proximity pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pairs(
        &self,
        effective_only: bool,
    ) -> impl Iterator<
        Item = (
            CollisionObjectSlabHandle,
            CollisionObjectSlabHandle,
            &dyn ProximityDetector<N>,
            Proximity,
        ),
    > {
        self.interactions.proximity_pairs(effective_only)
    }

    /// The potential interaction pair between the two specified collision objects.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pair(
        &self,
        handle1: CollisionObjectSlabHandle,
        handle2: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<(
        CollisionObjectSlabHandle,
        CollisionObjectSlabHandle,
        &Interaction<N>,
    )> {
        let co1 = self.objects.collision_object(handle1)?;
        let co2 = self.objects.collision_object(handle2)?;
        let id1 = co1.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = co2.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        self.interactions.interaction_pair(id1, id2, effective_only)
    }

    /// The potential contact pair between the two specified collision objects.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pair(
        &self,
        handle1: CollisionObjectSlabHandle,
        handle2: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<(
        CollisionObjectSlabHandle,
        CollisionObjectSlabHandle,
        &ContactAlgorithm<N>,
        &ContactManifold<N>,
    )> {
        let co1 = self.objects.collision_object(handle1)?;
        let co2 = self.objects.collision_object(handle2)?;
        let id1 = co1.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = co2.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        self.interactions.contact_pair(id1, id2, effective_only)
    }

    /// The potential proximity pair between the two specified collision objects.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pair(
        &self,
        handle1: CollisionObjectSlabHandle,
        handle2: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<(
        CollisionObjectSlabHandle,
        CollisionObjectSlabHandle,
        &dyn ProximityDetector<N>,
        Proximity,
    )> {
        let co1 = self.objects.collision_object(handle1)?;
        let co2 = self.objects.collision_object(handle2)?;
        let id1 = co1.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = co2.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        self.interactions.proximity_pair(id1, id2, effective_only)
    }

    /// All the interaction pairs involving the specified collision object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with(
        &self,
        handle: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<
        impl Iterator<
            Item = (
                CollisionObjectSlabHandle,
                CollisionObjectSlabHandle,
                &Interaction<N>,
            ),
        >,
    > {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.interactions_with(id, effective_only))
    }

    /// All the mutable interactions pairs involving the specified collision object.
    ///
    /// This also returns a mutable reference to the narrow-phase which is necessary for updating the interaction if needed.
    /// For interactions between a collision object and itself, only one mutable reference to the collision object is returned.
    pub fn interactions_with_mut(
        &mut self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<(
        &mut NarrowPhase<N, CollisionObjectSlabHandle>,
        impl Iterator<
            Item = (
                CollisionObjectSlabHandle,
                CollisionObjectSlabHandle,
                TemporaryInteractionIndex,
                &mut Interaction<N>,
            ),
        >,
    )> {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some((
            &mut self.narrow_phase,
            self.interactions.interactions_with_mut(id),
        ))
    }

    /// All the proximity pairs involving the specified collision object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximities_with(
        &self,
        handle: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<
        impl Iterator<
            Item = (
                CollisionObjectSlabHandle,
                CollisionObjectSlabHandle,
                &dyn ProximityDetector<N>,
                Proximity,
            ),
        >,
    > {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.proximities_with(id, effective_only))
    }

    /// All the contact pairs involving the specified collision object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contacts_with(
        &self,
        handle: CollisionObjectSlabHandle,
        effective_only: bool,
    ) -> Option<
        impl Iterator<
            Item = (
                CollisionObjectSlabHandle,
                CollisionObjectSlabHandle,
                &ContactAlgorithm<N>,
                &ContactManifold<N>,
            ),
        >,
    > {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.contacts_with(id, effective_only))
    }

    /// All the collision object handles of collision objects interacting with the specified collision object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_interacting_with<'a>(
        &'a self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<impl Iterator<Item = CollisionObjectSlabHandle> + 'a> {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.collision_objects_interacting_with(id))
    }

    /// All the collision object handles of collision objects in potential contact with the specified collision
    /// object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_in_contact_with<'a>(
        &'a self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<impl Iterator<Item = CollisionObjectSlabHandle> + 'a> {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.collision_objects_in_contact_with(id))
    }

    /// All the collision object handles of collision objects in potential proximity of with the specified
    /// collision object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn collision_objects_in_proximity_of<'a>(
        &'a self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<impl Iterator<Item = CollisionObjectSlabHandle> + 'a> {
        let co = self.objects.collision_object(handle)?;
        let id = co.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(self.interactions.collision_objects_in_proximity_of(id))
    }

    /*
     *
     * Events
     *
     */
    /// The contact events pool.
    pub fn contact_events(&self) -> &ContactEvents<CollisionObjectSlabHandle> {
        self.narrow_phase.contact_events()
    }

    /// The proximity events pool.
    pub fn proximity_events(&self) -> &ProximityEvents<CollisionObjectSlabHandle> {
        self.narrow_phase.proximity_events()
    }
}
