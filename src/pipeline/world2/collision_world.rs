use crate::bounding_volume::{self, BoundingVolume, AABB};
use crate::math::{Isometry, Point};
use na::RealField;
use crate::pipeline::broad_phase::{
    BroadPhase, BroadPhasePairFilter, BroadPhasePairFilters, DBVTBroadPhase, BroadPhaseProxyHandle,
    BroadPhaseInterferenceHandler, BroadPhasePairFilter2
};
use crate::pipeline::events::{ContactEvent, ContactEvents, ProximityEvents};
use crate::pipeline::narrow_phase::{
    DefaultContactDispatcher, NarrowPhase2, DefaultProximityDispatcher,
    CollisionObjectGraphIndex, Interaction, ContactAlgorithm, ProximityAlgorithm,
    TemporaryInteractionIndex,
};
use crate::pipeline::world::{
    CollisionGroups, CollisionGroupsPairFilter2, CollisionObject, CollisionObjectHandle,
    CollisionObjectSlab, CollisionObjects, GeometricQueryType,
};
use crate::pipeline::narrow_phase::InteractionGraph2;
use crate::query::{PointQuery, Ray, RayCast, RayIntersection, ContactManifold};
use crate::shape::Shape;
use crate::pipeline::world2::{CollisionObjectSet, CollisionObjectRef};
use std::vec::IntoIter;


struct CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N>,
          Filter: BroadPhasePairFilter2<'a, N, Objects::CollisionObject> {
    narrow_phase: &'b mut NarrowPhase2<N, Objects::Handle>,
    interactions: &'b mut InteractionGraph2<N, Objects::Handle>,
    objects: &'a Objects,
    pair_filters: Option<&'a Filter>,
}

impl <'a, 'b, N: RealField, Objects, Filter> BroadPhaseInterferenceHandler<Objects::Handle> for CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
    where Objects: CollisionObjectSet<'a, N>,
          Filter: BroadPhasePairFilter2<'a, N, Objects::CollisionObject> {
    fn is_interference_allowed(&mut self, b1: &Objects::Handle, b2: &Objects::Handle) -> bool {
        let o1 = try_ret!(self.objects.get(*b1), false);
        let o2 = try_ret!(self.objects.get(*b2), false);
        let filter_by_groups = CollisionGroupsPairFilter2;

        filter_by_groups.is_pair_valid(o1, o2) &&
            self.pair_filters.map(|f| f.is_pair_valid(o1, o2)).unwrap_or(true)
    }

    fn interference_started(&mut self, b1: &Objects::Handle, b2: &Objects::Handle) {
        self.narrow_phase.handle_interaction(
            self.interactions,
            self.objects,
            *b1, *b2,
            true
        )
    }

    fn interference_stopped(&mut self, b1: &Objects::Handle, b2: &Objects::Handle) {
        self.narrow_phase.handle_interaction(
            &mut self.interactions,
            self.objects,
            *b1, *b2,
            false
        )
    }
}

pub fn create_proxies<'a, N: RealField, Handle: Copy>(handle: Handle,
                                                    broad_phase: &mut impl BroadPhase<N,AABB<N>, Handle>,
                                                    interactions: &mut InteractionGraph2<N, Handle>,
                                                    position: &Isometry<N>,
                                                    shape: &impl Shape<N>,
                                                    query_type: &GeometricQueryType<N>)
                                                    -> (BroadPhaseProxyHandle, CollisionObjectGraphIndex) {
    let mut aabb = shape.aabb(position);
    aabb.loosen(query_type.query_limit());

    let proxy_handle = broad_phase.create_proxy(aabb, handle);
    let graph_index = interactions.add_node(handle);

    (proxy_handle, graph_index)
}


#[must_use = "The graph index of the collision object returned by this method has been changed to the returned graph index."]
pub fn delete_proxies<'a, N: RealField, Handle: Copy>(
     broad_phase: &mut impl BroadPhase<N,AABB<N>, Handle>,
     interactions: &mut InteractionGraph2<N, Handle>,
     proxy_handle: BroadPhaseProxyHandle,
     graph_index: CollisionObjectGraphIndex)
    -> Option<(Handle, CollisionObjectGraphIndex)> {
    // NOTE: no need to notify handle the removed pairs because the node
    // will be removed from the interaction graph anyway.
    broad_phase.remove(&[proxy_handle], &mut |_, _| {});
    interactions.remove_node(graph_index).map(|h| (h, graph_index))
}

pub fn perform_broad_phase<'a, N: RealField, Objects>(objects: &'a Objects,
                                                      broad_phase: &mut impl BroadPhase<N, AABB<N>, Objects::Handle>,
                                                      narrow_phase: &mut NarrowPhase2<N, Objects::Handle>,
                                                      interactions: &mut InteractionGraph2<N, Objects::Handle>,
                                                      pair_filters: Option<&'a impl BroadPhasePairFilter2<'a, N, Objects::CollisionObject>>)
    where Objects: CollisionObjectSet<'a, N> {

    // Take changes into account.
    for (handle, co) in objects.iter() {
        let flags = co.update_flags();

        if flags.needs_bounding_volume_update() {
            broad_phase.deferred_set_bounding_volume(co.proxy_handle(), co.compute_aabb());
        }

        if flags.needs_broad_phase_redispatch() {
            broad_phase.deferred_recompute_all_proximities_with(co.proxy_handle());
        }
    }

    // Update the broad-phase.
    broad_phase.update(&mut CollisionWorldInterferenceHandler {
        interactions,
        narrow_phase,
        pair_filters,
        objects,
    });
}

pub fn perform_narrow_phase<'a, N, Objects>(objects: &'a Objects,
                                            narrow_phase: &mut NarrowPhase2<N, Objects::Handle>,
                                            interactions: &mut InteractionGraph2<N, Objects::Handle>)
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    narrow_phase.clear_events();
    narrow_phase.update(interactions, objects);
}


pub fn perform_all_pipeline<'a, N, Objects>(objects: &'a Objects,
                                          broad_phase: &mut impl BroadPhase<N, AABB<N>, Objects::Handle>,
                                          narrow_phase: &mut NarrowPhase2<N, Objects::Handle>,
                                          interactions: &mut InteractionGraph2<N, Objects::Handle>,
                                          pair_filters: Option<&'a impl BroadPhasePairFilter2<'a, N, Objects::CollisionObject>>)
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    perform_broad_phase(objects, broad_phase, narrow_phase, interactions, pair_filters);
    perform_narrow_phase(objects, narrow_phase, interactions);
}

pub fn default_narrow_phase<N: RealField, Handle: Copy>() -> NarrowPhase2<N, Handle> {
    let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
    let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
    NarrowPhase2::new(coll_dispatcher, prox_dispatcher)
}

pub fn default_broad_phase<N: RealField, Handle: Copy>() -> DBVTBroadPhase<N, AABB<N>, Handle> {
    let default_margin = 0.01f64;
    DBVTBroadPhase::new(na::convert(default_margin))
}

pub fn default_interaction_graph<N: RealField, Handle: Copy>() -> InteractionGraph2<N, Handle> {
    InteractionGraph2::new()
}


/*
    /// The broad-phase aabb for the given collision object.
    pub fn broad_phase_aabb(&self, handle: CollisionObjectHandle) -> Option<&AABB<N>> {
        let co = self.objects.get(handle)?;
        self.broad_phase.proxy(co.proxy_handle()).map(|p| p.0)
    }
*/

pub fn interferences_with_ray<'a, 'b, N, Objects>(objects: &'a Objects,
                                                  broad_phase: &'a impl BroadPhase<N, AABB<N>, Objects::Handle>,
                                                  ray: &'b Ray<N>,
                                                  groups: &'b CollisionGroups)
        -> InterferencesWithRay<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_ray(ray, &mut handles);

    InterferencesWithRay {
        ray,
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, 'b, N: RealField, Objects: CollisionObjectSet<'a, N>> {
    ray: &'b Ray<N>,
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::Handle>,
}

impl<'a, 'b, N: RealField, Objects> Iterator for InterferencesWithRay<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    type Item = (Objects::CollisionObject, RayIntersection<N>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.get(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups) {
                    let inter = co
                        .shape()
                        .toi_and_normal_with_ray(&co.position(), self.ray, true);

                    if let Some(inter) = inter {
                        return Some((co, inter));
                    }
                }
            }
        }

        None
    }
}


pub fn interferences_with_point<'a, 'b, N, Objects>(objects: &'a Objects,
                                                  broad_phase: &'a impl BroadPhase<N, AABB<N>, Objects::Handle>,
                                                  point: &'b Point<N>,
                                                  groups: &'b CollisionGroups)
                                                  -> InterferencesWithPoint<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_point(point, &mut handles);

    InterferencesWithPoint {
        point,
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world that intersect a specific point.
pub struct InterferencesWithPoint<'a, 'b, N: RealField, Objects: CollisionObjectSet<'a, N>> {
    point: &'b Point<N>,
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::Handle>,
}

impl<'a, 'b, N: RealField, Objects> Iterator for InterferencesWithPoint<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    type Item = Objects::CollisionObject;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.get(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups)
                    && co.shape().contains_point(&co.position(), self.point)
                {
                    return Some(co);
                }
            }
        }

        None
    }
}


pub fn interferences_with_aabb<'a, 'b, N, Objects>(objects: &'a Objects,
                                                    broad_phase: &'a impl BroadPhase<N, AABB<N>, Objects::Handle>,
                                                    aabb: &AABB<N>,
                                                    groups: &'b CollisionGroups)
                                                    -> InterferencesWithAABB<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_bounding_volume(aabb, &mut handles);

    InterferencesWithAABB {
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world which bounding volume intersects a specific AABB.
pub struct InterferencesWithAABB<'a, 'b, N: RealField, Objects: CollisionObjectSet<'a, N>> {
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::Handle>,
}

impl<'a, 'b, N: RealField, Objects: CollisionObjectSet<'a, N>> Iterator for InterferencesWithAABB<'a, 'b, N, Objects> {
    type Item = Objects::CollisionObject;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.get(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups) {
                    return Some(co);
                }
            }
        }

        None
    }
}
