use na::RealField;

use crate::math::Isometry;
use crate::shape::Shape;
use crate::bounding_volume::{AABB, BoundingVolume};
use crate::pipeline::object::{CollisionObjectHandle, GeometricQueryType};
use crate::pipeline::broad_phase::{BroadPhase, DBVTBroadPhase, BroadPhaseProxyHandle};
use crate::pipeline::narrow_phase::{NarrowPhase, DefaultContactDispatcher, DefaultProximityDispatcher, InteractionGraph, CollisionObjectGraphIndex};

pub fn create_proxies<'a, N: RealField, Handle: CollisionObjectHandle>(handle: Handle,
                                                      broad_phase: &mut (impl BroadPhase<N,AABB<N>, Handle> + ?Sized),
                                                      interactions: &mut InteractionGraph<N, Handle>,
                                                      position: &Isometry<N>,
                                                      shape: &(impl Shape<N> + ?Sized),
                                                      query_type: GeometricQueryType<N>)
                                                      -> (BroadPhaseProxyHandle, CollisionObjectGraphIndex) {
    let mut aabb = shape.aabb(position);
    aabb.loosen(query_type.query_limit());

    let proxy_handle = broad_phase.create_proxy(aabb, handle);
    let graph_index = interactions.add_node(handle);

    (proxy_handle, graph_index)
}


#[must_use = "The graph index of the collision object returned by this method has been changed to the returned graph index."]
pub fn remove_proxies<'a, N: RealField, Handle: CollisionObjectHandle>(
    broad_phase: &mut (impl BroadPhase<N,AABB<N>, Handle> + ?Sized),
    interactions: &mut InteractionGraph<N, Handle>,
    proxy_handle: BroadPhaseProxyHandle,
    graph_index: CollisionObjectGraphIndex)
    -> Option<(Handle, CollisionObjectGraphIndex)> {
    // NOTE: no need to notify handle the removed pairs because the node
    // will be removed from the interaction graph anyway.
    broad_phase.remove(&[proxy_handle], &mut |_, _| {});
    interactions.remove_node(graph_index).map(|h| (h, graph_index))
}

pub fn default_narrow_phase<N: RealField, Handle: CollisionObjectHandle>() -> NarrowPhase<N, Handle> {
    let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
    let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
    NarrowPhase::new(coll_dispatcher, prox_dispatcher)
}

pub fn default_broad_phase<N: RealField, Handle: CollisionObjectHandle>() -> DBVTBroadPhase<N, AABB<N>, Handle> {
    let default_margin = 0.01f64;
    DBVTBroadPhase::new(na::convert(default_margin))
}

pub fn default_interaction_graph<N: RealField, Handle: CollisionObjectHandle>() -> InteractionGraph<N, Handle> {
    InteractionGraph::new()
}
