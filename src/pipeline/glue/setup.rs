use na::RealField;

use crate::math::Isometry;
use crate::shape::Shape;
use crate::bounding_volume::{AABB, BoundingVolume};
use crate::pipeline::object::{CollisionObjectHandle, GeometricQueryType};
use crate::pipeline::broad_phase::{BroadPhase, DBVTBroadPhase, BroadPhaseProxyHandle};
use crate::pipeline::narrow_phase::{NarrowPhase, DefaultContactDispatcher, DefaultProximityDispatcher, InteractionGraph, CollisionObjectGraphIndex};

/// Registers a collision object handle so it can be taken into acconut by the broad-phase and the narrow-phase.
///
/// This tells the broad-phase and the interaction graph exists and allow them to perform some setup to accomodate
/// for a new collision object with the given `handle`, and known to currently have the given `position`, `shape` and `query_type`.
/// The result of this registration is a pair of handles that must be stored by the user as the will be needed for various
/// queries (on the broad-phase and interaction graph), as well as for freeing (using `remove_proxies`) the resources allocated here.
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


/// Free all the resources allocated by the broad-phase and the interaction graph for the given proxy handles.
///
/// This will free the resoruces allocated by the `create_proxies` function. A special care should be
/// taken with the return value of this function.
///
/// # Return (important, please read)
///
/// Thus function either returns `None` or a pair composed of a collision object handle and a collision object graph index.
/// If `None` is returned, no extra work is required from the caller. If `Some` is returned, the it is necessary that
/// the collision object graph index of the collider identified by the returned handle is replaced by the returned
/// collision object graph index. This is caused by the fact that the indices used by the interaction graph to identify
///  collision objects are not stable wrt. the deletion of collision objects. Therefore, removing one collision object can
/// result in the collision object graph index of another collision object to be changed.
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

/// Allocate a default narrow-phase, configured with the default contact and proximity dispatchers.
pub fn default_narrow_phase<N: RealField, Handle: CollisionObjectHandle>() -> NarrowPhase<N, Handle> {
    let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
    let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
    NarrowPhase::new(coll_dispatcher, prox_dispatcher)
}

/// Allocate a default broad-phase, configured with a default coherence margin (set to 0.01).
pub fn default_broad_phase<N: RealField, Handle: CollisionObjectHandle>() -> DBVTBroadPhase<N, AABB<N>, Handle> {
    let default_margin = 0.01f64;
    DBVTBroadPhase::new(na::convert(default_margin))
}

/// Allocate a default interaction graph.
pub fn default_interaction_graph<N: RealField, Handle: CollisionObjectHandle>() -> InteractionGraph<N, Handle> {
    InteractionGraph::new()
}
