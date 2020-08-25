use na::RealField;

use crate::bounding_volume::AABB;
use crate::pipeline::broad_phase::{
    BroadPhase, BroadPhaseInterferenceHandler, BroadPhasePairFilter,
};
use crate::pipeline::narrow_phase::{InteractionGraph, NarrowPhase};
use crate::pipeline::object::{CollisionGroupsPairFilter, CollisionObjectRef, CollisionObjectSet};

struct CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
where
    N: RealField,
    Objects: CollisionObjectSet<N>,
    Filter:
        BroadPhasePairFilter<N, Objects> + ?Sized,
{
    narrow_phase: &'b mut NarrowPhase<N, Objects::CollisionObjectHandle>,
    interactions: &'b mut InteractionGraph<N, Objects::CollisionObjectHandle>,
    objects: &'a Objects,
    pair_filters: Option<&'a Filter>,
}

impl<'a, 'b, N: RealField, Objects, Filter>
    BroadPhaseInterferenceHandler<Objects::CollisionObjectHandle>
    for CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
where
    Objects: CollisionObjectSet<N>,
    Filter:
        BroadPhasePairFilter<N, Objects> + ?Sized,
{
    fn is_interference_allowed(
        &mut self,
        b1: &Objects::CollisionObjectHandle,
        b2: &Objects::CollisionObjectHandle,
    ) -> bool {
        let filter_by_groups = CollisionGroupsPairFilter;

        filter_by_groups.is_pair_valid(*b1, *b2, self.objects)
            && self
                .pair_filters
                .map(|f| f.is_pair_valid(*b1, *b2, self.objects))
                .unwrap_or(true)
    }

    fn interference_started(
        &mut self,
        b1: &Objects::CollisionObjectHandle,
        b2: &Objects::CollisionObjectHandle,
    ) {
        self.narrow_phase
            .handle_interaction(self.interactions, self.objects, *b1, *b2, true)
    }

    fn interference_stopped(
        &mut self,
        b1: &Objects::CollisionObjectHandle,
        b2: &Objects::CollisionObjectHandle,
    ) {
        self.narrow_phase
            .handle_interaction(&mut self.interactions, self.objects, *b1, *b2, false)
    }
}

/// Performs the broad-phase.
///
/// This will update the broad-phase internal structure, and create potential interaction pairs in the interaction graph.
/// A `pair_filters` can be provided to filter out pairs of object that should not be considered.
pub fn perform_broad_phase<N: RealField, Objects>(
    objects: &Objects,
    broad_phase: &mut (impl BroadPhase<N, AABB<N>, Objects::CollisionObjectHandle> + ?Sized),
    narrow_phase: &mut NarrowPhase<N, Objects::CollisionObjectHandle>,
    interactions: &mut InteractionGraph<N, Objects::CollisionObjectHandle>,
    pair_filters: Option<
        &(impl BroadPhasePairFilter<N, Objects>
              + ?Sized),
    >,
) where
    Objects: CollisionObjectSet<N>,
{
    // Take changes into account.
    objects.foreach(|_, co| {
        let flags = co.update_flags();
        let proxy_handle = co.proxy_handle().expect(crate::NOT_REGISTERED_ERROR);

        if flags.needs_bounding_volume_update() {
            broad_phase.deferred_set_bounding_volume(proxy_handle, co.compute_swept_aabb());
        }

        if flags.needs_broad_phase_redispatch() {
            broad_phase.deferred_recompute_all_proximities_with(proxy_handle);
        }
    });

    // Update the broad-phase.
    broad_phase.update(&mut CollisionWorldInterferenceHandler {
        interactions,
        narrow_phase,
        pair_filters,
        objects,
    });
}

/// Performs the narrow-phase.
///
/// This will update all interactions in the interaction graph by computing new contacts,
/// and proximities.
pub fn perform_narrow_phase<N, Objects>(
    objects: &Objects,
    narrow_phase: &mut NarrowPhase<N, Objects::CollisionObjectHandle>,
    interactions: &mut InteractionGraph<N, Objects::CollisionObjectHandle>,
) where
    N: RealField,
    Objects: CollisionObjectSet<N>,
{
    narrow_phase.update(interactions, objects);
}

/// Performs the broad-phase and the narrow-phase.
///
/// This execute a complete collision detection pipeline by performing the broad-phase first and then
/// the narrow-phase.
pub fn perform_all_pipeline<'a, N, Objects>(
    objects: &Objects,
    broad_phase: &mut (impl BroadPhase<N, AABB<N>, Objects::CollisionObjectHandle> + ?Sized),
    narrow_phase: &mut NarrowPhase<N, Objects::CollisionObjectHandle>,
    interactions: &mut InteractionGraph<N, Objects::CollisionObjectHandle>,
    pair_filters: Option<
        &'a (impl BroadPhasePairFilter<N, Objects>
                 + ?Sized),
    >,
) where
    N: RealField,
    Objects: CollisionObjectSet<N>,
{
    perform_broad_phase(
        objects,
        broad_phase,
        narrow_phase,
        interactions,
        pair_filters,
    );
    perform_narrow_phase(objects, narrow_phase, interactions);
}
