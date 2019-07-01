use na::RealField;

use crate::math::Isometry;
use crate::shape::Shape;
use crate::bounding_volume::{AABB, BoundingVolume};
use crate::pipeline::object::{GeometricQueryType, CollisionObjectSet, CollisionGroupsPairFilter, CollisionObjectRef};
use crate::pipeline::broad_phase::{BroadPhase, DBVTBroadPhase, BroadPhaseProxyHandle, BroadPhasePairFilter, BroadPhaseInterferenceHandler};
use crate::pipeline::narrow_phase::{NarrowPhase, DefaultContactDispatcher, DefaultProximityDispatcher, InteractionGraph, CollisionObjectGraphIndex};


struct CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
    where N: RealField,
          Objects: CollisionObjectSet<'a, N>,
          Filter: BroadPhasePairFilter<N, Objects::CollisionObject, Objects::Handle> + ?Sized {
    narrow_phase: &'b mut NarrowPhase<N, Objects::Handle>,
    interactions: &'b mut InteractionGraph<N, Objects::Handle>,
    objects: &'a Objects,
    pair_filters: Option<&'a Filter>,
}

impl <'a, 'b, N: RealField, Objects, Filter> BroadPhaseInterferenceHandler<Objects::Handle> for CollisionWorldInterferenceHandler<'a, 'b, N, Objects, Filter>
    where Objects: CollisionObjectSet<'a, N>,
          Filter: BroadPhasePairFilter<N, Objects::CollisionObject, Objects::Handle> + ?Sized {
    fn is_interference_allowed(&mut self, b1: &Objects::Handle, b2: &Objects::Handle) -> bool {
        let o1 = try_ret!(self.objects.get(*b1), false);
        let o2 = try_ret!(self.objects.get(*b2), false);
        let filter_by_groups = CollisionGroupsPairFilter;

        filter_by_groups.is_pair_valid(o1, o2, *b1, *b2) &&
            self.pair_filters.map(|f| f.is_pair_valid(o1, o2, *b1, *b2)).unwrap_or(true)
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

pub fn perform_broad_phase<'a, N: RealField, Objects>(objects: &'a Objects,
                                                      broad_phase: &mut (impl BroadPhase<N, AABB<N>, Objects::Handle> + ?Sized),
                                                      narrow_phase: &mut NarrowPhase<N, Objects::Handle>,
                                                      interactions: &mut InteractionGraph<N, Objects::Handle>,
                                                      pair_filters: Option<&'a (impl BroadPhasePairFilter<N, Objects::CollisionObject, Objects::Handle> + ?Sized)>)
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
                                            narrow_phase: &mut NarrowPhase<N, Objects::Handle>,
                                            interactions: &mut InteractionGraph<N, Objects::Handle>)
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    narrow_phase.clear_events();
    narrow_phase.update(interactions, objects);
}


pub fn perform_all_pipeline<'a, N, Objects>(objects: &'a Objects,
                                          broad_phase: &mut (impl BroadPhase<N, AABB<N>, Objects::Handle> + ?Sized),
                                          narrow_phase: &mut NarrowPhase<N, Objects::Handle>,
                                          interactions: &mut InteractionGraph<N, Objects::Handle>,
                                          pair_filters: Option<&'a (impl BroadPhasePairFilter<N, Objects::CollisionObject, Objects::Handle> + ?Sized)>)
    where N: RealField,
          Objects: CollisionObjectSet<'a, N> {
    perform_broad_phase(objects, broad_phase, narrow_phase, interactions, pair_filters);
    perform_narrow_phase(objects, narrow_phase, interactions);
}



/*
    /// The broad-phase aabb for the given collision object.
    pub fn broad_phase_aabb(&self, handle: CollisionObjectSlabHandle) -> Option<&AABB<N>> {
        let co = self.objects.get(handle)?;
        self.broad_phase.proxy(co.proxy_handle()).map(|p| p.0)
    }
*/
