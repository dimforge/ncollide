// types and traits
pub use broad::private::broad_phase::{BroadPhase,
                                      InterferencesBroadPhase,
                                      BoundingVolumeBroadPhase,
                                      RayCastBroadPhase};
pub use broad::private::dispatcher::{Dispatcher, NoIdDispatcher};
pub use broad::private::brute_force_broad_phase::BruteForceBroadPhase;
pub use broad::private::brute_force_bounding_volume_broad_phase::BruteForceBoundingVolumeBroadPhase;
pub use broad::private::dbvt_broad_phase::DBVTBroadPhase;

// modules
mod private { // FIXME: this is only to do the compiler's work: ensure invisibility of submodules.
    #[path = "../broad_phase.rs"]
    mod broad_phase;
    #[path = "../dispatcher.rs"]
    mod dispatcher;
    #[path = "../brute_force_broad_phase.rs"]
    mod brute_force_broad_phase;
    #[path = "../brute_force_bounding_volume_broad_phase.rs"]
    mod brute_force_bounding_volume_broad_phase;
    #[path = "../dbvt_broad_phase.rs"]
    mod dbvt_broad_phase;
}
