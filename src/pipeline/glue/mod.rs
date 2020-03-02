//! Glue code between each part of the collision-detection pipeline.

pub use self::query::{
    first_interference_with_ray, interferences_with_aabb, interferences_with_point,
    interferences_with_ray, FirstInterferenceWithRay, InterferencesWithAABB,
    InterferencesWithPoint, InterferencesWithRay,
};
pub use setup::{
    create_proxies, default_broad_phase, default_interaction_graph, default_narrow_phase,
    remove_proxies,
};
pub use update::{perform_all_pipeline, perform_broad_phase, perform_narrow_phase};

mod query;
mod setup;
mod update;
