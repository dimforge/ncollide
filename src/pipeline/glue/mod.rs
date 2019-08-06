//! Glue code between each part of the collision-detection pipeline.

pub use self::query::{
    InterferencesWithAABB, InterferencesWithPoint, InterferencesWithRay,
    interferences_with_ray, interferences_with_point, interferences_with_aabb
};
pub use update::{perform_narrow_phase, perform_broad_phase, perform_all_pipeline};
pub use setup::{
    create_proxies, remove_proxies,
    default_broad_phase, default_narrow_phase, default_interaction_graph
};


mod query;
mod setup;
mod update;
