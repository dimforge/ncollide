// types an traits
pub use ray::private::ray::{Ray, RayCast, RayCastWithTransform};

// functions
pub use ray::private::ray_plane::plane_toi_with_ray;

// modules
mod private { // FIXME: this is only to do the compiler's work: ensure invisibility of submodules.
    #[path = "../ray.rs"]
    mod ray;

    #[path = "../ray_plane.rs"]
    mod ray_plane;

    #[path = "../ray_ball.rs"]
    mod ray_ball;

    #[path = "../ray_box.rs"]
    mod ray_box;

    #[path = "../ray_aabb.rs"]
    mod ray_aabb;

    #[path = "../ray_compound.rs"]
    mod ray_compound;

    #[path = "../ray_implicit.rs"]
    mod ray_implicit;
}
