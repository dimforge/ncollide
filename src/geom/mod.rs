// traits and types
pub use geom::private::implicit::{Implicit, HasMargin};
pub use geom::private::ball::Ball;
pub use geom::private::plane::Plane;
pub use geom::private::box::Box;
pub use geom::private::capsule::Capsule;
pub use geom::private::cone::Cone;
pub use geom::private::cylinder::Cylinder;
pub use geom::private::convex_polytope::ConvexPolytope;
pub use geom::private::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use geom::private::reflection::Reflection;
pub use geom::private::compound::CompoundAABB;
pub use geom::private::geom_with_margin::GeomWithMargin;

// methods
pub use geom::private::minkowski_sum::cso_support_point;
pub use geom::private::minkowski_sum::cso_support_point_without_margin;

// modules
mod private { // FIXME: this is only to do the compiler's work: ensure invisibility of submodules.
    #[path = "../implicit.rs"]
    mod implicit;
    #[path = "../ball.rs"]
    mod ball;
    #[path = "../plane.rs"]
    mod plane;
    #[path = "../box.rs"]
    mod box;
    #[path = "../capsule.rs"]
    mod capsule;
    #[path = "../cone.rs"]
    mod cone;
    #[path = "../cylinder.rs"]
    mod cylinder;
    #[path = "../convex_polytope.rs"]
    mod convex_polytope;
    #[path = "../minkowski_sum.rs"]
    mod minkowski_sum;
    #[path = "../reflection.rs"]
    mod reflection;
    #[path = "../compound.rs"]
    mod compound;
    #[path = "../geom_with_margin.rs"]
    mod geom_with_margin;
}
