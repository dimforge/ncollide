extern crate nalgebra as na;
extern crate ncollide;

use na::{Vec2, Iso2};
use ncollide::bounding_volume::{self, BoundingVolume};
use ncollide::shape::{Cone, Cylinder};

fn main() {
    /*
     * Initialize the shapes.
     */
    let cone     = Cone::new(0.5, 0.5);
    let cylinder = Cylinder::new(1.0, 0.5);

    let cone_pos     = Iso2::new(Vec2::y(), na::zero()); // 1.0 along the `y` axis.
    let cylinder_pos = na::one::<Iso2<f32>>();           // Identity matrix.

    /*
     * Compute their axis-aligned bounding boxes.
     */
    let aabb_cone     = bounding_volume::aabb(&cone, &cone_pos);
    let aabb_cylinder = bounding_volume::aabb(&cylinder, &cylinder_pos);

    // Merge the two boxes.
    let bounding_aabb = aabb_cone.merged(&aabb_cylinder);

    // Enlarge the cylinder aabb.
    let loose_aabb_cylinder = aabb_cylinder.loosened(1.0);

    // Intersection and inclusion tests.
    assert!(aabb_cone.intersects(&aabb_cylinder));
    assert!(bounding_aabb.contains(&aabb_cone));
    assert!(bounding_aabb.contains(&aabb_cylinder));
    assert!(!aabb_cylinder.contains(&bounding_aabb));
    assert!(!aabb_cone.contains(&bounding_aabb));
    assert!(loose_aabb_cylinder.contains(&aabb_cylinder));
}
