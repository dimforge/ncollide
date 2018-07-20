extern crate nalgebra as na;
extern crate ncollide2d;

use na::{Isometry2, Vector2};
use ncollide2d::bounding_volume::{self, BoundingVolume};
use ncollide2d::shape::Cuboid;

fn main() {
    /*
     * Initialize the shapes.
     */
    let cube1 = Cuboid::new(Vector2::repeat(0.5));
    let cube2 = Cuboid::new(Vector2::new(1.0, 0.5));

    let cube1_pos = Isometry2::new(Vector2::y(), na::zero()); // 1.0 along the `y` axis.
    let cube2_pos = na::one::<Isometry2<f32>>(); // Identity matrix.

    /*
     * Compute their bounding spheres.
     */
    let bounding_sphere_cube1 = bounding_volume::bounding_sphere(&cube1, &cube1_pos);
    let bounding_sphere_cube2 = bounding_volume::bounding_sphere(&cube2, &cube2_pos);

    // Merge the two spheres.
    let bounding_bounding_sphere = bounding_sphere_cube1.merged(&bounding_sphere_cube2);

    // Enlarge the cube2 bounding sphere.
    let loose_bounding_sphere_cube2 = bounding_sphere_cube2.loosened(1.0);

    // Intersection and inclusion tests.
    assert!(bounding_sphere_cube1.intersects(&bounding_sphere_cube2));
    assert!(bounding_bounding_sphere.contains(&bounding_sphere_cube1));
    assert!(bounding_bounding_sphere.contains(&bounding_sphere_cube2));
    assert!(!bounding_sphere_cube2.contains(&bounding_bounding_sphere));
    assert!(!bounding_sphere_cube1.contains(&bounding_bounding_sphere));
    assert!(loose_bounding_sphere_cube2.contains(&bounding_sphere_cube2));
}
