extern crate "nalgebra" as na;
extern crate ncollide;

use na::{Vec3, Iso3};
use ncollide::bounding_volume::{BoundingVolume, HasBoundingSphere};
use ncollide::shape::{Cone, Cylinder};

fn main() {
    /*
     * Initialize the shapes.
     */
    let cone     = Cone::new(0.5f32, 0.5);
    let cylinder = Cylinder::new(1.0f32, 0.5);

    let cone_pos     = Iso3::new(Vec3::z(), na::zero()); // 1.0 along the `z` axis.
    let cylinder_pos = na::one::<Iso3<f32>>();                        // Identity matrix.

    /*
     * Compute their bounding spheres.
     */
    let bounding_sphere_cone     = cone.bounding_sphere(&cone_pos);
    let bounding_sphere_cylinder = cylinder.bounding_sphere(&cylinder_pos);

    // Merge the two spheres.
    let bounding_bounding_sphere = bounding_sphere_cone.merged(&bounding_sphere_cylinder);

    // Enlarge the cylinder bounding sphere.
    let loose_bounding_sphere_cylinder = bounding_sphere_cylinder.loosened(1.0);

    // Intersection and inclusion tests.
    assert!(bounding_sphere_cone.intersects(&bounding_sphere_cylinder));
    assert!(bounding_bounding_sphere.contains(&bounding_sphere_cone));
    assert!(bounding_bounding_sphere.contains(&bounding_sphere_cylinder));
    assert!(!bounding_sphere_cylinder.contains(&bounding_bounding_sphere));
    assert!(!bounding_sphere_cone.contains(&bounding_bounding_sphere));
    assert!(loose_bounding_sphere_cylinder.contains(&bounding_sphere_cylinder));
}
