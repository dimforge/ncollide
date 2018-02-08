extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry3, Vector3};
use na::{Isometry2, Vector2, Point2};
use ncollide::shape::Cuboid;
use ncollide::query::contacts_internal;
use ncollide::query::ContactPrediction;
use ncollide::narrow_phase::{ContactDispatcher, DefaultContactDispatcher};

#[test]
fn cuboid_cuboid_epa3() {
    let c = Cuboid::new(Vector3::new(2.0, 1.0, 1.0));
    let m1 = Isometry3::new(Vector3::new(3.5, 0.0, 0.0), na::zero());
    let m2 = Isometry3::identity();

    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, -Vector3::x_axis());

    let m1 = Isometry3::new(Vector3::new(0.0, 0.2, 0.0), na::zero());
    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 1.8);
    assert_eq!(res.normal, -Vector3::y_axis());
}

#[test]
fn cuboid_cuboid_epa2() {
    let c = Cuboid::new(Vector2::new(2.0, 1.0));
    let m1 = Isometry2::new(Vector2::new(3.5, 0.0), na::zero());
    let m2 = Isometry2::identity();

    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 0.5);
    assert_eq!(res.normal, -Vector2::x_axis());

    let m1 = Isometry2::new(Vector2::new(0.0, 0.2), na::zero());
    let res = contacts_internal::support_map_against_support_map(&m1, &c, &m2, &c, 10.0)
        .expect("Penetration not found.");
    assert_eq!(res.depth, 1.8);
    assert_eq!(res.normal, -Vector2::y_axis());
}

#[test]
fn cuboids_large_size_ratio_issue_181() {              
    let cuboid_a = Cuboid::new(Vector2::new(10.0, 10.0));
    let cuboid_b = Cuboid::new(Vector2::new(300.0, 1.5));

    let pos_b = Isometry2::new(Vector2::new(5.0, 0.0), 1.5);

    let dispatcher = DefaultContactDispatcher::<Point2<f32>, Isometry2<f32>>::new();
    let mut algorithm = dispatcher.get_contact_algorithm(&cuboid_a, &cuboid_b).unwrap();

    let prediction = ContactPrediction::new(0.0, 0.0, 0.0);
  
    let mut p = Vector2::new(0.0, 0.0); 
    let mut angle = 0.0; 
    
    // Used to panic at some point:
    // thread 'main' panicked at 'assertion failed: neg_dist <= gjk::eps_tol()', ncollide_geometry/query/algorithms/epa2.rs:26:9
    for _ in 1..200000 {
        p.x += 0.0001;   
        angle += 0.005;  
            
        let pos_a = Isometry2::new(p, angle); 
          
        algorithm.update(&dispatcher, &pos_a, &cuboid_a, &pos_b, &cuboid_b, &prediction);
          
        let mut contacts = Vec::new();
        algorithm.contacts(&mut contacts);
      
        for contact in contacts.iter() {
            p -= contact.normal.unwrap() * contact.depth;
        } 
    }
} 