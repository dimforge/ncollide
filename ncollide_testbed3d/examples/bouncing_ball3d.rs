extern crate nalgebra as na;
extern crate ncollide;
extern crate ncollide_testbed3d;

use std::cell::Cell;
use na::{Vector3, Point3, Isometry3, Translation};
use ncollide::world::{CollisionWorld, CollisionGroups, GeometricQueryType, CollisionObject3};
use ncollide::narrow_phase::{ProximityHandler, ContactHandler, ContactAlgorithm3};
use ncollide::shape::{Plane, Ball, Cuboid, ShapeHandle3};
use ncollide::query::Proximity;
use ncollide_testbed3d::Testbed;

/*
 *
 * Data associated to each object.
 *
 */
#[derive(Clone)]
struct CollisionObjectData {
    pub name:     &'static str,
    pub velocity: Option<Cell<Vector3<f32>>>
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector3<f32>>) -> CollisionObjectData {
        let init_velocity;
        if let Some(velocity) = velocity {
            init_velocity = Some(Cell::new(velocity))
        }
        else {
            init_velocity = None
        }

        CollisionObjectData {
            name:     name,
            velocity: init_velocity
        }
    }
}

/*
 *
 * Proximity handler.
 *
 */
struct ProximityMessage;

impl ProximityHandler<Point3<f32>, Isometry3<f32>, CollisionObjectData> for ProximityMessage {
    fn handle_proximity(&mut self,
                        co1: &CollisionObject3<f32, CollisionObjectData>,
                        co2: &CollisionObject3<f32, CollisionObjectData>,
                        _:             Proximity,
                        new_proximity: Proximity) {
        // The collision object with a None velocity is the coloured area.
        let area_name;

        if co1.data.velocity.is_none() {
            area_name = co1.data.name;
        }
        else {
            area_name = co2.data.name;
        }

        if new_proximity == Proximity::Intersecting {
            println!("The ball enters the {} area.", area_name);
        }
        else if new_proximity == Proximity::Disjoint {
            println!("The ball leaves the {} area.", area_name);
        }
    }
}

/*
 *
 * Contact handler.
 *
 */
struct VelocityBouncer;

impl ContactHandler<Point3<f32>, Isometry3<f32>, CollisionObjectData> for VelocityBouncer {
    fn handle_contact_started(&mut self,
                              co1: &CollisionObject3<f32, CollisionObjectData>,
                              co2: &CollisionObject3<f32, CollisionObjectData>,
                              alg: &ContactAlgorithm3<f32>) {
        // NOTE: real-life applications would avoid this systematic allocation.
        let mut collector = Vec::new();
        alg.contacts(&mut collector);

        // The ball is the one with a non-None velocity.
        if let Some(ref vel) = co1.data.velocity {
            let normal = collector[0].normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * normal);
        }
        if let Some(ref vel) = co2.data.velocity {
            let normal = -collector[0].normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * normal);
        }
    }

    fn handle_contact_stopped(&mut self,
                              _: &CollisionObject3<f32, CollisionObjectData>,
                              _: &CollisionObject3<f32, CollisionObjectData>) {
        // We don't care.
    }
}


fn main() {
    /*
     * Setup initial object properties.
     */
    // Plane shapes.
    let plane_left   = ShapeHandle3::new(Plane::new(Vector3::x()));
    let plane_bottom = ShapeHandle3::new(Plane::new(Vector3::y()));
    let plane_back   = ShapeHandle3::new(Plane::new(Vector3::z()));
    let plane_right  = ShapeHandle3::new(Plane::new(-Vector3::x()));
    let plane_top    = ShapeHandle3::new(Plane::new(-Vector3::y()));
    let plane_front  = ShapeHandle3::new(Plane::new(-Vector3::z()));

    // Shared cuboid for the rectangular areas.
    let rect = ShapeHandle3::new(Cuboid::new(Vector3::new(4.9f32, 4.9, 4.9)));

    // Ball shape.
    let ball = ShapeHandle3::new(Ball::new(0.5f32));

    // Positions of the planes.
    let planes_pos = [
        Isometry3::new(Vector3::new(-10.0, 0.0, 0.0),  na::zero()),
        Isometry3::new(Vector3::new(0.0, -10.0, 0.0),  na::zero()),
        Isometry3::new(Vector3::new(0.0,  0.0, -10.0), na::zero()),
        Isometry3::new(Vector3::new(10.0, 0.0,  0.0),  na::zero()),
        Isometry3::new(Vector3::new(0.0,  10.0, 0.0),  na::zero()),
        Isometry3::new(Vector3::new(0.0,  0.0,  10.0), na::zero())
    ];

    // Position of the rectangles.
    let rects_pos = [
        Isometry3::new(Vector3::new(-5.0, 5.0, 5.0),  na::zero()),
        Isometry3::new(Vector3::new(5.0, 5.0, 5.0),   na::zero()),
        Isometry3::new(Vector3::new(5.0, -5.0, 5.0),  na::zero()),
        Isometry3::new(Vector3::new(-5.0, -5.0, 5.0), na::zero()),
        Isometry3::new(Vector3::new(-5.0, 5.0, -5.0),  na::zero()),
        Isometry3::new(Vector3::new(5.0, 5.0, -5.0),   na::zero()),
        Isometry3::new(Vector3::new(5.0, -5.0, -5.0),  na::zero()),
        Isometry3::new(Vector3::new(-5.0, -5.0, -5.0), na::zero())
    ];

    // Position of the ball.
    let ball_pos = Isometry3::new(Vector3::new(5.0, 5.0, 5.0), na::zero());

    // The ball is part of group 1 and can interact with everything.
    let mut ball_groups = CollisionGroups::new();
    ball_groups.set_membership(&[1]);

    // All the other objects are part of the group 2 and interact only with the ball (but not with
    // each other).
    let mut others_groups = CollisionGroups::new();
    others_groups.set_membership(&[2]);
    others_groups.set_whitelist(&[1]);

    let plane_data       = CollisionObjectData::new("ground", None);
    let rect_data_purple = CollisionObjectData::new("purple", None);
    let rect_data_blue   = CollisionObjectData::new("blue", None);
    let rect_data_green  = CollisionObjectData::new("green", None);
    let rect_data_yellow = CollisionObjectData::new("yellow", None);
    let rect_data_red    = CollisionObjectData::new("red", None);
    let rect_data_grey   = CollisionObjectData::new("grey", None);
    let rect_data_pink   = CollisionObjectData::new("pink", None);
    let rect_data_cyan   = CollisionObjectData::new("cyan", None);
    let ball_data        = CollisionObjectData::new("ball", Some(Vector3::new(10.0, 5.0, 5.0)));

    /*
     * Setup the world.
     */
    // Collision world 0.02 optimization margin and small object identifiers.
    let mut world = CollisionWorld::new(0.02, true);

    // Add the planes to the world.
    let contacts_query  = GeometricQueryType::Contacts(0.0);
    let proximity_query = GeometricQueryType::Proximity(0.0);

    world.add(0, planes_pos[0], plane_left,   others_groups, contacts_query, plane_data.clone());
    world.add(1, planes_pos[1], plane_bottom, others_groups, contacts_query, plane_data.clone());
    world.add(2, planes_pos[2], plane_back,   others_groups, contacts_query, plane_data.clone());
    world.add(3, planes_pos[3], plane_right,  others_groups, contacts_query, plane_data.clone());
    world.add(4, planes_pos[4], plane_top,    others_groups, contacts_query, plane_data.clone());
    world.add(5, planes_pos[5], plane_front,  others_groups, contacts_query, plane_data.clone());

    // Add the colored rectangles to the world.
    world.add(6,  rects_pos[0], rect.clone(), others_groups, proximity_query, rect_data_purple);
    world.add(7,  rects_pos[1], rect.clone(), others_groups, proximity_query, rect_data_blue);
    world.add(8,  rects_pos[2], rect.clone(), others_groups, proximity_query, rect_data_green);
    world.add(9,  rects_pos[3], rect.clone(), others_groups, proximity_query, rect_data_yellow);
    world.add(10, rects_pos[4], rect.clone(), others_groups, proximity_query, rect_data_red);
    world.add(11, rects_pos[5], rect.clone(), others_groups, proximity_query, rect_data_grey);
    world.add(12, rects_pos[6], rect.clone(), others_groups, proximity_query, rect_data_pink);
    world.add(13, rects_pos[7], rect.clone(), others_groups, proximity_query, rect_data_cyan);

    // Add the ball to the world.
    world.add(14, ball_pos, ball, ball_groups, GeometricQueryType::Contacts(0.0), ball_data);

    // Register our handlers.
    world.register_proximity_handler("ProximityMessage", ProximityMessage);
    world.register_contact_handler("VelocityBouncer", VelocityBouncer);

    /*
     * Run indefinitely.
     */
    let timestep = 0.016;

    let mut testbed = Testbed::new();
    testbed.set_visible(0, false);
    testbed.set_visible(1, false);
    testbed.set_visible(2, false);
    testbed.set_visible(3, false);
    testbed.set_visible(4, false);
    testbed.set_visible(5, false);
    testbed.set_color(6, Point3::new(0.5, 0.0, 0.5));
    testbed.set_color(7, Point3::new(0.0, 0.0, 1.0));
    testbed.set_color(8, Point3::new(0.0, 1.0, 0.0));
    testbed.set_color(9, Point3::new(1.0, 1.0, 0.0));
    testbed.set_color(10, Point3::new(1.0, 0.0, 0.0));
    testbed.set_color(11, Point3::new(0.5, 0.5, 0.5));
    testbed.set_color(12, Point3::new(1.0, 0.75, 0.8));
    testbed.set_color(13, Point3::new(0.0, 1.0, 1.0));
    testbed.set_color(14, Point3::new(1.0, 1.0, 1.0));

    // let mut i = 0;
    // testbed.start_recording("bouncing_ball.mpg");

    while testbed.step(&mut world) {
        let ball_pos;

        {
            // Integrate the velocities.
            let ball_object   = world.collision_object(14).unwrap();
            let ball_velocity = ball_object.data.velocity.as_ref().unwrap();

            // Integrate the positions.
            ball_pos = ball_object.position.append_translation(&(timestep * ball_velocity.get()));
        }

        // Submit the position update to the world.
        world.deferred_set_position(14, ball_pos);

        // i += 1;
        // if i == 96 * 5 {
        //     testbed.stop_recording()
        // }
    }
}
