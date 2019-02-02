extern crate nalgebra as na;
extern crate ncollide2d;
extern crate ncollide_testbed2d;

use na::{Isometry2, Point2, Point3, Translation2, Vector2};
use ncollide2d::events::{ContactEvent, ProximityEvent};
use ncollide2d::narrow_phase::ContactAlgorithm;
use ncollide2d::query::Proximity;
use ncollide2d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use ncollide2d::world::{CollisionGroups, CollisionObject, CollisionWorld, GeometricQueryType};
use ncollide_testbed2d::Testbed;
use std::cell::Cell;

/*
 *
 * Data associated to each object.
 *
 */
#[derive(Clone)]
struct CollisionObjectData {
    pub name: &'static str,
    pub velocity: Option<Cell<Vector2<f32>>>,
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector2<f32>>) -> CollisionObjectData {
        let init_velocity;
        if let Some(velocity) = velocity {
            init_velocity = Some(Cell::new(velocity))
        } else {
            init_velocity = None
        }

        CollisionObjectData {
            name: name,
            velocity: init_velocity,
        }
    }
}

/*
 *
 * Proximity handler.
 *
 */
fn handle_proximity_event(
    world: &CollisionWorld<f32, CollisionObjectData>,
    event: &ProximityEvent,
) {
    // The collision object with a None velocity is the coloured area.
    let area_name;
    let co1 = world.collision_object(event.collider1).unwrap();
    let co2 = world.collision_object(event.collider2).unwrap();

    if co1.data().velocity.is_none() {
        area_name = co1.data().name;
    } else {
        area_name = co2.data().name;
    }

    if event.new_status == Proximity::Intersecting {
        println!("The ball enters the {} area.", area_name);
    } else if event.new_status == Proximity::Disjoint {
        println!("The ball leaves the {} area.", area_name);
    }
}

/*
 *
 * Contact handler.
 *
 */
fn handle_contact_event(world: &CollisionWorld<f32, CollisionObjectData>, event: &ContactEvent) {
    if let &ContactEvent::Started(collider1, collider2) = event {
        // NOTE: real-life applications would avoid this systematic allocation.
        let pair = world.contact_pair(collider1, collider2).unwrap();
        let mut collector = Vec::new();
        pair.contacts(&mut collector);

        let co1 = world.collision_object(collider1).unwrap();
        let co2 = world.collision_object(collider2).unwrap();

        // The ball is the one with a non-None velocity.
        if let Some(ref vel) = co1.data().velocity {
            let normal = collector[0].deepest_contact().unwrap().contact.normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal);
        }
        if let Some(ref vel) = co2.data().velocity {
            let normal = -collector[0].deepest_contact().unwrap().contact.normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal);
        }
    }
}

fn main() {
    /*
     * Setup initial object properties.
     */
    // Plane shapes.
    let plane_left = ShapeHandle::new(Plane::new(Vector2::x_axis()));
    let plane_bottom = ShapeHandle::new(Plane::new(Vector2::y_axis()));
    let plane_right = ShapeHandle::new(Plane::new(-Vector2::x_axis()));
    let plane_top = ShapeHandle::new(Plane::new(-Vector2::y_axis()));

    // Shared cuboid for the rectangular areas.
    let rect = ShapeHandle::new(Cuboid::new(Vector2::new(4.8f32, 4.8)));

    // Ball shape.
    let ball = ShapeHandle::new(Ball::new(0.5f32));

    // Positions of the planes.
    let planes_pos = [
        Isometry2::new(Vector2::new(-10.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(0.0, -10.0), na::zero()),
        Isometry2::new(Vector2::new(10.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(0.0, 10.0), na::zero()),
    ];

    // Position of the rectangles.
    let rects_pos = [
        Isometry2::new(Vector2::new(-5.0, 5.0), na::zero()),
        Isometry2::new(Vector2::new(5.0, -5.0), na::zero()),
        Isometry2::new(Vector2::new(5.0, 5.0), na::zero()),
        Isometry2::new(Vector2::new(-5.0, -5.0), na::zero()),
    ];

    // Position of the ball.
    let ball_pos = Isometry2::new(Vector2::new(5.0, 5.0), na::zero());

    // The ball is part of group 1 and can interact with everything.
    let ball_groups = CollisionGroups::new().with_membership(&[1]);

    // All the other objects are part of the group 2 and interact only with the ball (but not with
    // each other).
    let others_groups = CollisionGroups::new()
        .with_membership(&[2])
        .with_whitelist(&[1]);

    let plane_data = CollisionObjectData::new("ground", None);
    let rect_data_purple = CollisionObjectData::new("purple", None);
    let rect_data_blue = CollisionObjectData::new("blue", None);
    let rect_data_green = CollisionObjectData::new("green", None);
    let rect_data_yellow = CollisionObjectData::new("yellow", None);
    let ball_data = CollisionObjectData::new("ball", Some(Vector2::new(10.0, 5.0)));

    /*
     * Setup the world.
     */
    // Collision world 0.02 optimization margin and small object identifiers.
    let mut world = CollisionWorld::new(0.02);

    // Add the planes to the world.
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);
    let proximity_query = GeometricQueryType::Proximity(0.0);

    let handle1 = world.add(
        planes_pos[0],
        plane_left,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle2 = world.add(
        planes_pos[1],
        plane_bottom,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle3 = world.add(
        planes_pos[2],
        plane_right,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle4 = world.add(
        planes_pos[3],
        plane_top,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );

    // Add the colored rectangles to the world.
    let handle5 = world.add(
        rects_pos[0],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_purple,
    );
    let handle6 = world.add(
        rects_pos[1],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_blue,
    );
    let handle7 = world.add(
        rects_pos[2],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_green,
    );
    let handle8 = world.add(
        rects_pos[3],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_yellow,
    );

    // Add the ball to the world.
    let ball_handle = world.add(ball_pos, ball, ball_groups, contacts_query, ball_data);

    /*
     * Run indefinitely.
     */
    let timestep = 0.016;

    let mut testbed = Testbed::new();
    testbed.set_color(handle4, Point3::new(0.7, 0.0, 0.7));
    testbed.set_color(handle5, Point3::new(0.0, 0.0, 0.9));
    testbed.set_color(handle6, Point3::new(0.0, 0.8, 0.0));
    testbed.set_color(handle7, Point3::new(0.8, 0.8, 0.0));
    testbed.set_color(handle8, Point3::new(0.0, 0.0, 0.0));
    testbed.set_background_color(Point3::new(1.0, 1.0, 1.0));

    // let mut i = 0;
    // testbed.start_recording("bouncing_ball.mpg");

    while testbed.step(&mut world) {
        let ball_pos;

        for event in world.proximity_events() {
            handle_proximity_event(&world, event)
        }

        for event in world.contact_events() {
            handle_contact_event(&world, event)
        }

        {
            // Integrate the velocities.
            let ball_object = world.collision_object(ball_handle).unwrap();
            let ball_velocity = ball_object.data().velocity.as_ref().unwrap();

            // Integrate the positions.
            let displacement = Translation2::from_vector(timestep * ball_velocity.get());
            ball_pos = displacement * ball_object.position();
        }

        // Submit the position update to the world.
        world.set_position(ball_handle, ball_pos);

        // i += 1;
        // if i == 480 {
        //     testbed.stop_recording()
        // }
    }
}
