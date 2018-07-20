extern crate nalgebra as na;
extern crate ncollide3d;
extern crate ncollide_testbed3d;

use na::{Isometry3, Point3, Translation3, Vector3};
use ncollide3d::events::{ContactEvent, ProximityEvent};
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use ncollide3d::world::{CollisionGroups, CollisionWorld, GeometricQueryType};
use ncollide_testbed3d::Testbed;
use std::cell::Cell;

/*
 *
 * Data associated to each object.
 *
 */
#[derive(Clone)]
struct CollisionObjectData {
    pub name: &'static str,
    pub velocity: Option<Cell<Vector3<f32>>>,
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector3<f32>>) -> CollisionObjectData {
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
        let pair = world.contact_pair(collider1, collider2);
        let mut collector = Vec::new();
        pair.contacts(&mut collector);

        let co1 = world.collision_object(collider1).unwrap();
        let co2 = world.collision_object(collider2).unwrap();

        // The ball is the one with a non-None velocity.
        if let Some(ref vel) = co1.data().velocity {
            let normal = collector[0].normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * normal);
        }
        if let Some(ref vel) = co2.data().velocity {
            let normal = -collector[0].normal;
            vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * normal);
        }
    }
}

fn main() {
    /*
     * Setup initial object properties.
     */
    // Plane shapes.
    let plane_left = ShapeHandle::new(Plane::new(Vector3::x_axis()));
    let plane_bottom = ShapeHandle::new(Plane::new(Vector3::y_axis()));
    let plane_back = ShapeHandle::new(Plane::new(Vector3::z_axis()));
    let plane_right = ShapeHandle::new(Plane::new(-Vector3::x_axis()));
    let plane_top = ShapeHandle::new(Plane::new(-Vector3::y_axis()));
    let plane_front = ShapeHandle::new(Plane::new(-Vector3::z_axis()));

    // Shared cuboid for the rectangular areas.
    let rect = ShapeHandle::new(Cuboid::new(Vector3::new(4.9f32, 4.9, 4.9)));

    // Ball shape.
    let ball = ShapeHandle::new(Ball::new(0.5f32));

    // Positions of the planes.
    let planes_pos = [
        Isometry3::new(Vector3::new(-10.0, 0.0, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, -10.0, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, -10.0), na::zero()),
        Isometry3::new(Vector3::new(10.0, 0.0, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 10.0, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, 10.0), na::zero()),
    ];

    // Position of the rectangles.
    let rects_pos = [
        Isometry3::new(Vector3::new(-5.0, 5.0, 5.0), na::zero()),
        Isometry3::new(Vector3::new(5.0, 5.0, 5.0), na::zero()),
        Isometry3::new(Vector3::new(5.0, -5.0, 5.0), na::zero()),
        Isometry3::new(Vector3::new(-5.0, -5.0, 5.0), na::zero()),
        Isometry3::new(Vector3::new(-5.0, 5.0, -5.0), na::zero()),
        Isometry3::new(Vector3::new(5.0, 5.0, -5.0), na::zero()),
        Isometry3::new(Vector3::new(5.0, -5.0, -5.0), na::zero()),
        Isometry3::new(Vector3::new(-5.0, -5.0, -5.0), na::zero()),
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

    let plane_data = CollisionObjectData::new("ground", None);
    let rect_data_purple = CollisionObjectData::new("purple", None);
    let rect_data_blue = CollisionObjectData::new("blue", None);
    let rect_data_green = CollisionObjectData::new("green", None);
    let rect_data_yellow = CollisionObjectData::new("yellow", None);
    let rect_data_red = CollisionObjectData::new("red", None);
    let rect_data_grey = CollisionObjectData::new("grey", None);
    let rect_data_pink = CollisionObjectData::new("pink", None);
    let rect_data_cyan = CollisionObjectData::new("cyan", None);
    let ball_data = CollisionObjectData::new("ball", Some(Vector3::new(10.0, 5.0, 5.0)));

    /*
     * Setup the world.
     */
    // Collision world 0.02 optimization margin and small object identifiers.
    let mut world = CollisionWorld::new(0.02);

    // Add the planes to the world.
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);
    let proximity_query = GeometricQueryType::Proximity(0.0);

    let handle0 = world.add(
        planes_pos[0],
        plane_left,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle1 = world.add(
        planes_pos[1],
        plane_bottom,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle2 = world.add(
        planes_pos[2],
        plane_back,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle3 = world.add(
        planes_pos[3],
        plane_right,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle4 = world.add(
        planes_pos[4],
        plane_top,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );
    let handle5 = world.add(
        planes_pos[5],
        plane_front,
        others_groups,
        contacts_query,
        plane_data.clone(),
    );

    // Add the colored rectangles to the world.
    let handle6 = world.add(
        rects_pos[0],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_purple,
    );
    let handle7 = world.add(
        rects_pos[1],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_blue,
    );
    let handle8 = world.add(
        rects_pos[2],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_green,
    );
    let handle9 = world.add(
        rects_pos[3],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_yellow,
    );
    let handle10 = world.add(
        rects_pos[4],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_red,
    );
    let handle11 = world.add(
        rects_pos[5],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_grey,
    );
    let handle12 = world.add(
        rects_pos[6],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_pink,
    );
    let handle13 = world.add(
        rects_pos[7],
        rect.clone(),
        others_groups,
        proximity_query,
        rect_data_cyan,
    );

    // Add the ball to the world.
    let ball_handle = world.add(ball_pos, ball, ball_groups, contacts_query, ball_data);

    /*
     * Run indefinitely.
     */
    let timestep = 0.016;

    let mut testbed = Testbed::new();
    testbed.set_visible(handle0, false);
    testbed.set_visible(handle1, false);
    testbed.set_visible(handle2, false);
    testbed.set_visible(handle3, false);
    testbed.set_visible(handle4, false);
    testbed.set_visible(handle5, false);
    testbed.set_color(handle6, Point3::new(0.5, 0.0, 0.5));
    testbed.set_color(handle7, Point3::new(0.0, 0.0, 1.0));
    testbed.set_color(handle8, Point3::new(0.0, 1.0, 0.0));
    testbed.set_color(handle9, Point3::new(1.0, 1.0, 0.0));
    testbed.set_color(handle10, Point3::new(1.0, 0.0, 0.0));
    testbed.set_color(handle11, Point3::new(0.5, 0.5, 0.5));
    testbed.set_color(handle12, Point3::new(1.0, 0.75, 0.8));
    testbed.set_color(handle13, Point3::new(0.0, 1.0, 1.0));
    testbed.set_color(ball_handle, Point3::new(1.0, 1.0, 1.0));

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
            let displacement = Translation3::from_vector(timestep * ball_velocity.get());
            ball_pos = displacement * ball_object.position();
        }

        // Submit the position update to the world.
        world.set_position(ball_handle, ball_pos);

        // i += 1;
        // if i == 96 * 5 {
        //     testbed.stop_recording()
        // }
    }
}
