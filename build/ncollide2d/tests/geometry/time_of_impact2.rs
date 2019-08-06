use na::{self, Isometry2, Vector2};
use ncollide2d::query;
use ncollide2d::shape::{Ball, Cuboid};

#[test]
fn ball_cuboid_toi() {
    let cuboid = Cuboid::new(Vector2::new(1.0, 1.0));
    let ball = Ball::new(1.0);

    let cuboid_pos = Isometry2::identity();
    let ball_pos_intersecting = Isometry2::new(Vector2::new(1.0, 1.0), na::zero());
    let ball_pos_will_touch = Isometry2::new(Vector2::new(2.0, 2.0), na::zero());
    let ball_pos_wont_touch = Isometry2::new(Vector2::new(3.0, 3.0), na::zero());

    let box_vel1 = Vector2::new(-1.0, 1.0);
    let box_vel2 = Vector2::new(1.0, 1.0);

    let ball_vel1 = Vector2::new(2.0, 2.0);
    let ball_vel2 = Vector2::new(-0.5, -0.5);

    let toi_intersecting = query::time_of_impact(
        &ball_pos_intersecting,
        &ball_vel1,
        &ball,
        &cuboid_pos,
        &box_vel1,
        &cuboid,
        0.0,
    );
    let toi_will_touch = query::time_of_impact(
        &ball_pos_will_touch,
        &ball_vel2,
        &ball,
        &cuboid_pos,
        &box_vel2,
        &cuboid,
        0.0,
    );
    let toi_wont_touch = query::time_of_impact(
        &ball_pos_wont_touch,
        &ball_vel1,
        &ball,
        &cuboid_pos,
        &box_vel1,
        &cuboid,
        0.0,
    );

    assert_eq!(toi_intersecting.map(|toi| toi.toi), Some(0.0));
    assert!(relative_eq!(
        toi_will_touch.unwrap().toi,
        (2.0f64.sqrt() - 1.0) / (ball_vel2 - box_vel2).norm()
    ));
    assert_eq!(toi_wont_touch.map(|toi| toi.toi), None);
}

#[test]
fn cuboid_cuboid_toi_issue_214() {
    let shape1 = Cuboid::new(Vector2::new(1.0, 1.0));
    let shape2 = Cuboid::new(Vector2::new(1.0, 1.5));

    let pos1 = Isometry2::new(Vector2::new(0.0, 0.0), na::zero());
    let pos2 = Isometry2::new(Vector2::new(10.0, 0.0), na::zero());

    let vel1 = Vector2::new(1.0, 0.0);
    let vel2 = Vector2::new(0.0, 0.0);

    let toi = query::time_of_impact(&pos1, &vel1, &shape1, &pos2, &vel2, &shape2, 0.0);
    assert!(toi.is_some());
}
