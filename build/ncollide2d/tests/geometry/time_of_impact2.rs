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
    );
    let toi_will_touch = query::time_of_impact(
        &ball_pos_will_touch,
        &ball_vel2,
        &ball,
        &cuboid_pos,
        &box_vel2,
        &cuboid,
    );
    let toi_wont_touch = query::time_of_impact(
        &ball_pos_wont_touch,
        &ball_vel1,
        &ball,
        &cuboid_pos,
        &box_vel1,
        &cuboid,
    );

    assert_eq!(toi_intersecting, Some(0.0));
    assert!(relative_eq!(
        toi_will_touch.unwrap(),
        (2.0_f32.sqrt() - 1.0) / (ball_vel2 - box_vel2).norm()
    ));
    assert_eq!(toi_wont_touch, None);
}
