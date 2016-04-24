extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry2, Vector2};
use ncollide::shape::{Cuboid, Ball};
use ncollide::geometry::{self, Proximity};

fn main() {
    let cuboid = Cuboid::new(Vector2::new(1.0, 1.0));
    let ball   = Ball::new(1.0);
    let margin = 1.0;

    let cuboid_pos             = na::one();
    let ball_pos_intersecting  = Isometry2::new(Vector2::new(1.0, 1.0), na::zero());
    let ball_pos_within_margin = Isometry2::new(Vector2::new(2.0, 2.0), na::zero());
    let ball_pos_disjoint      = Isometry2::new(Vector2::new(3.0, 3.0), na::zero());

    let prox_intersecting = geometry::proximity(&ball_pos_intersecting, &ball,
                                                &cuboid_pos,            &cuboid,
                                                margin);
    let prox_within_margin = geometry::proximity(&ball_pos_within_margin, &ball,
                                                 &cuboid_pos,             &cuboid,
                                                 margin);
    let prox_disjoint = geometry::proximity(&ball_pos_disjoint, &ball,
                                            &cuboid_pos,        &cuboid,
                                            margin);

    assert_eq!(prox_intersecting, Proximity::Intersecting);
    assert_eq!(prox_within_margin, Proximity::WithinMargin);
    assert_eq!(prox_disjoint, Proximity::Disjoint);
}
