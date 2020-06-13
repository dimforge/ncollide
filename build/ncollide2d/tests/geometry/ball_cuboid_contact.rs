use nalgebra::{Isometry2, Point2, Vector2};
use ncollide2d::query;
use ncollide2d::shape::{Ball, Cuboid};
#[cfg(feature = "improved_fixed_point_support")]
use simba::scalar::FixedI40F24;

fn f64_from_f64(val: f64) -> f64 {
    val
}

fn f32_from_f64(val: f64) -> f32 {
    val as f32
}

#[cfg(feature = "improved_fixed_point_support")]
fn fi40f24_from_f64(val: f64) -> FixedI40F24 {
    na::convert(val)
}

fn test_collide<T>(type_from_f64: fn(f64) -> T)
where
    T: simba::scalar::RealField,
{
    let cuboid = Cuboid::new(Vector2::new(type_from_f64(0.5), type_from_f64(0.5)));
    let cuboid_point = Point2::new(type_from_f64(0.0), type_from_f64(4.0));
    let cuboid_pos = Isometry2::new(cuboid_point.coords, nalgebra::zero());
    let ball = Ball::new(type_from_f64(0.5));
    let ball_point = Point2::new(type_from_f64(0.0517938), type_from_f64(3.05178815));
    let ball_pos = Isometry2::new(ball_point.coords, nalgebra::zero());
    let ct = query::contact(&cuboid_pos, &cuboid, &ball_pos, &ball, nalgebra::zero());
    assert!(ct.is_some());
    let ct = query::contact(&ball_pos, &ball, &cuboid_pos, &cuboid, nalgebra::zero());
    assert!(ct.is_some());
}

#[test]
fn test_ball_cuboid_query_contact_fp64() {
    test_collide::<f64>(f64_from_f64);
}

#[test]
fn test_ball_cuboid_query_contact_fp32() {
    test_collide::<f32>(f32_from_f64);
}

#[test]
#[cfg(feature = "improved_fixed_point_support")]
fn test_ball_cuboid_query_contact_fixedi40f24() {
    test_collide::<FixedI40F24>(fi40f24_from_f64);
}
