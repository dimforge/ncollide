#[test]
use std::cmp::ApproxEq;
#[test]
use nalgebra::dim3::vec3::vec3;
#[test]
use nalgebra::traits::norm::Norm;
#[test]
use geom::ball::ball;
#[test]
use geom::implicit::Implicit;
#[test]
use geom::minkowski_sum::minkowski_sum;
#[test]
use geom::reflection::reflection;
#[test]
use geom::convex_polytope::convex_polytope;

#[test]
fn test_ball_support_function()
{
  let dir  = &vec3(1f64, 1f64, 1f64);
  let ball = ball(42f64);
  let diag = 42f64 / dir.norm();

  assert!(ball.support_point(dir).approx_eq(&vec3(diag, diag, diag)));
}

#[test]
fn test_convex_polytope_support_function()
{
  let dir    = &vec3(1f64, 1f64, 1f64);
  let bestpt = vec3(2f64, 2f64, 0f64);
  let pts    = ~[bestpt, vec3(-2f64, -2f64, -0f64)];
  let poly   = convex_polytope(pts);

  assert!(poly.support_point(dir).approx_eq(&bestpt));
}

#[test]
fn test_minkowski_sum_support_function()
{
  let dir  = vec3(1f64, 1f64, 1f64);
  let ball = @ball(42f64);
  let diag = 2.0f64 * 42f64 / dir.norm();

  let msum = minkowski_sum(ball, ball);

  assert!(msum.support_point(&dir).approx_eq(&vec3(diag, diag, diag)));
}

#[test]
fn test_reflection_support_function()
{
  let dir    = &vec3(1f64, 1f64, 1f64);
  let pts    = ~[vec3(2f64, 2f64, 2f64), vec3(-20f64, -20f64, -20f64)];
  let poly   = @convex_polytope(pts);

  assert!(reflection(poly).support_point(dir)
                          .approx_eq(&vec3(20f64, 20f64, 20f64)));
}
