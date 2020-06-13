use crate::math::{Isometry, Point};
use crate::query::Contact;
use crate::shape::{Ball, FeatureId, Shape};
use na::{self, RealField, Unit};

/// Contact between a ball and a convex polyhedron.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn contact_ball_convex_polyhedron<N: RealField>(
    ball_center1: &Point<N>,
    ball1: &Ball<N>,
    m2: &Isometry<N>,
    shape2: &(impl Shape<N> + ?Sized),
    prediction: N,
) -> Option<Contact<N>> {
    // NOTE: this code is mostly taken from the narrow-phase's BallConvexPolyhedronManifoldGenerator
    // after removal of all the code related to contact kinematics because it is not needed here
    // TODE: is there a way to refactor this to avoid duplication?.
    let poly2 = shape2
        .as_convex_polyhedron()
        .expect("The input shape does not implement the ConvexPolyhedron trait.");
    let pt_query2 = shape2
        .as_point_query()
        .expect("The input shape does not implement the PointQuery trait.");

    let (proj, f2) = pt_query2.project_point_with_feature(m2, &ball_center1);
    let world2 = proj.point;
    let dpt = world2 - ball_center1;

    let depth;
    let normal;
    if let Some((dir, dist)) = Unit::try_new_and_get(dpt, N::default_epsilon()) {
        if proj.is_inside {
            depth = dist + ball1.radius();
            normal = -dir;
        } else {
            depth = -dist + ball1.radius();
            normal = dir;
        }
    } else {
        if f2 == FeatureId::Unknown {
            // We cant do anything more at this point.
            return None;
        }

        depth = N::zero();
        normal = -poly2.feature_normal(f2);
    }

    if depth >= -prediction {
        let world1 = ball_center1 + normal.into_inner() * ball1.radius();
        return Some(Contact::new(world1, world2, normal, depth));
    }

    None
}

/// Contact between a convex polyhedron and a ball.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn contact_convex_polyhedron_ball<N: RealField>(
    m1: &Isometry<N>,
    poly1: &(impl Shape<N> + ?Sized),
    ball_center2: &Point<N>,
    ball2: &Ball<N>,
    prediction: N,
) -> Option<Contact<N>> {
    let mut res = contact_ball_convex_polyhedron(ball_center2, ball2, m1, poly1, prediction);
    if let Some(c) = &mut res {
        c.flip()
    }
    res
}
