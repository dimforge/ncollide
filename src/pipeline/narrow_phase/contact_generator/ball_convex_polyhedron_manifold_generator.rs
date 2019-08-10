use crate::math::{Isometry, Point};
use na::{RealField, Unit};
use crate::pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use crate::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry, ContactPreprocessor, PointQuery};
use crate::shape::{Ball, FeatureId, Shape, ConvexPolyhedron};
use std::marker::PhantomData;
use crate::utils::IsometryOps;

/// Collision detector between two balls.
#[derive(Clone)]
pub struct BallConvexPolyhedronManifoldGenerator<N: RealField> {
    phantom: PhantomData<N>,
    flip: bool,
}

impl<N: RealField> BallConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexPolyhedronManifoldGenerator<N> {
        BallConvexPolyhedronManifoldGenerator {
            phantom: PhantomData,
            flip,
        }
    }


    pub(crate) fn do_generate_with_exact_shapes(
        &self,
        m1: &Isometry<N>,
        ball: &Ball<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        pq2: &dyn PointQuery<N>,
        cp2: &dyn ConvexPolyhedron<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    )
    {
        let ball_center = Point::from(m1.translation.vector);
        let (proj, f2) = pq2.project_point_with_feature(m2, &ball_center);
        let world2 = proj.point;
        let dpt = world2 - ball_center;

        let depth;
        let normal;
        if let Some((dir, dist)) = Unit::try_new_and_get(dpt, N::default_epsilon()) {
            if proj.is_inside {
                depth = dist + ball.radius();
                normal = -dir;
            } else {
                depth = -dist + ball.radius();
                normal = dir;
            }
        } else {
            if f2 == FeatureId::Unknown {
                // We cant do anything more at this point.
                return;
            }

            depth = N::zero();
            normal = -cp2.feature_normal(f2);
        }

        if depth >= -prediction.linear() {
            let mut kinematic = ContactKinematic::new();
            let f1 = FeatureId::Face(0);
            let world1 = ball_center + normal.into_inner() * ball.radius();

            let contact;

            if !self.flip {
                contact = Contact::new(world1, world2, normal, depth);
                kinematic.set_approx1(
                    f1,
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_dilation1(ball.radius());
            } else {
                contact = Contact::new(world2, world1, -normal, depth);
                kinematic.set_approx2(
                    f1,
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_dilation2(ball.radius());
            }

            let local2 = m2.inverse_transform_point(&world2);
            let geom2;

            match f2 {
                FeatureId::Face { .. } => {
                    let n = m2.inverse_transform_unit_vector(&-normal);
                    geom2 = NeighborhoodGeometry::Plane(n);
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge { .. } => {
                    let edge = cp2.edge(f2);
                    let dir = Unit::new_normalize(edge.1 - edge.0);
                    geom2 = NeighborhoodGeometry::Line(dir);
                }
                FeatureId::Vertex { .. } => {
                    geom2 = NeighborhoodGeometry::Point;
                }
                FeatureId::Unknown => panic!("Feature id cannot be unknown."),
            }

            if !self.flip {
                kinematic.set_approx2(f2, local2, geom2);
                let _ = manifold.push(contact, kinematic, Point::origin(), proc1, proc2);
            } else {
                kinematic.set_approx1(f2, local2, geom2);
                let _ = manifold.push(contact, kinematic, Point::origin(), proc2, proc1);
            }
        }
    }

    fn do_generate(
        &self,
        m1: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if let (Some(ball), Some(pq2), Some(cp2)) = (
            a.as_shape::<Ball<N>>(),
            b.as_point_query(),
            b.as_convex_polyhedron(),
        ) {
            self.do_generate_with_exact_shapes(m1, ball, proc1, m2, pq2, cp2, proc2, prediction, manifold);
            true
        } else {
            false
        }
    }
}

impl<N: RealField> ContactManifoldGenerator<N> for BallConvexPolyhedronManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &dyn ContactDispatcher<N>,
        m1: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if !self.flip {
            self.do_generate(m1, a, proc1, m2, b, proc2, prediction, manifold)
        } else {
            self.do_generate(m2, b, proc2, m1, a, proc1, prediction, manifold)
        }
    }
}
