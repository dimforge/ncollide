use crate::math::{Isometry, Point};
use na::{Real, Unit};
use crate::pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use crate::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry, ContactPreprocessor};
use crate::shape::{Ball, FeatureId, Shape};
use std::marker::PhantomData;
use crate::utils::{IdAllocator, IsometryOps};

/// Collision detector between two balls.
#[derive(Clone)]
pub struct BallConvexPolyhedronManifoldGenerator<N: Real> {
    phantom: PhantomData<N>,
    flip: bool,
}

impl<N: Real> BallConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexPolyhedronManifoldGenerator<N> {
        BallConvexPolyhedronManifoldGenerator {
            phantom: PhantomData,
            flip,
        }
    }

    fn do_generate(
        &mut self,
        m1: &Isometry<N>,
        a: &Shape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        // NOTE: we use an underscore to silence a warning
        // for _cp2 because it is used in 3D but not in 2D.
        if let (Some(ball), Some(pq2), Some(cp2)) = (
            a.as_shape::<Ball<N>>(),
            b.as_point_query(),
            b.as_convex_polyhedron(),
        ) {
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
                    return true;
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
                    kinematic.set_approx2(f2, local2, geom2)
                } else {
                    kinematic.set_approx1(f2, local2, geom2)
                }

                let _ = manifold.push(contact, kinematic, Point::origin(), proc1, proc2, id_alloc);
            }

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for BallConvexPolyhedronManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &ContactDispatcher<N>,
        m1: &Isometry<N>,
        a: &Shape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if !self.flip {
            self.do_generate(m1, a, proc1, m2, b, proc2, prediction, id_alloc, manifold)
        } else {
            self.do_generate(m2, b, proc2, m1, a, proc1, prediction, id_alloc, manifold)
        }
    }
}
