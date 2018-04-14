use std::marker::PhantomData;
use approx::ApproxEq;

use na::Unit;
use alga::linear::Translation;
use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::bounding_volume::PolyhedralCone;
use geometry::shape::{Ball, FeatureId, Shape};
use geometry::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction};
use narrow_phase::{ContactDispatcher, ContactManifoldGenerator};

/// Collision detector between two balls.
pub struct BallConvexPolyhedronManifoldGenerator<N: Real, M> {
    flip: bool,
    contact_manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<N: Real, M> Clone for BallConvexPolyhedronManifoldGenerator<P, M> {
    fn clone(&self) -> BallConvexPolyhedronManifoldGenerator<P, M> {
        BallConvexPolyhedronManifoldGenerator {
            flip: self.flip,
            contact_manifold: self.contact_manifold.clone(),
            mat_type: PhantomData,
        }
    }
}

impl<N: Real> BallConvexPolyhedronManifoldGenerator<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexPolyhedronManifoldGenerator<P, M> {
        BallConvexPolyhedronManifoldGenerator {
            flip,
            contact_manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }

    fn do_update(
        &mut self,
        m1: &Isometry<N>,
        a: &Shape<N>,
        m2: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        flip: bool,
    ) -> bool {
        if let (Some(ball), Some(pq2), Some(cp2)) = (
            a.as_shape::<Ball<N>>(),
            b.as_point_query(),
            b.as_convex_polyhedron(),
        ) {
            self.contact_manifold.save_cache_and_clear(id_alloc);

            let ball_center = Point::from_coordinates(m1.translation().to_vector());
            let (proj, f2) = pq2.project_point_with_feature(m2, &ball_center);
            let world2 = proj.point;
            let dpt = world2 - ball_center;

            if let Some((dir, dist)) = Unit::try_new_and_get(dpt, N::default_epsilon()) {
                let depth;
                let normal;

                if proj.is_inside {
                    depth = dist + ball.radius();
                    normal = -dir;
                } else {
                    depth = -dist + ball.radius();
                    normal = dir;
                }

                if depth >= -prediction.linear {
                    let mut kinematic = ContactKinematic::new();
                    let f1 = FeatureId::Face(0);
                    let world1 = ball_center + normal.unwrap() * ball.radius();

                    let contact;

                    if !flip {
                        contact = Contact::new(world1, world2, normal, depth);
                        kinematic.set_point1(f1, Point::origin(), PolyhedralCone::Full);
                        kinematic.set_dilation1(ball.radius());
                    } else {
                        contact = Contact::new(world2, world1, -normal, depth);
                        kinematic.set_point2(f1, Point::origin(), PolyhedralCone::Full);
                        kinematic.set_dilation2(ball.radius());
                    }

                    let local2 = m2.inverse_transform_point(&world2);
                    let n2 = cp2.normal_cone(f2);

                    match f2 {
                        FeatureId::Face { .. } => {
                            if !flip {
                                kinematic.set_plane2(f2, local2, n2.unwrap_half_line())
                            } else {
                                kinematic.set_plane1(f2, local2, n2.unwrap_half_line())
                            }
                        }
                        FeatureId::Edge { .. } => {
                            let edge = cp2.edge(f2);
                            let dir = Unit::new_normalize(edge.1 - edge.0);

                            if !flip {
                                kinematic.set_line2(f2, local2, dir, n2)
                            } else {
                                kinematic.set_line1(f2, local2, dir, n2)
                            }
                        }
                        FeatureId::Vertex { .. } => {
                            if !flip {
                                kinematic.set_point2(f2, local2, n2)
                            } else {
                                kinematic.set_point1(f2, local2, n2)
                            }
                        }
                        FeatureId::Unknown => panic!("Feature id cannot be unknown."),
                    }

                    let _ = self.contact_manifold.push(contact, kinematic, id_alloc);
                }
            } else {
                // FIXME: unhandled case where the ball center is exactly on the polyhedra surface.
            }

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<P, M>
    for BallConvexPolyhedronManifoldGenerator<P, M>
{
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        id1: usize,
        m1: &Isometry<N>,
        a: &Shape<N>,
        id2: usize,
        m2: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        self.contact_manifold.set_subshape_id1(id1);
        self.contact_manifold.set_subshape_id2(id2);

        if !self.flip {
            self.do_update(m1, a, m2, b, prediction, id_alloc, false)
        } else {
            self.do_update(m2, b, m1, a, prediction, id_alloc, true)
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.contact_manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        if self.contact_manifold.len() != 0 {
            out.push(&self.contact_manifold)
        }
    }
}
