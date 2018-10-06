use bounding_volume::PolyhedralCone;
use math::{Isometry, Point};
use na::{Real, Unit};
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use query::{Contact, ContactKinematic, ContactManifold, ContactPrediction};
use shape::{Ball, FeatureId, Shape};
use utils::{IdAllocator, IsometryOps};

/// Collision detector between two balls.
pub struct BallConvexPolyhedronManifoldGenerator<N: Real> {
    flip: bool,
    contact_manifold: ContactManifold<N>,
}

impl<N: Real> Clone for BallConvexPolyhedronManifoldGenerator<N> {
    fn clone(&self) -> BallConvexPolyhedronManifoldGenerator<N> {
        BallConvexPolyhedronManifoldGenerator {
            flip: self.flip,
            contact_manifold: self.contact_manifold.clone(),
        }
    }
}

impl<N: Real> BallConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexPolyhedronManifoldGenerator<N> {
        BallConvexPolyhedronManifoldGenerator {
            flip,
            contact_manifold: ContactManifold::new(),
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

            let ball_center = Point::from_coordinates(m1.translation.vector);
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
                if f2 == FeatureId::Unknown() {
                    // We cant do anything more at this point.
                    return true;
                }

                depth = N::zero();
                normal = -cp2.feature_normal(f2);
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
                    #[cfg(feature = "dim3")]
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

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N>
for BallConvexPolyhedronManifoldGenerator<N>
{
    fn update(
        &mut self,
        _: &ContactDispatcher<N>,
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

    fn clear(&mut self, id_alloc: &mut IdAllocator) {
        self.contact_manifold.clear(id_alloc)
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.contact_manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        if self.contact_manifold.len() != 0 {
            out.push(&self.contact_manifold)
        }
    }
}
