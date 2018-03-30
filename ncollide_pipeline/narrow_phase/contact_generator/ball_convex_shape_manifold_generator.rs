use std::marker::PhantomData;
use approx::ApproxEq;

use na::Unit;
use alga::linear::Translation;
use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::bounding_volume::PolyhedralCone;
use geometry::shape::{Ball, FeatureId, Shape};
use geometry::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction};
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Collision detector between two balls.
pub struct BallConvexShapeManifoldGenerator<P: Point, M> {
    flip: bool,
    contact_manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallConvexShapeManifoldGenerator<P, M> {
    fn clone(&self) -> BallConvexShapeManifoldGenerator<P, M> {
        BallConvexShapeManifoldGenerator {
            flip: self.flip,
            contact_manifold: self.contact_manifold.clone(),
            mat_type: PhantomData,
        }
    }
}

impl<P: Point, M: Isometry<P>> BallConvexShapeManifoldGenerator<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexShapeManifoldGenerator<P, M> {
        BallConvexShapeManifoldGenerator {
            flip,
            contact_manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }

    fn do_update(
        &mut self,
        m1: &M,
        a: &Shape<P, M>,
        m2: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
        flip: bool,
    ) -> bool {
        if let (Some(ball), Some(pq2), Some(cp2)) = (
            a.as_shape::<Ball<P::Real>>(),
            b.as_point_query(),
            b.as_convex_polyhedron(),
        ) {
            self.contact_manifold.save_cache_and_clear(id_alloc);

            let ball_center = P::from_coordinates(m1.translation().to_vector());
            let (proj, f2) = pq2.project_point_with_feature(m2, &ball_center);
            let world2 = proj.point;
            let dpt = world2 - ball_center;

            if let Some((dir, dist)) = Unit::try_new_and_get(dpt, P::Real::default_epsilon()) {
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
                    let f1 = FeatureId::face(0, 0);
                    let world1 = ball_center + normal.unwrap() * ball.radius();

                    let local1 = m1.inverse_transform_point(&world1);
                    let local2 = m2.inverse_transform_point(&world2);
                    let n2 = cp2.normal_cone(f2);
                    let contact;

                    if !flip {
                        contact = Contact::new(world1, world2, normal, depth);
                        kinematic.set_point1(f1, P::origin(), PolyhedralCone::new());
                        kinematic.set_dilation1(ball.radius());
                    } else {
                        contact = Contact::new(world2, world1, -normal, depth);
                        kinematic.set_point2(f2, P::origin(), PolyhedralCone::new());
                        kinematic.set_dilation2(ball.radius());
                    }

                    // match f2 {
                    //     FeatureId::Face { id, .. } => {
                    //         kinematic.set_plane2(f2, local2, n2.generators()[0])
                    //     }
                    //     FeatureId::Edge { id, .. } => kinematic.set_line2(f2, local2, dir, n2),
                    //     FeatureId::Vertex { id, .. } => kinematic.set_point2(f2, local2, n2),
                    // }

                    let _ = self.contact_manifold.push(contact, kinematic, id_alloc);
                }
            } else {

            }

            true
        } else {
            false
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M> for BallConvexShapeManifoldGenerator<P, M> {
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        m1: &M,
        a: &Shape<P, M>,
        m2: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
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
