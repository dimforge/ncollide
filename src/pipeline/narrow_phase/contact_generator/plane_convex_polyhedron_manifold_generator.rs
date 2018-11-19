use bounding_volume::ConicalApproximation;
use math::{Isometry, Point};
use na::{self, Real};
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator, ContactGeneratorShapeContext};
use query::{Contact, ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry};
use shape::{ConvexPolygonalFeature, FeatureId, Plane, Shape};
use utils::{IdAllocator, IsometryOps};

/// Collision detector between g1 plane and g1 shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneConvexPolyhedronManifoldGenerator<N: Real> {
    flip: bool,
    feature: ConvexPolygonalFeature<N>,
    manifold: ContactManifold<N>,
}

impl<N: Real> PlaneConvexPolyhedronManifoldGenerator<N> {
    /// Creates g1 new persistent collision detector between g1 plane and g1 shape with g1 support
    /// mapping function.
    #[inline]
    pub fn new(flip: bool) -> PlaneConvexPolyhedronManifoldGenerator<N> {
        PlaneConvexPolyhedronManifoldGenerator {
            flip,
            feature: ConvexPolygonalFeature::new(),
            manifold: ContactManifold::new(),
        }
    }

    #[inline]
    fn do_update_to(
        m1: &Isometry<N>,
        g1: &Shape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        prediction: &ContactPrediction<N>,
        poly_feature: &mut ConvexPolygonalFeature<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
        flip: bool,
    ) -> bool
    {
        if let (Some(plane), Some(cp)) = (g1.as_shape::<Plane<N>>(), g2.as_convex_polyhedron()) {
            let plane_normal = m1 * plane.normal();
            let plane_center = Point::from_coordinates(m1.translation.vector);

            cp.support_face_toward(m2, &-plane_normal, poly_feature);

            for (i, world2) in poly_feature.vertices.iter().enumerate() {
                let dpt = *world2 - plane_center;
                let dist = na::dot(&dpt, plane_normal.as_ref());

                if dist <= prediction.linear() {
                    let world1 = *world2 + (-*plane_normal * dist);
                    let local1 = m1.inverse_transform_point(&world1);
                    let local2 = m2.inverse_transform_point(&world2);
                    let f1 = FeatureId::Face(0);
                    let f2 = poly_feature.vertices_id[i];
                    let mut kinematic = ContactKinematic::new();
                    let contact;

                    let approx_plane = NeighborhoodGeometry::Plane(*plane.normal());
                    let approx2 = NeighborhoodGeometry::Point;

                    if !flip {
                        contact = Contact::new(world1, *world2, plane_normal, -dist);
                        kinematic.set_approx1(f1, local1, approx_plane);
                        kinematic.set_approx2(f2, local2, approx2);
                    } else {
                        contact = Contact::new(*world2, world1, -plane_normal, -dist);
                        kinematic.set_approx1(f2, local2, approx2);
                        kinematic.set_approx2(f1, local1, approx_plane);
                    }
                    let _ = manifold.push(contact, local2, kinematic, id_alloc);
                }
            }

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for PlaneConvexPolyhedronManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &Shape<N>,
        ctxt1: Option<&ContactGeneratorShapeContext<N>>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        ctxt2: Option<&ContactGeneratorShapeContext<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        /*
        if !self.flip {
            Self::do_update_to(
                m1,
                g1,
                m2,
                g2,
                prediction,
                &mut self.feature,
                id_alloc,
                &mut self.manifold,
                false,
            )
        } else {
            Self::do_update_to(
                m2,
                g2,
                m1,
                g1,
                prediction,
                &mut self.feature,
                id_alloc,
                &mut self.manifold,
                true,
            )
        }
        */
        unimplemented!()
    }
}
