#[cfg(feature = "dim3")]
use alga::linear::FiniteDimInnerSpace;
use math::{Isometry, Vector};
use na::{self, Point2, Real, Unit};
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use query::algorithms::gjk::GJKResult;
use query::algorithms::VoronoiSimplex;
#[cfg(feature = "dim3")]
use query::closest_points_internal;
use query::contacts_internal;
#[cfg(feature = "dim3")]
use query::ray_internal;
use query::{Contact, ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry};
#[cfg(feature = "dim3")]
use shape::ClippingCache;
use shape::ConvexPolygonalFeature;
use shape::{ConvexPolyhedron, FeatureId, Segment, SegmentPointLocation, Shape};
#[cfg(feature = "dim3")]
use utils;
use utils::{IdAllocator, IsometryOps};

#[cfg(feature = "dim2")]
#[derive(Clone)]
pub struct ConvexPolyhedronConvexPolyhedronManifoldGenerator<N: Real> {
    simplex: VoronoiSimplex<N>,
    last_gjk_dir: Option<Unit<Vector<N>>>,
    last_optimal_dir: Option<Unit<Vector<N>>>,
    new_contacts: Vec<(Contact<N>, FeatureId, FeatureId)>,
    manifold1: ConvexPolygonalFeature<N>,
    manifold2: ConvexPolygonalFeature<N>,
}

#[cfg(feature = "dim3")]
#[derive(Clone)]
pub struct ConvexPolyhedronConvexPolyhedronManifoldGenerator<N: Real> {
    simplex: VoronoiSimplex<N>,
    last_gjk_dir: Option<Unit<Vector<N>>>,
    last_optimal_dir: Option<Unit<Vector<N>>>,
    clip_cache: ClippingCache<N>,
    new_contacts: Vec<(Contact<N>, FeatureId, FeatureId)>,
    manifold1: ConvexPolygonalFeature<N>,
    manifold2: ConvexPolygonalFeature<N>,
}

impl<N: Real> ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
    #[cfg(feature = "dim3")]
    pub fn new() -> Self {
        ConvexPolyhedronConvexPolyhedronManifoldGenerator {
            simplex: VoronoiSimplex::new(),
            last_gjk_dir: None,
            last_optimal_dir: None,
            clip_cache: ClippingCache::new(),
            new_contacts: Vec::new(),
            manifold1: ConvexPolygonalFeature::new(),
            manifold2: ConvexPolygonalFeature::new(),
        }
    }

    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        ConvexPolyhedronConvexPolyhedronManifoldGenerator {
            simplex: VoronoiSimplex::new(),
            last_gjk_dir: None,
            last_optimal_dir: None,
            new_contacts: Vec::new(),
            manifold1: ConvexPolygonalFeature::new(),
            manifold2: ConvexPolygonalFeature::new(),
        }
    }

    fn clip_polyfaces(&mut self, prediction: &ContactPrediction<N>, normal: &Unit<Vector<N>>) {
        #[cfg(feature = "dim2")]
        {
            self.manifold1
                .clip(&self.manifold2, normal, prediction, &mut self.new_contacts)
        }
        #[cfg(feature = "dim3")]
        {
            self.manifold1.clip(
                &self.manifold2,
                normal,
                prediction,
                &mut self.clip_cache,
                &mut self.new_contacts,
            )
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        fmap1: Option<&Fn(FeatureId) -> FeatureId>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        fmap2: Option<&Fn(FeatureId) -> FeatureId>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if let (Some(cpa), Some(cpb)) = (a.as_convex_polyhedron(), b.as_convex_polyhedron()) {
            let contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                cpa,
                mb,
                cpb,
                prediction.linear(),
                &mut self.simplex,
                self.last_gjk_dir,
            );

            // Generate a contact manifold.
            self.new_contacts.clear();
            self.manifold1.clear();
            self.manifold2.clear();

            match contact {
                GJKResult::ClosestPoints(world1, world2, dir) => {
                    self.last_gjk_dir = Some(dir);
                    let contact = Contact::new_wo_depth(world1, world2, dir);

                    if contact.depth > na::zero() {
                        cpa.support_face_toward(ma, &contact.normal, &mut self.manifold1);
                        cpb.support_face_toward(mb, &-contact.normal, &mut self.manifold2);
                        self.clip_polyfaces(prediction, &contact.normal);
                    } else {
                        cpa.support_feature_toward(
                            ma,
                            &contact.normal,
                            prediction.angular1(),
                            &mut self.manifold1,
                        );
                        cpb.support_feature_toward(
                            mb,
                            &-contact.normal,
                            prediction.angular2(),
                            &mut self.manifold2,
                        );

                        self.clip_polyfaces(prediction, &contact.normal);
                    }

                    if self.new_contacts.len() == 0 {
                        self.new_contacts.push((
                            contact.clone(),
                            self.manifold1.feature_id,
                            self.manifold2.feature_id,
                        ));
                    }
                }
                GJKResult::NoIntersection(dir) => self.last_gjk_dir = Some(dir),
                _ => {}
            }

            for (c, f1, f2) in self.new_contacts.drain(..) {
                self.manifold1.add_contact_to_manifold(
                    &self.manifold2,
                    c,
                    ma,
                    f1,
                    fmap1,
                    mb,
                    f2,
                    fmap2,
                    id_alloc,
                    manifold,
                )
            }

            true
        } else {
            false
        }
    }
}
