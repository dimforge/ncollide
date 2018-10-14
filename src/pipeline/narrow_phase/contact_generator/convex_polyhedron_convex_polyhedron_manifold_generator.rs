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
struct Cache<N: Real> {
    simplex: VoronoiSimplex<N>,
    last_gjk_dir: Option<Unit<Vector<N>>>,
    last_optimal_dir: Option<Unit<Vector<N>>>,
    new_contacts: Vec<(Contact<N>, FeatureId, FeatureId)>,
    manifold1: ConvexPolygonalFeature<N>,
    manifold2: ConvexPolygonalFeature<N>,
}

#[cfg(feature = "dim3")]
#[derive(Clone)]
struct Cache<N: Real> {
    simplex: VoronoiSimplex<N>,
    last_gjk_dir: Option<Unit<Vector<N>>>,
    last_optimal_dir: Option<Unit<Vector<N>>>,
    clip_cache: ClippingCache<N>,
    new_contacts: Vec<(Contact<N>, FeatureId, FeatureId)>,
    manifold1: ConvexPolygonalFeature<N>,
    manifold2: ConvexPolygonalFeature<N>,
}

impl<N: Real> Cache<N> {
    #[cfg(feature = "dim3")]
    pub fn new() -> Self {
        Cache {
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
        Cache {
            simplex: VoronoiSimplex::new(),
            last_gjk_dir: None,
            last_optimal_dir: None,
            new_contacts: Vec::new(),
            manifold1: ConvexPolygonalFeature::new(),
            manifold2: ConvexPolygonalFeature::new(),
        }
    }
}

/// Persistent contact manifold computation between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct ConvexPolyhedronConvexPolyhedronManifoldGenerator<N: Real> {
    cache: Cache<N>,
    contact_manifold: ContactManifold<N>,
}

impl<N: Real> ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistant collision detector between two convex polyhedra.
    pub fn new() -> ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
        ConvexPolyhedronConvexPolyhedronManifoldGenerator {
            contact_manifold: ContactManifold::new(),
            cache: Cache::new(),
        }
    }

    fn clip_polyfaces(
        cache: &mut Cache<N>,
        prediction: &ContactPrediction<N>,
        normal: &Unit<Vector<N>>,
    ) {
        #[cfg(feature = "dim2")]
        {
            cache.manifold1.clip(
                &cache.manifold2,
                normal,
                prediction,
                &mut cache.new_contacts,
            )
        }
        #[cfg(feature = "dim3")]
        {
            cache.manifold1.clip(
                &cache.manifold2,
                normal,
                prediction,
                &mut cache.clip_cache,
                &mut cache.new_contacts,
            )
        }
    }

    #[inline]
    fn do_update_to(
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        cache: &mut Cache<N>,
        ids: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if let (Some(cpa), Some(cpb)) = (a.as_convex_polyhedron(), b.as_convex_polyhedron()) {
            let contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                cpa,
                mb,
                cpb,
                prediction.linear(),
                &mut cache.simplex,
                cache.last_gjk_dir,
            );

            // Generate a contact manifold.
            cache.new_contacts.clear();
            cache.manifold1.clear();
            cache.manifold2.clear();

            match contact {
                GJKResult::ClosestPoints(world1, world2, dir) => {
                    cache.last_gjk_dir = Some(dir);
                    let contact = Contact::new_wo_depth(world1, world2, dir);

                    if contact.depth > na::zero() {
                        cpa.support_face_toward(ma, &contact.normal, &mut cache.manifold1);
                        cpb.support_face_toward(mb, &-contact.normal, &mut cache.manifold2);
                        Self::clip_polyfaces(cache, prediction, &contact.normal);
                    } else {
                        cpa.support_feature_toward(
                            ma,
                            &contact.normal,
                            prediction.angular1(),
                            &mut cache.manifold1,
                        );
                        cpb.support_feature_toward(
                            mb,
                            &-contact.normal,
                            prediction.angular2(),
                            &mut cache.manifold2,
                        );

                        Self::clip_polyfaces(cache, prediction, &contact.normal);
                    }

                    if cache.new_contacts.len() == 0 {
                        cache.new_contacts.push((
                            contact.clone(),
                            cache.manifold1.feature_id,
                            cache.manifold2.feature_id,
                        ));
                    }
                }
                GJKResult::NoIntersection(dir) => cache.last_gjk_dir = Some(dir),
                _ => {}
            }

            for (c, f1, f2) in cache.new_contacts.drain(..) {
                cache.manifold1.add_contact_to_manifold(
                    &cache.manifold2,
                    c,
                    ma,
                    f1,
                    mb,
                    f2,
                    ids,
                    manifold,
                )
            }

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        self.contact_manifold.save_cache_and_clear(id_alloc);

        Self::do_update_to(
            ida,
            ma,
            a,
            idb,
            mb,
            b,
            prediction,
            &mut self.cache,
            id_alloc,
            &mut self.contact_manifold,
        )
    }

    fn update_to(
        &mut self,
        _: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        Self::do_update_to(
            ida,
            ma,
            a,
            idb,
            mb,
            b,
            prediction,
            &mut self.cache,
            id_alloc,
            manifold,
        )
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
