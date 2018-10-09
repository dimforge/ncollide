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
use shape::ConvexPolygonalFeature;
use shape::{ConvexPolyhedron, FeatureId, Segment, SegmentPointLocation, Shape};
#[cfg(feature = "dim3")]
use utils;
use utils::{IdAllocator, IsometryOps};

#[derive(Clone)]
struct ClippingCache<N: Real> {
    poly1: Vec<Point2<N>>,
    poly2: Vec<Point2<N>>,
}

impl<N: Real> ClippingCache<N> {
    pub fn new() -> Self {
        ClippingCache {
            poly1: Vec::with_capacity(4),
            poly2: Vec::with_capacity(4),
        }
    }

    pub fn clear(&mut self) {
        self.poly1.clear();
        self.poly2.clear();
    }
}

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

    fn save_new_contacts_as_contact_manifold<G1: ?Sized, G2: ?Sized>(
        cache: &mut Cache<N>,
        m1: &Isometry<N>,
        g1: &G1,
        m2: &Isometry<N>,
        g2: &G2,
        ids: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) where
        G1: ConvexPolyhedron<N>,
        G2: ConvexPolyhedron<N>,
    {
        for (c, f1, f2) in cache.new_contacts.drain(..) {
            let mut kinematic = ContactKinematic::new();
            let local1 = m1.inverse_transform_point(&c.world1);
            let local2 = m2.inverse_transform_point(&c.world2);
            let n1 = g1.normal_cone(f1);
            let n2 = g2.normal_cone(f2);

            match f1 {
                FeatureId::Face(..) => kinematic.set_approx1(
                    f1,
                    local1,
                    NeighborhoodGeometry::Plane(n1.unwrap_half_line()),
                ),
                #[cfg(feature = "dim3")]
                FeatureId::Edge(..) => {
                    let e1 = cache.manifold1.edge(f1).expect("Invalid edge id.");
                    if let Some(dir1) = e1.direction() {
                        let local_dir1 = m1.inverse_transform_unit_vector(&dir1);
                        let approx1 = NeighborhoodGeometry::Line(local_dir1);
                        kinematic.set_approx1(f1, local1, approx1)
                    } else {
                        continue;
                    }
                }
                FeatureId::Vertex(..) => {
                    kinematic.set_approx1(f1, local1, NeighborhoodGeometry::Point)
                }
                FeatureId::Unknown => unreachable!(),
            }

            match f2 {
                FeatureId::Face(..) => {
                    let approx2 = NeighborhoodGeometry::Plane(n2.unwrap_half_line());
                    kinematic.set_approx2(f2, local2, approx2)
                }
                #[cfg(feature = "dim3")]
                FeatureId::Edge(..) => {
                    let e2 = cache.manifold2.edge(f2).expect("Invalid edge id.");
                    if let Some(dir2) = e2.direction() {
                        let local_dir2 = m2.inverse_transform_unit_vector(&dir2);
                        let approx2 = NeighborhoodGeometry::Line(local_dir2);
                        kinematic.set_approx2(f2, local2, approx2)
                    } else {
                        continue;
                    }
                }
                FeatureId::Vertex(..) => {
                    kinematic.set_approx2(f2, local2, NeighborhoodGeometry::Point)
                }
                FeatureId::Unknown => unreachable!(),
            }

            let _ = manifold.push(c, local1, kinematic, ids);
        }
    }

    fn clip_polyfaces(
        cache: &mut Cache<N>,
        prediction: &ContactPrediction<N>,
        normal: Unit<Vector<N>>,
    ) {
        cache.clip_cache.clear();

        #[cfg(feature = "dim2")]
        {
            if cache.manifold1.nvertices <= 1 || cache.manifold2.nvertices <= 1 {
                return;
            }
            // In 2D we always end up with two points.
            let mut ortho: Vector<N> = na::zero();
            ortho[0] = -normal.as_ref()[1];
            ortho[1] = normal.as_ref()[0];

            let mut seg1 = Segment::new(cache.manifold1.vertices[0], cache.manifold1.vertices[1]);
            let mut seg2 = Segment::new(cache.manifold2.vertices[0], cache.manifold2.vertices[1]);

            let ref_pt = *seg1.a();
            let mut range1 = [
                na::dot(&(*seg1.a() - ref_pt), &ortho),
                na::dot(&(*seg1.b() - ref_pt), &ortho),
            ];
            let mut range2 = [
                na::dot(&(*seg2.a() - ref_pt), &ortho),
                na::dot(&(*seg2.b() - ref_pt), &ortho),
            ];
            let mut features1 = [
                cache.manifold1.vertices_id[0],
                cache.manifold1.vertices_id[1],
            ];
            let mut features2 = [
                cache.manifold2.vertices_id[0],
                cache.manifold2.vertices_id[1],
            ];

            if range1[1] < range1[0] {
                range1.swap(0, 1);
                features1.swap(0, 1);
                seg1.swap();
            }

            if range2[1] < range2[0] {
                range2.swap(0, 1);
                features2.swap(0, 1);
                seg2.swap();
            }

            if range2[0] > range1[1] || range1[0] > range2[1] {
                return;
            }

            let _1: N = na::one();
            let length1 = range1[1] - range1[0];
            let length2 = range2[1] - range2[0];

            if range2[0] > range1[0] {
                let bcoord = (range2[0] - range1[0]) / length1;
                let p1 = seg1.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let p2 = *seg2.a();
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear() {
                    cache
                        .new_contacts
                        .push((contact, cache.manifold1.feature_id, features2[0]));
                }
            } else {
                let bcoord = (range1[0] - range2[0]) / length2;
                let p1 = *seg1.a();
                let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear() {
                    cache
                        .new_contacts
                        .push((contact, features1[0], cache.manifold2.feature_id));
                }
            }

            if range2[1] < range1[1] {
                let bcoord = (range2[1] - range1[0]) / length1;
                let p1 = seg1.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let p2 = *seg2.b();
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear() {
                    cache
                        .new_contacts
                        .push((contact, cache.manifold1.feature_id, features2[1]));
                }
            } else {
                let bcoord = (range1[1] - range2[0]) / length2;
                let p1 = *seg1.b();
                let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear() {
                    cache
                        .new_contacts
                        .push((contact, features1[1], cache.manifold2.feature_id));
                }
            }
        }
        #[cfg(feature = "dim3")]
        {
            // FIXME: don't compute contacts further than the prediction.

            if cache.manifold1.vertices.len() <= 2 && cache.manifold2.vertices.len() <= 2 {
                return;
            }

            // In 3D we may end up with more than two points.
            let mut basis = [na::zero(), na::zero()];
            let mut basis_i = 0;

            Vector::orthonormal_subspace_basis(&[normal.unwrap()], |dir| {
                basis[basis_i] = *dir;
                basis_i += 1;
                true
            });

            let ref_pt = cache.manifold1.vertices[0];

            for pt in &cache.manifold1.vertices {
                let dpt = *pt - ref_pt;
                let coords = Point2::new(na::dot(&basis[0], &dpt), na::dot(&basis[1], &dpt));
                cache.clip_cache.poly1.push(coords);
            }

            for pt in &cache.manifold2.vertices {
                let dpt = *pt - ref_pt;
                let coords = Point2::new(na::dot(&basis[0], &dpt), na::dot(&basis[1], &dpt));
                cache.clip_cache.poly2.push(coords);
            }

            if cache.clip_cache.poly2.len() > 2 {
                for i in 0..cache.clip_cache.poly1.len() {
                    let pt = &cache.clip_cache.poly1[i];

                    if utils::point_in_poly2d(pt, &cache.clip_cache.poly2) {
                        let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                        let n2 = cache.manifold2.normal.as_ref().unwrap().unwrap();
                        let p2 = &cache.manifold2.vertices[0];
                        if let Some(toi2) =
                            ray_internal::plane_toi_with_line(p2, &n2, &origin, &normal.unwrap())
                        {
                            let world2 = origin + normal.unwrap() * toi2;
                            let world1 = cache.manifold1.vertices[i];
                            let f2 = cache.manifold2.feature_id;
                            let f1 = cache.manifold1.vertices_id[i];
                            let contact = Contact::new_wo_depth(world1, world2, normal);

                            if -contact.depth <= prediction.linear() {
                                cache.new_contacts.push((contact, f1, f2));
                            }
                        }
                    }
                }
            }

            if cache.clip_cache.poly1.len() > 2 {
                for i in 0..cache.clip_cache.poly2.len() {
                    let pt = &cache.clip_cache.poly2[i];

                    if utils::point_in_poly2d(pt, &cache.clip_cache.poly1) {
                        let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                        let n1 = cache.manifold1.normal.as_ref().unwrap().unwrap();
                        let p1 = &cache.manifold1.vertices[0];
                        if let Some(toi1) =
                            ray_internal::plane_toi_with_line(p1, &n1, &origin, &normal.unwrap())
                        {
                            let world1 = origin + normal.unwrap() * toi1;
                            let world2 = cache.manifold2.vertices[i];
                            let f1 = cache.manifold1.feature_id;
                            let f2 = cache.manifold2.vertices_id[i];
                            let contact = Contact::new_wo_depth(world1, world2, normal);

                            if -contact.depth <= prediction.linear() {
                                cache.new_contacts.push((contact, f1, f2));
                            }
                        }
                    }
                }
            }

            let nedges1 = cache.manifold1.nedges();
            let nedges2 = cache.manifold2.nedges();

            for i1 in 0..nedges1 {
                let j1 = (i1 + 1) % cache.clip_cache.poly1.len();
                let seg1 = (&cache.clip_cache.poly1[i1], &cache.clip_cache.poly1[j1]);

                for i2 in 0..nedges2 {
                    let j2 = (i2 + 1) % cache.clip_cache.poly2.len();
                    let seg2 = (&cache.clip_cache.poly2[i2], &cache.clip_cache.poly2[j2]);

                    if let (SegmentPointLocation::OnEdge(e1), SegmentPointLocation::OnEdge(e2)) =
                        closest_points_internal::segment_against_segment_with_locations_nD(
                            seg1, seg2,
                        ) {
                        let original1 = Segment::new(
                            cache.manifold1.vertices[i1],
                            cache.manifold1.vertices[j1],
                        );
                        let original2 = Segment::new(
                            cache.manifold2.vertices[i2],
                            cache.manifold2.vertices[j2],
                        );
                        let world1 = original1.point_at(&SegmentPointLocation::OnEdge(e1));
                        let world2 = original2.point_at(&SegmentPointLocation::OnEdge(e2));
                        let f1 = cache.manifold1.edges_id[i1];
                        let f2 = cache.manifold2.edges_id[i2];
                        let contact = Contact::new_wo_depth(world1, world2, normal);

                        if -contact.depth <= prediction.linear() {
                            cache.new_contacts.push((contact, f1, f2));
                        }
                    }
                }
            }
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
                        Self::clip_polyfaces(cache, prediction, contact.normal);
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

                        Self::clip_polyfaces(cache, prediction, contact.normal);
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

            Self::save_new_contacts_as_contact_manifold(cache, ma, cpa, mb, cpb, ids, manifold);

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
