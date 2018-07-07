use na::{self, Point2, Real, Unit};

#[cfg(feature = "dim3")]
use alga::linear::FiniteDimInnerSpace;
use math::{Isometry, Vector};
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use query::algorithms::gjk::GJKResult;
use query::algorithms::VoronoiSimplex;
#[cfg(feature = "dim3")]
use query::closest_points_internal;
use query::contacts_internal;
#[cfg(feature = "dim3")]
use query::ray_internal;
use query::{Contact, ContactKinematic, ContactManifold, ContactPrediction};
use shape::ConvexPolygonalFeature;
use shape::{ConvexPolyhedron, FeatureId, Segment, SegmentPointLocation, Shape};
#[cfg(feature = "dim3")]
use utils;
use utils::{IdAllocator, IsometryOps};
use query::algorithms::CSOPoint;

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

/// Persistent contact manifold computation between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct ConvexPolyhedronConvexPolyhedronManifoldGenerator<N: Real> {
    simplex: VoronoiSimplex<N>,
    last_optimal_dir: Option<Unit<Vector<N>>>,
    contact_manifold: ContactManifold<N>,
    clip_cache: ClippingCache<N>,
    new_contacts: Vec<(Contact<N>, FeatureId, FeatureId)>,
    manifold1: ConvexPolygonalFeature<N>,
    manifold2: ConvexPolygonalFeature<N>,
    sep_axis: Option<Unit<Vector<N>>>
}

impl<N: Real> ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistant collision detector between two convex polyhedra.
    pub fn new() -> ConvexPolyhedronConvexPolyhedronManifoldGenerator<N> {
        ConvexPolyhedronConvexPolyhedronManifoldGenerator {
            simplex: VoronoiSimplex::new(),
            last_optimal_dir: None,
            contact_manifold: ContactManifold::new(),
            clip_cache: ClippingCache::new(),
            new_contacts: Vec::new(),
            manifold1: ConvexPolygonalFeature::new(),
            manifold2: ConvexPolygonalFeature::new(),
            sep_axis: None
        }
    }

    fn save_new_contacts_as_contact_manifold<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &Isometry<N>,
        g1: &G1,
        m2: &Isometry<N>,
        g2: &G2,
        ids: &mut IdAllocator,
    ) where
        G1: ConvexPolyhedron<N>,
        G2: ConvexPolyhedron<N>,
    {
        self.contact_manifold.save_cache_and_clear(ids);

        for (c, f1, f2) in self.new_contacts.drain(..) {
            let mut kinematic = ContactKinematic::new();
            let local1 = m1.inverse_transform_point(&c.world1);
            let local2 = m2.inverse_transform_point(&c.world2);
            let n1 = g1.normal_cone(f1);
            let n2 = g2.normal_cone(f2);

            match f1 {
                FeatureId::Face(..) => kinematic.set_plane1(f1, local1, n1.unwrap_half_line()),
                #[cfg(feature = "dim3")]
                FeatureId::Edge(..) => {
                    let e1 = self.manifold1.edge(f1).expect("Invalid edge id.");
                    if let Some(dir1) = e1.direction() {
                        let local_dir1 = m1.inverse_transform_unit_vector(&dir1);
                        kinematic.set_line1(f1, local1, local_dir1, n1)
                    } else {
                        continue;
                    }
                }
                FeatureId::Vertex(..) => kinematic.set_point1(f1, local1, n1),
                FeatureId::Unknown => unreachable!(),
            }

            match f2 {
                FeatureId::Face(..) => kinematic.set_plane2(f2, local2, n2.unwrap_half_line()),
                #[cfg(feature = "dim3")]
                FeatureId::Edge(..) => {
                    let e2 = self.manifold2.edge(f2).expect("Invalid edge id.");
                    if let Some(dir2) = e2.direction() {
                        let local_dir2 = m2.inverse_transform_unit_vector(&dir2);
                        kinematic.set_line2(f2, local2, local_dir2, n2)
                    } else {
                        continue;
                    }
                }
                FeatureId::Vertex(..) => kinematic.set_point2(f2, local2, n2),
                FeatureId::Unknown => unreachable!(),
            }

            let _ = self.contact_manifold.push(c, kinematic, ids);
        }
    }

    fn clip_polyfaces(&mut self, prediction: &ContactPrediction<N>, normal: Unit<Vector<N>>) {
        self.clip_cache.clear();

        #[cfg(feature = "dim2")]
        {
            if self.manifold1.nvertices <= 1 || self.manifold2.nvertices <= 1 {
                return;
            }
            // In 2D we always end up with two points.
            let mut ortho: Vector<N> = na::zero();
            ortho[0] = -normal.as_ref()[1];
            ortho[1] = normal.as_ref()[0];

            let mut seg1 = Segment::new(self.manifold1.vertices[0], self.manifold1.vertices[1]);
            let mut seg2 = Segment::new(self.manifold2.vertices[0], self.manifold2.vertices[1]);

            let ref_pt = *seg1.a();
            let mut range1 = [
                na::dot(&(*seg1.a() - ref_pt), &ortho),
                na::dot(&(*seg1.b() - ref_pt), &ortho),
            ];
            let mut range2 = [
                na::dot(&(*seg2.a() - ref_pt), &ortho),
                na::dot(&(*seg2.b() - ref_pt), &ortho),
            ];
            let mut features1 = [self.manifold1.vertices_id[0], self.manifold1.vertices_id[1]];
            let mut features2 = [self.manifold2.vertices_id[0], self.manifold2.vertices_id[1]];

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

                if -contact.depth <= prediction.linear {
                    self.new_contacts
                        .push((contact, self.manifold1.feature_id, features2[0]));
                }
            } else {
                let bcoord = (range1[0] - range2[0]) / length2;
                let p1 = *seg1.a();
                let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear {
                    self.new_contacts
                        .push((contact, features1[0], self.manifold2.feature_id));
                }
            }

            if range2[1] < range1[1] {
                let bcoord = (range2[1] - range1[0]) / length1;
                let p1 = seg1.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let p2 = *seg2.b();
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear {
                    self.new_contacts
                        .push((contact, self.manifold1.feature_id, features2[1]));
                }
            } else {
                let bcoord = (range1[1] - range2[0]) / length2;
                let p1 = *seg1.b();
                let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
                let contact = Contact::new_wo_depth(p1, p2, normal);

                if -contact.depth <= prediction.linear {
                    self.new_contacts
                        .push((contact, features1[1], self.manifold2.feature_id));
                }
            }
        }
        #[cfg(feature = "dim3")]
        {
            // FIXME: don't compute contacts further than the prediction.

            if self.manifold1.vertices.len() <= 2 && self.manifold2.vertices.len() <= 2 {
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

            let ref_pt = self.manifold1.vertices[0];

            for pt in &self.manifold1.vertices {
                let dpt = *pt - ref_pt;
                let coords = Point2::new(na::dot(&basis[0], &dpt), na::dot(&basis[1], &dpt));
                self.clip_cache.poly1.push(coords);
            }

            for pt in &self.manifold2.vertices {
                let dpt = *pt - ref_pt;
                let coords = Point2::new(na::dot(&basis[0], &dpt), na::dot(&basis[1], &dpt));
                self.clip_cache.poly2.push(coords);
            }

            if self.clip_cache.poly2.len() > 2 {
                for i in 0..self.clip_cache.poly1.len() {
                    let pt = &self.clip_cache.poly1[i];

                    if utils::point_in_poly2d(pt, &self.clip_cache.poly2) {
                        let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                        let n2 = self.manifold2.normal.as_ref().unwrap().unwrap();
                        let p2 = &self.manifold2.vertices[0];
                        if let Some(toi2) =
                            ray_internal::plane_toi_with_line(p2, &n2, &origin, &normal.unwrap())
                        {
                            let world2 = origin + normal.unwrap() * toi2;
                            let world1 = self.manifold1.vertices[i];
                            let f2 = self.manifold2.feature_id;
                            let f1 = self.manifold1.vertices_id[i];
                            let contact = Contact::new_wo_depth(world1, world2, normal);

                            if -contact.depth <= prediction.linear {
                                self.new_contacts.push((contact, f1, f2));
                            }
                        }
                    }
                }
            }

            if self.clip_cache.poly1.len() > 2 {
                for i in 0..self.clip_cache.poly2.len() {
                    let pt = &self.clip_cache.poly2[i];

                    if utils::point_in_poly2d(pt, &self.clip_cache.poly1) {
                        let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                        let n1 = self.manifold1.normal.as_ref().unwrap().unwrap();
                        let p1 = &self.manifold1.vertices[0];
                        if let Some(toi1) =
                            ray_internal::plane_toi_with_line(p1, &n1, &origin, &normal.unwrap())
                        {
                            let world1 = origin + normal.unwrap() * toi1;
                            let world2 = self.manifold2.vertices[i];
                            let f1 = self.manifold1.feature_id;
                            let f2 = self.manifold2.vertices_id[i];
                            let contact = Contact::new_wo_depth(world1, world2, normal);

                            if -contact.depth <= prediction.linear {
                                self.new_contacts.push((contact, f1, f2));
                            }
                        }
                    }
                }
            }

            let nedges1 = self.manifold1.nedges();
            let nedges2 = self.manifold2.nedges();

            for i1 in 0..nedges1 {
                let j1 = (i1 + 1) % self.clip_cache.poly1.len();
                let seg1 = (&self.clip_cache.poly1[i1], &self.clip_cache.poly1[j1]);

                for i2 in 0..nedges2 {
                    let j2 = (i2 + 1) % self.clip_cache.poly2.len();
                    let seg2 = (&self.clip_cache.poly2[i2], &self.clip_cache.poly2[j2]);

                    if let (SegmentPointLocation::OnEdge(e1), SegmentPointLocation::OnEdge(e2)) =
                        closest_points_internal::segment_against_segment_with_locations_nD(
                            seg1, seg2,
                        ) {
                        let original1 =
                            Segment::new(self.manifold1.vertices[i1], self.manifold1.vertices[j1]);
                        let original2 =
                            Segment::new(self.manifold2.vertices[i2], self.manifold2.vertices[j2]);
                        let world1 = original1.point_at(&SegmentPointLocation::OnEdge(e1));
                        let world2 = original2.point_at(&SegmentPointLocation::OnEdge(e2));
                        let f1 = self.manifold1.edges_id[i1];
                        let f2 = self.manifold2.edges_id[i2];
                        let contact = Contact::new_wo_depth(world1, world2, normal);

                        if -contact.depth <= prediction.linear {
                            self.new_contacts.push((contact, f1, f2));
                        }
                    }
                }
            }
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
        ids: &mut IdAllocator,
    ) -> bool {
        if let (Some(cpa), Some(cpb)) = (a.as_convex_polyhedron(), b.as_convex_polyhedron()) {
            if let Some(sep_axis) = self.sep_axis {
                let point = CSOPoint::from_shapes_toward(ma, cpa, mb, cpa, &sep_axis);
                if -point.point.coords.dot(&*sep_axis) > prediction.linear {
                    self.contact_manifold.save_cache_and_clear(ids);
                    return true;
                }

                self.sep_axis = None;
            }
            self.contact_manifold.set_subshape_id1(ida);
            self.contact_manifold.set_subshape_id2(idb);
            self.simplex.transform_points(ma, mb);

            let contact = contacts_internal::support_map_against_support_map_with_simplex(
                ma,
                cpa,
                mb,
                cpb,
                prediction.linear,
                &mut self.simplex,
            );

            self.simplex.transform_points(&ma.inverse(), &mb.inverse());

            // Generate a contact manifold.
            self.new_contacts.clear();
            self.manifold1.clear();
            self.manifold2.clear();

            match contact {
                GJKResult::ClosestPoints(world1, world2, dir) => {
                    let contact = Contact::new_wo_depth(world1, world2, dir);

                    if contact.depth > na::zero() {
                        cpa.support_face_toward(ma, &contact.normal, &mut self.manifold1);
                        cpb.support_face_toward(mb, &-contact.normal, &mut self.manifold2);
                        self.clip_polyfaces(prediction, contact.normal);
                    } else {
                        cpa.support_feature_toward(
                            ma,
                            &contact.normal,
                            prediction.angular1,
                            &mut self.manifold1,
                        );
                        cpb.support_feature_toward(
                            mb,
                            &-contact.normal,
                            prediction.angular2,
                            &mut self.manifold2,
                        );

                        self.clip_polyfaces(prediction, contact.normal);
                    }

                    if self.new_contacts.len() == 0 {
                        self.new_contacts.push((
                            contact.clone(),
                            self.manifold1.feature_id,
                            self.manifold2.feature_id,
                        ));
                    }
                }
                GJKResult::NoIntersection(sep_axis) => self.sep_axis = Some(sep_axis),
                _ => {}
            }

            self.save_new_contacts_as_contact_manifold(ma, cpa, mb, cpb, ids);

            true
        } else {
            false
        }
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
