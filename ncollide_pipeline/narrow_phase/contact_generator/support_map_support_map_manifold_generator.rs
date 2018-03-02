use std::marker::PhantomData;
use std::cell::RefCell;
use approx::ApproxEq;
use na::{self, Id, Real, Unit};
use math::{Isometry, Point};
use utils::{self, IdAllocator};
use geometry::shape::{AnnotatedPoint, FeatureId, Segment, SegmentPointLocation, Shape, SupportMap};
use geometry::query::algorithms::Simplex;
use geometry::query::algorithms::gjk::GJKResult;
use geometry::query::contacts_internal;
use geometry::query::closest_points_internal;
use geometry::query::{Contact, ContactManifold, ContactPrediction, PointQueryWithLocation};
use geometry::shape::ConvexPolyface;
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Persistent contact manifold computation between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapSupportMapManifoldGenerator<P: Point, M, S> {
    simplex: S,
    last_gjk_dir: Option<P::Vector>,
    last_optimal_dir: Option<Unit<P::Vector>>,
    contact_manifold: ContactManifold<P>,
    manifold1: ConvexPolyface<P>,
    manifold2: ConvexPolyface<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P, M, S> SupportMapSupportMapManifoldGenerator<P, M, S>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
{
    /// Creates a new persistant collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapManifoldGenerator<P, M, S> {
        SupportMapSupportMapManifoldGenerator {
            simplex: simplex,
            last_gjk_dir: None,
            last_optimal_dir: None,
            contact_manifold: ContactManifold::new(),
            manifold1: ConvexPolyface::new(),
            manifold2: ConvexPolyface::new(),
            mat_type: PhantomData,
        }
    }

    pub fn contain_optimal<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        m2: &M,
        g2: &G2,
        prediction: &ContactPrediction<P::Real>,
        ids: &mut IdAllocator,
    ) -> bool
    where
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        if let Some(dir) = self.last_optimal_dir {
            g1.support_area_toward(m1, &dir, prediction.angular1, &mut self.manifold1);
            g2.support_area_toward(m2, &-dir, prediction.angular2, &mut self.manifold2);
            self.quasi_conformal_contact_area(m1, g1, m2, g2, prediction, ids);

            if let Some(dir) = self.last_optimal_dir {
                let f1 = self.manifold1.feature_id;
                let f2 = self.manifold2.feature_id;

                g1.support_area_toward(m1, &dir, prediction.angular1, &mut self.manifold1);
                g2.support_area_toward(m2, &-dir, prediction.angular2, &mut self.manifold2);

                if self.manifold1.feature_id != f1 && self.manifold2.feature_id != f2 {
                    // println!("Transitioning to new features: {:?}, {:?}", f1, f2);
                    self.contact_manifold.save_cache_and_clear(ids);
                    self.quasi_conformal_contact_area(m1, g1, m2, g2, prediction, ids);
                }

                // println!(
                //     "Current features: {:?}, {:?}",
                //     self.manifold1.feature_id, self.manifold2.feature_id
                // );

                return true;
            }
            // println!(
            //     "Tried features: {:?}, {:?}",
            //     self.manifold1.feature_id, self.manifold2.feature_id
            // );
        }

        false
    }

    pub fn register_contact<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        fid1: FeatureId,
        m2: &M,
        g2: &G2,
        fid2: FeatureId,
        prediction: &ContactPrediction<P::Real>,
        contact: Contact<P>,
        ids: &mut IdAllocator,
    ) -> bool
    where
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        let keep = -contact.depth <= prediction.linear;

        if self.last_optimal_dir.is_none()
            && is_contact_optimal(m1, g1, fid1, m2, g2, fid2, &contact)
        {
            self.last_optimal_dir = Some(contact.normal);

            if !keep {
                // println!("Early exit.");
                return true;
            }
        }

        if keep {
            self.contact_manifold.push(contact, fid1, fid2, ids)
        }

        false
    }

    // FIXME: the method name is not very representative of what actually does.
    /// Computes the vertices of the quasi-conformal contact area between two convex polygonal faces.
    /// Output nothing to `contacts` if the contact is only punctual. In particular, this does nothing
    /// if either face has only one vertex.
    fn quasi_conformal_contact_area<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        m2: &M,
        g2: &G2,
        pred: &ContactPrediction<P::Real>,
        ids: &mut IdAllocator,
    ) where
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        self.last_optimal_dir = None;

        if self.manifold1.vertices.len() == 0 || self.manifold2.vertices.len() == 0 {
            return;
        }
        if self.manifold1.vertices.len() == 1 && self.manifold2.vertices.len() == 1 {
            let fid1 = self.manifold1.feature_id;
            let fid2 = self.manifold2.feature_id;
            let ab = self.manifold2.vertices[0] - self.manifold1.vertices[0];

            let contact;
            if let Some((normal, depth)) = Unit::try_new_and_get(ab, na::zero()) {
                if g1.is_direction_in_normal_cone(
                    m1,
                    self.manifold1.feature_id,
                    &self.manifold1.vertices[0],
                    &normal,
                ) {
                    contact = Contact::new(
                        self.manifold1.vertices[0],
                        self.manifold2.vertices[0],
                        normal,
                        -depth,
                    );
                } else {
                    return;
                }
            } else {
                let mut n = na::zero::<P::Vector>();
                n[0] = na::one(); // FIXME: ugly.
                let n = Unit::new_unchecked(n);
                contact = Contact::new(
                    self.manifold1.vertices[0],
                    self.manifold2.vertices[0],
                    n,
                    na::zero(),
                );
            }
            let _ = self.register_contact(m1, g1, fid1, m2, g2, fid2, pred, contact, ids);
            return;
        }

        if na::dimension::<P::Vector>() == 2 {
            if self.manifold1.vertices.len() > 1 {
                let s1 = Segment::new(self.manifold1.vertices[0], self.manifold1.vertices[1]);
                let n1 = self.manifold1.normal.expect("2D edges must have a normal.");

                for j in 0..self.manifold2.vertices.len() {
                    let p2 = self.manifold2.vertices[j];

                    if let Some(contact) = point_on_segment_contact_2d(&s1, &p2, &n1, false) {
                        let id1 = self.manifold1.feature_id;
                        let id2 = self.manifold2.vertices_id[0];

                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, ids) {
                            return;
                        }
                    }
                }
            }
            if self.manifold2.vertices.len() > 1 {
                let s2 = Segment::new(self.manifold2.vertices[0], self.manifold2.vertices[1]);
                let n2 = self.manifold2.normal.expect("2D edges must have a normal.");

                for i in 0..self.manifold1.vertices.len() {
                    let p1 = self.manifold1.vertices[i];

                    if let Some(contact) = point_on_segment_contact_2d(&s2, &p1, &n2, true) {
                        let id1 = self.manifold2.feature_id;
                        let id2 = self.manifold1.vertices_id[0];

                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, ids) {
                            return;
                        }
                    }
                }
            }
        } else if na::dimension::<P::Vector>() == 3 {
            if self.manifold1.vertices.len() == 1 && self.manifold2.vertices.len() == 2 {
            } else if self.manifold1.vertices.len() == 2 && self.manifold2.vertices.len() == 1 {
            } else {
                let sang1 = pred.angular1.sin();
                let sang2 = pred.angular2.sin();

                // Check segments.
                for i1 in 0..self.manifold1.nedges() {
                    for i2 in 0..self.manifold2.nedges() {
                        if self.segment_segment_contact_3d(
                            m1,
                            g1,
                            i1,
                            m2,
                            g2,
                            i2,
                            sang1,
                            sang2,
                            pred,
                            ids,
                        ) {
                            return;
                        }
                    }
                }

                // Check vertices.
                if self.manifold1.vertices.len() > 2 {
                    assert!(self.manifold1.edge_normals.len() == self.manifold1.vertices().len(),
                        "Convex polyhedral face: the number of edge normals should match the number of vertices.");

                    for j in 0..self.manifold2.vertices.len() {
                        // Project on face.
                        if let Some(contact) = point_face_contact_3d(
                            &self.manifold1,
                            &self.manifold2.vertices[j],
                            false,
                        ) {
                            let id1 = self.manifold1.feature_id;
                            let id2 = self.manifold2.vertices_id[j];

                            if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, ids) {
                                return;
                            }
                        }
                    }
                }
                if self.manifold2.vertices.len() > 2 {
                    assert!(self.manifold2.edge_normals.len() == self.manifold2.vertices().len(),
                        "Convex polyhedral face: the number of edge normals should match the number of vertices.");

                    for i in 0..self.manifold1.vertices.len() {
                        // Project on face.
                        if let Some(contact) = point_face_contact_3d(
                            &self.manifold2,
                            &self.manifold1.vertices[i],
                            true,
                        ) {
                            let id1 = self.manifold1.vertices_id[i];
                            let id2 = self.manifold2.feature_id;

                            if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, ids) {
                                return;
                            }
                        }
                    }
                }
            }
        }
    }

    fn segment_segment_contact_3d<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        i1: usize,
        m2: &M,
        g2: &G2,
        i2: usize,
        sang1: P::Real,
        sang2: P::Real,
        pred: &ContactPrediction<P::Real>,
        ids: &mut IdAllocator,
    ) -> bool
    where
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        let j1 = (i1 + 1) % self.manifold1.vertices.len();
        let j2 = (i2 + 1) % self.manifold2.vertices.len();

        let seg1 = Segment::new(self.manifold1.vertices[i1], self.manifold1.vertices[j1]);
        let seg2 = Segment::new(self.manifold2.vertices[i2], self.manifold2.vertices[j2]);

        let locs = closest_points_internal::segment_against_segment_with_locations(
            &Id::new(),
            &seg1,
            &Id::new(),
            &seg2,
        );

        let p1 = seg1.point_at(&locs.0);
        let p2 = seg2.point_at(&locs.1);
        let f1 = match locs.0 {
            SegmentPointLocation::OnEdge(_) => self.manifold1.edges_id[i1],
            SegmentPointLocation::OnVertex(i) => if i == 0 {
                self.manifold1.vertices_id[i1]
            } else {
                self.manifold1.vertices_id[j1]
            },
        };

        let f2 = match locs.0 {
            SegmentPointLocation::OnEdge(_) => self.manifold2.edges_id[i2],
            SegmentPointLocation::OnVertex(i) => if i == 0 {
                self.manifold2.vertices_id[i2]
            } else {
                self.manifold2.vertices_id[j2]
            },
        };

        if let Some((normal, depth)) = Unit::try_new_and_get(p2 - p1, P::Real::default_epsilon()) {
            // First assume there is no penetration.
            let mut contact = Contact::new(p1, p2, normal, -depth);

            // Detect penetration configurations.
            match locs {
                (SegmentPointLocation::OnEdge(_), SegmentPointLocation::OnEdge(_)) => {
                    if let Some(n1) = self.manifold1.normal {
                        // First manifold is a face.
                        let edge_n1 = &self.manifold1.edge_normals[i1];
                        if na::dot(contact.normal.as_ref(), n1.as_ref()) < na::zero() {
                            contact.normal = -contact.normal;
                            contact.depth = -contact.depth;
                        }

                        if na::dot(contact.normal.as_ref(), edge_n1) < -sang1 {
                            return false;
                        }
                    } else if let Some(n2) = self.manifold2.normal {
                        // Second manifold is a face but the first is an edge.
                        let edge_n2 = &self.manifold2.edge_normals[i2];
                        if na::dot(contact.normal.as_ref(), n2.as_ref()) > na::zero() {
                            contact.normal = -contact.normal;
                            contact.depth = -contact.depth;
                        }

                        if na::dot(contact.normal.as_ref(), edge_n2) > sang2 {
                            return false;
                        }
                    } else {
                        // Both manifolds are edges.

                        // Should we use an approximate normal cone test here
                        // instead of the real one?
                        if !is_contact_optimal(m1, g1, f1, m2, g2, f2, &contact) {
                            contact.normal = -contact.normal;
                            contact.depth = -contact.depth;
                        }
                    }
                }
                _ => {
                    if !is_contact_optimal(m1, g1, f1, m2, g2, f2, &contact) {
                        return false;
                    }
                }
            }

            return self.register_contact(m1, g1, f1, m2, g2, f2, pred, contact, ids);
        }

        return false;
    }
}

thread_local! {
    pub static NAVOID: RefCell<u32> = RefCell::new(0);
}

impl<P, M, S> ContactGenerator<P, M> for SupportMapSupportMapManifoldGenerator<P, M, S>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
{
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        ids: &mut IdAllocator,
    ) -> bool {
        if let (Some(sma), Some(smb)) = (a.as_support_map(), b.as_support_map()) {
            self.contact_manifold.save_cache_and_clear(ids);

            if self.contain_optimal(ma, sma, mb, smb, prediction, ids) {
                NAVOID.with(|e| *e.borrow_mut() += 1);
                return true;
            }

            let contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                prediction.linear,
                &mut self.simplex,
                self.last_gjk_dir,
            );

            // Generate a contact manifold.
            self.manifold1.clear();
            self.manifold2.clear();
            self.contact_manifold.save_cache_and_clear(ids);

            match contact {
                GJKResult::Projection(ref contact, dir) => {
                    self.last_gjk_dir = Some(dir.unwrap());

                    sma.support_area_toward(
                        ma,
                        &contact.normal,
                        prediction.angular1,
                        &mut self.manifold1,
                    );
                    smb.support_area_toward(
                        mb,
                        &-contact.normal,
                        prediction.angular2,
                        &mut self.manifold2,
                    );

                    // println!(
                    //     "Handling features: {:?}, {:?}",
                    //     self.manifold1.feature_id, self.manifold2.feature_id
                    // );
                    self.quasi_conformal_contact_area(ma, sma, mb, smb, prediction, ids);
                    // if self.last_optimal_dir.is_none() {
                    //     println!(
                    //         "No optimum was found: {}, {:?}, {:?}, ncontacts: {}",
                    //         self.last_optimal_dir.is_some(),
                    //         self.manifold1.feature_id,
                    //         self.manifold2.feature_id,
                    //         self.contact_manifold.len()
                    //     );
                    // }
                    // println!("Contacts found: {}", self.contact_manifold.len());

                    if self.contact_manifold.len() == 0 {
                        self.contact_manifold.push(
                            contact.clone(),
                            FeatureId::Unknown,
                            FeatureId::Unknown,
                            ids,
                        );
                    }
                }
                GJKResult::NoIntersection(dir) => self.last_gjk_dir = Some(dir),
                _ => {}
            }

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
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        out.push(&self.contact_manifold)
    }
}

fn point_on_segment_contact_2d<P: Point>(
    seg: &Segment<P>,
    pt: &P,
    normal: &Unit<P::Vector>,
    flip: bool,
) -> Option<Contact<P>> {
    let (proj, location) = seg.project_point_with_location(&Id::new(), pt, false);
    let dpt = *pt - proj.point;

    match location {
        SegmentPointLocation::OnEdge(..) => {
            let depth = -na::dot(normal.as_ref(), &dpt);
            if !flip {
                return Some(Contact::new(proj.point, *pt, *normal, depth));
            } else {
                return Some(Contact::new(*pt, proj.point, -*normal, depth));
            }
        }
        SegmentPointLocation::OnVertex(..) => {
            if utils::perp2(&dpt, normal.as_ref()).abs() < P::Real::default_epsilon() {
                let depth = -na::dot(normal.as_ref(), &dpt);
                if !flip {
                    return Some(Contact::new(proj.point, *pt, *normal, depth));
                } else {
                    return Some(Contact::new(*pt, proj.point, -*normal, depth));
                }
            }
        }
    }

    None
}

fn point_face_contact_3d<P: Point>(
    face: &ConvexPolyface<P>,
    pt: &P,
    flip: bool,
) -> Option<Contact<P>> {
    for i in 0..face.vertices.len() {
        let n = face.edge_normals[i];
        let dpt = *pt - face.vertices[i];

        if na::dot(&n, &dpt) > na::zero() {
            return None;
        }
    }

    let normal = face.normal.unwrap();
    let dist = -na::dot(normal.as_ref(), &(*pt - face.vertices[0]));
    let proj = *pt + *normal * dist;

    if !flip {
        Some(Contact::new(proj, *pt, normal, dist))
    } else {
        Some(Contact::new(*pt, proj, -normal, dist))
    }
}

fn is_contact_optimal<P, M, G1: ?Sized, G2: ?Sized>(
    m1: &M,
    g1: &G1,
    f1: FeatureId,
    m2: &M,
    g2: &G2,
    f2: FeatureId,
    contact: &Contact<P>,
) -> bool
where
    P: Point,
    M: Isometry<P>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    g1.is_direction_in_normal_cone(m1, f1, &contact.world1, &contact.normal)
        && g2.is_direction_in_normal_cone(m2, f2, &contact.world2, &-contact.normal)
}
