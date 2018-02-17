use approx::ApproxEq;
use na::{self, Id, Real, Unit};

use math::{Isometry, Point};
use utils;
use shape::{Segment, SegmentPointLocation, SupportMap};
use query::{Contact, ContactPrediction, PointQueryWithLocation};
use query::closest_points_internal;

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub enum FeatureId {
    Vertex(usize),
    Edge(usize),
    Face(usize),
}

/// Represents a convex polygonal approximation of a face of a solid.
///
/// It is never checked if the vertices actually form a convex polygon.
/// If they do not, results of any geometric query may end up being invalid.
#[derive(Clone, Debug)]
pub struct ConvexPolyface<P: Point> {
    /// The vertices of the polyhedral face.
    vertices: Vec<P>,
    edge_normals: Vec<P::Vector>,
    normal: Unit<P::Vector>,
    feature_id: FeatureId,
    vertices_id: Vec<FeatureId>,
    edges_id: Vec<FeatureId>,
    last_optimal_dir: Option<Unit<P::Vector>>,
}

impl<P: Point> ConvexPolyface<P> {
    /// Creates a new empty convex polygonal faces.
    pub fn new() -> Self {
        let mut y: P::Vector = na::zero();
        y[1] = na::one();

        ConvexPolyface {
            vertices: Vec::new(),
            edge_normals: Vec::new(),
            normal: Unit::new_unchecked(y),
            feature_id: FeatureId::Vertex(0),
            vertices_id: Vec::new(),
            edges_id: Vec::new(),
            last_optimal_dir: None,
        }
    }

    /// Removes all the vertices of this face.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.edge_normals.clear();
        self.vertices_id.clear();
        self.edges_id.clear();
    }

    /// Adds a vertex to this face.
    ///
    /// It is not checked whether `pt` breaks the convexity of the polyhedral face.
    pub fn push(&mut self, pt: P, id: FeatureId) {
        self.vertices.push(pt);
        self.vertices_id.push(id);
    }

    /// The number of vertices of this face.
    pub fn nvertices(&self) -> usize {
        self.vertices.len()
    }

    /// The vertices of this convex polygonal face.
    pub fn vertices(&self) -> &[P] {
        &self.vertices[..]
    }

    /// Adds a scaled edge normal to this face.
    pub fn push_scaled_edge_normal(&mut self, normal: P::Vector) {
        if let Some(normal) = na::try_normalize(&normal, P::Real::default_epsilon()) {
            self.edge_normals.push(normal)
        } else {
            self.edge_normals.push(na::zero())
        }
    }

    /// Adds an edge normal to this face.
    pub fn push_edge_normal(&mut self, normal: Unit<P::Vector>) {
        self.edge_normals.push(normal.unwrap())
    }

    /// Automatically recomputes the scaled edge normals (3D only).
    ///
    /// Panics if the ambient space is not 3D.
    pub fn recompute_edge_normals_3d(&mut self) {
        self.edge_normals.clear();

        for i1 in 0..self.vertices.len() {
            let i2 = (i1 + 1) % self.vertices.len();
            let dpt = self.vertices[i2] - self.vertices[i1];
            let scaled_normal = utils::cross3(&dpt, self.normal.as_ref());
            self.push_scaled_edge_normal(scaled_normal)
        }
    }

    /// Sets the outward normal of this convex polygonal face.
    pub fn set_normal(&mut self, normal: Unit<P::Vector>) {
        self.normal = normal
    }

    pub fn push_edge_feature_id(&mut self, id: FeatureId) {
        self.edges_id.push(id)
    }

    pub fn set_feature_id(&mut self, id: FeatureId) {
        self.feature_id = id
    }

    fn is_contact_optimal<M, G1: ?Sized, G2: ?Sized>(
        m1: &M,
        g1: &G1,
        f1: FeatureId,
        m2: &M,
        g2: &G2,
        f2: FeatureId,
        contact: &Contact<P>,
    ) -> bool
    where
        M: Isometry<P>,
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        g1.is_direction_in_normal_cone(m1, f1, &contact.world1, &contact.normal)
            && g2.is_direction_in_normal_cone(m2, f2, &contact.world2, &-contact.normal)
    }

    pub fn contains_optimal<M, G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        other: &mut Self,
        m2: &M,
        g2: &G2,
        prediction: &ContactPrediction<P::Real>,
        contacts: &mut Vec<Contact<P>>,
    ) -> bool
    where
        M: Isometry<P>,
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        if let Some(dir) = self.last_optimal_dir {
            g1.support_area_toward(m1, &dir, prediction.angular1, self);
            g2.support_area_toward(m2, &-dir, prediction.angular2, other);
            let f1 = self.feature_id;
            let f2 = other.feature_id;
            self.quasi_conformal_contact_area(m1, g1, other, m2, g2, prediction, contacts);
            if let Some(dir) = self.last_optimal_dir {
                g1.support_area_toward(m1, &dir, prediction.angular1, self);
                g2.support_area_toward(m2, &-dir, prediction.angular2, other);

                if self.feature_id != f1 && other.feature_id != f2 {
                    contacts.clear();
                    self.quasi_conformal_contact_area(m1, g1, other, m2, g2, prediction, contacts);
                }
                true
            } else {
                false
            }
        } else {
            false
        }
    }

    pub fn register_contact<M, G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        fid1: FeatureId,
        m2: &M,
        g2: &G2,
        fid2: FeatureId,
        prediction: &ContactPrediction<P::Real>,
        contact: Contact<P>,
        contacts: &mut Vec<Contact<P>>,
    ) -> bool
    where
        M: Isometry<P>,
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        let keep = -contact.depth <= prediction.linear;

        if Self::is_contact_optimal(m1, g1, fid1, m2, g2, fid2, &contact) {
            self.last_optimal_dir = Some(contact.normal);
            if !keep {
                return true;
            }
        }

        if keep {
            contacts.push(contact)
        }

        false
    }

    // FIXME: the method name is not very representative of what actually does.
    /// Computes the vertices of the quasi-conformal contact area between two convex polygonal faces.
    /// Output nothing to `contacts` if the contact is only punctual. In particular, this does nothing
    /// if either face has only one vertex.
    pub fn quasi_conformal_contact_area<M, G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &M,
        g1: &G1,
        other: &Self,
        m2: &M,
        g2: &G2,
        pred: &ContactPrediction<P::Real>,
        contacts: &mut Vec<Contact<P>>,
    ) where
        M: Isometry<P>,
        G1: SupportMap<P, M>,
        G2: SupportMap<P, M>,
    {
        self.last_optimal_dir = None;
        if self.vertices.len() == 0 || other.vertices.len() == 0 {
            return;
        }
        if self.vertices.len() == 1 && other.vertices.len() == 1 {
            let fid1 = self.feature_id;
            let fid2 = other.feature_id;
            let ab = other.vertices[0] - self.vertices[0];

            let contact;
            if let Some((normal, depth)) = Unit::try_new_and_get(ab, na::zero()) {
                if g1.is_direction_in_normal_cone(m1, self.feature_id, &self.vertices[0], &normal) {
                    contact = Contact::new(self.vertices[0], other.vertices[0], normal, -depth);
                } else {
                    return; // Impossible configuration, the input normal is probably wrong.
                            // contact = Contact::new(self.vertices[0], other.vertices[0], -normal, depth);
                            // println!("Found penetration: {:?}", contact);
                            // println!(
                            //     "Local points: {}, {}, Normals: {}, {}",
                            //     m1.inverse_transform_point(&contact.world1),
                            //     m2.inverse_transform_point(&contact.world2),
                            //     m1.inverse_transform_vector(&contact.normal),
                            //     m2.inverse_transform_vector(&-contact.normal)
                            // );
                            // panic!("");
                }
            } else {
                let mut n = na::zero::<P::Vector>();
                n[0] = na::one(); // FIXME: ugly.
                let n = Unit::new_unchecked(n);
                contact = Contact::new(self.vertices[0], other.vertices[0], n, na::zero());
            }
            let _ = self.register_contact(m1, g1, fid1, m2, g2, fid2, pred, contact, contacts);
            return;
        }

        match na::dimension::<P::Vector>() {
            2 => {
                if self.vertices.len() == 1 {
                    let s2 = Segment::new(other.vertices[0], other.vertices[1]);

                    if let Some(contact) =
                        point_on_segment_contact_2d(&s2, &self.vertices[0], &other.normal, true)
                    {
                        let id1 = self.feature_id;
                        let id2 = other.feature_id;
                        let _ = self.register_contact(
                            m1,
                            g1,
                            id1,
                            m2,
                            g2,
                            id2,
                            pred,
                            contact,
                            contacts,
                        );
                    }
                } else if other.vertices.len() == 1 {
                    let s1 = Segment::new(self.vertices[0].clone(), self.vertices[1].clone());

                    if let Some(contact) =
                        point_on_segment_contact_2d(&s1, &other.vertices[0], &self.normal, false)
                    {
                        let id1 = self.feature_id;
                        let id2 = other.feature_id;
                        let _ = self.register_contact(
                            m1,
                            g1,
                            id1,
                            m2,
                            g2,
                            id2,
                            pred,
                            contact,
                            contacts,
                        );
                    }
                } else {
                    let s1 = Segment::new(self.vertices[0].clone(), self.vertices[1].clone());
                    let s2 = Segment::new(other.vertices[0].clone(), other.vertices[1].clone());
                    let n1 = self.normal;
                    let n2 = other.normal;

                    if let Some(contact) = point_on_segment_contact_2d(&s1, s2.a(), &n1, false) {
                        let id1 = self.feature_id;
                        let id2 = other.vertices_id[0];
                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, contacts)
                        {
                            return;
                        }
                    }
                    if let Some(contact) = point_on_segment_contact_2d(&s1, s2.b(), &n1, false) {
                        let id1 = self.feature_id;
                        let id2 = other.vertices_id[1];
                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, contacts)
                        {
                            return;
                        }
                    }
                    if let Some(contact) = point_on_segment_contact_2d(&s2, s1.a(), &n2, true) {
                        let id1 = self.vertices_id[0];
                        let id2 = other.feature_id;
                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, contacts)
                        {
                            return;
                        }
                    }
                    if let Some(contact) = point_on_segment_contact_2d(&s2, s1.b(), &n2, true) {
                        let id1 = self.vertices_id[1];
                        let id2 = other.feature_id;
                        if self.register_contact(m1, g1, id1, m2, g2, id2, pred, contact, contacts)
                        {
                            return;
                        }
                    }
                }
            }
            3 => {
                if self.vertices.len() == 2 && other.vertices.len() == 2 {
                    // FIXME: edge-edge case (less useful as the contact is most likely a single point).
                } else {
                    let sang1 = pred.angular1.sin();
                    let sang2 = pred.angular2.sin();

                    // Check segments.
                    for i1 in 0..self.vertices.len() {
                        let i2 = (i1 + 1) % self.vertices.len();
                        let s1 = Segment::new(self.vertices[i1], self.vertices[i2]);

                        for j1 in 0..other.vertices.len() {
                            let j2 = (j1 + 1) % other.vertices.len();
                            let s2 = Segment::new(other.vertices[j1], other.vertices[j2]);

                            let contact = if self.vertices.len() > 2 {
                                segment_segment_contact_3d(&s1, &s2, &self.normal, false)
                            } else {
                                segment_segment_contact_3d(&s2, &s1, &other.normal, true)
                            };

                            if let Some(contact) = contact {
                                // FIXME: perform those checks _before_ the computation of the contact?
                                if self.vertices.len() > 2 {
                                    let n1 = &self.edge_normals[i1];
                                    let mut dot = na::dot(contact.normal.as_ref(), n1);
                                    if contact.depth > na::zero() {
                                        // penetrating.
                                        dot = -dot;
                                    }

                                    if dot < -sang1 {
                                        continue;
                                    }
                                }
                                if other.vertices.len() > 2 {
                                    let n2 = &other.edge_normals[j1];
                                    let mut dot = -na::dot(contact.normal.as_ref(), n2);
                                    if contact.depth > na::zero() {
                                        // penetrating.
                                        dot = -dot;
                                    }

                                    if dot < -sang2 {
                                        continue;
                                    }
                                }

                                contacts.push(contact)
                            }
                        }
                    }

                    // Check vertices.
                    if self.vertices.len() > 2 {
                        assert!(self.edge_normals.len() == self.vertices().len(),
                        "Convex polyhedral face: the number of edge normals should match the number of vertices.");

                        for v in &other.vertices {
                            // Project on face.
                            if let Some(contact) = point_face_contact_3d(self, v, false) {
                                contacts.push(contact)
                            }
                        }
                    }
                    if other.vertices.len() > 2 {
                        assert!(other.edge_normals.len() == other.vertices().len(),
                        "Convex polyhedral face: the number of edge normals should match the number of vertices.");

                        for v in &self.vertices {
                            // Project on face.
                            if let Some(contact) = point_face_contact_3d(other, v, true) {
                                contacts.push(contact)
                            }
                        }
                    }
                }
            }
            _ => {}
        }
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

fn segment_segment_contact_3d<P: Point>(
    seg1: &Segment<P>,
    seg2: &Segment<P>,
    n1: &Unit<P::Vector>,
    flip: bool,
) -> Option<Contact<P>> {
    let locs = closest_points_internal::segment_against_segment_with_locations(
        &Id::new(),
        seg1,
        &Id::new(),
        seg2,
    );

    match locs {
        (SegmentPointLocation::OnEdge(_), SegmentPointLocation::OnEdge(_)) => {
            let p1 = seg1.point_at(&locs.0);
            let p2 = seg2.point_at(&locs.1);

            // FIXME: check the normal cone.

            if let Some((mut normal, mut depth)) =
                Unit::try_new_and_get(p2 - p1, P::Real::default_epsilon())
            {
                if na::dot(normal.as_ref(), n1.as_ref()) > na::zero() {
                    depth = -depth;
                } else {
                    // Penetration, flip the normal s that it still points toward
                    // the outside of the first solid.
                    normal = -normal;
                }

                if !flip {
                    Some(Contact::new(p1, p2, normal, depth))
                } else {
                    Some(Contact::new(p2, p1, -normal, depth))
                }
            } else {
                None
            }
        }
        _ => None,
    }
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

    let dist = -na::dot(face.normal.as_ref(), &(*pt - face.vertices[0]));
    let proj = *pt + *face.normal * dist;

    if !flip {
        Some(Contact::new(proj, *pt, face.normal, dist))
    } else {
        Some(Contact::new(*pt, proj, -face.normal, dist))
    }
}
