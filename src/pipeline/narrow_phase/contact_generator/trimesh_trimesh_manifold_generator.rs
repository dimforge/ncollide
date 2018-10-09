use bounding_volume::{BoundingVolume, AABB};
use math::Isometry;
use na::{self, Real, Unit};
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::closest_points_internal;
use query::{
    visitors::AABBSetsInterferencesCollector, Contact, ContactKinematic, ContactManifold,
    ContactPrediction, NeighborhoodGeometry,
};
use shape::{CompositeShape, FeatureId, Segment, SegmentPointLocation, Shape, TriMesh, Triangle};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct TriMeshTriMeshManifoldGenerator<N: Real> {
    manifold: ContactManifold<N>,
    interferences: Vec<(usize, usize)>,
}

impl<N: Real> TriMeshTriMeshManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> TriMeshTriMeshManifoldGenerator<N> {
        TriMeshTriMeshManifoldGenerator {
            manifold: ContactManifold::new(),
            interferences: Vec::new(),
        }
    }
}

impl<N: Real> TriMeshTriMeshManifoldGenerator<N> {
    fn compute_faces_closest_points(
        m12: &Isometry<N>,
        m21: &Isometry<N>,
        m1: &Isometry<N>,
        mesh1: &TriMesh<N>,
        i1: usize,
        m2: &Isometry<N>,
        mesh2: &TriMesh<N>,
        i2: usize,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
        id_alloc: &mut IdAllocator,
    ) {
        let face1 = &mesh1.faces()[i1];
        let face2 = &mesh2.faces()[i2];
        let pts1 = mesh1.points();
        let pts2 = mesh2.points();
        let t1 = Triangle::new(
            pts1[face1.indices.x],
            pts1[face1.indices.y],
            pts1[face1.indices.z],
        );
        let t2 = Triangle::new(
            m12 * pts2[face2.indices.x],
            m12 * pts2[face2.indices.y],
            m12 * pts2[face2.indices.z],
        );

        if let (Some(n1), Some(n2)) = (face1.normal, face2.normal) {
            let n2 = m12 * n2;

            /*
             * Start with the SAT.
             */
            let mut sep_axis = None;

            #[inline(always)]
            fn penetration<N: Real>(a: (N, N), b: (N, N)) -> Option<N> {
                if a.0 > b.1 || b.0 > a.1 {
                    // The intervals are disjoint.
                    None
                } else {
                    Some((b.1 - a.0).min(a.1 - a.0))
                }
            }

            #[inline(always)]
            fn sort2<N: Real>(a: N, b: N) -> (N, N) {
                if a > b {
                    (b, a)
                } else {
                    (a, b)
                }
            }

            // This loop is a trick to be able to easily stop the search for a separating axis as
            // as we find one using `break 'search` (without having to do all this on a separate function
            // and do a return instead of breaks).
            'search: loop {
                // First, test normals.
                let interval1 = t1.a().coords.dot(&n1);
                let interval2 = t2.extents_on_dir(&n1);

                if let Some(overlap) = penetration((interval1, interval1), interval2) {
                    // XXX accept the contact.
                } else {
                    // The triangles are disjoint.
                    sep_axis = Some(n1);
                    break;
                }

                let interval2 = t2.a().coords.dot(&n2);
                let interval1 = t1.extents_on_dir(&n2);
                if let Some(overlap) = penetration(interval1, (interval2, interval2)) {
                    // XXX accept the contact.
                } else {
                    // The triangles are disjoint.
                    sep_axis = Some(n2);
                    break;
                }

                let edge_dirs_a = t1.edges_scaled_directions();
                let edge_dirs_b = t2.edges_scaled_directions();

                // Second, test edges cross products.
                for (i, e1) in edge_dirs_a.iter().enumerate() {
                    for (j, e2) in edge_dirs_b.iter().enumerate() {
                        if let Some(dir) = Unit::try_new(e1.cross(e2), N::default_epsilon()) {
                            let interval1 = sort2(
                                dir.dot(&t1.vertices()[i].coords),
                                dir.dot(&t1.vertices()[(i + 2) % 3].coords),
                            );
                            let interval2 = sort2(
                                dir.dot(&t2.vertices()[j].coords),
                                dir.dot(&t2.vertices()[(j + 2) % 3].coords),
                            );

                            if let Some(overlap) = penetration(interval1, interval2) {
                                // XXX accept the contact.
                            } else {
                                // Triangles are disjoint.
                                sep_axis = Some(dir);
                                break 'search;
                            }
                        }
                    }
                }

                // If we reached this point, no separating axis was found: the triangles intersect.
                // XXX accept the contact.
                return;
            }

            /*
             * The two triangles don't intersect.
             * Compute all the LMDs considering the given linear and angular tolerances.
             */
            for i in 0..3 {
                let id_e1 = face1.edges[i];
                let e1 = &mesh1.edges()[id_e1];
                let seg1 = Segment::new(pts1[e1.indices.x], pts1[e1.indices.y]);

                for j in 0..3 {
                    let id_e2 = face2.edges[j];
                    let e2 = &mesh2.edges()[id_e2];
                    // FIXME: don't transform the points at each loop.
                    // Use the corresponding edge from t2 instead.
                    let seg2 = Segment::new(m12 * pts2[e2.indices.x], m12 * pts2[e2.indices.y]);

                    let locs = closest_points_internal::segment_against_segment_with_locations_nD(
                        (seg1.a(), seg1.b()),
                        (seg2.a(), seg2.b()),
                    );
                    let p1 = seg1.point_at(&locs.0);
                    let p2 = seg2.point_at(&locs.1);
                    if let Some(dir) = Unit::try_new(p2 - p1, N::default_epsilon()) {
                        match locs {
                            (
                                SegmentPointLocation::OnVertex(i),
                                SegmentPointLocation::OnVertex(j),
                            ) => {
                                let ip1 = e1.indices[i];
                                let ip2 = e2.indices[j];
                                if mesh1.vertex_tangent_cone_polar_contains_dir(
                                    ip1,
                                    &dir,
                                    prediction.sin_angular1(),
                                ) && mesh2.vertex_tangent_cone_polar_contains_dir(
                                    ip2,
                                    &(m21 * -dir),
                                    prediction.sin_angular2(),
                                ) {
                                    // Accept the contact.
                                    let contact = Contact::new_wo_depth(m1 * p1, m1 * p2, m1 * dir);
                                    let mut kinematic = ContactKinematic::new();
                                    kinematic.set_approx1(
                                        FeatureId::Vertex(ip1),
                                        pts1[ip1],
                                        NeighborhoodGeometry::Point,
                                    );
                                    kinematic.set_approx2(
                                        FeatureId::Vertex(ip2),
                                        pts2[ip2],
                                        NeighborhoodGeometry::Point,
                                    );
                                    let _ = manifold.push(contact, p1, kinematic, id_alloc);
                                }
                            }
                            (
                                SegmentPointLocation::OnVertex(i),
                                SegmentPointLocation::OnEdge(_),
                            ) => {
                                let ip1 = e1.indices[i];
                                if mesh1.vertex_tangent_cone_polar_contains_dir(
                                    ip1,
                                    &dir,
                                    prediction.sin_angular1(),
                                ) && mesh2.edge_tangent_cone_polar_contains_orthogonal_dir(
                                    id_e2,
                                    &(m21 * -dir),
                                    prediction.sin_angular2(),
                                ) {
                                    // Accept the contact.
                                    let contact = Contact::new_wo_depth(m1 * p1, m1 * p2, m1 * dir);
                                    let mut kinematic = ContactKinematic::new();
                                    kinematic.set_approx1(
                                        FeatureId::Vertex(ip1),
                                        pts1[ip1],
                                        NeighborhoodGeometry::Point,
                                    );
                                    kinematic.set_approx2(
                                        FeatureId::Edge(id_e2),
                                        pts2[e2.indices.x],
                                        NeighborhoodGeometry::Line(m21 * seg2.direction().unwrap()),
                                    );
                                    let _ = manifold.push(contact, p1, kinematic, id_alloc);
                                }
                            }
                            (
                                SegmentPointLocation::OnEdge(_),
                                SegmentPointLocation::OnVertex(j),
                            ) => {
                                let ip2 = e2.indices[j];
                                if mesh1.edge_tangent_cone_polar_contains_orthogonal_dir(
                                    id_e1,
                                    &dir,
                                    prediction.sin_angular1(),
                                ) && mesh2.vertex_tangent_cone_polar_contains_dir(
                                    ip2,
                                    &(m21 * -dir),
                                    prediction.sin_angular2(),
                                ) {
                                    // Accept the contact.
                                    let contact = Contact::new_wo_depth(m1 * p1, m1 * p2, m1 * dir);
                                    let mut kinematic = ContactKinematic::new();
                                    kinematic.set_approx1(
                                        FeatureId::Edge(id_e1),
                                        pts1[e1.indices.x],
                                        NeighborhoodGeometry::Line(seg1.direction().unwrap()),
                                    );
                                    kinematic.set_approx2(
                                        FeatureId::Vertex(ip2),
                                        pts2[ip2],
                                        NeighborhoodGeometry::Point,
                                    );
                                    let _ = manifold.push(contact, p1, kinematic, id_alloc);
                                }
                            }
                            (SegmentPointLocation::OnEdge(_), SegmentPointLocation::OnEdge(_)) => {
                                if mesh1.edge_tangent_cone_polar_contains_orthogonal_dir(
                                    id_e1,
                                    &dir,
                                    prediction.sin_angular1(),
                                ) && mesh2.edge_tangent_cone_polar_contains_orthogonal_dir(
                                    id_e2,
                                    &(m21 * -dir),
                                    prediction.sin_angular2(),
                                ) {
                                    // Accept the contact.
                                    let contact = Contact::new_wo_depth(m1 * p1, m1 * p2, m1 * dir);
                                    let mut kinematic = ContactKinematic::new();
                                    kinematic.set_approx1(
                                        FeatureId::Edge(id_e1),
                                        pts1[e1.indices.x],
                                        NeighborhoodGeometry::Line(seg1.direction().unwrap()),
                                    );
                                    kinematic.set_approx2(
                                        FeatureId::Edge(id_e2),
                                        pts2[e2.indices.x],
                                        NeighborhoodGeometry::Line(m21 * seg2.direction().unwrap()),
                                    );
                                    let _ = manifold.push(contact, p1, kinematic, id_alloc);
                                }
                            }
                        }
                    }
                }
            }

            // Project vertices for face1 to the plane of face2.
            'vloop1: for iv in face1.indices.iter() {
                let p1 = pts1[*iv];

                for (side2, ref_pt2) in face2
                    .side_normals
                    .as_ref()
                    .unwrap()
                    .iter()
                    .zip(t2.vertices().iter())
                {
                    // FIXME: too bad we will re-transform side2 for each iv...
                    let dpt = p1 - ref_pt2;
                    if dpt.dot(&(m12 * side2)) >= N::zero() {
                        continue 'vloop1;
                    }
                }

                let dpt = p1 - t2.a();
                if mesh1.vertex_tangent_cone_polar_contains_dir(
                    *iv,
                    &(m12 * -n2),
                    prediction.sin_angular1(),
                ) {
                    let dist = dpt.dot(&n2);
                    let proj = p1 + *n2 * -dist;

                    // Accept the contact.
                    let contact = Contact::new(m1 * p1, m1 * proj, m1 * -n2, -dist);
                    let mut kinematic = ContactKinematic::new();
                    kinematic.set_approx1(FeatureId::Vertex(*iv), p1, NeighborhoodGeometry::Point);
                    kinematic.set_approx2(
                        FeatureId::Face(i2),
                        pts2[face2.indices.x],
                        NeighborhoodGeometry::Plane(face2.normal.unwrap()),
                    );
                    let _ = manifold.push(contact, p1, kinematic, id_alloc);
                }
            }

            // Project vertices for face2 to the plane of face1.
            'vloop2: for iv in face2.indices.iter() {
                // FIXME: don't re-transform the point.
                // Re-use the corresponding vertex from t2 instead.
                let p2 = m12 * pts2[*iv];

                for (side1, ref_pt1) in face1
                    .side_normals
                    .as_ref()
                    .unwrap()
                    .iter()
                    .zip(t1.vertices().iter())
                {
                    let dpt = p2 - ref_pt1;
                    if dpt.dot(side1) >= N::zero() {
                        continue 'vloop2;
                    }
                }

                let dpt = p2 - t1.a();
                if mesh2.vertex_tangent_cone_polar_contains_dir(
                    *iv,
                    &-n1,
                    prediction.sin_angular2(),
                ) {
                    let dist = dpt.dot(&n1);
                    let proj = p2 + *n1 * -dist;

                    // Accept the contact.
                    let contact = Contact::new(m1 * proj, m1 * p2, m1 * n1, -dist);
                    let mut kinematic = ContactKinematic::new();
                    kinematic.set_approx1(
                        FeatureId::Face(i1),
                        *t1.a(),
                        NeighborhoodGeometry::Plane(n1),
                    );
                    kinematic.set_approx2(
                        FeatureId::Vertex(*iv),
                        m21 * p2,
                        NeighborhoodGeometry::Point,
                    );
                    let _ = manifold.push(contact, proj, kinematic, id_alloc);
                }
            }
        }
    }

    fn do_update_to(
        _: &ContactDispatcher<N>,
        id1: usize,
        m1: &Isometry<N>,
        mesh1: &TriMesh<N>,
        id2: usize,
        m2: &Isometry<N>,
        mesh2: &TriMesh<N>,
        prediction: &ContactPrediction<N>,
        interferences: &mut Vec<(usize, usize)>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) {
        // Find new collisions
        let m12 = m1.inverse() * m2;
        let m21 = m12.inverse();

        // For transforming AABBs from mesh2 in the local space of mesh1.
        let m12_abs_rot = m12.rotation.to_rotation_matrix().matrix().abs();

        {
            let mut visitor =
                AABBSetsInterferencesCollector::new(&m12, &m12_abs_rot, interferences);
            mesh1.bvh().visit_bvtt(mesh2.bvh(), &mut visitor);
        }

        for id in interferences.drain(..) {
            Self::compute_faces_closest_points(
                &m12, &m21, m1, mesh1, id.0, m2, mesh2, id.1, prediction, manifold, id_alloc,
            );
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for TriMeshTriMeshManifoldGenerator<N> {
    fn update(
        &mut self,
        d: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        self.manifold.save_cache_and_clear(id_alloc);

        if let (Some(mesh1), Some(mesh2)) = (a.as_shape::<TriMesh<N>>(), b.as_shape::<TriMesh<N>>())
        {
            Self::do_update_to(
                d,
                ida,
                ma,
                mesh1,
                idb,
                mb,
                mesh2,
                prediction,
                &mut self.interferences,
                id_alloc,
                &mut self.manifold,
            );
            true
        } else {
            false
        }
    }

    fn update_to(
        &mut self,
        d: &ContactDispatcher<N>,
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
        if let (Some(mesh1), Some(mesh2)) = (a.as_shape::<TriMesh<N>>(), b.as_shape::<TriMesh<N>>())
        {
            Self::do_update_to(
                d,
                ida,
                ma,
                mesh1,
                idb,
                mb,
                mesh2,
                prediction,
                &mut self.interferences,
                id_alloc,
                manifold,
            );
            true
        } else {
            false
        }
    }

    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        out.push(&self.manifold)
    }
}
