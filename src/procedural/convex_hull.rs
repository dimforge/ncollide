use std::num::{Bounded, Zero};
use nalgebra::na::{Vec2, Vec3, Norm, FloatVec};
use nalgebra::na;
use math::Scalar;
use utils;
use procedural::{Polyline, TriMesh, UnifiedIndexBuffer};
use implicit;

/// Computes the convex hull of a set of 3d points.
pub fn convex_hull3d(points: &[Vec3<Scalar>]) -> TriMesh<Scalar, Vec3<Scalar>> {
    assert!(points.len() != 0, "Cannot compute the convex hull of an empty set of point.");

    let mut undecidable_points  = Vec::new();
    let mut horizon_loop_facets = Vec::new();
    let mut horizon_loop_ids    = Vec::new();
    let mut removed_facets      = Vec::new();

    let mut triangles;
    
    match get_initial_mesh(points, &mut undecidable_points) {
        Facets(facets)   => triangles = facets,
        ResultMesh(mesh) => return mesh
    }

    let mut i = 0;
    while i != triangles.len() {
        horizon_loop_facets.clear();
        horizon_loop_ids.clear();

        if !triangles[i].valid {
            i = i + 1;
            continue;
        }

        let pt_id = support_point(&triangles[i].normal,
                                  points,
                                  triangles[i].visible_points.as_slice());

        match pt_id {
            Some(point) => {
                removed_facets.clear();

                triangles.get_mut(i).valid = false;
                removed_facets.push(i);

                for j in range(0u, 3) {
                    compute_silhouette(triangles[i].adj[j],
                                       triangles[i].indirect_adj_id[j],
                                       point,
                                       &mut horizon_loop_facets,
                                       &mut horizon_loop_ids,
                                       points,
                                       &mut removed_facets,
                                       triangles.as_mut_slice());
                }

                if horizon_loop_facets.is_empty() {
                    // Due to inaccuracies, the silhouette could not be computed
                    // (the point seems to be visible from… every triangle).
                    // Force it to be at least the curren tiangle adjascent faces.
                    // FIXME: check that this is OK even in weird cases of coplanarity…
                    for j in range(0, 3) {
                        horizon_loop_facets.push(triangles[i].adj[j]);
                        horizon_loop_ids.push(triangles[i].indirect_adj_id[j]);
                    }
                }

                attach_and_push_facets_3d(horizon_loop_facets.as_slice(),
                                          horizon_loop_ids.as_slice(),
                                          point,
                                          points,
                                          &mut triangles,
                                          removed_facets.as_slice(),
                                          &mut undecidable_points);
            },
            None => { }
        }

        i = i + 1;
    }

    let mut pts = Vec::from_slice(points);
    let mut idx = Vec::new();

    for facet in triangles.iter() {
        if facet.valid {
            idx.push(Vec3::new(facet.pts[0] as u32, facet.pts[1] as u32, facet.pts[2] as u32));
        }
    }

    utils::remove_unused_points(&mut pts, idx.as_mut_slice());

    assert!(pts.len() != 0, "Internal error: empty output mesh.");

    TriMesh::new(pts, None, None, Some(UnifiedIndexBuffer(idx)))
}

enum InitialMesh {
    Facets(Vec<TriangleFacet>),
    ResultMesh(TriMesh<Scalar, Vec3<Scalar>>)
}

fn build_degenerate_mesh_point(point: Vec3<Scalar>) -> TriMesh<Scalar, Vec3<Scalar>> {
    let ta = Vec3::new(0u32, 0, 0);
    let tb = Vec3::new(0u32, 0, 0);

    TriMesh::new(vec!(point), None, None, Some(UnifiedIndexBuffer(vec!(ta, tb))))
}

fn build_degenerate_mesh_segment(dir: &Vec3<Scalar>, points: &[Vec3<Scalar>]) -> TriMesh<Scalar, Vec3<Scalar>> {
    let a = implicit::point_cloud_support_point(dir, points);
    let b = implicit::point_cloud_support_point(&-dir, points);

    let ta = Vec3::new(0u32, 1, 0);
    let tb = Vec3::new(1u32, 0, 0);

    TriMesh::new(vec!(a, b), None, None, Some(UnifiedIndexBuffer(vec!(ta, tb))))
}

fn get_initial_mesh(points: &[Vec3<Scalar>], undecidable: &mut Vec<uint>) -> InitialMesh {
    let mut res = Vec::new();

    let p1 = 0;
    let mut p2 = 1;
    let mut p3 = 2;

    while p2 != points.len() {
        let p1p2 = points[p2] - points[p1];

        if !na::approx_eq(&p1p2, &na::zero()) { // FIXME: use a smaller threshold?
            break;
        }

        p2 = p2 + 1;
    }

    if p2 == points.len() {
        return ResultMesh(build_degenerate_mesh_point(points[p1]));
    }

    while p3 != points.len() {
        if !utils::is_affinely_dependent_triangle(&points[p1], &points[p2], &points[p3]) {
            break;
        }

        p3 = p3 + 1;
    }

    if p3 == points.len() {
        return ResultMesh(build_degenerate_mesh_segment(&(points[p2] - points[p1]), points));
    }

    // Build two facets with opposite normals
    let mut f1 = TriangleFacet::new(p1, p2, p3, points);
    let mut f2 = TriangleFacet::new(p2, p1, p3, points);

    // Link the facets together
    f1.set_facets_adjascency(1, 1, 1, 0, 2, 1);
    f2.set_facets_adjascency(0, 0, 0, 0, 2, 1);

    // Attribute points to each facet.
    for i in range(1, points.len()) {
        if i == p2 || i == p3 {
            continue;
        }
        if f1.can_be_seen_by(i, points) {
            f1.visible_points.push(i);
        }
        else if f2.can_be_seen_by(i, points) {
            f2.visible_points.push(i);
        }
        else { // the point is coplanar
            undecidable.push(i);
        }
    }

    res.push(f1);
    res.push(f2);

    verify_facet_links(0, res.as_slice());
    verify_facet_links(1, res.as_slice());

    Facets(res)
}

fn support_point<N: Float, V: FloatVec<N>>(direction: &V, points : &[V], idx: &[uint]) -> Option<uint> {
    let mut argmax = None;
    let _M: N      = Bounded::max_value();
    let mut max    = -_M;

    for i in idx.iter() {
        let dot = na::dot(direction, &points[*i]);

        if dot > max {
            argmax = Some(*i);
            max    = dot;
        }
    }

    argmax
}

// FIXME: uggly, find a way to refactor all the support point functions!
fn support_point_2<N: Float, V: FloatVec<N>>(direction: &V, points : &[V]) -> Option<uint> {
    let mut argmax = None;
    let _M: N      = Bounded::max_value();
    let mut max    = -_M;

    for (id, pt) in points.iter().enumerate() {
        let dot = na::dot(direction, pt);

        if dot > max {
            argmax = Some(id);
            max    = dot;
        }
    }

    argmax
}

fn compute_silhouette(facet:          uint,
                      indirectID :    uint,
                      point:          uint,
                      out_facets:     &mut Vec<uint>,
                      out_adj_idx:    &mut Vec<uint>,
                      points:         &[Vec3<Scalar>],
                      removedFacets : &mut Vec<uint>,
                      triangles:      &mut [TriangleFacet]) {
    if triangles[facet].valid {
        if !triangles[facet].can_be_seen_by(point, points) {
            out_facets.push(facet);
            out_adj_idx.push(indirectID);
        }
        else {
            triangles[facet].valid = false; // the facet must be removed from the convex hull
            removedFacets.push(facet);

            compute_silhouette(triangles[facet].adj[(indirectID + 1) % 3],
                               triangles[facet].indirect_adj_id[(indirectID + 1) % 3],
                               point,
                               out_facets,
                               out_adj_idx,
                               points,
                               removedFacets,
                               triangles);
            compute_silhouette(triangles[facet].adj[(indirectID + 2) % 3],
                               triangles[facet].indirect_adj_id[(indirectID + 2) % 3],
                               point,
                               out_facets,
                               out_adj_idx,
                               points,
                               removedFacets,
                               triangles);
        }
    }
}

fn verify_facet_links(ifacet: uint, facets: &[TriangleFacet]) {
    let facet = &facets[ifacet];

    for i in range(0u, 3) {
        let adji = &facets[facet.adj[i]];

        assert!(
            adji.adj[facet.indirect_adj_id[i]] == ifacet &&
            adji.first_point_from_edge(facet.indirect_adj_id[i]) == facet.second_point_from_edge(adji.indirect_adj_id[facet.indirect_adj_id[i]]) &&
            adji.second_point_from_edge(facet.indirect_adj_id[i]) == facet.first_point_from_edge(adji.indirect_adj_id[facet.indirect_adj_id[i]]))
    }
}

fn attach_and_push_facets_3d(horizon_loop_facets: &[uint],
                             horizon_loop_ids:    &[uint],
                             point:               uint,
                             points:              &[Vec3<Scalar>],
                             triangles:           &mut Vec<TriangleFacet>,
                             removed_facets:      &[uint],
                             undecidable:         &mut Vec<uint>) {
    // horizon is built to be in CCW order
    let mut new_facets = Vec::with_capacity(horizon_loop_facets.len());

    // create new facets
    let mut adj_facet:  uint;
    let mut indirectId: uint;

    for i in range(0, horizon_loop_facets.len()) {
        adj_facet  = horizon_loop_facets[i];
        indirectId = horizon_loop_ids[i];

        let facet = TriangleFacet::new(point,
                                       (*triangles)[adj_facet].second_point_from_edge(indirectId),
                                       (*triangles)[adj_facet].first_point_from_edge(indirectId),
                                       points);
        new_facets.push(facet);
    }

    // link facets together
    for i in range(0, horizon_loop_facets.len()) {
        let prev_facet;

        if i == 0 {
            prev_facet = triangles.len() + horizon_loop_facets.len() - 1;
        }
        else {
            prev_facet = triangles.len() + i - 1;
        }

        let middle_facet = horizon_loop_facets[i];
        let next_facet   = triangles.len() + (i + 1) % horizon_loop_facets.len();
        let middle_id    = horizon_loop_ids[i];

        new_facets.get_mut(i).set_facets_adjascency(prev_facet, middle_facet, next_facet,
                                                    2         , middle_id   , 0);
        triangles.get_mut(middle_facet).adj[middle_id] = triangles.len() + i; // the future id of curr_facet
        triangles.get_mut(middle_facet).indirect_adj_id[middle_id] = 1;
    }

    // assign to each facets some of the points which can see it
    for curr_facet in removed_facets.iter() {
        for visible_point in (*triangles)[*curr_facet].visible_points.iter() {
            if *visible_point == point {
                continue;
            }

            for curr_facet in new_facets.mut_iter() {
                if curr_facet.can_be_seen_by(*visible_point, points) {
                    curr_facet.visible_points.push(*visible_point);
                    break;
                }
            }
            // if none of the facet can be seen from the point, it is naturally deleted
        }

        // try to assign collinear points to one of the new facets
        let mut i = 0;

        while i != undecidable.len() {
            for curr_facet in new_facets.mut_iter() {
                if curr_facet.can_be_seen_by((*undecidable)[i], points) {
                    curr_facet.visible_points.push((*undecidable)[i]);
                    let _ = undecidable.swap_remove(i);
                    i = i - 1;
                    break;
                }
            }

            i = i + 1;
        }
    }

    // push facets
    // FIXME: can we avoid the tmp vector `new_facets` ?
    for curr_facet in new_facets.move_iter() {
        triangles.push(curr_facet);
    }
}


struct TriangleFacet {
    // FIXME: could some of those Vecs be replaced by a [type, ..3] ?
    pub valid:           bool,
    pub normal:          Vec3<Scalar>,
    pub adj:             [uint, ..3],
    pub indirect_adj_id: [uint, ..3],
    pub pts:             [uint, ..3],
    pub visible_points:  Vec<uint>
}


impl TriangleFacet {
    pub fn new(p1: uint, p2: uint, p3: uint, points: &[Vec3<Scalar>]) -> TriangleFacet {
        let p1p2 = points[p2] - points[p1];
        let p1p3 = points[p3] - points[p1];

        let mut normal = na::cross(&p1p2, &p1p3);
        if normal.normalize().is_zero() {
            fail!("A facet must not be affinely dependent.");
        }

        TriangleFacet {
            valid:           true,
            normal:          normal,
            adj:             [0, 0, 0],
            indirect_adj_id: [0, 0, 0],
            pts:             [p1, p2, p3],
            visible_points:  Vec::new()
        }
    }

    pub fn set_facets_adjascency(&mut self,
                                 adj1:   uint,
                                 adj2:   uint,
                                 adj3:   uint,
                                 idAdj1: uint,
                                 idAdj2: uint,
                                 idAdj3: uint) {
        self.indirect_adj_id[0] = idAdj1;
        self.indirect_adj_id[1] = idAdj2;
        self.indirect_adj_id[2] = idAdj3;

        self.adj[0] = adj1;
        self.adj[1] = adj2;
        self.adj[2] = adj3;
    }

    pub fn first_point_from_edge(&self, id: uint) -> uint {
        self.pts[id]
    }

    pub fn second_point_from_edge(&self, id: uint) -> uint {
        self.pts[(id + 1) % 3]
    }

    /*
    pub fn opposite_point_to_edge(&self, id: uint) -> uint {
        self.pts[(id + 2) % 3]
    }
    */

    pub fn can_be_seen_by(&self, point: uint, points: &[Vec3<Scalar>]) -> bool {
        let p0 = &points[self.pts[0]];
        let p1 = &points[self.pts[1]];
        let p2 = &points[self.pts[2]];
        let pt = &points[point];

        let _eps: Scalar = Float::epsilon();

        na::dot(&(*pt - *p0), &self.normal) > _eps * na::cast(100.0f64) &&
        !utils::is_affinely_dependent_triangle(p0, p1, pt) &&
        !utils::is_affinely_dependent_triangle(p0, p2, pt) &&
        !utils::is_affinely_dependent_triangle(p1, p2, pt)
    }
}


/// Computes the convex hull of a set of 2d points.
pub fn convex_hull2d(points: &[Vec2<Scalar>]) -> Polyline<Scalar, Vec2<Scalar>> {
    let mut undecidable_points = Vec::new();
    let mut segments           = get_initial_polyline(points, &mut undecidable_points);

    let mut i = 0;
    while i != segments.len() {
        if !segments[i].valid {
            i = i + 1;
            continue;
        }

        let pt_id = support_point(&segments[i].normal,
                                  points,
                                  segments[i].visible_points.as_slice());

        match pt_id {
            Some(point) => {
                segments.get_mut(i).valid = false;

                attach_and_push_facets_2d(segments[i].prev,
                                          segments[i].next,
                                          point,
                                          points.as_slice(),
                                          &mut segments,
                                          i,
                                          &mut undecidable_points);
            },
            None => { }
        }

        i = i + 1;
    }

    let mut pts        = Vec::new();
    let mut curr_facet = 0;

    while !segments[curr_facet].valid {
        curr_facet = curr_facet + 1
    }

    let first_facet = curr_facet;

    loop {
        let curr = &segments[curr_facet];

        assert!(curr.valid);

        pts.push(points[curr.pts[0]].clone());

        curr_facet = curr.next;

        if curr_facet == first_facet {
            break;
        }
    }

    Polyline::new(pts, None)
}

pub fn get_initial_polyline(points: &[Vec2<Scalar>], undecidable: &mut Vec<uint>) -> Vec<SegmentFacet> {
    let mut res = Vec::new();

    assert!(points.len() >= 2);

    let p1     = support_point_2(&Vec2::x(), points).unwrap();
    let mut p2 = p1;

    let direction = [
        -Vec2::x(),
        Vec2::y(),
        -Vec2::y()
    ];

    for dir in direction.iter() {
        p2 = support_point_2(dir, points).unwrap();

        let p1p2 = points[p2] - points[p1];

        if !na::sqnorm(&p1p2).is_zero() {
            break;
        }
    }

    assert!(p1 != p2, "Failed to build the 2d convex hull of this point cloud.");

    // Build two facets with opposite normals.
    let mut f1 = SegmentFacet::new(p1, p2, 1, 1, points);
    let mut f2 = SegmentFacet::new(p2, p1, 0, 0, points);

    // Attribute points to each facet.
    for i in range(1, points.len()) {
        if i == p2 {
            continue;
        }
        if f1.can_be_seen_by(i, points) {
            f1.visible_points.push(i);
        }
        else if f2.can_be_seen_by(i, points) {
            f2.visible_points.push(i);
        }
        else { // The point is collinear.
            undecidable.push(i);
        }
    }

    res.push(f1);
    res.push(f2);

    res
}

fn attach_and_push_facets_2d(prev_facet:    uint,
                             next_facet:    uint,
                             point:         uint,
                             points:        &[Vec2<Scalar>],
                             segments:      &mut Vec<SegmentFacet>,
                             removed_facet: uint,
                             undecidable:   &mut Vec<uint>) {

    let new_facet1_id = segments.len();
    let new_facet2_id = new_facet1_id + 1;
    let prev_pt       = (*segments)[prev_facet].pts[1];
    let next_pt       = (*segments)[next_facet].pts[0];

    let mut new_facet1 = SegmentFacet::new(prev_pt, point, prev_facet, new_facet2_id, points);
    let mut new_facet2 = SegmentFacet::new(point, next_pt, new_facet1_id, next_facet, points);

    segments.get_mut(prev_facet).next = new_facet1_id;
    segments.get_mut(next_facet).prev = new_facet2_id;

    // Assign to each facets some of the points which can see it.
    for visible_point in (*segments)[removed_facet].visible_points.iter() {
        if *visible_point == point {
            continue;
        }

        if new_facet1.can_be_seen_by(*visible_point, points) {
            new_facet1.visible_points.push(*visible_point);
        }
        else if new_facet2.can_be_seen_by(*visible_point, points) {
            new_facet2.visible_points.push(*visible_point);
        }
        // If none of the facet can be seen from the point, it is naturally deleted.
    }

    // Try to assign collinear points to one of the new facets
    let mut i = 0;

    while i != undecidable.len() {
        if new_facet1.can_be_seen_by((*undecidable)[i], points) {
            new_facet1.visible_points.push((*undecidable)[i]);
            let _ = undecidable.swap_remove(i);
        }
        else if new_facet2.can_be_seen_by((*undecidable)[i], points) {
            new_facet2.visible_points.push((*undecidable)[i]);
            let _ = undecidable.swap_remove(i);
        }
        else {
            i = i + 1;
        }
    }

    segments.push(new_facet1);
    segments.push(new_facet2);
}

struct SegmentFacet {
    pub valid:          bool,
    pub normal:         Vec2<Scalar>,
    pub next:           uint,
    pub prev:           uint,
    pub pts:            [uint, ..2],
    pub dto:            Scalar,
    pub visible_points: Vec<uint>
}

impl SegmentFacet {
    pub fn new(p1: uint, p2: uint, prev: uint, next: uint, points: &[Vec2<Scalar>]) -> SegmentFacet {
        let p1p2 = points[p2] - points[p1];

        let mut normal = Vec2::new(-p1p2.y, p1p2.x);
        if normal.normalize().is_zero() {
            fail!("A facet must not be affinely dependent.");
        }

        let dto = na::dot(&points[p1], &normal);

        SegmentFacet {
            valid:          true,
            normal:         normal,
            prev:           prev,
            next:           next,
            pts:            [p1, p2],
            dto:            dto,
            visible_points: Vec::new()
        }
    }

    pub fn can_be_seen_by(&self, point: uint, points: &[Vec2<Scalar>]) -> bool {
        let pt = &points[point];

        na::dot(pt, &self.normal) > self.dto
    }
}



#[cfg(test)]
mod test {
    use nalgebra::na::Vec2;
    use procedural;

    #[test]
    fn test_simple_convex_hull2d() {
        let points = [
            Vec2::new(4.723881, 3.597233),
            Vec2::new(3.333363, 3.429991),
            Vec2::new(3.137215, 2.812263)
            ];

        let chull = super::convex_hull2d(points);

        assert!(chull.coords.len() == 3);
    }

    #[test]
    fn test_ball_convex_hull() {
        // This trigerred a failure to an affinely dependent facet.
        let sphere = procedural::sphere(&0.4f32, 20, 20, true);
        let points = sphere.coords;
        let chull  = procedural::convex_hull3d(points.as_slice());

        // dummy test, we are just checking that the construction did not fail.
        assert!(chull.coords.len() == chull.coords.len());
    }
}
