use num::Bounded;
use std::cmp::Ordering;

use na::{self, Matrix3, Point2, Point3, Real, Vector3};
use crate::procedural::{IndexBuffer, TriMesh};
use crate::transformation::{
    self,
    convex_hull_utils::{denormalize, indexed_support_point_id, normalize, support_point_id},
};
use crate::utils;

/// Computes the convariance matrix of a set of points.
fn cov<N: Real>(pts: &[Point3<N>]) -> Matrix3<N> {
    let center = utils::center(pts);
    let mut cov: Matrix3<N> = na::zero();
    let normalizer: N = na::convert(1.0 / (pts.len() as f64));

    for p in pts.iter() {
        let cp = *p - center;
        cov = cov + cp * (cp * normalizer).transpose();
    }

    cov
}

/// Computes the convex hull of a set of 3d points.
pub fn convex_hull3<N: Real>(points: &[Point3<N>]) -> TriMesh<N> {
    assert!(
        points.len() != 0,
        "Cannot compute the convex hull of an empty set of point."
    );

    let mut points = points.to_vec();

    let (norm_center, norm_diag) = normalize(&mut points[..]);

    let mut undecidable_points = Vec::new();
    let mut horizon_loop_facets = Vec::new();
    let mut horizon_loop_ids = Vec::new();
    let mut removed_facets = Vec::new();

    let mut triangles;
    let denormalizer;

    match get_initial_mesh(&mut points[..], &mut undecidable_points) {
        InitialMesh::Facets(facets, denorm) => {
            triangles = facets;
            denormalizer = denorm;
        }
        InitialMesh::ResultMesh(mut mesh) => {
            denormalize(&mut mesh.coords[..], &norm_center, norm_diag);
            return mesh;
        }
    }

    let mut i = 0;
    while i != triangles.len() {
        horizon_loop_facets.clear();
        horizon_loop_ids.clear();

        if !triangles[i].valid {
            i = i + 1;
            continue;
        }

        // FIXME: use triangles[i].furthest_point instead.
        let pt_id = indexed_support_point_id(
            &triangles[i].normal,
            &points[..],
            &triangles[i].visible_points[..],
        );

        match pt_id {
            Some(point) => {
                removed_facets.clear();

                triangles[i].valid = false;
                removed_facets.push(i);

                for j in 0usize..3 {
                    compute_silhouette(
                        triangles[i].adj[j],
                        triangles[i].indirect_adj_id[j],
                        point,
                        &mut horizon_loop_facets,
                        &mut horizon_loop_ids,
                        &points[..],
                        &mut removed_facets,
                        &mut triangles[..],
                    );
                }

                if horizon_loop_facets.is_empty() {
                    // Due to inaccuracies, the silhouette could not be computed
                    // (the point seems to be visible from… every triangle).
                    let mut any_valid = false;
                    for j in i + 1..triangles.len() {
                        if triangles[j].valid {
                            any_valid = true;
                        }
                    }

                    if any_valid {
//                        println!("Warning: exitting an unfinished work.");
                    }

                    // FIXME: this is verry harsh.
                    triangles[i].valid = true;
                    break;
                }

                attach_and_push_facets3(
                    &horizon_loop_facets[..],
                    &horizon_loop_ids[..],
                    point,
                    &points[..],
                    &mut triangles,
                    &removed_facets[..],
                    &mut undecidable_points,
                );
            }
            None => {}
        }

        i = i + 1;
    }

    let mut idx = Vec::new();

    for facet in triangles.iter() {
        if facet.valid {
            idx.push(Point3::new(
                facet.pts[0] as u32,
                facet.pts[1] as u32,
                facet.pts[2] as u32,
            ));
        }
    }

    utils::remove_unused_points(&mut points, &mut idx[..]);

    assert!(points.len() != 0, "Internal error: empty output mesh.");

    for point in points.iter_mut() {
        *point = denormalizer * *point;
    }

    denormalize(&mut points[..], &norm_center, norm_diag);

    TriMesh::new(points, None, None, Some(IndexBuffer::Unified(idx)))
}

enum InitialMesh<N: Real> {
    Facets(Vec<TriangleFacet<N>>, Matrix3<N>),
    ResultMesh(TriMesh<N>),
}

fn build_degenerate_mesh_point<N: Real>(point: Point3<N>) -> TriMesh<N> {
    let ta = Point3::new(0u32, 0, 0);
    let tb = Point3::new(0u32, 0, 0);

    TriMesh::new(
        vec![point],
        None,
        None,
        Some(IndexBuffer::Unified(vec![ta, tb])),
    )
}

fn build_degenerate_mesh_segment<N: Real>(dir: &Vector3<N>, points: &[Point3<N>]) -> TriMesh<N> {
    let a = utils::point_cloud_support_point(dir, points);
    let b = utils::point_cloud_support_point(&-*dir, points);

    let ta = Point3::new(0u32, 1, 0);
    let tb = Point3::new(1u32, 0, 0);

    TriMesh::new(
        vec![a, b],
        None,
        None,
        Some(IndexBuffer::Unified(vec![ta, tb])),
    )
}

fn get_initial_mesh<N: Real>(
    points: &mut [Point3<N>],
    undecidable: &mut Vec<usize>,
) -> InitialMesh<N>
{
    /*
     * Compute the eigenvectors to see if the input datas live on a subspace.
     */
    let cov_mat = cov(points);
    let eig = cov_mat.symmetric_eigen();
    let (eigvec, eigval) = (eig.eigenvectors, eig.eigenvalues);
    let mut eigpairs = [
        (eigvec.column(0).into_owned(), eigval[0]),
        (eigvec.column(1).into_owned(), eigval[1]),
        (eigvec.column(2).into_owned(), eigval[2]),
    ];

    /*
     * Sort in deacreasing order wrt. eigenvalues.
     */
    eigpairs.sort_by(|a, b| {
        if a.1 > b.1 {
            Ordering::Less // `Less` and `Greater` are reversed.
        } else if a.1 < b.1 {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });

    /*
     * Count the dimension the data lives in.
     */
    let mut dimension = 0;
    while dimension < 3 {
        if relative_eq!(
            eigpairs[dimension].1,
            na::zero(),
            epsilon = na::convert(1.0e-7f64)
        ) {
            break;
        }

        dimension = dimension + 1;
    }

    match dimension {
        0 => {
            // The hull is a point.
            InitialMesh::ResultMesh(build_degenerate_mesh_point(points[0].clone()))
        }
        1 => {
            // The hull is a segment.
            InitialMesh::ResultMesh(build_degenerate_mesh_segment(&eigpairs[0].0, points))
        }
        2 => {
            // The hull is a triangle.
            // Project into the principal plane…
            let axis1 = &eigpairs[0].0;
            let axis2 = &eigpairs[1].0;

            let mut subspace_points = Vec::with_capacity(points.len());

            for point in points.iter() {
                subspace_points.push(Point2::new(
                    point.coords.dot(axis1),
                    point.coords.dot(axis2),
                ))
            }

            // … and compute the 2d convex hull.
            let idx = transformation::convex_hull2_idx(&subspace_points[..]);

            // Finalize the result, triangulating the polyline.
            let npoints = idx.len();
            let coords = idx.into_iter().map(|i| points[i].clone()).collect();
            let mut triangles = Vec::with_capacity(npoints + npoints - 4);

            let a = 0u32;

            for id in 1u32..npoints as u32 - 1 {
                triangles.push(Point3::new(a, id, id + 1));
                triangles.push(Point3::new(id, a, id + 1));
            }

            InitialMesh::ResultMesh(TriMesh::new(
                coords,
                None,
                None,
                Some(IndexBuffer::Unified(triangles)),
            ))
        }
        3 => {
            // The hull is a polyedra.
            // Find a initial triangle lying on the principal plane…
            let _1: N = na::one();
            let diag = Vector3::new(_1 / eigval[0], _1 / eigval[1], _1 / eigval[2]);
            let diag = Matrix3::from_diagonal(&diag);
            let icov = eigvec * diag * eigvec.transpose();

            for point in points.iter_mut() {
                *point = Point3::origin() + icov * point.coords;
            }

            let p1 = support_point_id(&eigpairs[0].0, points).unwrap();
            let p2 = support_point_id(&-eigpairs[0].0, points).unwrap();

            let mut max_area = na::zero();
            let mut p3 = usize::max_value();

            for (i, point) in points.iter().enumerate() {
                let area = utils::triangle_area(&points[p1], &points[p2], point);

                if area > max_area {
                    max_area = area;
                    p3 = i;
                }
            }

            assert!(
                p3 != usize::max_value(),
                "Internal convex hull error: no triangle found."
            );

            // Build two facets with opposite normals
            let mut f1 = TriangleFacet::new(p1, p2, p3, points);
            let mut f2 = TriangleFacet::new(p2, p1, p3, points);

            // Link the facets together
            f1.set_facets_adjascency(1, 1, 1, 0, 2, 1);
            f2.set_facets_adjascency(0, 0, 0, 0, 2, 1);

            let mut facets = vec![f1, f2];

            // … and attribute visible points to each one of them.
            // FIXME: refactor this with the two others.
            let mut ignored = 0usize;
            for point in 0..points.len() {
                if point == p1 || point == p2 || point == p3 {
                    continue;
                }

                let mut furthest = usize::max_value();
                let mut furthest_dist = na::zero();

                for (i, curr_facet) in facets.iter().enumerate() {
                    if curr_facet.can_be_seen_by(point, points) {
                        let distance = curr_facet.distance_to_point(point, points);

                        if distance > furthest_dist {
                            furthest = i;
                            furthest_dist = distance;
                        }
                    }
                }

                if furthest != usize::max_value() {
                    facets[furthest].add_visible_point(point, points);
                } else {
                    undecidable.push(point);
                    ignored = ignored + 1;
                }

                // If none of the facet can be seen from the point, it is naturally deleted.
            }

            verify_facet_links(0, &facets[..]);
            verify_facet_links(1, &facets[..]);

            InitialMesh::Facets(facets, cov_mat)
        }
        _ => unreachable!(),
    }
}

fn compute_silhouette<N: Real>(
    facet: usize,
    indirect_id: usize,
    point: usize,
    out_facets: &mut Vec<usize>,
    out_adj_idx: &mut Vec<usize>,
    points: &[Point3<N>],
    removed_facets: &mut Vec<usize>,
    triangles: &mut [TriangleFacet<N>],
)
{
    if triangles[facet].valid {
        if !triangles[facet].can_be_seen_by_or_is_affinely_dependent_with_contour(
            point,
            points,
            indirect_id,
        ) {
            out_facets.push(facet);
            out_adj_idx.push(indirect_id);
        } else {
            triangles[facet].valid = false; // The facet must be removed from the convex hull.
            removed_facets.push(facet);

            compute_silhouette(
                triangles[facet].adj[(indirect_id + 1) % 3],
                triangles[facet].indirect_adj_id[(indirect_id + 1) % 3],
                point,
                out_facets,
                out_adj_idx,
                points,
                removed_facets,
                triangles,
            );
            compute_silhouette(
                triangles[facet].adj[(indirect_id + 2) % 3],
                triangles[facet].indirect_adj_id[(indirect_id + 2) % 3],
                point,
                out_facets,
                out_adj_idx,
                points,
                removed_facets,
                triangles,
            );
        }
    }
}

fn verify_facet_links<N: Real>(ifacet: usize, facets: &[TriangleFacet<N>]) {
    let facet = &facets[ifacet];

    for i in 0usize..3 {
        let adji = &facets[facet.adj[i]];

        assert!(
            adji.adj[facet.indirect_adj_id[i]] == ifacet
                && adji.first_point_from_edge(facet.indirect_adj_id[i])
                    == facet.second_point_from_edge(adji.indirect_adj_id[facet.indirect_adj_id[i]])
                && adji.second_point_from_edge(facet.indirect_adj_id[i])
                    == facet.first_point_from_edge(adji.indirect_adj_id[facet.indirect_adj_id[i]])
        )
    }
}

fn attach_and_push_facets3<N: Real>(
    horizon_loop_facets: &[usize],
    horizon_loop_ids: &[usize],
    point: usize,
    points: &[Point3<N>],
    triangles: &mut Vec<TriangleFacet<N>>,
    removed_facets: &[usize],
    undecidable: &mut Vec<usize>,
)
{
    // The horizon is built to be in CCW order.
    let mut new_facets = Vec::with_capacity(horizon_loop_facets.len());

    // Create new facets.
    let mut adj_facet: usize;
    let mut indirect_id: usize;

    for i in 0..horizon_loop_facets.len() {
        adj_facet = horizon_loop_facets[i];
        indirect_id = horizon_loop_ids[i];

        let facet = TriangleFacet::new(
            point,
            triangles[adj_facet].second_point_from_edge(indirect_id),
            triangles[adj_facet].first_point_from_edge(indirect_id),
            points,
        );
        new_facets.push(facet);
    }

    // Link the facets together.
    for i in 0..horizon_loop_facets.len() {
        let prev_facet;

        if i == 0 {
            prev_facet = triangles.len() + horizon_loop_facets.len() - 1;
        } else {
            prev_facet = triangles.len() + i - 1;
        }

        let middle_facet = horizon_loop_facets[i];
        let next_facet = triangles.len() + (i + 1) % horizon_loop_facets.len();
        let middle_id = horizon_loop_ids[i];

        new_facets[i].set_facets_adjascency(prev_facet, middle_facet, next_facet, 2, middle_id, 0);
        triangles[middle_facet].adj[middle_id] = triangles.len() + i; // The future id of curr_facet.
        triangles[middle_facet].indirect_adj_id[middle_id] = 1;
    }

    // Assign to each facets some of the points which can see it.
    // FIXME: refactor this with the others.
    for curr_facet in removed_facets.iter() {
        for visible_point in triangles[*curr_facet].visible_points.iter() {
            if *visible_point == point {
                continue;
            }

            let mut furthest = usize::max_value();
            let mut furthest_dist = na::zero();

            for (i, curr_facet) in new_facets.iter_mut().enumerate() {
                if curr_facet.can_be_seen_by(*visible_point, points) {
                    let distance = curr_facet.distance_to_point(*visible_point, points);

                    if distance > furthest_dist {
                        furthest = i;
                        furthest_dist = distance;
                    }
                }
            }

            if furthest != usize::max_value() {
                new_facets[furthest].add_visible_point(*visible_point, points);
            }

            // If none of the facet can be seen from the point, it is naturally deleted.
        }
    }

    // Try to assign collinear points to one of the new facets.
    let mut i = 0;

    while i != undecidable.len() {
        let mut furthest = usize::max_value();
        let mut furthest_dist = na::zero();
        let undecidable_point = undecidable[i];

        for (j, curr_facet) in new_facets.iter_mut().enumerate() {
            if curr_facet.can_be_seen_by(undecidable_point, points) {
                let distance = curr_facet.distance_to_point(undecidable_point, points);

                if distance > furthest_dist {
                    furthest = j;
                    furthest_dist = distance;
                }
            }
        }

        if furthest != usize::max_value() {
            new_facets[furthest].add_visible_point(undecidable_point, points);
            let _ = undecidable.swap_remove(i);
        } else {
            i = i + 1;
        }
    }

    // Push facets.
    // FIXME: can we avoid the tmp vector `new_facets` ?
    for curr_facet in new_facets.into_iter() {
        triangles.push(curr_facet);
    }
}

struct TriangleFacet<N: Real> {
    valid: bool,
    normal: Vector3<N>,
    adj: [usize; 3],
    indirect_adj_id: [usize; 3],
    pts: [usize; 3],
    visible_points: Vec<usize>,
    furthest_point: usize,
    furthest_distance: N,
}

impl<N: Real> TriangleFacet<N> {
    pub fn new(p1: usize, p2: usize, p3: usize, points: &[Point3<N>]) -> TriangleFacet<N> {
        let p1p2 = points[p2] - points[p1];
        let p1p3 = points[p3] - points[p1];

        let mut normal = p1p2.cross(&p1p3);
        if normal.normalize_mut().is_zero() {
            panic!("ConvexHull hull failure: a facet must not be affinely dependent.");
        }

        TriangleFacet {
            valid: true,
            normal: normal,
            adj: [0, 0, 0],
            indirect_adj_id: [0, 0, 0],
            pts: [p1, p2, p3],
            visible_points: Vec::new(),
            furthest_point: Bounded::max_value(),
            furthest_distance: na::zero(),
        }
    }

    pub fn add_visible_point(&mut self, pid: usize, points: &[Point3<N>]) {
        let distance = self.distance_to_point(pid, points);

        if distance > self.furthest_distance {
            self.furthest_distance = distance;
            self.furthest_point = pid;
        }

        self.visible_points.push(pid);
    }

    pub fn distance_to_point(&self, point: usize, points: &[Point3<N>]) -> N {
        self.normal.dot(&(points[point] - points[self.pts[0]]))
    }

    pub fn set_facets_adjascency(
        &mut self,
        adj1: usize,
        adj2: usize,
        adj3: usize,
        id_adj1: usize,
        id_adj2: usize,
        id_adj3: usize,
    )
    {
        self.indirect_adj_id[0] = id_adj1;
        self.indirect_adj_id[1] = id_adj2;
        self.indirect_adj_id[2] = id_adj3;

        self.adj[0] = adj1;
        self.adj[1] = adj2;
        self.adj[2] = adj3;
    }

    pub fn first_point_from_edge(&self, id: usize) -> usize {
        self.pts[id]
    }

    pub fn second_point_from_edge(&self, id: usize) -> usize {
        self.pts[(id + 1) % 3]
    }

    pub fn can_be_seen_by(&self, point: usize, points: &[Point3<N>]) -> bool {
        let p0 = &points[self.pts[0]];
        let p1 = &points[self.pts[1]];
        let p2 = &points[self.pts[2]];
        let pt = &points[point];

        let _eps = N::default_epsilon();

        (*pt - *p0).dot(&self.normal) > _eps * na::convert(100.0f64)
            && !utils::is_affinely_dependent_triangle(p0, p1, pt)
            && !utils::is_affinely_dependent_triangle(p0, p2, pt)
            && !utils::is_affinely_dependent_triangle(p1, p2, pt)
    }

    pub fn can_be_seen_by_or_is_affinely_dependent_with_contour(
        &self,
        point: usize,
        points: &[Point3<N>],
        edge: usize,
    ) -> bool
    {
        let p0 = &points[self.first_point_from_edge(edge)];
        let p1 = &points[self.second_point_from_edge(edge)];
        let pt = &points[point];

        let aff_dep = utils::is_affinely_dependent_triangle(p0, p1, pt)
            || utils::is_affinely_dependent_triangle(p0, pt, p1)
            || utils::is_affinely_dependent_triangle(p1, p0, pt)
            || utils::is_affinely_dependent_triangle(p1, pt, p0)
            || utils::is_affinely_dependent_triangle(pt, p0, p1)
            || utils::is_affinely_dependent_triangle(pt, p1, p0);

        (*pt - *p0).dot(&self.normal) >= na::zero() || aff_dep
    }
}

#[cfg(test)]
mod test {
    #[cfg(feature = "dim2")]
    use na::Point2;
    #[cfg(feature = "dim3")]
    use crate::procedural;
    use crate::transformation;

    #[cfg(feature = "dim2")]
    #[test]
    fn test_simple_convex_hull() {
        let points = [
            Point2::new(4.723881f32, 3.597233),
            Point2::new(3.333363, 3.429991),
            Point2::new(3.137215, 2.812263),
        ];

        let chull = transformation::convex_hull(points.as_slice());

        assert!(chull.coords.len() == 3);
    }

    #[cfg(feature = "dim3")]
    #[test]
    fn test_ball_convex_hull() {
        // This trigerred a failure to an affinely dependent facet.
        let sphere = procedural::sphere(0.4f32, 20, 20, true);
        let points = sphere.coords;
        let chull = transformation::convex_hull(points.as_slice());

        // dummy test, we are just checking that the construction did not fail.
        assert!(chull.coords.len() == chull.coords.len());
    }
}
