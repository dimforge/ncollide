use na::Point2;

use ncollide2d::utils;

#[test]
pub fn disjoint_no_intersection() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(1.0, 0.0),
        Point2::new(1.1, 0.0),
        Point2::new(1.1, 0.1),
        Point2::new(1.0, 0.1),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert_eq!(res.len(), 0);
}

#[test]
pub fn full_inclusion() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.025, 0.025),
        Point2::new(0.075, 0.025),
        Point2::new(0.075, 0.075),
        Point2::new(0.025, 0.075),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.025, 0.025)));
    assert!(res.contains(&Point2::new(0.075, 0.025)));
    assert!(res.contains(&Point2::new(0.075, 0.075)));
    assert!(res.contains(&Point2::new(0.025, 0.075)));
    assert_eq!(res.len(), 4);
}

#[test]
pub fn two_edges_overlap() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.05, 0.05),
        Point2::new(0.1, 0.05),
        Point2::new(0.1, 0.1),
        Point2::new(0.05, 0.1),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.1, 0.05)));
    assert!(res.contains(&Point2::new(0.1, 0.1)));
    assert!(res.contains(&Point2::new(0.05, 0.1)));
    assert_eq!(res.len(), 3);
}

#[test]
pub fn quadrilateral_intersection() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.3, 0.0),
        Point2::new(0.3, 0.3),
        Point2::new(0.0, 0.3),
    ];

    let poly2 = [
        Point2::new(0.1, 0.4),
        Point2::new(0.5, 0.1),
        Point2::new(0.2, -0.1),
        Point2::new(0.1, 0.2),
    ];

    /*
    let poly2 = [
        Point2::new(0.1, 0.2),
        Point2::new(0.2, -0.1),
        Point2::new(0.5, 0.1),
        Point2::new(0.1, 0.4),
    ];*/

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    /*
    Point2::new(0.16666666666666666,0.0),
    Point2::new(0.3, 0.0)
    Point2::new(0.3, 0.25)
    Point2::new(0.23333333333333336, 0.3)
    Point2::new(0.09999999999999998, 0.3)
    Point2::new(0.1, 0.2)
    */
    println!("{:?}", res);
    assert_eq!(res.len(), 6);
    assert!(false);
}

#[test]
pub fn single_segment_intersection() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.0 + 0.1, 0.025),
        Point2::new(0.1 + 0.1, 0.025),
        Point2::new(0.1 + 0.1, 0.075),
        Point2::new(0.0 + 0.1, 0.075),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.1, 0.025)));
    assert!(res.contains(&Point2::new(0.1, 0.075)));
    assert_eq!(res.len(), 2);
}

#[test]
pub fn single_edge_intersection() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.0 + 0.1, 0.0),
        Point2::new(0.1 + 0.1, 0.0),
        Point2::new(0.1 + 0.1, 0.1),
        Point2::new(0.0 + 0.1, 0.1),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.1, 0.0)));
    assert!(res.contains(&Point2::new(0.1, 0.1)));
    assert_eq!(res.len(), 2);
}

#[test]
pub fn intersect_vertices_only() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.05, 0.0),
        Point2::new(0.1, 0.05),
        Point2::new(0.05, 0.1),
        Point2::new(0.0, 0.05),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.05, 0.0)));
    assert!(res.contains(&Point2::new(0.1, 0.05)));
    assert!(res.contains(&Point2::new(0.05, 0.1)));
    assert!(res.contains(&Point2::new(0.0, 0.05)));
    assert_eq!(res.len(), 4);
}

#[test]
pub fn partial_overlap() {
    let mut res = Vec::new();
    let poly1 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    let poly2 = [
        Point2::new(0.0, 0.0),
        Point2::new(0.05, 0.0),
        Point2::new(0.05, 0.1),
        Point2::new(0.0, 0.1),
    ];

    utils::convex_polygons_intersection_points(&poly1, &poly2, &mut res);

    assert!(res.contains(&Point2::new(0.0, 0.0)));
    assert!(res.contains(&Point2::new(0.05, 0.0)));
    assert!(res.contains(&Point2::new(0.05, 0.1)));
    assert!(res.contains(&Point2::new(0.0, 0.1)));
    assert_eq!(res.len(), 4);
}

#[test]
pub fn full_overlap() {
    let mut res = Vec::new();
    let poly = [
        Point2::new(0.0, 0.0),
        Point2::new(0.1, 0.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.1),
    ];

    utils::convex_polygons_intersection_points(&poly, &poly, &mut res);

    assert!(res.contains(&Point2::new(0.0, 0.0)));
    assert!(res.contains(&Point2::new(0.1, 0.0)));
    assert!(res.contains(&Point2::new(0.1, 0.1)));
    assert!(res.contains(&Point2::new(0.0, 0.1)));
    assert_eq!(res.len(), 4);
}
