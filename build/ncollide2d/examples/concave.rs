use gnuplot::*;
use ncollide2d::math::Point;
use ncollide2d::nalgebra::geometry::Isometry;
use ncollide2d::query::PointQuery;
use ncollide2d::shape::{Compound, ConvexPolygon};

fn main() {
    let hull = [
        (5., 1.),
        (4., 0.),
        (4.5, 1.),
        (3.5, 1.5),
        (3., 0.),
        (2., 1.),
        (1., 0.),
        (0., 1.),
        (0.5, 1.5),
        (0., 2.),
        (1., 3.),
        (2., 3.),
        (4.25, 3.25),
        (4.25, 2.25),
        (4., 3.),
        (4., 2.),
        (5., 2.),
        (5., 1.),
    ];

    let polygon = Compound::new_concave_polygon(Isometry::identity(), &hull);

    let mut fg = Figure::new();
    let mut axe = fg.axes2d();

    display_polygon(&mut axe, &polygon);
    display_hull(&mut axe, &hull);

    // Points inside the concave hull
    must_be_inside(&mut axe, &polygon, &[(2., 2.)]);

    // Points at the edge of the hull
    must_be_inside(&mut axe, &polygon, &[(5., 1.), (2., 1.)]);

    // Points outside the convex hull
    must_be_outside(&mut axe, &polygon, &[(-1., -1.), (3., 3.2), (4.5, 3.)]);

    // Points outside the concave hull, and the convex hull
    must_be_outside(
        &mut axe,
        &polygon,
        &[
            (0.25, 0.25),
            (0.25, 1.5),
            (2., 0.5),
            (3.5, 0.5),
            (3.5, 1.),
            (3.5, 1.25),
            (4.25, 0.75),
            (4.5, 2.5),
            (4.25, 2.1),
            (4.1, 2.3),
        ],
    );

    // Points inside the concave hull
    must_be_inside(
        &mut axe,
        &polygon,
        &[
            (4.5, 0.75),
            (2.75, 0.75),
            (1., 0.75),
            (4.75, 1.75),
            (4.1, 2.75),
            (4.1, 3.1),
            (3.5, 3.1),
        ],
    );

    fg.show().unwrap();
}

/// Asserts that all points are inside the polygon, and displays them in green
fn must_be_inside(axe: &mut gnuplot::Axes2D, polygon: &Compound<f64>, points: &[(f64, f64)]) {
    axe.points(
        points.iter().map(|(x, _y)| *x),
        points.iter().map(|(_x, y)| *y),
        &[Color("green")],
    );
    for &(x, y) in points {
        assert!(polygon.contains_point(&Isometry::identity(), &Point::new(x, y)));
    }
}

/// Asserts that all points are inside the polygon, and displays them in red
fn must_be_outside(axe: &mut gnuplot::Axes2D, polygon: &Compound<f64>, points: &[(f64, f64)]) {
    axe.points(
        points.iter().map(|(x, _y)| *x),
        points.iter().map(|(_x, y)| *y),
        &[Color("red")],
    );
    for &(x, y) in points {
        assert!(!polygon.contains_point(&Isometry::identity(), &Point::new(x, y)));
    }
}

/// Displays all the triangles contained inside the concave polygon in orange
fn display_polygon(axe: &mut gnuplot::Axes2D, polygon: &Compound<f64>) {
    for (_isometry, triangle) in polygon.shapes() {
        let triangle = triangle.as_shape::<ConvexPolygon<f64>>().unwrap();
        let p = triangle.points();
        assert!(p.len() == 3);
        let points = [p[0], p[1], p[2], p[0]];
        axe.lines(
            points.iter().map(|point| point.iter().nth(0).unwrap()),
            points.iter().map(|point| point.iter().nth(1).unwrap()),
            &[Color("orange"), LineWidth(5.)],
        );
    }
}

/// Displays the hull in black
fn display_hull(axe: &mut gnuplot::Axes2D, hull: &[(f64, f64)]) {
    axe.lines(
        hull.iter().map(|(x, _y)| *x),
        hull.iter().map(|(_x, y)| *y),
        &[Color("black")],
    );
}
