#![allow(unused_variables)]
#![allow(unused_mut)]

extern crate sfml;
extern crate nalgebra as na;
extern crate ncollide;

use sfml::system::Vector2f;
use sfml::window::{ContextSettings, VideoMode};
use sfml::window::window_style;
use sfml::graphics::{Shape, Transformable, RenderWindow, RenderTarget, ConvexShape, Color};
use na::{Point2, Vector2, Identity};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder, Reflection, MinkowskiSum};
use ncollide::procedural::Polyline;
use ncollide::transformation::{self, ToPolyline};

static SZ: u32 = 200;

fn main () -> () {
    // Create a new RenderWindow.
    let mut setting = ContextSettings::default();

    let mode = VideoMode::new_init(SZ, SZ, 32);
    let mut window = match RenderWindow::new(mode, "SFML Short Example", window_style::CLOSE, &setting) {
        Some(window) => window,
        None         => panic!("Cannot create a new Render Window.")
    };

    let mut polyline = Ball::new(1.0).to_polyline(1000);
    draw_save(&mut window, polyline, true, "./out/ball2d.png");

    let mut polyline = Cuboid::new(Vector2::new(1.5, 1.0)).to_polyline(());
    draw_save(&mut window, polyline, true, "./out/cuboid2d.png");

    let mut polyline = Capsule::new(0.5, 0.75).to_polyline(1000);
    draw_save(&mut window, polyline, true, "./out/capsule2d.png");

    let cone     = Cone::new(0.75, 0.75);
    let cylinder = Cylinder::new(0.5, 0.75);
    let rcone    = Reflection::new(&cone);

    let cone2    = Cone::new(0.5, 0.75);
    let rcone2   = Reflection::new(&cone2);

    let mut polyline = rcone2.to_polyline(());
    draw_save(&mut window, polyline, true, "./out/refl2d.png");

    let mut polyline = MinkowskiSum::new(&Identity::new(), &cylinder, &Identity::new(), &cone).to_polyline(((), ()));
    draw_save(&mut window, polyline, true, "./out/msum2d.png");

    let mut polyline = MinkowskiSum::new(&Identity::new(), &cylinder, &Identity::new(), &rcone).to_polyline(((), ()));
    draw_save(&mut window, polyline, true, "./out/cso2d.png");


    /*
     * Here come the more complicate shapes.
     */

    // Compound
    let mut poly1 = Cuboid::new(Vector2::new(0.25f32, 1.5)).to_polyline(());
    poly1.translate_by(&Vector2::new(1.5f32, 0.0));

    let mut poly2 = Cuboid::new(Vector2::new(0.25f32, 1.5)).to_polyline(());
    poly2.translate_by(&Vector2::new(-1.5f32, 0.0));

    let mut poly3 = Cuboid::new(Vector2::new(1.5f32, 0.25)).to_polyline(());
    poly3.translate_by(&Vector2::new(0.0f32, -1.5));

    window.clear(&Color::new_rgb(255, 255, 255));
    draw_polyline(&mut window, poly1, true);
    draw_polyline(&mut window, poly2, true);
    draw_polyline(&mut window, poly3, true);
    let capture = window.capture().unwrap();
    capture.save_to_file("./out/compound2d.png");

    // Convex
    let pts = vec!(
        Point2::new(-1.0, 1.0), Point2::new(-0.5, -0.5),
        Point2::new(0.0, 0.5),  Point2::new(0.5, -0.5),
        Point2::new(1.0, 1.0));

    let polyline = transformation::convex_hull2(pts.as_slice());
    draw_save(&mut window, polyline, false, "./out/convex2d.png");

    // Mesh
    let pts = vec!(
        Point2::new(0.0, 1.0),  Point2::new(-1.0, -1.0),
        Point2::new(0.0, -0.5), Point2::new(1.0, -1.0));

    let polyline = Polyline::new(pts, None);
    draw_save(&mut window, polyline, false, "./out/mesh2d.png");

}

fn draw_save(window: &mut RenderWindow, polyline: Polyline<Point2<f32>>, fill: bool, out: &str) {
    window.clear(&Color::new_rgb(255, 255, 255));
    draw_polyline(window, polyline, fill);
    let capture = window.capture().unwrap();
    capture.save_to_file(out);
}

fn draw_polyline(window: &mut RenderWindow, mut polyline: Polyline<Point2<f32>>, fill: bool) {
    polyline.scale_by_scalar(&50.0);

    let mut convex = match ConvexShape::new(polyline.coords().len() as u32) {
        Some(convex)  => convex,
        None          => panic!("Error, cannot create a Circle Shape.")
    };

    for (i, c) in polyline.coords().iter().enumerate() {
        convex.set_point(i as u32, &Vector2f::new(c.x, -c.y));
    }

    convex.set_outline_thickness(5.0);
    convex.set_outline_color(&Color::new_rgb(15, 109, 255));

    if fill {
        convex.set_fill_color(&Color::new_rgb(25, 183, 255));
    }

    convex.set_position(&Vector2f::new(SZ as f32 / 2.0, SZ as f32 / 2.0));

    window.draw(&convex);
}
