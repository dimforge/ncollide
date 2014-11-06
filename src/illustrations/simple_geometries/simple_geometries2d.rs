#![allow(unused_variables)]
#![allow(unused_mut)]

extern crate rsfml;
extern crate "nalgebra" as na;
extern crate ncollide;

use rsfml::system::Vector2f;
use rsfml::window::{ContextSettings, Close, VideoMode};
use rsfml::graphics::{RenderWindow, RenderTarget, ConvexShape, Color};
use na::{Pnt2, Vec2, Identity};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder, Reflection, MinkowskiSum};
use ncollide::procedural::{Polyline, ToPolyline};
use ncollide::procedural;

static SZ: uint = 200;

fn main () -> () {
    // Create a new RenderWindow.
    let mut setting = ContextSettings::default();
    setting.antialiasing_level = 12;

    let mut window = match RenderWindow::new(VideoMode::new_init(SZ, SZ, 32), "SFML Short Example", Close, &setting) {
        Some(window) => window,
        None         => panic!("Cannot create a new Render Window.")
    };

    let mut polyline = Ball::new(1.0).to_polyline(1000);
    draw_save(&mut window, polyline, true, "../../../img/ball2d.png");

    let mut polyline = Cuboid::new(Vec2::new(1.5, 1.0)).to_polyline(());
    draw_save(&mut window, polyline, true, "../../../img/cuboid2d.png");

    let mut polyline = Capsule::new(0.5, 0.75).to_polyline(1000);
    draw_save(&mut window, polyline, true, "../../../img/capsule2d.png");

    let cone     = Cone::new(0.75, 0.75);
    let cylinder = Cylinder::new(0.5, 0.75);
    let rcone    = Reflection::new(&cone);

    let cone2    = Cone::new(0.5, 0.75);
    let rcone2   = Reflection::new(&cone2);

    let mut polyline = rcone2.to_polyline(());
    draw_save(&mut window, polyline, true, "../../../img/refl2d.png");

    let mut polyline = MinkowskiSum::new(&Identity::new(), &cylinder, &Identity::new(), &cone).to_polyline(((), ()));
    draw_save(&mut window, polyline, true, "../../../img/msum2d.png");

    let mut polyline = MinkowskiSum::new(&Identity::new(), &cylinder, &Identity::new(), &rcone).to_polyline(((), ()));
    draw_save(&mut window, polyline, true, "../../../img/cso2d.png");


    /*
     * Here come the more complicate shapes.
     */

    // Compound
    let mut poly1 = Cuboid::new(Vec2::new(0.25f32, 1.5)).to_polyline(());
    poly1.translate_by(&Vec2::new(1.5f32, 0.0));

    let mut poly2 = Cuboid::new(Vec2::new(0.25f32, 1.5)).to_polyline(());
    poly2.translate_by(&Vec2::new(-1.5f32, 0.0));

    let mut poly3 = Cuboid::new(Vec2::new(1.5f32, 0.25)).to_polyline(());
    poly3.translate_by(&Vec2::new(0.0f32, -1.5));

    window.clear(&Color::new_RGB(255, 255, 255));
    draw_polyline(&mut window, poly1, true);
    draw_polyline(&mut window, poly2, true);
    draw_polyline(&mut window, poly3, true);
    let capture = window.capture().unwrap();
    capture.save_to_file("../../../img/compound2d.png");

    // Convex
    let pts = vec!(
        Pnt2::new(-1.0, 1.0), Pnt2::new(-0.5, -0.5),
        Pnt2::new(0.0, 0.5),  Pnt2::new(0.5, -0.5),
        Pnt2::new(1.0, 1.0));

    let polyline = procedural::convex_hull2(pts.as_slice());
    draw_save(&mut window, polyline, false, "../../../img/convex2d.png");

    // Mesh
    let pts = vec!(
        Pnt2::new(0.0, 1.0),  Pnt2::new(-1.0, -1.0),
        Pnt2::new(0.0, -0.5), Pnt2::new(1.0, -1.0));

    let polyline = Polyline::new(pts, None);
    draw_save(&mut window, polyline, false, "../../../img/mesh2d.png");

}

fn draw_save(window: &mut RenderWindow, polyline: Polyline<f32, Pnt2<f32>, Vec2<f32>>, fill: bool, out: &str) {
    window.clear(&Color::new_RGB(255, 255, 255));
    draw_polyline(window, polyline, fill);
    let capture = window.capture().unwrap();
    capture.save_to_file(out);
}

fn draw_polyline(window: &mut RenderWindow, mut polyline: Polyline<f32, Pnt2<f32>, Vec2<f32>>, fill: bool) {
    polyline.scale_by_scalar(&50.0);

    let mut convex = match ConvexShape::new(polyline.coords.len()) {
        Some(convex)  => convex,
        None          => panic!("Error, cannot create a Circle Shape.")
    };

    for (i, c) in polyline.coords.iter().enumerate() {
        convex.set_point(i, &Vector2f::new(c.x, -c.y));
    }

    convex.set_outline_thickness(5.0);
    convex.set_outline_color(&Color::new_RGB(15, 109, 255));

    if fill {
        convex.set_fill_color(&Color::new_RGB(25, 183, 255));
    }

    convex.set_position(&Vector2f::new(SZ as f32 / 2.0, SZ as f32 / 2.0));

    window.draw(&convex);
}
