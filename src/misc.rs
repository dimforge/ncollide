#![allow(unused_variable)]

extern crate "ncollide3df32" as ncollide;

use ncollide::geom::{Ball, Cylinder, Cone};
use ncollide::narrow::{GeomGeomDispatcher, PlaneImplicit,
                       IncrementalContactManifoldGenerator, OneShotContactManifoldGenerator};


fn main() {
    let dispatcher = GeomGeomDispatcher::new(0.10);
    let geom1 = Ball::new(0.5);
    let geom2 = Cylinder::new(0.5, 1.0);
    let geom3 = Cone::new(0.5, 1.0);

    let ball_vs_cylinder_detector = dispatcher.dispatch(&geom1, &geom2);
    let ball_vs_cone_detector     = dispatcher.dispatch(&geom1, &geom3);
    let cylinder_vs_cone_detector = dispatcher.dispatch(&geom2, &geom3);


    let plane_vs_ball: PlaneImplicit<Ball> = PlaneImplicit::new(0.04);
    let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(0.04, plane_vs_ball);


    let plane_vs_ball: PlaneImplicit<Ball> = PlaneImplicit::new(0.04);
    let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(0.04, plane_vs_ball);
}
