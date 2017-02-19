use std::sync::Arc;
use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::Color;

use alga::general::Real;
use na::{Point2, Point3, Isometry2};
use ncollide::world::CollisionObject2;
use draw_helper::draw_line;

pub struct Lines<N: Real> {
    color:      Point3<u8>,
    base_color: Point3<u8>,
    delta:      Isometry2<N>,
    indices:    Arc<Vec<Point2<usize>>>,
    vertices:   Arc<Vec<Point2<N>>>
}

impl<N: Real> Lines<N> {
    pub fn new(delta:    Isometry2<N>,
               vertices: Arc<Vec<Point2<N>>>,
               indices:  Arc<Vec<Point2<usize>>>,
               color:    Point3<u8>)
               -> Lines<N> {
        Lines {
            color:      color,
            base_color: color,
            delta:      delta,
            vertices:   vertices,
            indices:    indices
        }
    }
}

impl<N: Real + ToPrimitive> Lines<N> {
    pub fn update(&mut self) {
    }

    pub fn draw<T>(&self, rw: &mut graphics::RenderWindow, object: &CollisionObject2<N, T>) {
        let transform = object.position * self.delta;
        let color     = Color::new_rgb(self.color.x, self.color.y, self.color.z);

        let vs = &*self.vertices;

        for is in self.indices.iter() {
            let gsv0 = transform * vs[is.x];
            let gsv1 = transform * vs[is.y];
            draw_line(rw, &gsv0, &gsv1, &color);
        }
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        self.color      = color;
        self.base_color = color;
    }

    pub fn select(&mut self) {
        self.color = Point3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
