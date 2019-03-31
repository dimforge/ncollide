use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::Color;

use alga::general::RealField;
use draw_helper::draw_line;
use na::{Isometry2, Point2, Point3};
use ncollide2d::world::CollisionObject;

pub struct Lines<N: RealField> {
    color: Point3<u8>,
    base_color: Point3<u8>,
    delta: Isometry2<N>,
    vertices: Vec<Point2<N>>,
}

impl<N: RealField> Lines<N> {
    pub fn new(delta: Isometry2<N>, vertices: Vec<Point2<N>>, color: Point3<u8>) -> Lines<N> {
        Lines {
            color: color,
            base_color: color,
            delta: delta,
            vertices: vertices,
        }
    }
}

impl<N: RealField + ToPrimitive> Lines<N> {
    pub fn update(&mut self) {}

    pub fn draw<T>(&self, rw: &mut graphics::RenderWindow, object: &CollisionObject<N, T>) {
        let transform = object.position() * self.delta;
        let color = Color::new_rgb(self.color.x, self.color.y, self.color.z);

        for v in self.vertices.windows(2) {
            let gsv0 = transform * v[0];
            let gsv1 = transform * v[1];
            draw_line(rw, &gsv0, &gsv1, &color);
        }
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        self.color = color;
        self.base_color = color;
    }

    pub fn select(&mut self) {
        self.color = Point3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
