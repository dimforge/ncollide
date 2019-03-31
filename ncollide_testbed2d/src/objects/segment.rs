use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::Color;

use alga::general::RealField;
use draw_helper::draw_line;
use na::{Isometry2, Point2, Point3};
use ncollide2d::world::CollisionObject;

pub struct Segment<N: RealField> {
    color: Point3<u8>,
    base_color: Point3<u8>,
    delta: Isometry2<N>,
    a: Point2<N>,
    b: Point2<N>,
}

impl<N: RealField + ToPrimitive> Segment<N> {
    pub fn new(delta: Isometry2<N>, a: Point2<N>, b: Point2<N>, color: Point3<u8>) -> Segment<N> {
        Segment {
            color: color,
            base_color: color,
            delta: delta,
            a: a,
            b: b,
        }
    }
}

impl<N: RealField + ToPrimitive> Segment<N> {
    pub fn update(&mut self) {}

    pub fn draw<T>(&self, rw: &mut graphics::RenderWindow, object: &CollisionObject<N, T>) {
        let transform = object.position() * self.delta;
        let color = Color::new_rgb(self.color.x, self.color.y, self.color.z);

        let ga = transform * self.a;
        let gb = transform * self.b;
        draw_line(rw, &ga, &gb, &color);
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
