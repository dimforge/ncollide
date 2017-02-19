use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::Color;

use alga::general::Real;
use na::{Point2, Point3, Isometry2};
use ncollide::world::CollisionObject2;
use draw_helper::draw_line;

pub struct Segment<N: Real> {
    color:      Point3<u8>,
    base_color: Point3<u8>,
    delta:      Isometry2<N>,
    a:          Point2<N>,
    b:          Point2<N>,
}

impl<N: Real + ToPrimitive> Segment<N> {
    pub fn new(delta:  Isometry2<N>,
               a:      Point2<N>,
               b:      Point2<N>,
               color:  Point3<u8>)
               -> Segment<N> {
        Segment {
            color:      color,
            base_color: color,
            delta:      delta,
            a:          a,
            b:          b
        }
    }
}

impl<N: Real + ToPrimitive> Segment<N> {
    pub fn update(&mut self) {
    }

    pub fn draw<T>(&self, rw: &mut graphics::RenderWindow, object: &CollisionObject2<N, T>) {
        let transform = object.position * self.delta;
        let color     = Color::new_rgb(self.color.x, self.color.y, self.color.z);

        let ga = transform * self.a;
        let gb = transform * self.b;
        draw_line(rw, &ga, &gb, &color);
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
