use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::{CircleShape, Color, RenderTarget, Shape, Transformable};
use sfml::system::Vector2f;

use alga::general::Real;
use na::{Point3, Isometry2};
use ncollide::world::CollisionObject2;
use draw_helper::DRAW_SCALE;
use objects;

pub struct Ball<'a, N: Real> {
    color:      Point3<u8>,
    base_color: Point3<u8>,
    delta:      Isometry2<N>,
    gfx:        CircleShape<'a>
}

impl<'a, N: Real + ToPrimitive> Ball<'a, N> {
    pub fn new(delta: Isometry2<N>, radius: N, color:  Point3<u8>) -> Ball<'a, N> {
        let dradius = radius.to_f32().unwrap() * DRAW_SCALE;

        let mut res = Ball {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        CircleShape::new().unwrap(),
        };

        res.gfx.set_fill_color(&Color::new_rgb(color.x, color.y, color.z));
        res.gfx.set_radius(dradius);
        res.gfx.set_origin(&Vector2f::new(dradius, dradius)); 

        res
    }
}

impl<'a, N: Real + ToPrimitive> Ball<'a, N> {
    pub fn update<T>(&mut self, object: &CollisionObject2<N, T>) {
        objects::update_scene_node(&mut self.gfx, &object, &self.color, &self.delta)
    }

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        rw.draw(&self.gfx);
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
