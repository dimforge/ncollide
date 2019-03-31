use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::{Color, RenderTarget, Vertex, VertexArray};
use sfml::system::Vector2f;

use alga::general::RealField;
use na;
use na::Point2;
use ncollide2d::world::CollisionWorld;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls<N: RealField + ToPrimitive, T>(
    window: &mut graphics::RenderWindow,
    world: &mut CollisionWorld<N, T>,
) {
    for m in world.contact_manifolds() {
        for c in m.2.contacts() {
            draw_line(
                window,
                &c.contact.world1,
                &c.contact.world2,
                &Color::new_rgb(255, 255, 255),
            );

            let center = na::center(&c.contact.world1, &c.contact.world2);
            draw_line(
                window,
                &center,
                &(center + c.contact.normal.as_ref() * c.contact.depth),
                &Color::new_rgb(255, 0, 0),
            );

            draw_line(
                window,
                &center,
                &(center + c.contact.normal.unwrap()),
                &Color::new_rgb(0, 0, 255),
            );
        }
    }
}

pub fn draw_line<N: RealField + ToPrimitive>(
    window: &mut graphics::RenderWindow,
    v1: &Point2<N>,
    v2: &Point2<N>,
    color: &Color,
) {
    let mut vertices = VertexArray::new().unwrap();

    vertices.append(&Vertex::new(
        &Vector2f::new(
            v1.x.to_f32().unwrap() * DRAW_SCALE,
            v1.y.to_f32().unwrap() * DRAW_SCALE,
        ),
        color,
        &Vector2f::new(0.0, 0.0),
    ));

    vertices.append(&Vertex::new(
        &Vector2f::new(
            v2.x.to_f32().unwrap() * DRAW_SCALE,
            v2.y.to_f32().unwrap() * DRAW_SCALE,
        ),
        color,
        &Vector2f::new(0.0, 0.0),
    ));

    vertices.set_primitive_type(graphics::Lines);

    window.draw(&vertices);
}
