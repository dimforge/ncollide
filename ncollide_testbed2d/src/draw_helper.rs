use num::ToPrimitive;
use sfml::graphics;
use sfml::graphics::{Color, RenderTarget, Vertex, VertexArray};
use sfml::system::Vector2f;

use alga::general::Real;
use na::Point2;
use na;
use ncollide::world::CollisionWorld2;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls<N: Real + ToPrimitive, T>(
    window: &mut graphics::RenderWindow,
    world: &mut CollisionWorld2<N, T>,
) {
    for c in world.contacts() {
        draw_line(
            window,
            &c.2.world1,
            &c.2.world2,
            &Color::new_rgb(255, 255, 255),
        );

        let center = na::center(&c.2.world1, &c.2.world2);
        draw_line(
            window,
            &center,
            &(center + c.2.normal * c.2.depth),
            &Color::new_rgb(255, 0, 0),
        );

        draw_line(
            window,
            &center,
            &(center + c.2.normal),
            &Color::new_rgb(0, 0, 255),
        );
    }
}

pub fn draw_line<N: Real + ToPrimitive>(
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
