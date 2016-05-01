use std::f32;
use sfml::graphics::{Color, Shape, Transformable};
use sfml::system::Vector2f;
use na::{self, Point3, Isometry2};
use ncollide::world::{CollisionObject2, GeometricQueryType};
use ncollide::math::Scalar;
use objects::{Ball, Box, Lines, Segment};
use draw_helper::DRAW_SCALE;

pub enum SceneNode<'a, N> {
    BallNode(Ball<'a, N>),
    BoxNode(Box<'a, N>),
    LinesNode(Lines<N>),
    SegmentNode(Segment<N>)
}

impl<'a, N: Scalar> SceneNode<'a, N> {
    pub fn select(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.select(),
            SceneNode::BoxNode(ref mut n)     => n.select(),
            SceneNode::LinesNode(ref mut n)   => n.select(),
            SceneNode::SegmentNode(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.unselect(),
            SceneNode::BoxNode(ref mut n)     => n.unselect(),
            SceneNode::LinesNode(ref mut n)   => n.unselect(),
            SceneNode::SegmentNode(ref mut n) => n.unselect(),
        }
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.set_color(color),
            SceneNode::BoxNode(ref mut n)     => n.set_color(color),
            SceneNode::LinesNode(ref mut n)   => n.set_color(color),
            SceneNode::SegmentNode(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node<'a, N: Scalar, T, SN>(node:   &mut SN,
                                               object: &CollisionObject2<N, T>,
                                               color:  &Point3<u8>,
                                               delta:  &Isometry2<N>)
        where SN: Transformable + Shape<'a> {
    let transform = object.position * *delta;
    let pos       = na::translation(&transform);
    let rot       = na::rotation(&transform);

    node.set_position(&Vector2f::new(pos.x.to_f32().unwrap() as f32 * DRAW_SCALE,
                                     pos.y.to_f32().unwrap() as f32 * DRAW_SCALE));
    node.set_rotation(rot.x.to_f32().unwrap() * 180.0 / f32::consts::PI as f32);

    match object.query_type {
        GeometricQueryType::Proximity(_) => {
            // FIXME: what is the impact on performance of doing those at each update ?
            node.set_fill_color(&Color::new_rgba(0, 0, 0, 0));
            node.set_outline_thickness(2.0);
            node.set_outline_color(&Color::new_rgb(color.x, color.y, color.z));
        },
        GeometricQueryType::Contacts(_) => {
            node.set_fill_color(&Color::new_rgb(color.x, color.y, color.z));
        }
    }
}