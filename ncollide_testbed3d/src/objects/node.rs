use kiss3d::scene::SceneNode;
use na::{Isometry3, Point3};
use ncollide::world::CollisionObject3;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use objects::convex::Convex;

pub enum Node {
    Ball(Ball),
    Box(Box),
    Cylinder(Cylinder),
    Cone(Cone),
    Mesh(Mesh),
    Plane(Plane),
    Convex(Convex)
}

impl Node {
    pub fn set_visible(&mut self, visible: bool) {
        self.scene_node_mut().set_visible(visible)
    }

    pub fn select(&mut self) {
        match *self {
            Node::Plane(ref mut n)    => n.select(),
            Node::Ball(ref mut n)     => n.select(),
            Node::Box(ref mut n)      => n.select(),
            Node::Cylinder(ref mut n) => n.select(),
            Node::Cone(ref mut n)     => n.select(),
            Node::Mesh(ref mut n)     => n.select(),
            Node::Convex(ref mut n)   => n.select()
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n)    => n.unselect(),
            Node::Ball(ref mut n)     => n.unselect(),
            Node::Box(ref mut n)      => n.unselect(),
            Node::Cylinder(ref mut n) => n.unselect(),
            Node::Cone(ref mut n)     => n.unselect(),
            Node::Mesh(ref mut n)     => n.unselect(),
            Node::Convex(ref mut n)   => n.unselect()
        }
    }

    pub fn update<T>(&mut self, object: &CollisionObject3<f32, T>) {
        match *self {
            Node::Plane(ref mut n)    => n.update(),
            Node::Ball(ref mut n)     => n.update(object),
            Node::Box(ref mut n)      => n.update(object),
            Node::Cylinder(ref mut n) => n.update(object),
            Node::Cone(ref mut n)     => n.update(object),
            Node::Mesh(ref mut n)     => n.update(object),
            Node::Convex(ref mut n)   => n.update(object)
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        match *self {
            Node::Plane(ref n)    => n.scene_node(),
            Node::Ball(ref n)     => n.scene_node(),
            Node::Box(ref n)      => n.scene_node(),
            Node::Cylinder(ref n) => n.scene_node(),
            Node::Cone(ref n)     => n.scene_node(),
            Node::Mesh(ref n)     => n.scene_node(),
            Node::Convex(ref n)   => n.scene_node()
        }
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        match *self {
            Node::Plane(ref mut n)    => n.scene_node_mut(),
            Node::Ball(ref mut n)     => n.scene_node_mut(),
            Node::Box(ref mut n)      => n.scene_node_mut(),
            Node::Cylinder(ref mut n) => n.scene_node_mut(),
            Node::Cone(ref mut n)     => n.scene_node_mut(),
            Node::Mesh(ref mut n)     => n.scene_node_mut(),
            Node::Convex(ref mut n)   => n.scene_node_mut()
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>)  {
        match *self {
            Node::Plane(ref mut n)    => n.set_color(color),
            Node::Ball(ref mut n)     => n.set_color(color),
            Node::Box(ref mut n)      => n.set_color(color),
            Node::Cylinder(ref mut n) => n.set_color(color),
            Node::Cone(ref mut n)     => n.set_color(color),
            Node::Mesh(ref mut n)     => n.set_color(color),
            Node::Convex(ref mut n)   => n.set_color(color)
        }
    }
}


pub fn update_scene_node<T>(node:   &mut SceneNode,
                            object: &CollisionObject3<f32, T>,
                            color:  &Point3<f32>,
                            delta:  &Isometry3<f32>) {
    node.set_local_transformation(object.position * *delta);
    node.set_color(color.x, color.y, color.z);
}
