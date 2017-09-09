use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::resource;
use na::{Vector3, Point3, Isometry3};
use ncollide::world::{CollisionObject3, GeometricQueryType};
use objects::node;

pub struct Mesh {
    color:      Point3<f32>,
    base_color: Point3<f32>,
    delta:      Isometry3<f32>,
    gfx:        SceneNode
}

impl Mesh {
    pub fn new<T>(object:   &CollisionObject3<f32, T>,
                  delta:    Isometry3<f32>,
                  vertices: Vec<Point3<f32>>,
                  indices:  Vec<Point3<u32>>,
                  color:    Point3<f32>,
                  window:   &mut Window) -> Mesh {
        let vs = vertices;
        let is = indices;

        let mesh = resource::Mesh::new(vs, is, None, None, false);

        let mut res = Mesh {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_mesh(Rc::new(RefCell::new(mesh)), Vector3::from_element(1.0)),
        };

        if let GeometricQueryType::Proximity(_) = object.query_type {
            res.gfx.set_surface_rendering_activation(false);
            res.gfx.set_lines_width(1.0);
        }

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(object.position * res.delta);
        res.update(object);

        res
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update<T>(&mut self, object: &CollisionObject3<f32, T>) {
        node::update_scene_node(&mut self.gfx, &object, &self.color, &self.delta);
    }

    pub fn scene_node(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        &mut self.gfx
    }
}
