use std::rc::Rc;
use std::cell::RefCell;
use std::collections::HashMap;
use rand::{Rng, SeedableRng, XorShiftRng};
use na::{self, Isometry3, Point3};
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::camera::{ArcBall, Camera, FirstPerson};
use ncollide::shape::{Ball3, Compound3, Cone3, ConvexHull3, Cuboid3, Cylinder3, Plane3, Shape3,
                      TriMesh3};
use ncollide::transformation;
use ncollide::world::{CollisionObject3, CollisionWorld3};
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use objects::convex::Convex;
use objects::node::Node;

pub type GraphicsManagerHandle = Rc<RefCell<GraphicsManager>>;

pub struct GraphicsManager {
    rand: XorShiftRng,
    // FIXME: merge thote three hashaps into a single one.
    uid2sn: HashMap<usize, Vec<Node>>,
    uid2visible: HashMap<usize, bool>,
    uid2color: HashMap<usize, Point3<f32>>,
    arc_ball: ArcBall,
    first_person: FirstPerson,
    curr_is_arc_ball: bool,
    aabbs: Vec<SceneNode>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let arc_ball = ArcBall::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
        let first_person =
            FirstPerson::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));

        let mut rng: XorShiftRng = SeedableRng::from_seed([0, 2, 4, 8]);

        // the first colors are boring.
        for _ in 0usize..100 {
            let _: Point3<f32> = rng.gen();
        }

        GraphicsManager {
            arc_ball: arc_ball,
            first_person: first_person,
            curr_is_arc_ball: true,
            rand: rng,
            uid2visible: HashMap::new(),
            uid2sn: HashMap::new(),
            uid2color: HashMap::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.uid2sn.values() {
            for sn in sns.iter() {
                window.remove(&mut sn.scene_node().clone());
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove(aabb);
        }

        self.uid2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove(&mut self, window: &mut Window, uid: usize) {
        match self.uid2sn.get(&uid) {
            Some(sns) => for sn in sns.iter() {
                window.remove(&mut sn.scene_node().clone());
            },
            None => {}
        }

        self.uid2sn.remove(&uid);
    }

    pub fn set_color(&mut self, uid: usize, color: Point3<f32>) {
        self.uid2color.insert(uid, color);

        if let Some(ns) = self.uid2sn.get_mut(&uid) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_visible(&mut self, uid: usize, visible: bool) {
        if let Some(ns) = self.uid2sn.get_mut(&uid) {
            for n in ns.iter_mut() {
                n.set_visible(visible)
            }
        } else {
            self.uid2visible.insert(uid, visible);
        }
    }

    pub fn update<T>(&mut self, window: &mut Window, world: &CollisionWorld3<f32, T>) {
        for object in world.collision_objects() {
            if !self.uid2sn.contains_key(&object.uid) {
                self.add(window, object);

                let visible = match self.uid2visible.get(&object.uid) {
                    Some(visible) => *visible,
                    None => true,
                };

                if !visible {
                    self.set_visible(object.uid, false);
                }
            }
        }
    }

    pub fn add<T>(&mut self, window: &mut Window, object: &CollisionObject3<f32, T>) {
        let color = match self.uid2color.get(&object.uid) {
            Some(c) => *c,
            None => self.rand.gen(),
        };

        self.add_with_color(window, object, color)
    }

    pub fn add_with_color<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        color: Point3<f32>,
    ) {
        let nodes = {
            let mut nodes = Vec::new();

            self.add_shape(
                window,
                object,
                na::one(),
                object.shape.as_ref(),
                color,
                &mut nodes,
            );

            nodes
        };

        self.uid2sn.insert(object.uid, nodes);
        self.set_color(object.uid, color);
    }

    fn add_shape<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &Shape3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        if let Some(s) = shape.as_shape::<Plane3<f32>>() {
            self.add_plane(window, object, s, color, out)
        } else if let Some(s) = shape.as_shape::<Ball3<f32>>() {
            self.add_ball(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cuboid3<f32>>() {
            self.add_box(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<ConvexHull3<f32>>() {
            self.add_convex(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cylinder3<f32>>() {
            self.add_cylinder(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cone3<f32>>() {
            self.add_cone(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Compound3<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(window, object.clone(), delta * t, s.as_ref(), color, out)
            }
        } else if let Some(s) = shape.as_shape::<TriMesh3<f32>>() {
            self.add_mesh(window, object, delta, s, color, out);
        } else {
            panic!("Not yet implemented.")
        }
    }

    fn add_plane<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        shape: &Plane3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let position = Point3::from_coordinates(object.position.translation.vector);
        let normal = object.position.rotation * shape.normal();

        out.push(Node::Plane(Plane::new(
            object,
            &position,
            &normal,
            color,
            window,
        )))
    }

    fn add_mesh<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &TriMesh3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = &**shape.vertices();
        let indices = &**shape.indices();

        let is = indices
            .iter()
            .map(|p| Point3::new(p.x as u32, p.y as u32, p.z as u32))
            .collect();

        out.push(Node::Mesh(Mesh::new(
            object,
            delta,
            vertices.clone(),
            is,
            color,
            window,
        )))
    }

    fn add_ball<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &Ball3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::Ball(Ball::new(
            object,
            delta,
            shape.radius(),
            color,
            window,
        )))
    }

    fn add_box<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cuboid3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;
        let rz = shape.half_extents().z;

        out.push(Node::Box(Box::new(
            object,
            delta,
            rx,
            ry,
            rz,
            color,
            window,
        )))
    }

    fn add_convex<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &ConvexHull3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::Convex(Convex::new(
            object,
            delta,
            &transformation::convex_hull3(shape.points()),
            color,
            window,
        )))
    }

    fn add_cylinder<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cylinder3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cylinder(Cylinder::new(
            object,
            delta,
            r,
            h,
            color,
            window,
        )))
    }

    fn add_cone<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject3<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cone3<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cone(Cone::new(object, delta, r, h, color, window)))
    }

    pub fn draw<T>(&mut self, world: &CollisionWorld3<f32, T>) {
        for (uid, ns) in self.uid2sn.iter_mut() {
            if let Some(object) = world.collision_object(*uid) {
                for n in ns.iter_mut() {
                    n.update(object)
                }
            }
        }
    }

    pub fn draw_positions<T>(&mut self, window: &mut Window, world: &CollisionWorld3<f32, T>) {
        for object in world.collision_objects() {
            let t = object.position;
            let center = Point3::from_coordinates(t.translation.vector);

            let rotmat = t.rotation.to_rotation_matrix();

            let x = rotmat.matrix().column(0) * 0.25f32;
            let y = rotmat.matrix().column(1) * 0.25f32;
            let z = rotmat.matrix().column(2) * 0.25f32;

            window.draw_line(&center, &(center + x), &Point3::new(1.0, 0.0, 0.0));
            window.draw_line(&center, &(center + y), &Point3::new(0.0, 1.0, 0.0));
            window.draw_line(&center, &(center + z), &Point3::new(0.0, 0.0, 1.0));
        }
    }

    pub fn switch_cameras(&mut self) {
        if self.curr_is_arc_ball {
            self.first_person
                .look_at(self.arc_ball.eye(), self.arc_ball.at());
        } else {
            self.arc_ball
                .look_at(self.first_person.eye(), self.first_person.at());
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn camera<'a>(&'a self) -> &'a Camera {
        if self.curr_is_arc_ball {
            &self.arc_ball as &'a Camera
        } else {
            &self.first_person as &'a Camera
        }
    }

    pub fn camera_mut<'a>(&'a mut self) -> &'a mut Camera {
        if self.curr_is_arc_ball {
            &mut self.arc_ball as &'a mut Camera
        } else {
            &mut self.first_person as &'a mut Camera
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.arc_ball.look_at(eye, at);
        self.first_person.look_at(eye, at);
    }

    pub fn scene_nodes(&self, uid: usize) -> Option<&Vec<Node>> {
        self.uid2sn.get(&uid)
    }

    pub fn scene_nodes_mut(&mut self, uid: usize) -> Option<&mut Vec<Node>> {
        self.uid2sn.get_mut(&uid)
    }
}
