use kiss3d::camera::{ArcBall, Camera, FirstPerson};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{self, Isometry3, Point3};
use ncollide3d::shape::{Ball, Compound, ConvexHull, Cuboid, Plane, Shape, TriMesh};
use ncollide3d::transformation;
use ncollide3d::world::{CollisionObject, CollisionObjectSlabHandle, CollisionWorld};
use objects::{self, Box, Mesh, Node};
use rand::{Rng, SeedableRng, XorShiftRng};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

pub type GraphicsManagerHandle = Rc<RefCell<GraphicsManager>>;

pub struct GraphicsManager {
    rand: XorShiftRng,
    // FIXME: merge thote three hashaps into a single one.
    handle2sn: HashMap<CollisionObjectSlabHandle, Vec<Node>>,
    handle2visible: HashMap<CollisionObjectSlabHandle, bool>,
    handle2color: HashMap<CollisionObjectSlabHandle, Point3<f32>>,
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

        let mut rng: XorShiftRng = SeedableRng::from_seed([0; 16]);

        // the first colors are boring.
        for _ in 0usize..100 {
            let _: Point3<f32> = rng.gen();
        }

        GraphicsManager {
            arc_ball: arc_ball,
            first_person: first_person,
            curr_is_arc_ball: true,
            rand: rng,
            handle2visible: HashMap::new(),
            handle2sn: HashMap::new(),
            handle2color: HashMap::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.handle2sn.values() {
            for sn in sns.iter() {
                window.remove(&mut sn.scene_node().clone());
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove(aabb);
        }

        self.handle2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove(&mut self, window: &mut Window, handle: CollisionObjectSlabHandle) {
        self.handle2sn.get(&handle).flatten().map(|sn| {
            window.remove(&mut sn.scene_node().clone());
        });

        self.handle2sn.remove(&handle);
    }

    pub fn set_color(&mut self, handle: CollisionObjectSlabHandle, color: Point3<f32>) {
        self.handle2color.insert(handle, color);

        if let Some(ns) = self.handle2sn.get_mut(&handle) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_visible(&mut self, handle: CollisionObjectSlabHandle, visible: bool) {
        if let Some(ns) = self.handle2sn.get_mut(&handle) {
            for n in ns.iter_mut() {
                n.set_visible(visible)
            }
        } else {
            self.handle2visible.insert(handle, visible);
        }
    }

    pub fn update<T>(&mut self, window: &mut Window, world: &CollisionWorld<f32, T>) {
        for object in world.collision_objects() {
            if !self.handle2sn.contains_key(&object.handle()) {
                self.add(window, object);

                let visible = self
                    .handle2visible
                    .get(&object.handle())
                    .get_or_insert(true);

                if !visible {
                    self.set_visible(object.handle(), false);
                }
            }
        }
    }

    pub fn add<T>(&mut self, window: &mut Window, object: &CollisionObject<f32, T>) {
        let color = self
            .handle2color
            .get(&object.handle())
            .get_or_insert_with(|| self.rand.gen());

        self.add_with_color(window, object, color)
    }

    pub fn add_with_color<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        color: Point3<f32>,
    ) {
        let nodes = {
            let mut nodes = Vec::new();

            self.add_shape(
                window,
                object,
                na::one(),
                object.shape().as_ref(),
                color,
                &mut nodes,
            );

            nodes
        };

        self.handle2sn.insert(object.handle(), nodes);
        self.set_color(object.handle(), color);
    }

    fn add_shape<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &Shape<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        if let Some(s) = shape.as_shape::<Plane<f32>>() {
            self.add_plane(window, object, s, color, out)
        } else if let Some(s) = shape.as_shape::<Ball<f32>>() {
            self.add_ball(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cuboid<f32>>() {
            self.add_box(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<ConvexHull<f32>>() {
            self.add_convex(window, object, delta, s, color, out)
        // } else if let Some(s) = shape.as_shape::<Cylinder<f32>>() {
        //     self.add_cylinder(window, object, delta, s, color, out)
        // } else if let Some(s) = shape.as_shape::<Cone<f32>>() {
        //     self.add_cone(window, object, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Compound<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(window, object.clone(), delta * t, s.as_ref(), color, out)
            }
        } else if let Some(s) = shape.as_shape::<TriMesh<f32>>() {
            self.add_mesh(window, object, delta, s, color, out);
        } else {
            panic!("Not yet implemented.")
        }
    }

    fn add_plane<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        shape: &Plane<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let position = Point3::from_coordinates(object.position().translation.vector);
        let normal = object.position().rotation * shape.normal();

        out.push(Node::Plane(objects::Plane::new(
            object, &position, &normal, color, window,
        )))
    }

    fn add_mesh<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &TriMesh<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = &**shape.vertices();
        let indices = &**shape.indices();

        let is = indices
            .iter()
            .map(|p| Point3::new(p.x as u16, p.y as u16, p.z as u16))
            .collect();

        out.push(Node::Mesh(Mesh::new(
            object,
            delta,
            vertices.clone().to_vec(),
            is,
            color,
            window,
        )))
    }

    fn add_ball<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &Ball<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::Ball(objects::Ball::new(
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
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cuboid<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;
        let rz = shape.half_extents().z;

        out.push(Node::Box(Box::new(
            object, delta, rx, ry, rz, color, window,
        )))
    }

    fn add_convex<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &ConvexHull<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::Convex(objects::Convex::new(
            object,
            delta,
            &transformation::convex_hull(shape.points()),
            color,
            window,
        )))
    }

    /*
    fn add_cylinder<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cylinder<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cylinder(objects::Cylinder::new(
            object, delta, r, h, color, window,
        )))
    }

    fn add_cone<T>(
        &mut self,
        window: &mut Window,
        object: &CollisionObject<f32, T>,
        delta: Isometry3<f32>,
        shape: &Cone<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cone(objects::Cone::new(
            object, delta, r, h, color, window,
        )))
    }
    */

    pub fn draw<T>(&mut self, world: &CollisionWorld<f32, T>) {
        for (handle, ns) in self.handle2sn.iter_mut() {
            if let Some(object) = world.collision_object(*handle) {
                for n in ns.iter_mut() {
                    n.update(object)
                }
            }
        }
    }

    pub fn draw_positions<T>(&mut self, window: &mut Window, world: &CollisionWorld<f32, T>) {
        for object in world.collision_objects() {
            let t = object.position();
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

    pub fn scene_nodes(&self, handle: CollisionObjectSlabHandle) -> Option<&Vec<Node>> {
        self.handle2sn.get(&handle)
    }

    pub fn scene_nodes_mut(&mut self, handle: CollisionObjectSlabHandle) -> Option<&mut Vec<Node>> {
        self.handle2sn.get_mut(&handle)
    }
}
