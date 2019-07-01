use num::ToPrimitive;
use rand::{Rng, SeedableRng, XorShiftRng};
use sfml::graphics::RenderWindow;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use alga::general::RealField;
use camera::Camera;
use na;
use na::{Isometry2, Point3};
use ncollide2d::shape::{Ball, Compound, ConvexPolygon, Cuboid, Plane, Polyline, Segment, Shape};
use ncollide2d::transformation;
use ncollide2d::world::{CollisionObject, CollisionObjectSlabHandle, CollisionWorld};
use objects::{self, Box, Lines, SceneNode};

pub type GraphicsManagerHandle<N> = Rc<RefCell<GraphicsManager<'static, N>>>;

pub struct GraphicsManager<'a, N: RealField> {
    rand: XorShiftRng,
    handle2sn: HashMap<CollisionObjectSlabHandle, Vec<SceneNode<'a, N>>>,
    handle2color: HashMap<CollisionObjectSlabHandle, Point3<u8>>,
}

impl<'a, N: RealField + ToPrimitive> GraphicsManager<'a, N> {
    pub fn new() -> GraphicsManager<'a, N> {
        GraphicsManager {
            rand: SeedableRng::from_seed([0; 16]),
            handle2sn: HashMap::new(),
            handle2color: HashMap::new(),
        }
    }

    pub fn update<T>(&mut self, world: &CollisionWorld<N, T>) {
        for object in world.collision_objects() {
            if !self.handle2sn.contains_key(&object.handle()) {
                self.add(object);
            }
        }
    }

    pub fn add<T>(&mut self, object: &CollisionObject<N, T>) {
        let nodes = {
            let mut nodes = Vec::new();

            self.add_shape(object, &na::one(), object.shape().as_ref(), &mut nodes);

            nodes
        };

        self.handle2sn.insert(object.handle(), nodes);
    }

    fn add_shape<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &Shape<N>,
        out: &mut Vec<SceneNode<'a, N>>,
    ) {
        if let Some(s) = shape.as_shape::<Plane<N>>() {
            self.add_plane(object, s, out)
        } else if let Some(s) = shape.as_shape::<Ball<N>>() {
            self.add_ball(object, delta, s, out)
        } else if let Some(s) = shape.as_shape::<Cuboid<N>>() {
            self.add_box(object, delta, s, out)
        } else if let Some(s) = shape.as_shape::<ConvexPolygon<N>>() {
            self.add_convex(object, delta, s, out)
        } else if let Some(s) = shape.as_shape::<Segment<N>>() {
            self.add_segment(object, delta, s, out)
        } else if let Some(s) = shape.as_shape::<Compound<N>>() {
            for &(t, ref s) in s.shapes().iter() {
                let new_delta = *delta * t;
                self.add_shape(object.clone(), &new_delta, s.as_ref(), out)
            }
        } else if let Some(s) = shape.as_shape::<Polyline<N>>() {
            self.add_lines(object, delta, s, out)
        } else {
            panic!("Not yet implemented.")
        }
    }

    fn add_plane<T>(&mut self, _: &CollisionObject<N, T>, _: &Plane<N>, _: &mut Vec<SceneNode<N>>) {
    }

    fn add_ball<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &Ball<N>,
        out: &mut Vec<SceneNode<N>>,
    ) {
        let color = self.color_for_object(object.handle());
        out.push(SceneNode::BallNode(objects::Ball::new(
            *delta,
            shape.radius(),
            color,
        )))
    }

    fn add_convex<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &ConvexPolygon<N>,
        out: &mut Vec<SceneNode<N>>,
    ) {
        let color = self.color_for_object(object.handle());
        let vs = transformation::convex_hull(shape.points()).unwrap().0;

        out.push(SceneNode::LinesNode(Lines::new(*delta, vs, color)))
    }

    fn add_lines<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &Polyline<N>,
        out: &mut Vec<SceneNode<N>>,
    ) {
        let color = self.color_for_object(object.handle());
        let vs = shape.vertices().iter().cloned().collect();

        out.push(SceneNode::LinesNode(Lines::new(*delta, vs, color)))
    }

    fn add_box<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &Cuboid<N>,
        out: &mut Vec<SceneNode<N>>,
    ) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;

        let color = self.color_for_object(object.handle());

        out.push(SceneNode::BoxNode(Box::new(*delta, rx, ry, color)))
    }

    fn add_segment<T>(
        &mut self,
        object: &CollisionObject<N, T>,
        delta: &Isometry2<N>,
        shape: &Segment<N>,
        out: &mut Vec<SceneNode<N>>,
    ) {
        let a = shape.a();
        let b = shape.b();

        let color = self.color_for_object(object.handle());

        out.push(SceneNode::SegmentNode(objects::Segment::new(
            *delta, *a, *b, color,
        )))
    }

    pub fn clear(&mut self) {
        self.handle2sn.clear();
    }

    pub fn draw<T>(&mut self, world: &CollisionWorld<N, T>, rw: &mut RenderWindow, c: &Camera) {
        c.activate_scene(rw);

        for (handle, ns) in self.handle2sn.iter_mut() {
            if let Some(object) = world.collision_object(*handle) {
                for n in ns.iter_mut() {
                    match *n {
                        SceneNode::BoxNode(ref mut n) => n.update(object),
                        SceneNode::BallNode(ref mut n) => n.update(object),
                        SceneNode::LinesNode(ref mut n) => n.update(),
                        SceneNode::SegmentNode(ref mut n) => n.update(),
                    }
                }
            }
            // FIXME: else, the object has been removed! Destroy its graphical representation as
            // well?
        }

        // First, draw sensors.
        for (handle, ns) in self.handle2sn.iter_mut() {
            if let Some(object) = world.collision_object(*handle) {
                if object.query_type().is_proximity_query() {
                    for n in ns.iter_mut() {
                        match *n {
                            SceneNode::BoxNode(ref n) => n.draw(rw),
                            SceneNode::BallNode(ref n) => n.draw(rw),
                            SceneNode::LinesNode(ref n) => n.draw(rw, object),
                            SceneNode::SegmentNode(ref n) => n.draw(rw, object),
                        }
                    }
                }
            }
        }

        // Sencond, draw solid objets.
        for (handle, ns) in self.handle2sn.iter_mut() {
            if let Some(object) = world.collision_object(*handle) {
                if object.query_type().is_contacts_query() {
                    for n in ns.iter_mut() {
                        match *n {
                            SceneNode::BoxNode(ref n) => n.draw(rw),
                            SceneNode::BallNode(ref n) => n.draw(rw),
                            SceneNode::LinesNode(ref n) => n.draw(rw, object),
                            SceneNode::SegmentNode(ref n) => n.draw(rw, object),
                        }
                    }
                }
            }
        }

        c.activate_ui(rw);
    }

    pub fn set_color(&mut self, handle: CollisionObjectSlabHandle, color: Point3<f32>) {
        let color = Point3::new(
            (color.x * 255.0) as u8,
            (color.y * 255.0) as u8,
            (color.z * 255.0) as u8,
        );

        self.handle2color.insert(handle, color);

        if let Some(sns) = self.handle2sn.get_mut(&handle) {
            for sn in sns.iter_mut() {
                sn.set_color(color)
            }
        }
    }

    pub fn color_for_object(&mut self, handle: CollisionObjectSlabHandle) -> Point3<u8> {
        match self.handle2color.get(&handle) {
            Some(color) => return *color,
            None => {}
        }

        let color = Point3::new(
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8,
        );

        self.handle2color.insert(handle, color);

        color
    }

    pub fn scene_node(
        &mut self,
        handle: CollisionObjectSlabHandle,
    ) -> Option<&mut Vec<SceneNode<'a, N>>> {
        self.handle2sn.get_mut(&handle)
    }
}
