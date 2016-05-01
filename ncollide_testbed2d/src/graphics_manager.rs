use std::rc::Rc;
use std::cell::RefCell;
use std::sync::Arc;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use sfml::graphics::RenderWindow;
use na::{Point2, Point3, Isometry2};
use na;
use ncollide::world::{CollisionWorld2, CollisionObject2};
use ncollide::transformation;
use ncollide::shape::{Shape2, Plane2, Ball2, Cuboid2, Compound2, Polyline2, ConvexHull2, Segment2};
use ncollide::math::Scalar;
use camera::Camera;
use objects::{SceneNode, Ball, Box, Lines, Segment};

pub type GraphicsManagerHandle<N> = Rc<RefCell<GraphicsManager<'static, N>>>;

pub struct GraphicsManager<'a, N> {
    rand:      XorShiftRng,
    uid2sn:    HashMap<usize, Vec<SceneNode<'a, N>>>,
    uid2color: HashMap<usize, Point3<u8>>
}

impl<'a, N: Scalar> GraphicsManager<'a, N> {
    pub fn new() -> GraphicsManager<'a, N> {
        GraphicsManager {
            rand:      SeedableRng::from_seed([0, 1, 2, 3]),
            uid2sn:    HashMap::new(),
            uid2color: HashMap::new()
        }
    }

    pub fn update<T>(&mut self, world: &CollisionWorld2<N, T>) {
        for object in world.collision_objects() {
            if !self.uid2sn.contains_key(&object.uid) {
                self.add(object);
            }
        }
    }

    pub fn add<T>(&mut self, object: &CollisionObject2<N, T>) {
        let nodes = {
            let mut nodes = Vec::new();

            self.add_shape(object, &na::one(), object.shape.as_ref(), &mut nodes);

            nodes
        };

        self.uid2sn.insert(object.uid, nodes);
    }

    fn add_shape<T>(&mut self,
                    object: &CollisionObject2<N, T>,
                    delta:  &Isometry2<N>,
                    shape:  &Shape2<N>,
                    out:    &mut Vec<SceneNode<'a, N>>) {
        if let Some(s) = shape.as_shape::<Plane2<N>>() {
            self.add_plane(object, s, out)
        }
        else if let Some(s) = shape.as_shape::<Ball2<N>>() {
            self.add_ball(object, delta, s, out)
        }
        else if let Some(s) = shape.as_shape::<Cuboid2<N>>() {
            self.add_box(object, delta, s, out)
        }
        else if let Some(s) = shape.as_shape::<ConvexHull2<N>>() {
            self.add_convex(object, delta, s, out)
        }
        else if let Some(s) = shape.as_shape::<Segment2<N>>() {
            self.add_segment(object, delta, s, out)
        }
        else if let Some(s) = shape.as_shape::<Compound2<N>>() {
            for &(t, ref s) in s.shapes().iter() {
                let new_delta = *delta * t;
                self.add_shape(object.clone(), &new_delta, s.as_ref(), out)
            }
        }
        else if let Some(s) = shape.as_shape::<Polyline2<N>>() {
            self.add_lines(object, delta, s, out)
        }
        else {
            panic!("Not yet implemented.")
        }

    }

    fn add_plane<T>(&mut self,
                    _: &CollisionObject2<N, T>,
                    _: &Plane2<N>,
                    _: &mut Vec<SceneNode<N>>) {
    }

    fn add_ball<T>(&mut self,
                   object: &CollisionObject2<N, T>,
                   delta:  &Isometry2<N>,
                   shape:  &Ball2<N>,
                   out:    &mut Vec<SceneNode<N>>) {
        let color = self.color_for_object(object.uid);
        out.push(SceneNode::BallNode(Ball::new(*delta, shape.radius(), color)))
    }

    fn add_convex<T>(&mut self,
                     object: &CollisionObject2<N, T>,
                     delta:  &Isometry2<N>,
                     shape:  &ConvexHull2<N>,
                     out:    &mut Vec<SceneNode<N>>) {
        let color = self.color_for_object(object.uid);
        let vs = Arc::new(transformation::convex_hull2(shape.points()).unwrap().0);

        let is = {
            let limit = vs.len();
            Arc::new((0 .. limit as usize).map(|x| Point2::new(x, (x + (1 as usize)) % limit)).collect())
        };

        out.push(SceneNode::LinesNode(Lines::new(*delta, vs, is, color)))
    }

    fn add_lines<T>(&mut self,
                    object: &CollisionObject2<N, T>,
                    delta:  &Isometry2<N>,
                    shape:  &Polyline2<N>,
                    out:    &mut Vec<SceneNode<N>>) {

        let color = self.color_for_object(object.uid);

        let vs = shape.vertices().clone();
        let is = shape.indices().clone();

        out.push(SceneNode::LinesNode(Lines::new(*delta, vs, is, color)))
    }


    fn add_box<T>(&mut self,
                  object: &CollisionObject2<N, T>,
                  delta:  &Isometry2<N>,
                  shape:  &Cuboid2<N>,
                  out:    &mut Vec<SceneNode<N>>) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;

        let color = self.color_for_object(object.uid);

        out.push(SceneNode::BoxNode(Box::new(*delta, rx, ry, color)))
    }

    fn add_segment<T>(&mut self,
                      object: &CollisionObject2<N, T>,
                      delta:  &Isometry2<N>,
                      shape:  &Segment2<N>,
                      out:    &mut Vec<SceneNode<N>>) {
        let a = shape.a();
        let b = shape.b();

        let color = self.color_for_object(object.uid);

        out.push(SceneNode::SegmentNode(Segment::new(*delta, *a, *b, color)))
    }


    pub fn clear(&mut self) {
        self.uid2sn.clear();
    }

    pub fn draw<T>(&mut self, world: &CollisionWorld2<N, T>, rw: &mut RenderWindow, c: &Camera) {
        c.activate_scene(rw);

        for (uid, ns) in self.uid2sn.iter_mut() {
            if let Some(object) = world.collision_object(*uid) {
                for n in ns.iter_mut() {
                    match *n {
                        SceneNode::BoxNode(ref mut n)     => n.update(object),
                        SceneNode::BallNode(ref mut n)    => n.update(object),
                        SceneNode::LinesNode(ref mut n)   => n.update(),
                        SceneNode::SegmentNode(ref mut n) => n.update(),
                    }
                }
            }
            // FIXME: else, the object has been removed! Destroy its graphical representation as
            // well?
        }

        // First, draw sensors.
        for (uid, ns) in self.uid2sn.iter_mut() {
            if let Some(object) = world.collision_object(*uid) {
                if object.query_type.is_proximity_query() {
                    for n in ns.iter_mut() {
                        match *n {
                            SceneNode::BoxNode(ref n)     => n.draw(rw),
                            SceneNode::BallNode(ref n)    => n.draw(rw),
                            SceneNode::LinesNode(ref n)   => n.draw(rw, object),
                            SceneNode::SegmentNode(ref n) => n.draw(rw, object),
                        }
                    }
                }
            }
        }

        // Sencond, draw solid objets.
        for (uid, ns) in self.uid2sn.iter_mut() {
            if let Some(object) = world.collision_object(*uid) {
                if object.query_type.is_contacts_query() {
                    for n in ns.iter_mut() {
                        match *n {
                            SceneNode::BoxNode(ref n)     => n.draw(rw),
                            SceneNode::BallNode(ref n)    => n.draw(rw),
                            SceneNode::LinesNode(ref n)   => n.draw(rw, object),
                            SceneNode::SegmentNode(ref n) => n.draw(rw, object),
                        }
                    }
                }
            }
        }

        c.activate_ui(rw);
    }

    pub fn set_color(&mut self, uid: usize, color: Point3<f32>) {
        let color = Point3::new(
            (color.x * 255.0) as u8,
            (color.y * 255.0) as u8,
            (color.z * 255.0) as u8
        );

        self.uid2color.insert(uid, color);

        if let Some(sns) = self.uid2sn.get_mut(&uid) {
            for sn in sns.iter_mut() {
                sn.set_color(color)
            }
        }
    }

    pub fn color_for_object(&mut self, uid: usize) -> Point3<u8> {
        match self.uid2color.get(&uid) {
            Some(color) => return *color,
            None        => { }
        }

        let color = Point3::new(
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8);

        self.uid2color.insert(uid, color);

        color
    }

    pub fn scene_node(&mut self, uid: usize) -> Option<&mut Vec<SceneNode<'a, N>>> {
        self.uid2sn.get_mut(&uid)
    }
}
