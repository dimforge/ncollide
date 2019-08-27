use graphics_manager::{GraphicsManager, GraphicsManagerHandle};
use kiss3d::event::{Action, Key, Modifiers, MouseButton, WindowEvent};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::text::Font;
use kiss3d::window::Window;
use na::{self, Isometry3, Point2, Point3, Vector3};
use ncollide3d::query::{ray_internal, Ray};
use ncollide3d::world::{CollisionGroups, CollisionObjectSlabHandle, CollisionWorld};
use num::Bounded;
use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;
use time;

#[cfg(feature = "recording")]
use mpeg_encoder::Encoder;

#[derive(PartialEq, Eq, Debug)]
enum RunMode {
    Run,
    Stop,
    Step,
}

pub struct Testbed {
    window: Window,
    graphics: GraphicsManagerHandle,
    running: RunMode,
    font: Rc<Font>,
    grabbed_object: Option<CollisionObjectSlabHandle>,
    grabbed_object_plane: (Point3<f32>, Vector3<f32>),
    cursor_pos: Point2<f32>,
    draw_colls: bool,
    update_time: f64,

    #[cfg(feature = "recording")]
    recorder: Option<Encoder>,
    #[cfg(not(feature = "recording"))]
    recorder: Option<()>,
}

impl Testbed {
    pub fn new() -> Testbed {
        let graphics = GraphicsManager::new();
        let mut window = Window::new("nphysics: 3d demo");

        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        Testbed {
            window: window,
            graphics: Rc::new(RefCell::new(graphics)),
            running: RunMode::Run,
            recorder: None,
            font: Font::default(),
            grabbed_object: None,
            grabbed_object_plane: (Point3::origin(), Point3::zero()),
            cursor_pos: Point2::origin(),
            draw_colls: false,
            update_time: 0.0,
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.borrow_mut().look_at(eye, at);
    }

    pub fn set_color(&mut self, handle: CollisionObjectSlabHandle, color: Point3<f32>) {
        self.graphics.borrow_mut().set_color(handle, color);
    }

    pub fn graphics(&self) -> GraphicsManagerHandle {
        self.graphics.clone()
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "")
            .ok()
            .expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices = m.faces().read().unwrap().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.into_iter() {
                flat_indices.push(i.x as usize);
                flat_indices.push(i.y as usize);
                flat_indices.push(i.z as usize);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    pub fn set_visible(&mut self, handle: CollisionObjectSlabHandle, visible: bool) {
        self.graphics.borrow_mut().set_visible(handle, visible);
    }

    #[cfg(feature = "recording")]
    pub fn start_recording<P: AsRef<Path>>(&mut self, path: Point<N>) {
        let sz = self.window.size();

        self.recorder = Some(Encoder::new(path, sz.x as usize, sz.y as usize));
    }

    pub fn stop_recording(&mut self) {
        self.recorder = None;
    }

    pub fn step<T>(&mut self, world: &mut CollisionWorld<f32, T>) -> bool {
        self.graphics.borrow_mut().update(&mut self.window, world);

        loop {
            self.update(world);
            if !self.render(world) {
                break;
            }

            if self.running != RunMode::Stop {
                if self.running == RunMode::Step {
                    self.running = RunMode::Stop;
                }

                return true;
            }
        }

        return false;
    }

    pub fn update<T>(&mut self, world: &mut CollisionWorld<f32, T>) {
        self.process_events(world);

        let before = time::precise_time_s();
        world.update();
        self.update_time = time::precise_time_s() - before;
    }

    pub fn render<T>(&mut self, world: &mut CollisionWorld<f32, T>) -> bool {
        self.graphics.borrow_mut().draw(world);

        if self.draw_colls {
            self.graphics
                .borrow_mut()
                .draw_positions(&mut self.window, world);
            draw_collisions(&mut self.window, world);
        }

        let color = Point3::new(1.0, 1.0, 1.0);

        let time_str = format!("Update time: {:.*}sec.", 4, self.update_time);
        self.window
            .draw_text(&time_str[..], &Point2::origin(), 30.0, &self.font, &color);
        if self
            .window
            .render_with_camera(self.graphics.borrow_mut().camera_mut())
        {
            self.record_frame_if_enabled();

            true
        } else {
            false
        }
    }

    #[cfg(feature = "recording")]
    fn record_frame_if_enabled(&mut self) {
        if let Some(ref mut recorder) = self.recorder {
            let mut memory = Vec::new();
            let sz = self.window.size();
            self.window.snap(&mut memory);
            recorder.encode_rgba(sz.x as usize, sz.y as usize, &memory[..], false);
        }
    }

    #[cfg(not(feature = "recording"))]
    fn record_frame_if_enabled(&self) {
        // Do nothing.
    }

    fn process_events<T>(&mut self, world: &mut CollisionWorld<f32, T>) {
        for mut event in self.window.events().iter() {
            match event.value {
                WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                    if modifier.contains(Modifiers::Shift) {
                        // XXX: huge and uggly code duplication
                        let size = self.window.size();
                        let (pos, dir) = self
                            .graphics
                            .borrow()
                            .camera()
                            .unproject(&self.cursor_pos, &na::convert(size));
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut mintoi = Bounded::max_value();
                        let mut minhandle = None;

                        let all_groups = &CollisionGroups::new();

                        for (object, inter) in world.interferences_with_ray(&ray, all_groups) {
                            if inter.toi < mintoi {
                                mintoi = inter.toi;
                                minhandle = Some(object.handle());
                            }
                        }

                        if let Some(handle) = minhandle {
                            world.remove(&[handle]);
                            self.graphics.borrow_mut().remove(&mut self.window, handle);
                        }

                        event.inhibited = true;
                    } else if modifier.contains(Modifiers::Control) {
                        if let Some(handle) = self.grabbed_object {
                            for n in self
                                .graphics
                                .borrow_mut()
                                .scene_nodes_mut(handle)
                                .unwrap()
                                .iter_mut()
                            {
                                n.unselect()
                            }
                        }

                        // XXX: huge and uggly code duplication
                        let size = self.window.size();
                        let (pos, dir) = self
                            .graphics
                            .borrow()
                            .camera()
                            .unproject(&self.cursor_pos, &na::convert(size));
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut mintoi = Bounded::max_value();

                        let all_groups = CollisionGroups::new();
                        for (object, inter) in world.interferences_with_ray(&ray, &all_groups) {
                            if inter.toi < mintoi {
                                mintoi = inter.toi;
                                self.grabbed_object = Some(object.handle());
                            }
                        }

                        if let Some(handle) = self.grabbed_object {
                            for n in self
                                .graphics
                                .borrow_mut()
                                .scene_nodes_mut(handle)
                                .unwrap()
                                .iter_mut()
                            {
                                let attach = Isometry3::new(
                                    (ray.origin + ray.dir * mintoi).coords,
                                    na::zero(),
                                );
                                self.grabbed_object_plane = (
                                    Point3::from_coordinates(attach.translation.vector),
                                    -ray.dir,
                                );
                                n.select()
                            }
                        }

                        event.inhibited = true;
                    }
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    let mut graphics = self.graphics.borrow_mut();
                    if let Some(handle) = self.grabbed_object {
                        for n in graphics.scene_nodes_mut(handle).unwrap().iter_mut() {
                            n.unselect()
                        }
                    }

                    self.grabbed_object = None;
                }
                WindowEvent::CursorPos(x, y, _) => {
                    self.cursor_pos.x = x as f32;
                    self.cursor_pos.y = y as f32;

                    if let Some(handle) = self.grabbed_object {
                        let size = self.window.size();
                        let (pos, dir) = self
                            .graphics
                            .borrow()
                            .camera()
                            .unproject(&self.cursor_pos, &na::convert(size));

                        let (ref ppos, ref pdir) = self.grabbed_object_plane;

                        if let Some(inter) =
                            ray_internal::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir))
                        {
                            let new_pos = Isometry3::new((pos + dir * inter).coords, na::zero());
                            world.set_position(handle, new_pos);
                        }
                    }

                    event.inhibited = self.window.get_key(Key::RShift) != Action::Release
                        || self.window.get_key(Key::LShift) != Action::Release
                        || self.window.get_key(Key::RControl) != Action::Release
                        || self.window.get_key(Key::LControl) != Action::Release;
                }
                WindowEvent::Key(Key::Tab, Action::Release, _) => {
                    self.graphics.borrow_mut().switch_cameras()
                }
                WindowEvent::Key(Key::B, Action::Release, _) => {
                    // XXX: there is a bug on kiss3d with the removal of objects.
                    // draw_aabbs = !draw_aabbs;
                    // if draw_aabbs {
                    //     graphics.enable_aabb_draw(&mut self.window);
                    // }
                    // else {
                    //     graphics.disable_aabb_draw(&mut self.window);
                    // }
                }
                WindowEvent::Key(Key::S, Action::Release, _) => self.running = RunMode::Step,
                WindowEvent::Key(Key::T, Action::Release, _) => {
                    if self.running == RunMode::Stop {
                        self.running = RunMode::Run;
                    } else {
                        self.running = RunMode::Stop;
                    }
                }
                WindowEvent::Key(Key::Space, Action::Release, _) => {
                    let mut graphics = self.graphics.borrow_mut();
                    self.draw_colls = !self.draw_colls;
                    for object in world.collision_objects() {
                        // FIXME: ugly clone.
                        if let Some(ns) = graphics.scene_nodes_mut(object.handle()) {
                            for n in ns.iter_mut() {
                                if self.draw_colls || object.query_type().is_proximity_query() {
                                    n.scene_node_mut().set_lines_width(1.0);
                                    n.scene_node_mut().set_surface_rendering_activation(false);
                                } else {
                                    n.scene_node_mut().set_lines_width(0.0);
                                    n.scene_node_mut().set_surface_rendering_activation(true);
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

fn draw_collisions<T>(window: &mut Window, world: &mut CollisionWorld<f32, T>) {
    for m in world.contact_manifolds() {
        for c in m.2.contacts() {
            window.draw_line(
                &c.contact.world1,
                &c.contact.world2,
                &Point3::new(1.0, 0.0, 0.0),
            );

            let center = na::center(&c.contact.world1, &c.contact.world2);
            let end = center + c.contact.normal.into_inner() * 0.4f32;
            window.draw_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
        }
    }
}
