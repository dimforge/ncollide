use std::rc::Rc;
use std::cell::RefCell;
use std::path::Path;
use time;
use glfw;
use glfw::{MouseButton, Key, Action, WindowEvent};
use na::{Point2, Point3, Vector3, Translate, Isometry3, Bounded};
use na;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::loader::obj;
use ncollide::ray::{self, Ray};
use ncollide::world::{CollisionWorld3, CollisionGroups};
use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

#[cfg(feature = "recording")]
use mpeg_encoder::Encoder;


#[derive(PartialEq, Eq, Debug)]
enum RunMode {
    Run,
    Stop,
    Step
}


pub struct Testbed {
    window:               Window,
    graphics:             GraphicsManagerHandle,
    running:              RunMode,
    font:                 Rc<Font>,
    grabbed_object:       Option<usize>,
    grabbed_object_plane: (Point3<f32>, Vector3<f32>),
    cursor_pos:           Point2<f32>,
    draw_colls:           bool,
    update_time:          f64,

    #[cfg(feature = "recording")]
    recorder: Option<Encoder>,
    #[cfg(not(feature = "recording"))]
    recorder: Option<()>,
}

impl Testbed {
    pub fn new() -> Testbed {
        let graphics   = GraphicsManager::new();
        let mut window = Window::new("nphysics: 3d demo");

        let font_mem = include_bytes!("Inconsolata.otf");

        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        Testbed {
            window:               window,
            graphics:             Rc::new(RefCell::new(graphics)),
            running:              RunMode::Run,
            recorder:             None,
            font:                 Font::from_memory(font_mem, 60),
            grabbed_object:       None,
            grabbed_object_plane: (na::origin(), na::zero()),
            cursor_pos:           na::origin(),
            draw_colls:           false,
            update_time:          0.0
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.borrow_mut().look_at(eye, at);
    }

    pub fn set_color(&mut self, uid: usize, color: Point3<f32>) {
        self.graphics.borrow_mut().set_color(uid, color);
    }

    pub fn graphics(&self) -> GraphicsManagerHandle {
        self.graphics.clone()
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path    = Path::new(path);
        let empty   = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").ok().expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices  = m.faces().read().unwrap().to_owned().unwrap();

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

    pub fn set_visible(&mut self, uid: usize, visible: bool) {
        self.graphics.borrow_mut().set_visible(uid, visible);
    }

    #[cfg(feature = "recording")]
    pub fn start_recording<P: AsRef<Path>>(&mut self, path: P) {
        let sz = self.window.size();

        self.recorder = Some(Encoder::new(path, sz.x as usize, sz.y as usize));
    }

    pub fn stop_recording(&mut self) {
        self.recorder = None;
    }

    pub fn step<T>(&mut self, world: &mut CollisionWorld3<f32, T>) -> bool {
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

                return true
            }
        }

        return false
    }

    pub fn update<T>(&mut self, world: &mut CollisionWorld3<f32, T>) {
        self.process_events(world);

        let before = time::precise_time_s();
        world.update();
        self.update_time = time::precise_time_s() - before;
    }

    pub fn render<T>(&mut self, world: &mut CollisionWorld3<f32, T>) -> bool {
        self.graphics.borrow_mut().draw(world);

        if self.draw_colls {
            self.graphics.borrow_mut().draw_positions(&mut self.window, world);
            draw_collisions(&mut self.window, world);
        }

        let color = Point3::new(1.0, 1.0, 1.0);

        let time_str = format!("Update time: {:.*}sec.", 4, self.update_time);
        self.window.draw_text(&time_str[..], &na::origin(), &self.font, &color);
        if self.window.render_with_camera(self.graphics.borrow_mut().camera_mut()) {

            self.record_frame_if_enabled();

            true
        }
        else {
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


    fn process_events<T>(&mut self, world: &mut CollisionWorld3<f32, T>) {
        for mut event in self.window.events().iter() {
            match event.value {
                WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                    if modifier.contains(glfw::Shift) {
                        // XXX: huge and uggly code duplication
                        let size = self.window.size();
                        let (pos, dir) = self.graphics.borrow().camera().unproject(&self.cursor_pos, &size);
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut mintoi = Bounded::max_value();
                        let mut minuid = None;

                        let all_groups = &CollisionGroups::new();

                        for (object, inter) in world.interferences_with_ray(&ray, all_groups) {
                            if  inter.toi < mintoi {
                                mintoi = inter.toi;
                                minuid = Some(object.uid);
                            }
                        }

                        if let Some(uid) = minuid {
                            world.deferred_remove(uid);
                            self.graphics.borrow_mut().remove(&mut self.window, uid);
                        }

                        event.inhibited = true;
                    }
                    else if modifier.contains(glfw::Control) {
                        if let Some(uid) = self.grabbed_object {
                            for n in self.graphics.borrow_mut().scene_nodes_mut(uid).unwrap().iter_mut() {
                                n.unselect()
                            }
                        }

                        // XXX: huge and uggly code duplication
                        let size = self.window.size();
                        let (pos, dir) = self.graphics.borrow().camera().unproject(&self.cursor_pos, &size);
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut mintoi = Bounded::max_value();

                        let all_groups = CollisionGroups::new();
                        for (object, inter) in world.interferences_with_ray(&ray, &all_groups) {
                            if  inter.toi < mintoi {
                                mintoi              = inter.toi;
                                self.grabbed_object = Some(object.uid);
                            }
                        }

                        if let Some(uid) = self.grabbed_object {
                            for n in self.graphics.borrow_mut().scene_nodes_mut(uid).unwrap().iter_mut() {
                                let attach = Isometry3::new((ray.origin + ray.dir * mintoi).to_vector(), na::zero());
                                self.grabbed_object_plane = (attach.translate(&na::origin()), -ray.dir);
                                n.select()
                            }
                        }

                        event.inhibited = true;
                    }
                },
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    let mut graphics = self.graphics.borrow_mut();
                    if let Some(uid) = self.grabbed_object {
                        for n in graphics.scene_nodes_mut(uid).unwrap().iter_mut() {
                            n.unselect()
                        }
                    }

                    self.grabbed_object = None;
                },
                WindowEvent::CursorPos(x, y) => {
                    self.cursor_pos.x = x as f32;
                    self.cursor_pos.y = y as f32;

                    if let Some(uid) = self.grabbed_object {
                        let size       = self.window.size();
                        let (pos, dir) = self.graphics.borrow().camera().unproject(&self.cursor_pos, &size);

                        let (ref ppos, ref pdir) = self.grabbed_object_plane;

                        match ray::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir)) {
                            Some(inter) => {
                                let new_pos = Isometry3::new((pos + dir * inter).to_vector(), na::zero());
                                world.deferred_set_position(uid, new_pos);
                            },
                            None => { }
                        }
                    }

                    event.inhibited =
                        self.window.glfw_window().get_key(Key::RightShift)   != Action::Release ||
                        self.window.glfw_window().get_key(Key::LeftShift)    != Action::Release ||
                        self.window.glfw_window().get_key(Key::RightControl) != Action::Release ||
                        self.window.glfw_window().get_key(Key::LeftControl)  != Action::Release;
                },
                WindowEvent::Key(Key::Tab, _, Action::Release, _) => self.graphics.borrow_mut().switch_cameras(),
                WindowEvent::Key(Key::B, _, Action::Release, _) => {
                    // XXX: there is a bug on kiss3d with the removal of objects.
                    // draw_aabbs = !draw_aabbs;
                    // if draw_aabbs {
                    //     graphics.enable_aabb_draw(&mut self.window);
                    // }
                    // else {
                    //     graphics.disable_aabb_draw(&mut self.window);
                    // }
                },
                WindowEvent::Key(Key::S, _, Action::Release, _) => self.running = RunMode::Step,
                WindowEvent::Key(Key::T, _, Action::Release, _) => {
                    if self.running == RunMode::Stop {
                        self.running = RunMode::Run;
                    }
                    else {
                        self.running = RunMode::Stop;
                    }
                },
                WindowEvent::Key(Key::Space, _, Action::Release, _) => {
                    let mut graphics = self.graphics.borrow_mut();
                    self.draw_colls = !self.draw_colls;
                    for object in world.collision_objects() {
                        // FIXME: ugly clone.
                        if let Some(ns) = graphics.scene_nodes_mut(object.uid) {
                            for n in ns.iter_mut() {
                                if self.draw_colls || object.query_type.is_proximity_query() {
                                    n.scene_node_mut().set_lines_width(1.0);
                                    n.scene_node_mut().set_surface_rendering_activation(false);
                                }
                                else {
                                    n.scene_node_mut().set_lines_width(0.0);
                                    n.scene_node_mut().set_surface_rendering_activation(true);
                                }
                            }
                        }
                    }
                },
                _ => { }
            }
        }
    }
}

fn draw_collisions<T>(window: &mut Window, world: &mut CollisionWorld3<f32, T>) {
    for c in world.contacts() {
        window.draw_line(&c.2.world1, &c.2.world2, &Point3::new(1.0, 0.0, 0.0));

        let center = na::center(&c.2.world1, &c.2.world2);
        let end    = center + c.2.normal * 0.4f32;
        window.draw_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
    }
}
