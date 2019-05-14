use num::ToPrimitive;
use sfml::graphics::Color;
use sfml::graphics::{RenderTarget, RenderWindow};
use sfml::system::Vector2i;
use sfml::window::event::Event;
use sfml::window::window_style;
use sfml::window::{ContextSettings, VideoMode};
use sfml::window::{Key, MouseButton};
use std::cell::RefCell;
use std::rc::Rc;

use alga::general::RealField;
use alga::linear::ProjectiveTransformation;
use camera::Camera;
use na::{self, Point2, Point3, Translation2};
use ncollide2d::shape::Plane;
use ncollide2d::world::{CollisionGroups, CollisionObjectHandle, CollisionWorld};
// use fps::Fps;
use draw_helper;
use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

#[cfg(feature = "recording")]
use mpeg_encoder::Encoder;
#[cfg(feature = "recording")]
use std::path::Path;

#[derive(PartialEq, Eq, Debug)]
enum RunMode {
    Run,
    Stop,
    Step,
}

pub struct Testbed<N: RealField> {
    window: RenderWindow,
    graphics: GraphicsManagerHandle<N>,
    running: RunMode,
    background: Color,

    draw_colls: bool,
    camera: Camera,
    // font:           Font,
    // fps:            Fps<'static>,
    grabbed_object: Option<CollisionObjectHandle>,
    grab_anchor: Point2<N>,

    #[cfg(feature = "recording")]
    recorder: Option<Encoder>,
    #[cfg(not(feature = "recording"))]
    recorder: Option<()>,
}

impl<N: RealField + ToPrimitive> Testbed<N> {
    pub fn new() -> Testbed<N> {
        let mode = VideoMode::new_init(800, 600, 32);
        let ctxt = ContextSettings::default();
        // ctxt.antialiasing(3);

        let window_style = window_style::CLOSE | window_style::RESIZE | window_style::CLOSE;
        let mut window = match RenderWindow::new(mode, "ncollide 2d testbed", window_style, &ctxt) {
            Some(rwindow) => rwindow,
            None => panic!("Error on creating the sfml window."),
        };

        window.set_framerate_limit(60);

        // let fnt_mem = include_bytes!("Inconsolata.otf");
        // let fnt     = Font::new_from_memory(fnt_mem).unwrap();
        let camera = Camera::new(&window);

        Testbed {
            window: window,
            graphics: Rc::new(RefCell::new(GraphicsManager::new())),
            running: RunMode::Run,
            recorder: None,
            background: Color::black(),

            draw_colls: false,
            camera: camera,
            // font:           fnt,
            // fps:            Fps::new(&fnt),
            grabbed_object: None,
            grab_anchor: Point2::origin(),
        }
    }

    pub fn graphics(&self) -> GraphicsManagerHandle<N> {
        self.graphics.clone()
    }

    pub fn set_color(&mut self, handle: CollisionObjectHandle, color: Point3<f32>) {
        self.graphics.borrow_mut().set_color(handle, color);
    }

    pub fn is_open(&self) -> bool {
        self.window.is_open()
    }

    pub fn take_snapshot(&mut self, path: &str) {
        let capture = self.window.capture().unwrap();
        capture.save_to_file(path);
    }

    #[cfg(feature = "recording")]
    pub fn start_recording<P: AsRef<Path>>(&mut self, path: Point<N>) {
        let sz = self.window.get_size();

        self.recorder = Some(Encoder::new(path, sz.x as usize, sz.y as usize));
    }

    pub fn stop_recording(&mut self) {
        self.recorder = None;
    }

    pub fn step<T>(&mut self, world: &mut CollisionWorld<N, T>) -> bool {
        self.graphics.borrow_mut().update(world);

        while self.window.is_open() {
            self.update(world);
            self.render(world);

            if self.running != RunMode::Stop {
                if self.running == RunMode::Step {
                    self.running = RunMode::Stop;
                }

                return true;
            }
        }

        return false;
    }

    pub fn update<T>(&mut self, world: &mut CollisionWorld<N, T>) {
        self.process_events(world);

        // self.fps.reset();
        // self.fps.register_delta();
        world.update();
    }

    pub fn set_background_color(&mut self, color: Point3<f32>) {
        self.background = Color::new_rgb(
            (color.x * 255.0) as u8,
            (color.y * 255.0) as u8,
            (color.z * 255.0) as u8,
        );
    }

    pub fn render<T>(&mut self, world: &mut CollisionWorld<N, T>) {
        self.window.clear(&self.background);
        // self.fps.draw_registered(&mut self.window);
        self.graphics
            .borrow_mut()
            .draw(world, &mut self.window, &self.camera);
        self.camera.activate_scene(&mut self.window);

        self.draw_collisions(world);

        self.camera.activate_ui(&mut self.window);

        self.window.display();

        self.record_frame_if_enabled();
    }

    #[cfg(feature = "recording")]
    fn record_frame_if_enabled(&mut self) {
        if let Some(ref mut recorder) = self.recorder {
            let sz = self.window.get_size();
            let capture = self.window.capture().unwrap();
            let memory = capture.get_memory();
            recorder.encode_rgba(sz.x as usize, sz.y as usize, memory, false);
        }
    }

    #[cfg(not(feature = "recording"))]
    fn record_frame_if_enabled(&self) {
        // Do nothing.
    }

    fn process_events<T>(&mut self, world: &mut CollisionWorld<N, T>) {
        loop {
            match self.window.poll_event() {
                Event::KeyPressed { code, .. } => self.process_key_press(code),
                Event::MouseButtonPressed { button, x, y } => {
                    self.process_mouse_press(world, button, x, y)
                }
                Event::MouseButtonReleased { button, x, y } => {
                    self.process_mouse_release(button, x, y)
                }
                Event::MouseMoved { x, y } => self.process_mouse_moved(world, x, y),
                Event::Closed => self.window.close(),
                Event::NoEvent => break,
                e => self.camera.handle_event(&e),
            }
        }
    }

    fn process_key_press(&mut self, code: Key) {
        match code {
            Key::Escape => self.window.close(),
            Key::Space => self.draw_colls = !self.draw_colls,
            Key::S => self.running = RunMode::Step,
            Key::T => {
                if self.running == RunMode::Stop {
                    self.running = RunMode::Run;
                } else {
                    self.running = RunMode::Stop;
                }
            }
            _ => {}
        }
    }

    fn process_mouse_press<T>(
        &mut self,
        world: &CollisionWorld<N, T>,
        button: MouseButton,
        x: i32,
        y: i32,
    ) {
        match button {
            MouseButton::Left => {
                let mapped_coords = self.camera.map_pixel_to_coords(Vector2i::new(x, y));
                let mapped_point = Point2::new(mapped_coords.x as f64, mapped_coords.y as f64);

                // FIXME: use the collision groups to filter out sensors.
                let all_groups = &CollisionGroups::new();
                // We give the priority to contacts-enabled objects.
                let mut grabbed_solid = false; // The grabbed is not a sensor.

                for object in world.interferences_with_point(&na::convert(mapped_point), all_groups)
                {
                    // Planes are infinite so it is more ergonomic to prevent them from being
                    // grabbed.
                    if !object.shape().is_shape::<Plane<N>>() {
                        if object.query_type().is_contacts_query() || !grabbed_solid {
                            self.grab_anchor = object
                                .position()
                                .inverse_transform_point(&na::convert(mapped_point));
                            self.grabbed_object = Some(object.handle());
                            grabbed_solid = object.query_type().is_contacts_query();
                        }
                    }
                }

                if let Some(handle) = self.grabbed_object {
                    for node in self
                        .graphics
                        .borrow_mut()
                        .scene_node(handle)
                        .unwrap()
                        .iter_mut()
                    {
                        node.select()
                    }
                }
            }
            _ => self.camera.handle_event(&Event::MouseButtonPressed {
                button: button,
                x: x,
                y: y,
            }),
        }
    }

    fn process_mouse_release(&mut self, button: MouseButton, x: i32, y: i32) {
        match button {
            MouseButton::Left => {
                if let Some(handle) = self.grabbed_object {
                    for node in self
                        .graphics
                        .borrow_mut()
                        .scene_node(handle)
                        .unwrap()
                        .iter_mut()
                    {
                        node.unselect()
                    }
                }

                self.grabbed_object = None;
            }
            _ => self.camera.handle_event(&Event::MouseButtonReleased {
                button: button,
                x: x,
                y: y,
            }),
        }
    }

    fn process_mouse_moved<T>(&mut self, world: &mut CollisionWorld<N, T>, x: i32, y: i32) {
        let mapped_coords = self.camera.map_pixel_to_coords(Vector2i::new(x, y));
        let mapped_point = Point2::new(mapped_coords.x as f64, mapped_coords.y as f64);

        if let Some(handle) = self.grabbed_object {
            let mut new_pos = na::one();

            if let Some(obj) = world.collision_object(handle) {
                let anchor = obj.position() * self.grab_anchor;
                let new_pt: Point2<N> = na::convert(mapped_point);

                let t = Translation2::from_vector(new_pt - anchor);
                new_pos = t * obj.position();
            }

            world.set_position(handle, new_pos);
        } else {
            self.camera.handle_event(&Event::MouseMoved { x: x, y: y })
        }
    }

    fn draw_collisions<T>(&mut self, world: &mut CollisionWorld<N, T>) {
        if self.draw_colls {
            draw_helper::draw_colls(&mut self.window, world);
        }
    }
}
